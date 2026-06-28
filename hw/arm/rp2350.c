// SPDX-License-Identifier: GPL-2.0-or-later
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/boards.h"

#include "hw/arm/boot.h"
#include "hw/core/qdev-clock.h"
#include "hw/char/pl011.h"
#include "system/system.h"
#include "hw/misc/unimp.h"
#include "hw/arm/armv7m.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/arm/machines-qom.h"
#include "system/address-spaces.h"
#include "exec/cpu-common.h"
#include "ui/console.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

#define TYPE_RP2350 "rp2350"
#define TYPE_RP2350_MACHINE MACHINE_TYPE_NAME(TYPE_RP2350)

/* ── PWM block constants ───────────────────────────────────────────────── */
#define PWM_APB_BASE        0xa8000  /* offset within the APB region          */
#define PWM_NUM_CHANNELS    12
#define PWM_CH_STRIDE       0x14     /* 5 registers × 4 bytes per channel     */
#define PWM_REGS_SIZE       (PWM_CH_STRIDE * PWM_NUM_CHANNELS + 0x20)

/* Per-channel register offsets (relative to channel base) */
#define PWM_CHx_CSR         0x00
#define PWM_CHx_DIV         0x04
#define PWM_CHx_CTR         0x08
#define PWM_CHx_CC          0x0c
#define PWM_CHx_TOP         0x10

/* CSR bits */
#define PWM_CSR_EN          (1u << 0)

/* Counter advance per read — enough for spin-loops to notice progress */
#define PWM_CTR_STEP        17

typedef struct RP2350State RP2350State;

/* Per-channel PWM state */
typedef struct {
    bool     enabled;   /* CSR.EN                  */
    uint16_t counter;   /* free-running counter    */
    uint16_t top;       /* wrap value (default 0xFFFF) */
} PwmChannel;

struct RP2350State {
    /*< private >*/
    MachineState parent_obj;
    /*< public >*/

    MemoryRegion flash;
    MemoryRegion sram;
    MemoryRegion apb_peripherals;
    uint32_t apb_regs[0x140000 / 4];
    MemoryRegion ahb_peripherals;
    uint32_t dma_regs[0x1000 / 4];

    int spi0_rx_fifo;
    int spi1_rx_fifo;
    MemoryRegion sio_peripherals;
    MemoryRegion usb_dpram;
    Object *armv7m;
    Clock *sysclk;
    QemuConsole *con;

    /* SIO inter-core FIFO emulation (loopback for single-core QEMU) */
    uint32_t sio_fifo[16];
    int sio_fifo_count;
    int sio_fifo_write_count; /* tracks spawn protocol progress */

    /* PWM emulation */
    PwmChannel pwm[PWM_NUM_CHANNELS];
};

/* SIO register offsets */
#define SIO_FIFO_ST  0x50  /* FIFO status: bit 0 = VLD, bit 1 = RDY */
#define SIO_FIFO_WR  0x54  /* FIFO write (to other core) */
#define SIO_FIFO_RD  0x58  /* FIFO read  (from other core) */

/* Number of words in the multicore spawn command sequence */
#define SPAWN_CMD_SEQ_LEN  6  /* [0, 0, 1, vtor, sp, entry] */

static uint64_t rp2350_sio_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350State *s = opaque;

    /* FIFO status: RDY (bit 1) always set; VLD (bit 0) set when data queued */
    if (addr == SIO_FIFO_ST) {
        uint32_t st = 0x02; /* RDY = 1 */
        if (s->sio_fifo_count > 0) {
            st |= 0x01;     /* VLD = 1 */
        }
        return st;
    }

    /* FIFO read: pop front of ring buffer */
    if (addr == SIO_FIFO_RD) {
        if (s->sio_fifo_count > 0) {
            uint32_t val = s->sio_fifo[0];
            s->sio_fifo_count--;
            memmove(&s->sio_fifo[0], &s->sio_fifo[1],
                    s->sio_fifo_count * sizeof(uint32_t));
            return val;
        }
        return 0;
    }

    /* Spinlocks: always acquired successfully */
    if (addr >= 0x100 && addr <= 0x17c) {
        return 1;
    }

    /* CPUID = 0 (core 0) */
    return 0;
}

static void rp2350_sio_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned int size)
{
    RP2350State *s = opaque;

    /*
     * FIFO write: loopback mode.
     *
     * On real hardware core0 writes to FIFO_WR and the ROM bootloader on
     * core1 echoes each word back.  In single-core QEMU we short-circuit
     * this by making every write immediately readable from FIFO_RD.
     *
     * After the 6-word spawn sequence [0, 0, 1, vtor, sp, entry] the HAL
     * expects one more read of value 1 (the core1-entry ack from
     * multicore.rs).  Since core1 never runs, we auto-enqueue that ack.
     */
    if (addr == SIO_FIFO_WR) {
        if (s->sio_fifo_count < 16) {
            s->sio_fifo[s->sio_fifo_count++] = (uint32_t)val;
        }
        s->sio_fifo_write_count++;
        if (s->sio_fifo_write_count == SPAWN_CMD_SEQ_LEN
                && s->sio_fifo_count < 16) {
            s->sio_fifo[s->sio_fifo_count++] = 1; /* core1 ack */
        }
        return;
    }

    /* FIFO_ST write-to-clear (ROE/WOF bits) — safe to ignore */
}

static const MemoryRegionOps rp2350_sio_ops = {
    .read = rp2350_sio_read,
    .write = rp2350_sio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ── PWM helpers ───────────────────────────────────────────────────────── */

/*
 * Advance the counter of channel `ch` by `PWM_CTR_STEP`, wrapping at
 * TOP + 1.  Returns the new counter value.  This is intentionally a
 * simple "increment on read" model — no wall-clock dependency — which
 * is sufficient for firmware spin-loops that just need to observe the
 * counter changing.
 */
static uint16_t pwm_advance_counter(PwmChannel *ch)
{
    uint32_t wrap = (uint32_t)ch->top + 1;
    uint32_t next = (uint32_t)ch->counter + PWM_CTR_STEP;
    ch->counter = (uint16_t)(next % wrap);
    return ch->counter;
}

static uint64_t rp2350_apb_dummy_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350State *s = opaque;
    /* ── PWM counter reads ──────────────────────────────────────────── */
    if (addr >= PWM_APB_BASE
            && addr < PWM_APB_BASE + PWM_NUM_CHANNELS * PWM_CH_STRIDE) {
        uint32_t offset  = addr - PWM_APB_BASE;
        int      ch_idx  = offset / PWM_CH_STRIDE;
        uint32_t ch_off  = offset % PWM_CH_STRIDE;

        if (ch_off == PWM_CHx_CTR && s->pwm[ch_idx].enabled) {
            /* Simulate time passing: advance and return the counter */
            return pwm_advance_counter(&s->pwm[ch_idx]);
        }
        /* CSR / DIV / CC / TOP — fall through to generic apb_regs[] */
    }

    /* XOSC STATUS */
    if (addr == 0x48004) {
        return 0x80000000; /* STABLE */
    }
    /* PLL_SYS CS / PLL_USB CS */
    if (addr == 0x50000 || addr == 0x58000) {
        return 0x80000000; /* LOCK */
    }
    
    /* CLOCKS SELECTED registers */
    if (addr == 0x10038 || addr == 0x10044 || addr == 0x10050 || addr == 0x1005c || addr == 0x10068 || addr == 0x10074) {
        uint32_t ctrl = s->apb_regs[(addr - 8) / 4];
        uint32_t src = ctrl & 3;
        return 1 << src;
    }

    /* RESETS RESET_DONE register */
    if (addr == 0x20008) {
        return 0xffffffff; /* All peripherals resets are done */
    }

    /* TIMER0 / TIMER1 TIMELR / TIMERAWL */
    if (addr == 0xb000c || addr == 0xb0028 || addr == 0xb800c || addr == 0xb8028) {
        static int timer_reads = 0;
        static int64_t fake_us = 0;
        fake_us += 100; // Warp forward 100 microseconds per iteration
        int64_t usec = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + fake_us;
        if (timer_reads++ > 5000 && timer_reads % 10000 == 0) {
            printf("HEARTBEAT: Firmware is running in main loop! timer_reads=%d usec=%ld\n", timer_reads, (long)usec);
        }
        return usec;
    }
    if (addr != 0x88008 && addr != 0x8800c && addr != 0x80008 && addr != 0x8000c) {
        static int unhandled_reads_global = 0;
        if (unhandled_reads_global++ < 100) {
            // printf("APB GLOBAL READ! addr=0x%lx\n", addr);
        }
    }

    /* I2C0/1 IC_STATUS register is 0x90070 / 0x98070 */
    if (addr == 0x90070 || addr == 0x98070) {
        static int i2c_status_reads = 0;
        if (i2c_status_reads++ < 100) {
            printf("I2C STAT READ! addr=0x%lx returning 0xE\n", addr);
        }
        return 0x6 | 0x8; /* TFNF | TFE | RFNE (RX Not Empty) */
    }
    
    /* I2C0/1 IC_RXFLR is 0x90078 / 0x98078 */
    if (addr == 0x90078 || addr == 0x98078) {
        return 1; /* Always pretend there is 1 byte in the RX FIFO for dummy reads */
    }
    
    /* I2C0/1 IC_DATA_CMD is 0x90010 / 0x98010 */
    if (addr == 0x90010 || addr == 0x98010) {
        /* On dummy read of data, we set STOP_DET and TX_EMPTY in the RAW_INTR_STAT so the next status check finishes */
        if (addr == 0x90010) s->apb_regs[0x90034 / 4] |= 0x210;
        if (addr == 0x98010) s->apb_regs[0x98034 / 4] |= 0x210;
        return 0x00; /* Dummy I2C read data */
    }
    /* I2C0/1 IC_CLR_INTR is 0x90040 / 0x98040 */
    if (addr == 0x90040 || addr == 0x98040) {
        if (addr == 0x90040) s->apb_regs[0x90034 / 4] = 0x0;
        if (addr == 0x98040) s->apb_regs[0x98034 / 4] = 0x0;
        return 0x0;
    }
    if (addr == 0x80008 || addr == 0x88008) {
        // Handled below specifically
    } else if (addr == 0x1000) {
        return 0; // dummy
    }
    if (addr == 0x8000c) {
        uint64_t sr = 0x3; /* TFE (0) and TNF (1) always empty/not full */
        if (s->spi0_rx_fifo > 0) sr |= 0x4; // RNE
        // Bit 4 is BSY. We keep it 0 to avoid blocking the firmware.
        static int spi0_reads = 0;
        if (spi0_reads++ % 1000000 == 0) {
            printf("APB LOOPING ON SPI0_SR (0x8000c) (count=%d) TNF=1 RNE=%ld rx_fifo=%d\n", spi0_reads, (long)(sr & 0x4) >> 2, s->spi0_rx_fifo);
        }
        return sr;
    }
    if (addr == 0x8800c) {
        uint64_t sr = 0x3; /* TFE (0) and TNF (1) always empty/not full */
        if (s->spi1_rx_fifo > 0) sr |= 0x4; // RNE
        static int spi1_reads = 0;
        if (spi1_reads++ % 1000000 == 0) {
            printf("APB LOOPING ON SPI1_SR (0x8800c) (count=%d) TNF=1 RNE=%ld rx_fifo=%d\n", spi1_reads, (long)(sr & 0x4) >> 2, s->spi0_rx_fifo);
        }
        return sr;
    }

    if (addr == 0x80008) {
        uint32_t val = 0xFF; // Default for disconnected SPI0 (SD Card)
        if (s->spi0_rx_fifo > 0) {
            s->spi0_rx_fifo--;
            val = 0x0; // Dummy response for now
        }
        return val;
    }
    if (addr == 0x88008) {
        uint32_t val = 0x0;
        if (s->spi1_rx_fifo > 0) {
            s->spi1_rx_fifo--;
        }
        return val;
    }

    /* RESETS block: always report everything as out-of-reset (RESET_DONE = 1) */
    if (addr == 0x20008) return 0xFFFFFFFF; /* RESET_DONE */
    if (addr == 0x20000) return 0x00000000; /* RESET (0 = not in reset) */

    /* XOSC / ROSC status: always report STABLE (bit 31) */
    if (addr == 0x24004 || addr == 0x1c000 || addr == 0x24000) return 0x80000000;

    /* ADC block status: always report READY */
    if (addr >= 0x50000 && addr <= 0x50020) return 0xFFFFFFFF;

    /* RTC or Watchdog status: always report ACTIVE/READY */
    if (addr >= 0x58000 && addr <= 0x5800c) return 0xFFFFFFFF;

    /* PSM Power State Machine: always report DONE */
    if (addr >= 0x38000 && addr <= 0x38fff) return 0xFFFFFFFF;

    /* CLOCKS module selected glitch-less mux */
    if (addr == 0x10030 || addr == 0x1003c || addr == 0x10044 || addr == 0x10050 || addr == 0x1005c || addr == 0x10068 || addr == 0x10074 || addr == 0x10080 || addr == 0x10008 || addr == 0x10014 || addr == 0x10020 || addr == 0x1002c) {
        uint32_t ctrl = s->apb_regs[(addr - 4) / 4];
        return 1 << (ctrl & 0x7);
    }
    /* Rest of CLOCKS block: blanket ready response */
    if (addr >= 0x10000 && addr <= 0x100fc) return 0xFFFFFFFF;

    static hwaddr last_addr = 0;
    static int repeat_count = 0;
    if (addr == last_addr) {
        repeat_count++;
        if (repeat_count == 100 || repeat_count == 10000) {
            printf("APB LOOPING ON addr=0x%lx (count=%d)\n", addr, repeat_count);
        }
    } else {
        last_addr = addr;
        repeat_count = 0;
    }
    
    if (addr < 0x140000) {
        return s->apb_regs[addr / 4];
    }
    
    return 0;
}

static void rp2350_apb_dummy_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size)
{
    RP2350State *s = opaque;

    /* ── PWM register writes ───────────────────────────────────────── */
    if (addr >= PWM_APB_BASE
            && addr < PWM_APB_BASE + PWM_NUM_CHANNELS * PWM_CH_STRIDE) {
        uint32_t offset = addr - PWM_APB_BASE;
        int      ch_idx = offset / PWM_CH_STRIDE;
        uint32_t ch_off = offset % PWM_CH_STRIDE;

        switch (ch_off) {
        case PWM_CHx_CSR:
            s->pwm[ch_idx].enabled = (val & PWM_CSR_EN) != 0;
            if (s->pwm[ch_idx].enabled) {
                s->pwm[ch_idx].counter = 0; /* reset on enable */
            }
            break;
        case PWM_CHx_TOP:
            s->pwm[ch_idx].top = (uint16_t)(val & 0xFFFF);
            break;
        default:
            break; /* DIV, CC, CTR — stored in apb_regs[] below */
        }
        /* fall through to generic store so reads of CSR/DIV/CC/TOP work */
    }
    
    /* I2C0/1 IC_DATA_CMD is 0x90010 / 0x98010 */
    if (addr == 0x90010 || addr == 0x98010) {
        if (addr == 0x90010) s->apb_regs[0x90034 / 4] |= 0x210;
        if (addr == 0x98010) s->apb_regs[0x98034 / 4] |= 0x210;
    }

    
    if (addr == 0x80008) { /* SPI0 DR */
        s->spi0_rx_fifo++;
    }
    if (addr == 0x88008) { /* SPI1 DR */
        s->spi1_rx_fifo++;
        if (s->spi1_rx_fifo > 10) printf("SPI1 RX FIFO OVERFLOW! %d\n", s->spi1_rx_fifo);
    }
    if ((addr & 0xfff) == 0x008 && (addr & 0xf0000) == 0x80000) {
        /* Catch all aliases/sizes for SPI DR */
        static int spi_dr_writes = 0;
        if (spi_dr_writes++ % 100000 == 0) {
            // printf("SPI DR WRITE PROGRESS! addr=0x%lx count=%d\n", addr, spi_dr_writes);
        }
    }


    if (addr == 0x30000) {
        printf("apb_dummy intercepted UART0 write! val=0x%lx\n", val);
    }

    if (addr < 0x140000) {
        if (size == 1) {
            uint32_t shift = (addr & 3) * 8;
            s->apb_regs[addr / 4] = (s->apb_regs[addr / 4] & ~(0xff << shift)) | ((val & 0xff) << shift);
        } else if (size == 2) {
            uint32_t shift = (addr & 2) * 8;
            s->apb_regs[addr / 4] = (s->apb_regs[addr / 4] & ~(0xffff << shift)) | ((val & 0xffff) << shift);
        } else {
            s->apb_regs[addr / 4] = val;
        }
    }
}

static const MemoryRegionOps rp2350_apb_dummy_ops = {
    .read = rp2350_apb_dummy_read,
    .write = rp2350_apb_dummy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static uint64_t rp2350_ahb_dummy_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350State *s = opaque;
    hwaddr base_addr = addr & ~0x3fff;
    hwaddr real_addr = addr & 0xfff;
    
    if (base_addr == 0x0) { /* DMA is at 0x50000000. So offset 0x0. */
        static int64_t last_audio_time = 0;
        int64_t now_us = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
        if (last_audio_time == 0) last_audio_time = now_us;
        
        /* Complete Audio DMA every 5ms to pace the firmware */
        if (now_us > last_audio_time + 5000) {
            last_audio_time = now_us;
            /* If CH0 is busy (meaning it was started but not finished yet), finish it */
            if (s->dma_regs[0x0c / 4] & 1) {
                s->dma_regs[0x0c / 4] &= ~((1<<24) | 1); /* Clear BUSY and EN */
                s->dma_regs[0x400 / 4] |= 1;
            } else if (s->dma_regs[0x4c / 4] & 1) {
                s->dma_regs[0x4c / 4] &= ~((1<<24) | 1);
                s->dma_regs[0x400 / 4] |= 2;
            }
        }

        uint32_t val = s->dma_regs[real_addr / 4];
        
        if (real_addr == 0x400) {
            static int intr_read_count = 0;
            if (intr_read_count++ % 10000 == 0) {
                printf("DMA INTR READ! count=%d val=0x%08x now_us=%ld last_audio=%ld\n", intr_read_count, val, (long)now_us, (long)last_audio_time);
                fflush(stdout);
            }
        }
        
        return val;
    }
    return 0;
}

static void rp2350_invalidate_display(void *d)
{
}

static void rp2350_update_display(void *d)
{
}

static const GraphicHwOps rp2350_gfx_ops = {
    .invalidate  = rp2350_invalidate_display,
    .gfx_update  = rp2350_update_display,
};

static void rp2350_ahb_dummy_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size)
{
    RP2350State *s = opaque;
    hwaddr base_addr = addr & ~0x3fff;
    hwaddr real_addr = addr & 0xfff;
    hwaddr alias = addr & 0x3000;

    if (base_addr == 0x0) { /* DMA block */
        if (size == 4) {
             uint32_t val32 = (uint32_t)val;
             uint32_t *reg = &s->dma_regs[real_addr / 4];
            // printf("DMA WRITE! addr=0x%lx val=0x%08x alias=0x%lx\n", real_addr, val32, alias);

             if (real_addr == 0xa00) {
                 printf("\n\n\n=========================================\n");
                 printf("          FIRMWARE PANICKED!!!!!         \n");
                 printf("=========================================\n");
             }
             if (real_addr == 0xa04) {
                 printf("%c", (char)(val32 & 0xff));
                 fflush(stdout);
             }
             if (real_addr == 0x800) {
                 printf("!!! BACKDOOR TRIGGERED! ENABLING QEMU TRACING !!!\n");
                 qemu_set_log(CPU_LOG_EXEC | CPU_LOG_TB_IN_ASM, &error_abort);
                 fflush(stdout);
             }

             if (real_addr == 0x400) {
                 /* INTR is Write-to-Clear (WC) */
                 if (alias == 0x1000) { *reg ^= val32; } /* XOR -> emulate normally for bits? Actually WC XOR means what? Just clear matching bits. */
                 else if (alias == 0x2000) { *reg &= ~val32; } /* Atomic Set -> WC means clear these bits! */
                 else if (alias == 0x3000) { *reg &= ~val32; } /* Atomic Clear -> no effect on WC or clear? */
                 else { *reg &= ~val32; } /* Standard write to WC clears matching bits */
             } else {
                 if (alias == 0x1000) { *reg ^= val32; }
                 else if (alias == 0x2000) { *reg |= val32; }
                 else if (alias == 0x3000) { *reg &= ~val32; }
                 else { *reg = val32; }
             }

             /* Just sync the ALIAS regs manually to base regs for CH2 so our crude logic works */
             if (real_addr == 0x90) s->dma_regs[0x8c / 4] = val32; /* AL1_CTRL */
             if (real_addr == 0x9c) s->dma_regs[0x80 / 4] = val32; /* AL2_READ_ADDR */
             if (real_addr == 0x98) s->dma_regs[0x88 / 4] = val32; /* AL2_TRANS_COUNT */
             /* We don't bother for every single alias combinations, just mirror locally if needed. */

             /* Generalize trigger interception for all channels */
             int ch = real_addr / 0x40;
             int offset = real_addr % 0x40;
             int is_trigger = (offset == 0x0c) || (offset == 0x10) || (offset == 0x1c) || (offset == 0x20) || (offset == 0x24);

             if (is_trigger && (val32 & 1 || offset != 0x10)) {
                 if (ch == 2) {
                     /* HACK: let's resolve read_addr from ALL possible aliases just in case */
                     uint32_t read_addr = s->dma_regs[0x80 / 4]; if (!read_addr) read_addr = s->dma_regs[0x9c / 4];
                     uint32_t write_addr = s->dma_regs[0x84 / 4];
                     uint32_t count_words = s->dma_regs[0x88 / 4]; if (!count_words) count_words = s->dma_regs[0x98 / 4];
                     
                     static int print_count = 0;
                     if (print_count++ < 20) {
                         printf("DMA CH2 TRIG_ATTEMPT! r=0x%08x w=0x%08x c=%d real_addr=0x%lx val=0x%08x\n", read_addr, write_addr, count_words, real_addr, val32);
                     }

                     /* Check if writing to SPI1 SSPDR (0x40088008 or SPI0 0x40080008) */
                     if ((write_addr == 0x40080008 || write_addr == 0x40088008) && count_words == 240 * 320) {
                         /* We caught the Framebuffer transfer! */
                         uint16_t fb[240 * 320];
                         cpu_physical_memory_read(read_addr, fb, sizeof(fb));

                         /* Ensure QemuConsole exists and is 240x320 */
                         DisplaySurface *surface = qemu_console_surface(s->con);
                         if (surface && surface_width(surface) == 240 && surface_height(surface) == 320) {
                             /* Render the 16-bit RGB565 to the exact display format */
                             /* The firmware swapped byte order for SPI, so we swap it back */
                             uint32_t *dest = (uint32_t *)surface_data(surface);
                             for (int i = 0; i < 240 * 320; i++) {
                                 uint16_t pix = fb[i];
                                 uint8_t r = (pix >> 11) & 0x1F;
                                 uint8_t g = (pix >> 5) & 0x3F;
                                 uint8_t b = pix & 0x1F;
                                 dest[i] = ((r << 3) | (r >> 2)) << 16 |
                                           ((g << 2) | (g >> 4)) << 8  |
                                           ((b << 3) | (b >> 2));
                             }
                             dpy_gfx_update(s->con, 0, 0, 240, 320);
                             
                             static int frame_count = 0;
                             if (frame_count++ % 30 == 0) {
                                 printf("FRAMEBUFFER RENDERED! Frame %d\n", frame_count);
                             }
                         }
                         /* Return status immediately: done, no busy */
                         s->dma_regs[(ch * 0x40 + 0x0c) / 4] &= ~((1 << 24) | 1); /* Clear BUSY and EN */
                     }
                 } else if (ch == 0 || ch == 1) {
                     /* Audio DMA: Extract PCM frame and output to raw file */
                     uint32_t read_addr = s->dma_regs[(ch * 0x40 + 0x00) / 4];
                     if (!read_addr) read_addr = s->dma_regs[(ch * 0x40 + 0x1c) / 4];
                     uint32_t count_words = s->dma_regs[(ch * 0x40 + 0x08) / 4];
                     if (!count_words) count_words = s->dma_regs[(ch * 0x40 + 0x18) / 4];
                     
                     if (count_words > 0 && count_words <= 4096 && read_addr != 0) {
                         static FILE *audio_file = NULL;
                         if (!audio_file) {
                             audio_file = fopen("qemu_audio_out.raw", "wb");
                         }
                         if (audio_file) {
                             uint8_t buffer[16384]; /* max 4096 words * 4 */
                             int bytes_to_read = count_words * 4;
                             cpu_physical_memory_read(read_addr, buffer, bytes_to_read);
                             fwrite(buffer, 1, bytes_to_read, audio_file);
                             fflush(audio_file);
                         }
                     }

                     /* Delay completion to prevent firmware lockup */
                     s->dma_regs[(ch * 0x40 + 0x0c) / 4] &= ~((1 << 24)); 
                     static int64_t last_audio_dma_us = 0;
                     if (last_audio_dma_us == 0) last_audio_dma_us = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
                 }
             }
        }
    }
}

static const MemoryRegionOps rp2350_ahb_dummy_ops = {
    .read = rp2350_ahb_dummy_read,
    .write = rp2350_ahb_dummy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void rp2350_init(Object *obj)
{
    /* QOM instance init */
}

static void rp2350_machine_init(MachineState *machine)
{
    RP2350State *s = (RP2350State *)machine;
    MemoryRegion *sysmem = get_system_memory();

    /* Initialise PWM channels to datasheet reset values */
    for (int i = 0; i < PWM_NUM_CHANNELS; i++) {
        s->pwm[i].enabled = false;
        s->pwm[i].counter = 0;
        s->pwm[i].top     = 0xFFFF;  /* default wrap value per datasheet */
    }

    /* initialize the memory regions */
    memory_region_init_rom(&s->flash, NULL, "rp2350.flash", 0x200000, &error_fatal);
    memory_region_init_ram(&s->sram, NULL, "rp2350.sram", 0x82000, &error_fatal);
    memory_region_init_ram(&s->usb_dpram, NULL, "rp2350.usb_dpram", 4 * 1024, &error_fatal);

    /* map the memory regions */
    memory_region_add_subregion(sysmem, 0x10000000, &s->flash);
    memory_region_add_subregion(sysmem, 0x20000000, &s->sram);
    memory_region_add_subregion_overlap(sysmem, 0x50100000, &s->usb_dpram, 1);

    memory_region_init_io(&s->apb_peripherals, OBJECT(s), &rp2350_apb_dummy_ops, s, "rp2350.apb_dummy", 0x140000);
    memory_region_add_subregion_overlap(sysmem, 0x40000000, &s->apb_peripherals, -999);

    memory_region_init_io(&s->ahb_peripherals, OBJECT(s), &rp2350_ahb_dummy_ops, s, "rp2350.ahb", 0x8000000);
    memory_region_add_subregion(sysmem, 0x50000000, &s->ahb_peripherals);

    memory_region_init_io(&s->sio_peripherals, OBJECT(s), &rp2350_sio_ops, s, "rp2350.sio", 0x10000);
    memory_region_add_subregion(sysmem, 0xd0000000, &s->sio_peripherals);

    /* initialize the CPU */
    s->armv7m = object_new(TYPE_ARMV7M);
    qdev_prop_set_string(DEVICE(s->armv7m), "cpu-type", machine->cpu_type);
    qdev_prop_set_bit(DEVICE(s->armv7m), "enable-bitband", false);
    qdev_prop_set_uint32(DEVICE(s->armv7m), "init-svtor", 0x10000000);
    qdev_prop_set_uint32(DEVICE(s->armv7m), "init-nsvtor", 0x10000000);
    object_property_set_link(s->armv7m, "memory", OBJECT(sysmem), &error_fatal);

    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, 150000000);
    qdev_connect_clock_in(DEVICE(s->armv7m), "cpuclk", s->sysclk);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->armv7m), &error_fatal);

    /* instantiate peripherals */
    pl011_create(0x40070000, qdev_get_gpio_in(DEVICE(s->armv7m), 20), serial_hd(0));
    /* instantiate UART1 for MIDI */
    pl011_create(0x40078000, qdev_get_gpio_in(DEVICE(s->armv7m), 21), serial_hd(1));

    if (machine->kernel_filename) {
        armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename, 0x200000, 0x200000);
    }

    s->con = graphic_console_init(NULL, 0, &rp2350_gfx_ops, s);
    qemu_console_resize(s->con, 240, 320);

    /* Initialize DMA interrupts to 0b11 to jump-start the firmware audio loop in Qemu */
    s->dma_regs[0x400 / 4] = 3;
}

static void rp2350_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RP2350 board";
    mc->init = rp2350_machine_init;
    mc->max_cpus = 2;
    mc->default_ram_size = 0x82000;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
}

static const TypeInfo rp2350_machine_info = {
    .name = TYPE_RP2350_MACHINE,
    .parent = TYPE_MACHINE,
    .class_size = sizeof(MachineClass),
    .instance_size = sizeof(RP2350State),
    .instance_init = rp2350_init,
    .class_init = rp2350_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void rp2350_register_types(void)
{
    type_register_static(&rp2350_machine_info);
}

type_init(rp2350_register_types)
