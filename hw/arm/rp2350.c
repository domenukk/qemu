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

typedef struct RP2350State RP2350State;

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
};

static uint64_t rp2350_sio_read(void *opaque, hwaddr addr, unsigned int size)
{
    if (addr >= 0x100 && addr <= 0x17c) {
        return 1; /* Spinlock: Always acquired successfully */
    }
    /* CPUID: 0x000 (defaults to 0, which is correct for core 0) */
    return 0;
}

static void rp2350_sio_write(void *opaque, hwaddr addr, uint64_t val, unsigned int size)
{
    /* Ignore writes for now */
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

static uint64_t rp2350_apb_dummy_read(void *opaque, hwaddr addr, unsigned int size)
{
    RP2350State *s = opaque;
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
            printf("APB GLOBAL READ! addr=0x%lx\n", addr);
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
    if (addr == 0x88008) {
        if (s->spi1_rx_fifo > 0) s->spi1_rx_fifo--;
        return 0x0;
    }
    /* SPI0 / SPI1 */
    if (addr == 0x8000c) {
        uint64_t sr = 0x3; /* TFE (0) and TNF (1) always empty/not full */
        if (s->spi0_rx_fifo > 0) sr |= 0x4; // RNE
        static int spi0_reads = 0;
        if (spi0_reads++ % 10000 == 0) {
            printf("APB LOOPING ON SPI0_SR (0x8000c) (count=%d) TFE=1 TNF=1 RNE=%ld rx_fifo=%d\n", spi0_reads, (long)(sr & 0x4) >> 2, s->spi0_rx_fifo);
        }
        return sr;
    }
    if (addr == 0x8800c) {
        uint64_t sr = 0x3; /* TFE and TNF always 1 */
        if (s->spi1_rx_fifo > 0) sr |= 0x4;
        static int spi1_reads = 0;
        if (spi1_reads++ % 100000 == 0) {
            printf("APB LOOPING ON SPI1_SR (0x8800c) (count=%d) TFE=1 TNF=1 RNE=%ld rx_fifo=%d\n", spi1_reads, (long)(sr & 0x4) >> 2, s->spi1_rx_fifo);
        }
        return sr;
    }
    if (addr == 0x80008) {
        if (s->spi0_rx_fifo > 0) s->spi0_rx_fifo--;
        return 0x0;
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
    
    /* I2C0/1 IC_DATA_CMD is 0x90010 / 0x98010 */
    if (addr == 0x90010 || addr == 0x98010) {
        if (addr == 0x90010) s->apb_regs[0x90034 / 4] |= 0x210;
        if (addr == 0x98010) s->apb_regs[0x98034 / 4] |= 0x210;
    }

    
    if (addr == 0x88008) { /* SPI1 DR */
        s->spi1_rx_fifo++;
        if (s->spi1_rx_fifo > 10) printf("SPI1 RX FIFO OVERFLOW! %d\n", s->spi1_rx_fifo);
    }
    if ((addr & 0xfff) == 0x008 && (addr & 0xf0000) == 0x80000) {
        /* Catch all aliases/sizes for SPI DR */
        static int spi1_dr_writes = 0;
        if (spi1_dr_writes++ % 10000 == 0) {
            printf("SPI DR WRITE PROGRESS! count=%d\n", spi1_dr_writes);
        }
        if ((addr & 0x7f000) == 0x08000) {
            /* spi1 */
            if (addr != 0x88008) s->spi1_rx_fifo++; /* duplicate increment if we missed the exact match */
        } else {
            /* spi0 */
            if (addr != 0x80008) s->spi0_rx_fifo++;
        }
    }
    
    if (addr == 0x80008) { /* SPI0 DR */
        s->spi0_rx_fifo++;
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
             
             printf("DMA WRITE! addr=0x%lx val=0x%08x alias=0x%lx\n", real_addr, val32, alias);

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
                     /* Dummy Audio DMA */
                     /* We must not finish immediately, otherwise the firmware's audio loop starves the main loop! */
                     /* Instead, we record the start time to complete it later in the read dummy. */
                     s->dma_regs[(ch * 0x40 + 0x0c) / 4] &= ~((1 << 24)); /* Clear BUSY - wait, leave BUSY set so it's not done yet */
                     /* Or just set a "target time" for when this channel finishes */
                     static int64_t last_audio_dma_us = 0;
                     if (last_audio_dma_us == 0) last_audio_dma_us = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
                     /* It will complete in read dummy when time > last_audio_dma_us + 1000 */
                 }
             }
        }
    }
    
    if (addr == 0x800) {
        printf("\n\n!!! BACKDOOR TRIGGERED! ENABLING QEMU TRACING !!!\n\n");
        fflush(stdout);
        qemu_set_log(CPU_LOG_EXEC | CPU_LOG_TB_IN_ASM, &error_abort);
    }
    if (addr == 0xa00) {
        printf("\n\n!!! FIRMWARE PANIC HANDLER CALLED !!!\n\n");
        fflush(stdout);
    }
    if (addr == 0xa04) {
        printf("%c", (char)(val & 0xff));
        fflush(stdout);
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
