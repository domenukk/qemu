#include "qemu/osdep.h"
#include "cpu.h"
#include "helper.h"

/* bit_cast helper */
static inline double u64_to_double(uint64_t v) {
    union { uint64_t u; double d; } u;
    u.u = v;
    return u.d;
}

static inline uint64_t double_to_u64(double d) {
    union { uint64_t u; double d; } u;
    u.d = d;
    return u.u;
}

void helper_dcp_mcrr(CPUARMState *env, uint32_t cp, uint32_t opc1, uint32_t crm, uint64_t val)
{
    int idx = (cp == 5) ? 1 : (env->v7m.secure ? 0 : 1);
    
    // printf("DCP MCRR: cp=%d opc1=%d crm=%d val=0x%016lx\n", cp, opc1, crm, val);
    
    if (opc1 == 1 && crm == 0) {
        /* WXUP: write X unpacked double-precision */
        env->dcp.xm[idx] = val;
    } else if (opc1 == 1 && crm == 1) {
        /* WYUP: write Y unpacked double-precision */
        env->dcp.ym[idx] = val;
    } else if (opc1 == 2) {
        if (crm == 0) {
            env->dcp.xm[idx] = (env->dcp.xm[idx] & 0xFFFFFFFF00000000ULL) | (val & 0xFFFFFFFFULL);
        } else if (crm == 1) {
            env->dcp.ym[idx] = (env->dcp.ym[idx] & 0xFFFFFFFF00000000ULL) | (val & 0xFFFFFFFFULL);
        }
    } else if (opc1 == 3) {
        if (crm == 0) {
            env->dcp.xm[idx] = (env->dcp.xm[idx] & 0x00000000FFFFFFFFULL) | ((val & 0xFFFFFFFFULL) << 32);
        } else if (crm == 1) {
            env->dcp.ym[idx] = (env->dcp.ym[idx] & 0x00000000FFFFFFFFULL) | ((val & 0xFFFFFFFFULL) << 32);
        }
    } else {
        printf("UNHANDLED DCP MCRR: cp=%d opc1=%d crm=%d val=0x%016lx\n", cp, opc1, crm, val);
        fflush(stdout);
    }
}

uint64_t helper_dcp_mrrc(CPUARMState *env, uint32_t cp, uint32_t opc1, uint32_t crm)
{
    int idx = (cp == 5) ? 1 : (env->v7m.secure ? 0 : 1);
    
    // printf("DCP MRRC: cp=%d opc1=%d crm=%d\n", cp, opc1, crm);
    
    if (opc1 == 1 && crm == 0) {
        /* RDDA: read DADD result */
        return env->dcp.xm[idx];
    } else if (opc1 == 5 && crm == 0) {
        /* RDR: read MUL result */
        double x = u64_to_double(env->dcp.xm[idx]);
        double y = u64_to_double(env->dcp.ym[idx]);
        return double_to_u64(x * y);
    } else if (opc1 == 0 && crm == 4) {
        return env->dcp.ym[idx];
    } else if (opc1 == 0 && crm == 5) {
        return env->dcp.xm[idx];
    } else {
        printf("UNHANDLED DCP MRRC: cp=%d opc1=%d crm=%d\n", cp, opc1, crm);
        fflush(stdout);
    }
    
    return 0;
}

void helper_dcp_cdp(CPUARMState *env, uint32_t cp, uint32_t opc1, uint32_t crn, uint32_t crm, uint32_t opc2)
{
    int idx = (cp == 5) ? 1 : (env->v7m.secure ? 0 : 1);
    
    // printf("DCP CDP: cp=%d opc1=%d crn=%d crm=%d opc2=%d\n", cp, opc1, crn, crm, opc2);
    
    if (opc1 == 0 && crn == 0 && crm == 1 && opc2 == 0) {
        /* ADD0: compare X and Y, set status */
        double x = u64_to_double(env->dcp.xm[idx]);
        double y = u64_to_double(env->dcp.ym[idx]);
        
        /* Set comparison status bits (bits 6-7) */
        if (x > y) {
            env->dcp.status[idx] = (env->dcp.status[idx] & ~(0x3 << 6)) | (0x1 << 6);
        } else if (x == y) {
            env->dcp.status[idx] = (env->dcp.status[idx] & ~(0x3 << 6)) | (0x0 << 6);
        } else {
            env->dcp.status[idx] = (env->dcp.status[idx] & ~(0x3 << 6)) | (0x2 << 6);
        }
    } else if (opc1 == 1 && crn == 0 && crm == 1 && opc2 == 0) {
        /* ADD1: add/sub */
        double x = u64_to_double(env->dcp.xm[idx]);
        double y = u64_to_double(env->dcp.ym[idx]);
        double r = x + y;
        env->dcp.xm[idx] = double_to_u64(r);
    } else if (opc1 == 8 && crn == 0 && crm == 0 && opc2 == 1) {
        /* NRDD: normalize and round double (NOP for us as we use C double) */
    } else {
        printf("UNHANDLED DCP CDP: cp=%d opc1=%d crn=%d crm=%d opc2=%d\n", cp, opc1, crn, crm, opc2);
        fflush(stdout);
    }
}

void helper_dcp_mcr(CPUARMState *env, uint32_t cp, uint32_t opc1, uint32_t crn, uint32_t crm, uint32_t opc2, uint32_t val)
{
    printf("UNHANDLED DCP MCR: cp=%d opc1=%d crn=%d crm=%d opc2=%d val=0x%08x\n", cp, opc1, crn, crm, opc2, val);
    fflush(stdout);
}

uint32_t helper_dcp_mrc(CPUARMState *env, uint32_t cp, uint32_t opc1, uint32_t crn, uint32_t crm, uint32_t opc2)
{
    int idx = (cp == 5) ? 1 : (env->v7m.secure ? 0 : 1);
    
    if (opc1 == 0 && crn == 0 && crm == 0 && opc2 == 1) {
        /* RCMP: read processed status (NZCV flags) */
        uint32_t status = env->dcp.status[idx];
        if ((status & (0x3 << 6)) == (0x1 << 6)) {
            return 0x20000000; // C=1
        } else if ((status & (0x3 << 6)) == (0x0 << 6)) {
            return 0x60000000; // Z=1, C=1
        } else {
            return 0x80000000; // N=1
        }
    }
    
    printf("UNHANDLED DCP MRC: cp=%d opc1=%d crn=%d crm=%d opc2=%d\n", cp, opc1, crn, crm, opc2);
    fflush(stdout);
    return 0;
}
