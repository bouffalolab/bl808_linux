/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2022 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */
#include "bflb_platform.h"
#include "rv_hpm.h"
#include "rv_pmp.h"
#include "hal_mtimer.h"
// #include "bl808_lz4d.h"
#include "bl808_psram_uhs.h"
#include "bl808_glb.h"
#include "bl808_gpio.h"

extern void unlz4(const void *aSource, void *aDestination, uint32_t FileLen);

#define PSRAM_BASIC_ADDR 0x50000000
#define VRAM_BASIC_ADDR  0x3f008000

#define VM_LINUX_SRC_ADDR 0x580A0000 // 4M 3980268
#define VM_LINUX_DST_ADDR 0x50000000

#define OPENSBI_SRC_ADDR 0x58090000 // 64K 0xc000
#define OPENSBI_DST_ADDR 0x3eff0000

#define DTB_SRC_ADDR 0x58080000 // 64k
#define DTB_DST_ADDR 0x51ff8000

#if (__riscv_xlen == 64)
/* linux pmp setting */
const pmp_config_entry_t pmp_entry_tab[8] = {
    [0] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x20000000,
        .entry_pa_length = 0x10000,
    },

    [1] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x30000000,
        .entry_pa_length = PMP_REG_SZ_1M,
    },

    [2] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x3eff0000,
        .entry_pa_length = 0x10000,
    },

    [3] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x40000000,
        .entry_pa_length = PMP_REG_SZ_16K,
    },

    [4] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_X | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x50000000,
        .entry_pa_length = 0x4000000,
    },

    [5] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0x58000000,
        .entry_pa_length = 0x4000000,
    },

    [6] = {
        .entry_flag = ENTRY_FLAG_ADDR_NAPOT | ENTRY_FLAG_PERM_W | ENTRY_FLAG_PERM_R,
        .entry_pa_base = 0xe0000000,
        .entry_pa_length = 0x8000000,
    },

    [7] = {
        .entry_flag = ENTRY_FLAG_ADDR_TOR,
        .entry_pa_base = 0xffffffffff, /* 40-bit PA */
        .entry_pa_length = 0,
    },
};

#endif

void linux_load()
{
    MSG("linux load start... \r\n");
    uint32_t *pSrc, *pDest;
    uint32_t header_kernel_len = 0;
    header_kernel_len = *(volatile uint32_t *)(VM_LINUX_SRC_ADDR - 4);
    /* Copy and unlz4 vm linux code */
    MSG("len:0x%08x\r\n", header_kernel_len);
    __NOP_BARRIER();

    unlz4((const void *)VM_LINUX_SRC_ADDR, (void *)VM_LINUX_DST_ADDR, header_kernel_len /*3980268*/ /*3993116 v0.3.0*/ /*4010168*/);

    /* let's start */
    /* there are 7bytes file head that lz4d HW IP does not need, skip! */
    // LZ4D_Decompress((const void *)(VM_LINUX_SRC_ADDR + 7), (void *)VM_LINUX_DST_ADDR);

    /* method 1: wait when done */
    // while (!LZ4D_GetStatus(LZ4D_STATUS_DONE))
    // ;
    // __ISB();
    MSG("vm linux load done!\r\n");

    /* Copy dtb code */
    pSrc = (uint32_t *)DTB_SRC_ADDR;
    pDest = (uint32_t *)DTB_DST_ADDR;
    memcpy((void *)pDest, (void *)pSrc, 0x10000);
    MSG("dtb load done!\r\n");

    /* Copy opensbi code */
    pSrc = (uint32_t *)OPENSBI_SRC_ADDR;
    pDest = (uint32_t *)OPENSBI_DST_ADDR;
    memcpy((void *)pDest, (void *)pSrc, 0xc000);
    MSG("opensbi load done!\r\n");

    csi_dcache_clean_invalid();
    // csi_dcache_clean();
}

int main(void)
{
    bflb_platform_init(0);
#ifdef CPU_M0
    MSG("E907 start...\r\n");
    mtimer_init();
    MSG("mtimer clk:%d\r\n", CPU_Get_MTimer_Clock());

    MSG("psram clk init ok!\r\n");
    // MSG("m0 main! size_t:%d\r\n", sizeof(size_t));

    csi_dcache_disable();
#ifdef DUALCORE
    BL_WR_WORD(IPC_SYNC_ADDR1, IPC_SYNC_FLAG);
    BL_WR_WORD(IPC_SYNC_ADDR2, IPC_SYNC_FLAG);
    L1C_DCache_Clean_By_Addr(IPC_SYNC_ADDR1, 8);
#endif
    while (1) {
#ifdef __riscv_muldiv
        int dummy;
        /* In lieu of a halt instruction, induce a long-latency stall. */
        __asm__ __volatile__("div %0, %0, zero"
                             : "=r"(dummy));
#endif
    }

#endif

#ifdef CPU_D0

    // #define GLB_AHB_CLOCK_LZ4 (0x0008000000000000UL)

    // GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_LZ4);
    MSG("C906 start...\r\n");
    uint64_t start_time, stop_time;
    mtimer_init();
    MSG("mtimer clk:%d\r\n", CPU_Get_MTimer_Clock());
    // MSG("C906 main! size_t:%d\r\n", sizeof(size_t));
    bflb_platform_delay_ms(100);

    void (*opensbi)(int hart_id, int fdt_addr) = (void (*)(int hart_id, int fdt_addr))OPENSBI_DST_ADDR;

    start_time = bflb_platform_get_time_us();
    linux_load();
    stop_time = bflb_platform_get_time_us();
    MSG("\r\nload time: %ld us \r\n", (stop_time - start_time));

    __ASM volatile("csrw mcor, %0"
                   :
                   : "r"(0x30013));
    // csi_dcache_disable();
    // csi_icache_disable();
    // __set_MHINT(0x450c);
    rvpmp_init(pmp_entry_tab, sizeof(pmp_entry_tab) / sizeof(pmp_config_entry_t));
    __ISB();

    /* Linux clk and rtc clk setting */
    // unsigned int reg;
    // reg = *(volatile unsigned int *)0x30007004;
    // reg |= (0x0 << 0); // pll div --> 400Mhz
    // *(volatile unsigned int *)0x30000018 = 0x8000017b;

    /* set core volt 1.2 */
    // *(volatile unsigned int *)0x2000f814 = 0x14476c20;

    /* set dtb addr */
    /* go to run linux ... */
    opensbi(0, DTB_DST_ADDR);
#endif

    while (1) {
#ifdef __riscv_muldiv
        int dummy;
        /* In lieu of a halt instruction, induce a long-latency stall. */
        __asm__ __volatile__("div %0, %0, zero"
                             : "=r"(dummy));
#endif
    }
}
