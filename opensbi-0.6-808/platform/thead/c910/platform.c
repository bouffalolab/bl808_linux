/*
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sbi/riscv_encoding.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_console.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/sys/clint.h>
#include <sbi_utils/serial/uart8250.h>
#include "platform.h"

#include "bflb_uart.h"

#define BFLB_UART_CLK (40000000UL)
#define BFLB_UART_BAUDRATE (2000000UL)
#define BFLB_CONSOLE_BASE (0x30002000)
// #define BFLB_CONSOLE_BASE (0x2000A000)

static struct c910_regs_struct c910_regs;

static void c910_delegate_traps()
{
	unsigned long exceptions = csr_read(CSR_MEDELEG);

	/* Delegate 0 ~ 7 exceptions to S-mode */
	exceptions |=
		((1U << CAUSE_MISALIGNED_FETCH) | (1U << CAUSE_FETCH_ACCESS) |
		 (1U << CAUSE_ILLEGAL_INSTRUCTION) | (1U << CAUSE_BREAKPOINT) |
		 (1U << CAUSE_MISALIGNED_LOAD) | (1U << CAUSE_LOAD_ACCESS) |
		 (1U << CAUSE_MISALIGNED_STORE) | (1U << CAUSE_STORE_ACCESS));

	csr_write(CSR_MEDELEG, exceptions);
}

static int c910_early_init(bool cold_boot)
{
	if (cold_boot) {
		/* Load from boot core */
		c910_regs.pmpaddr0 = csr_read(CSR_PMPADDR0);
		c910_regs.pmpaddr1 = csr_read(CSR_PMPADDR1);
		c910_regs.pmpaddr2 = csr_read(CSR_PMPADDR2);
		c910_regs.pmpaddr3 = csr_read(CSR_PMPADDR3);
		c910_regs.pmpaddr4 = csr_read(CSR_PMPADDR4);
		c910_regs.pmpaddr5 = csr_read(CSR_PMPADDR5);
		c910_regs.pmpaddr6 = csr_read(CSR_PMPADDR6);
		c910_regs.pmpaddr7 = csr_read(CSR_PMPADDR7);
		c910_regs.pmpcfg0  = csr_read(CSR_PMPCFG0);

		c910_regs.mcor	   = csr_read(CSR_MCOR);
		c910_regs.mhcr	   = csr_read(CSR_MHCR);
		c910_regs.mccr2	   = csr_read(CSR_MCCR2);
		c910_regs.mhint	   = csr_read(CSR_MHINT);
		c910_regs.mxstatus = csr_read(CSR_MXSTATUS);

		c910_regs.plic_base_addr = csr_read(CSR_PLIC_BASE);
		c910_regs.clint_base_addr =
			c910_regs.plic_base_addr + C910_PLIC_CLINT_OFFSET;
	} else {
		/* Store to other core */
		csr_write(CSR_PMPADDR0, c910_regs.pmpaddr0);
		csr_write(CSR_PMPADDR1, c910_regs.pmpaddr1);
		csr_write(CSR_PMPADDR2, c910_regs.pmpaddr2);
		csr_write(CSR_PMPADDR3, c910_regs.pmpaddr3);
		csr_write(CSR_PMPADDR4, c910_regs.pmpaddr4);
		csr_write(CSR_PMPADDR5, c910_regs.pmpaddr5);
		csr_write(CSR_PMPADDR6, c910_regs.pmpaddr6);
		csr_write(CSR_PMPADDR7, c910_regs.pmpaddr7);
		csr_write(CSR_PMPCFG0, c910_regs.pmpcfg0);

		csr_write(CSR_MCOR, c910_regs.mcor);
		csr_write(CSR_MHCR, c910_regs.mhcr);
		csr_write(CSR_MHINT, c910_regs.mhint);
		csr_write(CSR_MXSTATUS, c910_regs.mxstatus);
	}

	{
#if 1
		unsigned long tmp;

		/* clock */
		tmp = readl((void *)(0x30007000));
		tmp |= (1 << 10);
		writel(tmp, (void *)(0x30007000));
		/* pinmux */
		tmp = readl((void *)(0x200008e4));
		writel(0x00401502, (void *)0x200008e4); //IO8, TXD
		writel(0x00401517, (void *)0x200008d8); //IO5, RXD
		//UART_IntMask(UART_DBG_ID, UART_INT_ALL, MASK);
		tmp = readl((void *)(BFLB_CONSOLE_BASE + UART_INT_MASK_OFFSET));
		tmp |= 0xfff;
		writel(tmp, (void *)(BFLB_CONSOLE_BASE + UART_INT_MASK_OFFSET));

		//UART_Disable(UART_DBG_ID,UART_TXRX);
		tmp = readl(
			(void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		tmp &= UART_CR_UTX_EN_UMSK;
		writel(tmp,
		       (void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		tmp = readl(
			(void *)(BFLB_CONSOLE_BASE + UART_URX_CONFIG_OFFSET));
		tmp &= UART_CR_URX_EN_UMSK;
		writel(tmp,
		       (void *)(BFLB_CONSOLE_BASE + UART_URX_CONFIG_OFFSET));

		//UART_Init(UART_DBG_ID, &uart_dbg_cfg);
		tmp = BFLB_UART_CLK / BFLB_UART_BAUDRATE - 1;
		tmp = (tmp << 16) | tmp;
		writel(tmp, (void *)(BFLB_CONSOLE_BASE +
				     UART_BIT_PRD_OFFSET)); /* 2000000bps */
		writel(0xf04,
		       (void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		writel(0x700,
		       (void *)(BFLB_CONSOLE_BASE + UART_URX_CONFIG_OFFSET));
		writel(0,
		       (void *)(BFLB_CONSOLE_BASE + UART_DATA_CONFIG_OFFSET));
		writel(0, (void *)(BFLB_CONSOLE_BASE + UART_SW_MODE_OFFSET));
		//UART_FifoConfig(UART_DBG_ID,&fifoCfg);
		writel(0xf0f0080,
		       (void *)(BFLB_CONSOLE_BASE + UART_FIFO_CONFIG_1_OFFSET));
		writel(0x80,
		       (void *)(BFLB_CONSOLE_BASE + UART_FIFO_CONFIG_0_OFFSET));
		//UART_TxFreeRun(UART_DBG_ID,ENABLE);
		writel(0xf04,
		       (void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		//UART_SetRxTimeoutValue(UART_DBG_ID,80);
		writel(0x4f,
		       (void *)(BFLB_CONSOLE_BASE + UART_URX_RTO_TIMER_OFFSET));
		//UART_IntMask(UART_DBG_ID,UART_INT_RX_FIFO_REQ,UNMASK);
		tmp = readl((void *)(BFLB_CONSOLE_BASE + UART_INT_MASK_OFFSET));
		tmp &= UART_CR_URX_FIFO_MASK_UMSK;
		//writel(tmp, BFLB_CONSOLE_BASE + UART_INT_MASK_OFFSET);
		//UART_Enable(UART_DBG_ID,UART_TXRX);
		tmp = readl(
			(void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		tmp |= UART_CR_UTX_EN_MSK;
		writel(tmp,
		       (void *)(BFLB_CONSOLE_BASE + UART_UTX_CONFIG_OFFSET));
		tmp = readl(
			(void *)(BFLB_CONSOLE_BASE + UART_URX_CONFIG_OFFSET));
		tmp |= UART_CR_URX_EN_MSK;
		writel(tmp,
		       (void *)(BFLB_CONSOLE_BASE + UART_URX_CONFIG_OFFSET));
#endif
	}

	return 0;
}

static int c910_final_init(bool cold_boot)
{
	c910_delegate_traps();

	return 0;
}

static int c910_irqchip_init(bool cold_boot)
{
	/* Delegate plic enable into S-mode */
	writel(C910_PLIC_DELEG_ENABLE,
	       (void *)c910_regs.plic_base_addr + C910_PLIC_DELEG_OFFSET);

	return 0;
}

static int c910_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_ipi_init(c910_regs.clint_base_addr,
					 C910_HART_COUNT);
		if (rc)
			return rc;
	}

	return clint_warm_ipi_init();
}

static u64 c910_timer_value(void)
{
	/* Read from CSR */
	return csr_read(CSR_TIME);
}

static int c910_timer_init(bool cold_boot)
{
	int ret;

	/* clock enable */
	unsigned int reg;

	reg = *(volatile unsigned int *)0x30007000;
	reg |= (1 << 10); // mm_xclk <--- xtal
	reg |= (3 << 4);  // mm_uart_clk <--- mm_xclk
	// reg &= ~(1 << 11); // mm_cpu_clk <--- xclk
	*(volatile unsigned int *)0x30007000 = reg;
	__asm volatile("fence.i" ::: "memory");

	/* setting c906 rtc timer 1000000Hz */
	*(volatile unsigned int *)0x30000018 = 0x8000017b;
	if (cold_boot) {
		ret = clint_cold_timer_init(c910_regs.clint_base_addr,
					    C910_HART_COUNT, FALSE);
		if (ret)
			return ret;
	}

	return clint_warm_timer_init();
}

static int c910_system_shutdown(u32 type)
{
	asm volatile("ebreak");
	return 0;
}

static void c910_uart_putc(char ch)
{
	/* wait for FIFO */
	while (!(
		readl((void *)(BFLB_CONSOLE_BASE + UART_FIFO_CONFIG_1_OFFSET)) &
		UART_TX_FIFO_CNT_MSK))
		;
	writel(ch, (void *)(BFLB_CONSOLE_BASE + UART_FIFO_WDATA_OFFSET));
}

void sbi_set_pmu()
{
	unsigned long interrupts;

	interrupts = csr_read(CSR_MIDELEG) | (1 << 17);
	csr_write(CSR_MIDELEG, interrupts);

	/* CSR_MCOUNTEREN has already been set in mstatus_init() */
	csr_write(CSR_MCOUNTERWEN, 0xffffffff);
	csr_write(CSR_MHPMEVENT3, 1);
	csr_write(CSR_MHPMEVENT4, 2);
	csr_write(CSR_MHPMEVENT5, 3);
	csr_write(CSR_MHPMEVENT6, 4);
	csr_write(CSR_MHPMEVENT7, 5);
	csr_write(CSR_MHPMEVENT8, 6);
	csr_write(CSR_MHPMEVENT9, 7);
	csr_write(CSR_MHPMEVENT10, 8);
	csr_write(CSR_MHPMEVENT11, 9);
	csr_write(CSR_MHPMEVENT12, 10);
	csr_write(CSR_MHPMEVENT13, 11);
	csr_write(CSR_MHPMEVENT14, 12);
	csr_write(CSR_MHPMEVENT15, 13);
	csr_write(CSR_MHPMEVENT16, 14);
	csr_write(CSR_MHPMEVENT17, 15);
	csr_write(CSR_MHPMEVENT18, 16);
	csr_write(CSR_MHPMEVENT19, 17);
	csr_write(CSR_MHPMEVENT20, 18);
	csr_write(CSR_MHPMEVENT21, 19);
	csr_write(CSR_MHPMEVENT22, 20);
	csr_write(CSR_MHPMEVENT23, 21);
	csr_write(CSR_MHPMEVENT24, 22);
	csr_write(CSR_MHPMEVENT25, 23);
	csr_write(CSR_MHPMEVENT26, 24);
	csr_write(CSR_MHPMEVENT27, 25);
	csr_write(CSR_MHPMEVENT28, 26);
}

void sbi_boot_other_core(int hartid)
{
	csr_write(CSR_MRVBR, FW_TEXT_START);
	csr_write(CSR_MRMR, csr_read(CSR_MRMR) | (1 << hartid));
}

static int c910_vendor_ext_provider(long extid, long funcid,
				    unsigned long *args,
				    unsigned long *out_value,
				    struct sbi_trap_info *out_trap)
{
	switch (extid) {
	case SBI_EXT_VENDOR_C910_BOOT_OTHER_CORE:
		sbi_boot_other_core((int)args[0]);
		break;
	case SBI_EXT_VENDOR_C910_SET_PMU:
		sbi_set_pmu();
		break;
	default:
		sbi_printf("Unsupported private sbi call: %ld\n", extid);
		asm volatile("ebreak");
	}
	return 0;
}

const struct sbi_platform_operations platform_ops = {
	.early_init = c910_early_init,
	.final_init = c910_final_init,

	.console_putc = c910_uart_putc,

	.irqchip_init = c910_irqchip_init,

	.ipi_init  = c910_ipi_init,
	.ipi_send  = clint_ipi_send,
	.ipi_clear = clint_ipi_clear,

	.timer_init	   = c910_timer_init,
	.timer_value	   = c910_timer_value,
	.timer_event_start = clint_timer_event_start,

	.system_shutdown = c910_system_shutdown,

	.vendor_ext_provider = c910_vendor_ext_provider,
};

const struct sbi_platform platform = { .opensbi_version = OPENSBI_VERSION,
				       .platform_version =
					       SBI_PLATFORM_VERSION(0x0, 0x01),
				       .name		= "T-HEAD Xuantie c910",
				       .features	= SBI_THEAD_FEATURES,
				       .hart_count	= C910_HART_COUNT,
				       .hart_stack_size = C910_HART_STACK_SIZE,
				       .disabled_hart_mask = 0,
				       .platform_ops_addr =
					       (unsigned long)&platform_ops };
