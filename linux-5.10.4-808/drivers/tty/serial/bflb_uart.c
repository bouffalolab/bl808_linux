// SPDX-License-Identifier: GPL-2.0+
/*
 * bflb_uart.c -- BFLB UART driver
 *
 * Based on mcf.c -- Freescale ColdFire UART driver
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include "bflb_uart.h"

#define DRV_NAME "bflb_uart"

#define BFLB_UART_MAJOR 204
#define BFLB_UART_MINOR 186

#define BFLB_UART_SIZE 1024
#define CONFIG_SERIAL_BFLB_UART_CONSOLE

#define BFLB_UART_BAUD 2000000
#define BFLB_UART_RX_FIFO_TH 7

/*
 * Local per-uart structure.
 */
struct bflb_uart {
	struct uart_port port;
	struct clk *clk; /* uart clock */
	unsigned int sigs; /* Local copy of line sigs */
	unsigned long imr; /* Local IMR mirror */
	bool tx_stopped; /* tx state = stop */
	struct dentry *debugfs_root;
	struct {
		u32 rx_fifo_underflow_count;
		u32 rx_fifo_overflow_count;
		u32 rx_fifo_ready_count;
		u32 rx_rto_count;
		u32 tx_fifo_ready_count;
		u32 tx_end_count;
	} debug_stats;
};

static inline struct bflb_uart *to_bflb_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct bflb_uart, port);
}

static inline u32 bflb_uart_readl(struct uart_port *port, u32 reg)
{
	return __raw_readl(port->membase + reg);
}

static inline void bflb_uart_writel(struct uart_port *port, u32 reg, u32 value)
{
	__raw_writel(value, port->membase + reg);
}

static inline u8 bflb_uart_read_char(struct uart_port *port)
{
	return __raw_readb(port->membase + UART_FIFO_RDATA_OFFSET);
}

static inline void bflb_uart_write_char(struct uart_port *port, u8 value)
{
	__raw_writeb(value, port->membase + UART_FIFO_WDATA_OFFSET);
}

static unsigned int bflb_uart_tx_empty(struct uart_port *port)
{
	return (readl(port->membase + UART_FIFO_CONFIG_1_OFFSET) &
		UART_TX_FIFO_CNT_MSK) ?
			     TIOCSER_TEMT :
			     0;
}

static unsigned int bflb_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void bflb_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
}

static void bflb_uart_start_tx(struct uart_port *port)
{
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned long tmp;

	tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
	tmp |= UART_CR_UTX_EN_MSK;
	writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);

	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp &= UART_CR_UTX_END_MASK_UMSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);

	tmp = readl(port->membase + UART_FIFO_CONFIG_1_OFFSET);
	tmp &= UART_TX_FIFO_TH_UMSK;
	tmp |= 15 << UART_TX_FIFO_TH_POS;
	writel(tmp, port->membase + UART_FIFO_CONFIG_1_OFFSET);

	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp &= UART_CR_UTX_FIFO_MASK_UMSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);
}

static void bflb_uart_stop_tx(struct uart_port *port)
{
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned long tmp;

	tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
	//	tmp &= UART_CR_UTX_EN_UMSK;
	writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);

	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp |= UART_CR_UTX_END_MASK_MSK | UART_CR_UTX_FIFO_MASK_MSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);
}

static void bflb_uart_stop_rx(struct uart_port *port)
{
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned long tmp;

	tmp = readl(port->membase + UART_URX_CONFIG_OFFSET);
	tmp &= UART_CR_URX_EN_UMSK;
	writel(tmp, port->membase + UART_URX_CONFIG_OFFSET);

	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp |= UART_CR_URX_FIFO_MASK_MSK | UART_CR_URX_RTO_MASK_MSK |
	       UART_CR_URX_FER_MASK_MSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);
}

static void bflb_uart_break_ctl(struct uart_port *port, int break_state)
{
}

static void bflb_uart_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  struct ktermios *old)
{
	/* Just copy the old termios settings back */
	if (old)
		tty_termios_copy_hw(termios, old);
}

static void bflb_uart_rx_chars(struct bflb_uart *pp)
{
	struct uart_port *port = &pp->port;
	unsigned char ch, flag;
	unsigned long status;

	while ((status = readl(port->membase + UART_FIFO_CONFIG_1_OFFSET)) &
	       UART_RX_FIFO_CNT_MSK) {
		//ch = status & BFLB_UART_DATA_DATA_MSK;
		ch = readl(port->membase + UART_FIFO_RDATA_OFFSET) &
		     UART_FIFO_RDATA_MSK;
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, 0, 0, ch, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
	spin_lock(&port->lock);
}

static void bflb_uart_tx_chars(struct bflb_uart *pp)
{
	struct uart_port *port = &pp->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int pending, count;

	if (port->x_char) {
		/* Send special char - probably flow control */
		writel(port->x_char, port->membase + UART_FIFO_WDATA_OFFSET);
		port->x_char = 0;
		port->icount.tx++;
		return;
	}

	pending = uart_circ_chars_pending(xmit);
	if (pending > 0) {
		count = (readl(port->membase + UART_FIFO_CONFIG_1_OFFSET) &
			 UART_TX_FIFO_CNT_MSK) >>
			UART_TX_FIFO_CNT_POS;
		if (count > pending)
			count = pending;
		if (count > 0) {
			pending -= count;
			while (count--) {
				writel(xmit->buf[xmit->tail],
				       port->membase + UART_FIFO_WDATA_OFFSET);
				xmit->tail =
					(xmit->tail + 1) & (UART_XMIT_SIZE - 1);
				port->icount.tx++;
			}
			if (pending < WAKEUP_CHARS)
				uart_write_wakeup(port);
		}
	}

	if (pending == 0) {
		/* stop tx */
		//unsigned long tmp;
		//tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
		//tmp &= UART_CR_UTX_EN_UMSK;
		//writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);
		bflb_uart_stop_tx(port);
	}
}

static irqreturn_t bflb_uart_interrupt(int irq, void *data)
{
	struct uart_port *port = data;
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned int isr;
	unsigned int tmp;

	isr = (readl(port->membase + UART_INT_STS_OFFSET));
	writel(isr, port->membase + UART_INT_CLEAR_OFFSET);

	isr &= ~(readl(port->membase + UART_INT_MASK_OFFSET));

	spin_lock(&port->lock);

	if (isr & UART_URX_FER_INT_MSK) {
		//RX FIFO error interrupt
		tmp = readl(port->membase + UART_FIFO_CONFIG_0_OFFSET);
		if (tmp & UART_RX_FIFO_OVERFLOW_MSK)
			pp->debug_stats.rx_fifo_overflow_count++;
		if (tmp & UART_RX_FIFO_UNDERFLOW_MSK)
			pp->debug_stats.rx_fifo_underflow_count++;

		tmp |= UART_RX_FIFO_CLR_MSK;
		writel(tmp, port->membase + UART_FIFO_CONFIG_0_OFFSET);
	}

	if (isr & UART_CR_URX_FIFO_MASK_MSK)
		pp->debug_stats.rx_fifo_ready_count++;
	if (isr & UART_CR_URX_RTO_MASK_MSK)
		pp->debug_stats.rx_rto_count++;
	if (isr & UART_CR_UTX_FIFO_MASK_MSK)
		pp->debug_stats.tx_fifo_ready_count++;
	if (isr & UART_UTX_END_INT_MSK)
		pp->debug_stats.tx_end_count++;

	if (isr & (UART_CR_URX_FIFO_MASK_MSK | UART_CR_URX_RTO_MASK_MSK)) {
		bflb_uart_rx_chars(pp);
	}
	if (isr & (UART_CR_UTX_FIFO_MASK_MSK | UART_UTX_END_INT_MSK)) {
		bflb_uart_tx_chars(pp);
	}

	spin_unlock(&port->lock);

	return IRQ_RETVAL(isr);
}

static void bflb_uart_config_port(struct uart_port *port, int flags)
{
	unsigned int tmp;

	port->type = PORT_BFLB_UART;

	/* Clear mask, so no surprise interrupts. */
	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp |= UART_CR_UTX_END_MASK_MSK;
	tmp |= UART_CR_UTX_FIFO_MASK_MSK;
	tmp |= UART_CR_URX_FIFO_MASK_MSK;
	tmp |= UART_CR_URX_RTO_MASK_MSK;
	tmp |= UART_CR_URX_FER_MASK_MSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);
}

static int bflb_uart_startup(struct uart_port *port)
{
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned long flags;
	int ret;
	unsigned int tmp;

	ret = request_irq(port->irq, bflb_uart_interrupt, 0, DRV_NAME, port);
	if (ret) {
		pr_err(DRV_NAME ": unable to attach BFLB UART %d "
				"interrupt vector=%d\n",
		       port->line, port->irq);
		return ret;
	}

	spin_lock_irqsave(&port->lock, flags);

	/* Enable RX interrupts now */
	tmp = readl(port->membase + UART_FIFO_CONFIG_1_OFFSET);
	tmp &= UART_RX_FIFO_TH_UMSK;
	tmp |= BFLB_UART_RX_FIFO_TH << UART_RX_FIFO_TH_POS;
	writel(tmp, port->membase + UART_FIFO_CONFIG_1_OFFSET);
	tmp = readl(port->membase + UART_INT_MASK_OFFSET);
	tmp &= UART_CR_URX_FIFO_MASK_UMSK;
	tmp &= UART_CR_URX_RTO_MASK_UMSK;
	tmp &= UART_CR_URX_FER_MASK_UMSK;
	writel(tmp, port->membase + UART_INT_MASK_OFFSET);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static void bflb_uart_shutdown(struct uart_port *port)
{
	struct bflb_uart *pp = container_of(port, struct bflb_uart, port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Disable all interrupts now */
	writel(UART_CR_UTX_END_MASK_MSK | UART_CR_URX_END_MASK_MSK,
	       port->membase + UART_INT_MASK_OFFSET);

	spin_unlock_irqrestore(&port->lock, flags);

	free_irq(port->irq, port);
}

static const char *bflb_uart_type(struct uart_port *port)
{
	return (port->type == PORT_BFLB_UART) ? "BFLB UART" : NULL;
}

static int bflb_uart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void bflb_uart_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int bflb_uart_verify_port(struct uart_port *port,
				 struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_BFLB_UART)
		return -EINVAL;
	return 0;
}

/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops bflb_uart_ops = {
	.tx_empty = bflb_uart_tx_empty,
	.get_mctrl = bflb_uart_get_mctrl,
	.set_mctrl = bflb_uart_set_mctrl,
	.start_tx = bflb_uart_start_tx,
	.stop_tx = bflb_uart_stop_tx,
	.stop_rx = bflb_uart_stop_rx,
	.break_ctl = bflb_uart_break_ctl,
	.startup = bflb_uart_startup,
	.shutdown = bflb_uart_shutdown,
	.set_termios = bflb_uart_set_termios,
	.type = bflb_uart_type,
	.request_port = bflb_uart_request_port,
	.release_port = bflb_uart_release_port,
	.config_port = bflb_uart_config_port,
	.verify_port = bflb_uart_verify_port,
};

#define BFLB_UART_MAXPORTS 1
static struct bflb_uart bflb_uart_ports[BFLB_UART_MAXPORTS];

#if defined(CONFIG_SERIAL_BFLB_UART_CONSOLE)

static void bflb_console_putchar(struct uart_port *port, int ch)
{
	while (!(bflb_uart_readl(port, UART_FIFO_CONFIG_1_OFFSET) &
		 UART_TX_FIFO_CNT_MSK))
		cpu_relax();
	bflb_uart_write_char(port, ch);
}

/*
 * Interrupts are disabled on entering
 */
static void bflb_uart_console_write(struct console *co, const char *s,
				    u_int count)
{
	struct uart_port *port = &bflb_uart_ports[co->index].port;
	struct bflb_uart *bflb_port = to_bflb_uart_port(port);
	unsigned int status, reg, mask;

	/* save then disable interrupts */
	mask = bflb_uart_readl(port, UART_INT_MASK_OFFSET);
	reg = -1;
	bflb_uart_writel(port, UART_INT_MASK_OFFSET, reg);

	/* Make sure that tx is enabled */
	reg = bflb_uart_readl(port, UART_UTX_CONFIG_OFFSET);
	reg |= UART_CR_UTX_EN_MSK;
	bflb_uart_writel(port, UART_UTX_CONFIG_OFFSET, reg);
	bflb_port->tx_stopped = false;

	uart_console_write(port, s, count, bflb_console_putchar);

	/* wait for TX done */
	do {
		status = bflb_uart_readl(port, UART_STATUS_OFFSET);
	} while ((status & UART_STS_UTX_BUS_BUSY_MSK));

	/* restore IRQ mask */
	bflb_uart_writel(port, UART_INT_MASK_OFFSET, mask);
}

/*
 * If the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init bflb_console_get_options(struct uart_port *port, int *baud,
					    int *parity, int *bits)
{
	unsigned int div;

	div = bflb_uart_readl(port, UART_BIT_PRD_OFFSET) &
	      UART_CR_UTX_BIT_PRD_MSK;
	if (!div)
		return;

	*bits = 8;

	/*
	 * The serial core only rounds down when matching this to a
	 * supported baud rate. Make sure we don't end up slightly
	 * lower than one of those, as it would make us fall through
	 * to a much lower baud rate than we really want.
	 */
	*baud = port->uartclk / (div + 1);
}

static int __init bflb_uart_console_setup(struct console *co, char *options)
{
	struct uart_port *port = &bflb_uart_ports[co->index].port;
	struct bflb_uart *bflb_uart = to_bflb_uart_port(port);
	int baud = BFLB_UART_BAUD;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	unsigned int reg;

	if (port->membase == NULL) {
		/* Port not initialized yet - delay setup */
		return -ENODEV;
	}

	/* TODO: clk */

	reg = bflb_uart_readl(port, UART_UTX_CONFIG_OFFSET);
	reg |= UART_CR_UTX_EN_MSK;
	bflb_uart_writel(port, UART_UTX_CONFIG_OFFSET, reg);
	reg = bflb_uart_readl(port, UART_URX_CONFIG_OFFSET);
	reg |= UART_CR_URX_EN_MSK;
	bflb_uart_writel(port, UART_URX_CONFIG_OFFSET, reg);
	bflb_uart->tx_stopped = false;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		bflb_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, co, baud, parity, bits, flow);
}
static struct uart_driver bflb_uart_driver;

static struct console bflb_uart_console = {
	.name = "ttyS",
	.write = bflb_uart_console_write,
	.device = uart_console_device,
	.setup = bflb_uart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &bflb_uart_driver,
};

static int __init bflb_uart_console_init(void)
{
	register_console(&bflb_uart_console);
	return 0;
}

console_initcall(bflb_uart_console_init);

#define BFLB_UART_CONSOLE (&bflb_uart_console)

static void bflb_uart_earlycon_write(struct console *co, const char *s,
				     unsigned int count)
{
	struct earlycon_device *dev = co->data;

	uart_console_write(&dev->port, s, count, bflb_console_putchar);
}

static int __init bflb_uart_earlycon_setup(struct earlycon_device *dev,
					   const char *options)
{
	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = bflb_uart_earlycon_write;

	return 0;
}

OF_EARLYCON_DECLARE(juart, "bflb-uart,uart0", bflb_uart_earlycon_setup);

#else

#define BFLB_UART_CONSOLE NULL

#endif /* CONFIG_SERIAL_BFLB_UART_CONSOLE */

static struct uart_driver bflb_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "bflb_uart",
	.dev_name = "ttyS",
	.major = BFLB_UART_MAJOR,
	.minor = BFLB_UART_MINOR,
	.nr = BFLB_UART_MAXPORTS,
	.cons = BFLB_UART_CONSOLE,
};

static int bflb_uart_probe(struct platform_device *pdev)
{
	struct bflb_uart_platform_uart *platp = dev_get_platdata(&pdev->dev);
	struct uart_port *port;
	struct bflb_uart *bflb_uart;
	struct resource *res_irq, *res_mem;
	int i = pdev->id;
	int clk_rate;
	int ret = 0;
	unsigned long tmp;

	/* -1 emphasizes that the platform must have one port, no .N suffix */
	if (i == -1)
		i = 0;

	if (i >= BFLB_UART_MAXPORTS)
		return -EINVAL;

	port = &bflb_uart_ports[i].port;
	bflb_uart = to_bflb_uart_port(port);

	port->mapbase = pdev->resource[0].start;
	port->irq = pdev->resource[1].start;

	port->membase = ioremap(port->mapbase, BFLB_UART_SIZE);
	if (!port->membase)
		return -ENOMEM;

	port->line = i;
	port->type = PORT_BFLB_UART;
	port->iotype = SERIAL_IO_MEM;
	port->ops = &bflb_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = &pdev->dev;

	bflb_uart->clk = clk_get(port->dev, "baudclk");
	if (IS_ERR(bflb_uart->clk)) {
		dev_err(&pdev->dev,
			"Unable to get baud rate generator clock\n");
		ret = PTR_ERR(bflb_uart->clk);
		return ret;
	}
	clk_prepare_enable(bflb_uart->clk);
	clk_rate = clk_get_rate(bflb_uart->clk);

	// debugfs
	bflb_uart->debugfs_root = debugfs_create_dir("bluart", NULL);
	bflb_uart->debug_stats.rx_fifo_overflow_count = 0;
	bflb_uart->debug_stats.rx_fifo_underflow_count = 0;
	bflb_uart->debug_stats.rx_fifo_ready_count = 0;
	bflb_uart->debug_stats.rx_rto_count = 0;
	bflb_uart->debug_stats.tx_fifo_ready_count = 0;
	bflb_uart->debug_stats.tx_end_count = 0;
	if (bflb_uart->debugfs_root) {
		debugfs_create_u32(
			"rx_fifo_overflow_count", 0644, bflb_uart->debugfs_root,
			&bflb_uart->debug_stats.rx_fifo_overflow_count);
		debugfs_create_u32(
			"rx_fifo_underflow_count", 0644,
			bflb_uart->debugfs_root,
			&bflb_uart->debug_stats.rx_fifo_underflow_count);
		debugfs_create_u32("rx_fifo_ready_count", 0644,
				   bflb_uart->debugfs_root,
				   &bflb_uart->debug_stats.rx_fifo_ready_count);
		debugfs_create_u32("rx_rto_count", 0644,
				   bflb_uart->debugfs_root,
				   &bflb_uart->debug_stats.rx_rto_count);

		debugfs_create_u32("tx_fifo_ready_count", 0644,
				   bflb_uart->debugfs_root,
				   &bflb_uart->debug_stats.tx_fifo_ready_count);
		debugfs_create_u32("tx_end_count", 0644,
				   bflb_uart->debugfs_root,
				   &bflb_uart->debug_stats.tx_end_count);
	}

	{
		/* TODO: move to setup */

		//UART_IntMask(UART_DBG_ID, UART_INT_ALL, MASK);
		tmp = readl(port->membase + UART_INT_MASK_OFFSET);
		tmp |= 0xfff;
		writel(tmp, port->membase + UART_INT_MASK_OFFSET);

		//UART_Disable(UART_DBG_ID,UART_TXRX);
		tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
		tmp &= UART_CR_UTX_EN_UMSK;
		writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);
		tmp = readl(port->membase + UART_URX_CONFIG_OFFSET);
		tmp &= UART_CR_URX_EN_UMSK;
		writel(tmp, port->membase + UART_URX_CONFIG_OFFSET);

		//UART_Init(UART_DBG_ID, &uart_dbg_cfg);
		tmp = clk_rate / BFLB_UART_BAUD - 1; /* 2000000bps */
		tmp = (tmp << 16) | tmp;
		writel(tmp, port->membase + UART_BIT_PRD_OFFSET);
		writel(0xf04, port->membase + UART_UTX_CONFIG_OFFSET);
		writel(0x700, port->membase + UART_URX_CONFIG_OFFSET);
		writel(0, port->membase + UART_DATA_CONFIG_OFFSET);
		writel(0, port->membase + UART_SW_MODE_OFFSET);
		//UART_FifoConfig(UART_DBG_ID,&fifoCfg);
		//	writel(0xf0f0080, port->membase + UART_FIFO_CONFIG_1_OFFSET);
		writel(0x00000080, port->membase + UART_FIFO_CONFIG_1_OFFSET);
		writel(0x80, port->membase + UART_FIFO_CONFIG_0_OFFSET);
		//UART_TxFreeRun(UART_DBG_ID,ENABLE);
		writel(0xf04, port->membase + UART_UTX_CONFIG_OFFSET);
		//UART_SetRxTimeoutValue(UART_DBG_ID,80);
		writel(0x4f, port->membase + UART_URX_RTO_TIMER_OFFSET);
		//UART_IntMask(UART_DBG_ID,UART_INT_RX_FIFO_REQ,UNMASK);
		//tmp = readl(port->membase + UART_INT_MASK_OFFSET);
		//tmp &= UART_CR_URX_FIFO_MASK_UMSK;
		//writel(tmp, port->membase + UART_INT_MASK_OFFSET);
		//UART_Enable(UART_DBG_ID,UART_TXRX);
		tmp = readl(port->membase + UART_UTX_CONFIG_OFFSET);
		tmp |= UART_CR_UTX_EN_MSK;
		writel(tmp, port->membase + UART_UTX_CONFIG_OFFSET);
		tmp = readl(port->membase + UART_URX_CONFIG_OFFSET);
		tmp |= UART_CR_URX_EN_MSK;
		writel(tmp, port->membase + UART_URX_CONFIG_OFFSET);
	}

	/* bflb uart does not need to divide the clock by 16 */
	tmp = bflb_uart_readl(port, UART_BIT_PRD_OFFSET) &
	      UART_CR_UTX_BIT_PRD_MSK;
	port->uartclk = clk_rate * 16 / (tmp + 1);

	uart_add_one_port(&bflb_uart_driver, port);

	return 0;
}

static int bflb_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port;
	int i = pdev->id;

	if (i == -1)
		i = 0;

	port = &bflb_uart_ports[i].port;
	uart_remove_one_port(&bflb_uart_driver, port);
	iounmap(port->membase);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bflb_uart_match[] = {
	{
		.compatible = "bflb-uart,uart0",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bflb_uart_match);
#endif /* CONFIG_OF */

static struct platform_driver bflb_uart_platform_driver = {
	.probe	= bflb_uart_probe,
	.remove	= bflb_uart_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(bflb_uart_match),
	},
};

static int __init bflb_uart_init(void)
{
	int rc;

	rc = uart_register_driver(&bflb_uart_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&bflb_uart_platform_driver);
	if (rc)
		uart_unregister_driver(&bflb_uart_driver);
	return rc;
}

static void __exit bflb_uart_exit(void)
{
	platform_driver_unregister(&bflb_uart_platform_driver);
	uart_unregister_driver(&bflb_uart_driver);
}

module_init(bflb_uart_init);
module_exit(bflb_uart_exit);

MODULE_DESCRIPTION("BFLB UART driver");
MODULE_AUTHOR("abc <123@abc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
