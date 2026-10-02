// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 FAP firmware: log, local memory tests, access timings,
 * mailbox ping/pong and CP0 timer on interrupts
 */

#include "fap_shared.h"
#include "fap4ke.h"

/* CP0 Count ticks the 4KE waits for the host to take a message */
#define MBOX_ACK_TIMEOUT	(10 * FAP_TIMER_PERIOD)

/*
 * CP0 Count ticks the 4KE waits for the iuDMA to take a frame. With the
 * mailbox wait it stays well within the host ping timeout, so the host
 * never frees a frame the engine may still read.
 */
#define TX_DONE_TIMEOUT		(10 * FAP_TIMER_PERIOD)

#define __stringify_1(x)	#x
#define __stringify(x)		__stringify_1(x)

#define read_c0(reg, sel) ({						\
	unsigned int __v;						\
	__asm__ volatile("mfc0 %0, " __stringify(reg) ", " #sel		\
			 : "=r" (__v));					\
	__v;								\
})

#define write_c0(reg, sel, val) ({					\
	__asm__ volatile("mtc0 %z0, " __stringify(reg) ", " #sel	\
			 "\n\tehb" : : "Jr" (val));			\
})

#define cache_op(op, addr) ({						\
	__asm__ volatile("cache %0, 0(%1)\n\tehb"			\
			 : : "i" (op), "r" (addr) : "memory");		\
})

void fap_main(void);
void fap_irq(unsigned int cause);
void fap_exception(unsigned int cause, unsigned int epc,
		   unsigned int badvaddr);

unsigned int bench_alu(volatile unsigned int *p, unsigned int iters);
unsigned int bench_load(volatile unsigned int *p, unsigned int iters);
unsigned int bench_store(volatile unsigned int *p, unsigned int iters);
unsigned int bench_load_pair(volatile unsigned int *p,
			     volatile unsigned int *q, unsigned int iters);

extern char _dspram_end[], _stack_bottom[];

static volatile struct fap_shared *const shared =
	(volatile struct fap_shared *)KSEG1(FAP_4KE_BOOT_BASE + FAP_SHARED_OFFSET);

/* a DDR word for the access timings, the image block is in DDR */
static volatile unsigned int ddr_word;

/* main loop iterations, the timer turns them into the heartbeat */
static volatile unsigned int main_loops __attribute__((section(".dspram")));
static unsigned int main_loops_seen __attribute__((section(".dspram")));

/* in PSM, the single descriptor of the transmit test */
struct iudma_desc {
	unsigned int len_status;
	unsigned int addr;
};

/* host command left by the interrupt handler for the main loop */
static volatile unsigned int command __attribute__((section(".dspram")));

static volatile struct fap_rx_qsm *const rxq =
	(volatile struct fap_rx_qsm *)(FAP_4KE_QSM + FAP_RX_QSM_OFFSET);

/* receive test state */
static struct {
	unsigned int active;
	unsigned int window;		/* kseg1 address of host address 0 */
	volatile unsigned int *comp;
	unsigned int len;
	unsigned int period;
	unsigned int next;		/* CP0 Count of the next frame */
	unsigned int free_cons;
	unsigned int comp_prod;
	unsigned int idle_free;
	unsigned int idle_ring;
	unsigned int busy;
} rx __attribute__((section(".dspram")));

static unsigned int reg_rd(unsigned int off)
{
	return *(volatile unsigned int *)(FAP_4KE_REGS + off);
}

static void reg_wr(unsigned int off, unsigned int val)
{
	*(volatile unsigned int *)(FAP_4KE_REGS + off) = val;
}

static unsigned int count_res(void)
{
	unsigned int res;

	__asm__ volatile("rdhwr %0, $3" : "=r" (res));

	return res;
}

static void log_putc(char c)
{
	unsigned int head = shared->log_head;

	shared->log[head % FAP_LOG_SIZE] = c;
	__asm__ volatile("sync" ::: "memory");
	shared->log_head = head + 1;
}

static void log_puts(const char *s)
{
	while (*s)
		log_putc(*s++);
}

static void log_reg(const char *name, unsigned int val)
{
	int shift;

	log_puts(name);
	log_puts(" 0x");
	for (shift = 28; shift >= 0; shift -= 4)
		log_putc("0123456789abcdef"[(val >> shift) & 0xf]);
	log_putc('\n');
}

static unsigned int irq_save(void)
{
	unsigned int status;

	__asm__ volatile("di %0\n\tehb" : "=r" (status) : : "memory");

	return status;
}

static void irq_restore(unsigned int status)
{
	if (status & ST0_IE)
		__asm__ volatile("ei\n\tehb" : : : "memory");
}

/* the host clears FAP_GP_MBOX_IN once it has read the previous message */
/* also used by the main loop, so interrupts are off while it runs */
static void mbox_send(unsigned int type, unsigned int val)
{
	unsigned int start = read_c0(CP0_COUNT, 0);
	unsigned int status = irq_save();

	while (reg_rd(FAP_REG_GP_STATUS) & FAP_GP_MBOX_IN) {
		if (read_c0(CP0_COUNT, 0) - start > MBOX_ACK_TIMEOUT) {
			shared->mbox_lost++;
			break;
		}
	}

	/* shared memory updates first, the host reads them on the message */
	__asm__ volatile("sync" ::: "memory");
	reg_wr(FAP_REG_MBOX_IN, FAP_MSG(type, val));

	irq_restore(status);
}

static void mbox_pong(unsigned int cmd)
{
	shared->pong_count = read_c0(CP0_COUNT, 0);
	mbox_send(FAP_MSG_PONG, cmd);
}

static void irq_timer(void)
{
	unsigned int compare = read_c0(CP0_COMPARE, 0);
	unsigned int lat = read_c0(CP0_COUNT, 0) - compare;
	unsigned int next = compare + FAP_TIMER_PERIOD;

	if ((int)(read_c0(CP0_COUNT, 0) - next) >= 0)
		next = read_c0(CP0_COUNT, 0) + FAP_TIMER_PERIOD;
	write_c0(CP0_COMPARE, 0, next);

	if (lat < shared->timer_lat_min)
		shared->timer_lat_min = lat;
	if (lat > shared->timer_lat_max)
		shared->timer_lat_max = lat;

	/* a live core is not enough, the main loop has to be moving */
	if (main_loops != main_loops_seen) {
		main_loops_seen = main_loops;
		shared->heartbeat++;
	}
}

static void irq_fap(void)
{
	unsigned int cmd;

	if (!(reg_rd(FAP_REG_IRQ_4KE_STATUS) & FAP_IRQ_4KE_MBOX_OUT))
		return;

	reg_wr(FAP_REG_GP_STATUS, FAP_GP_MBOX_OUT);
	cmd = reg_rd(FAP_REG_MBOX_OUT);

	if (FAP_MSG_TYPE(cmd) == FAP_CMD_PING) {
		shared->pings++;
		mbox_pong(cmd);
	} else {
		command = cmd;
	}
}

void fap_irq(unsigned int cause)
{
	if (cause & CAUSE_TI)
		irq_timer();
	if (cause & read_c0(CP0_STATUS, 0) & ST0_IM_FAP)
		irq_fap();
}

static unsigned int dspram_load_tag(unsigned int offset)
{
	unsigned int errctl = read_c0(CP0_ERRCTL, 0);
	unsigned int tag;

	write_c0(CP0_ERRCTL, 0, errctl | ERRCTL_SPR);
	cache_op(INDEX_LOAD_TAG_D, KSEG0(offset));
	tag = read_c0(CP0_TAGLO, 0);
	write_c0(CP0_ERRCTL, 0, errctl);

	return tag;
}

static unsigned int mem_test(volatile unsigned int *mem, unsigned int words)
{
	unsigned int errors = 0;
	unsigned int i;

	for (i = 0; i < words; i++)
		mem[i] = FAP_MEM_PATTERN(i);

	for (i = 0; i < words; i++)
		if (mem[i] != FAP_MEM_PATTERN(i))
			errors++;

	return errors;
}

static unsigned int mem_pattern_count(volatile unsigned int *mem,
				      unsigned int words)
{
	unsigned int count = 0;
	unsigned int i;

	for (i = 0; i < words; i++)
		if (mem[i] == FAP_MEM_PATTERN(i))
			count++;

	return count;
}

/*
 * start.S mapped the DSPRAM; the variables at its bottom and the stack at
 * its top are left alone.
 */
static int dspram_test(void)
{
	unsigned int start = (unsigned int)_dspram_end - KSEG0(0);
	unsigned int words = ((unsigned int)_stack_bottom -
			      (unsigned int)_dspram_end) / 4;
	unsigned int errors, leaked;

	log_reg("dspram tag0", dspram_load_tag(0));
	log_reg("dspram tag1", dspram_load_tag(SPRAM_TAG_STRIDE));

	errors = mem_test((volatile unsigned int *)KSEG0(start), words);

	/*
	 * Without the scratchpad the same physical range goes through the
	 * low translator to the memory block, which the boot translator
	 * shows at FAP_4KE_BOOT_BASE.
	 */
	leaked = mem_pattern_count((volatile unsigned int *)
				   KSEG1(FAP_4KE_BOOT_BASE + start), words);

	log_reg("dspram errors", errors);
	log_reg("dspram words in ddr", leaked);

	return !errors && !leaked;
}

static int local_mem_test(const char *name, unsigned int addr,
			  unsigned int size)
{
	unsigned int errors;

	errors = mem_test((volatile unsigned int *)addr, size / 4);
	log_puts(name);
	log_reg(" errors", errors);

	return !errors;
}

static void bench(void)
{
	static volatile unsigned int *const target[FAP_BENCH_NR] = {
		[FAP_BENCH_DSPRAM] = &main_loops,
		[FAP_BENCH_PSM] = (volatile unsigned int *)FAP_4KE_PSM,
		[FAP_BENCH_QSM] = (volatile unsigned int *)FAP_4KE_QSM,
		[FAP_BENCH_REGS] = (volatile unsigned int *)
				   (FAP_4KE_REGS + FAP_REG_SCRATCH),
		[FAP_BENCH_DDR] = &ddr_word,
	};
	int i;

	shared->bench_alu = bench_alu(target[0], FAP_BENCH_ITERS);

	for (i = 0; i < FAP_BENCH_NR; i++) {
		shared->bench[i].load = bench_load(target[i], FAP_BENCH_ITERS);
		shared->bench[i].store = bench_store(target[i], FAP_BENCH_ITERS);
	}

	/* words far apart in the image block, to leave the open DDR row */
	for (i = 0; i < 2; i++) {
		volatile unsigned int *p = (volatile unsigned int *)
					   KSEG1(FAP_4KE_BOOT_BASE);

		shared->bench_ddr_far[i] =
			bench_load_pair(p, p + FAP_BENCH_DDR_FAR(i) / 4,
					FAP_BENCH_ITERS);
	}
}

/*
 * The 4KE reads the iuDMA at the address the host uses. The host driver
 * owns it, so nothing is written.
 */
static void iudma_probe(void)
{
	volatile unsigned int *dma = (volatile unsigned int *)KSEG1(ENETSW_DMA);
	int ch;

	shared->iudma.controller_cfg = dma[0];
	for (ch = 0; ch < FAP_IUDMA_PROBE_CHANNELS; ch++) {
		shared->iudma.ch[ch].cfg =
			dma[(IUDMA_CHANNELS + ch * IUDMA_CH_STRIDE) / 4];
		shared->iudma.ch[ch].ring =
			dma[(IUDMA_SRAM + ch * IUDMA_CH_STRIDE) / 4];
	}

	shared->iudma.load_regs = bench_load(dma, FAP_BENCH_ITERS);
	shared->iudma.load_sram = bench_load(&dma[IUDMA_SRAM / 4],
					     FAP_BENCH_ITERS);
}

/*
 * Reads the shared area at its host physical address, outside both
 * translator windows: does the 4KE reach any DDR address directly?
 */
static void ddr_probe(unsigned int block)
{
	volatile unsigned int *magic = (volatile unsigned int *)
		KSEG1((block << 16) + FAP_SHARED_OFFSET);

	shared->ddr_direct_magic = *magic;
	shared->ddr_direct_load = bench_load(magic, FAP_BENCH_ITERS);
}

/* one load only: the address may be a register with read side effects */
static void peek(void)
{
	volatile unsigned int *p = (volatile unsigned int *)shared->peek_addr;
	unsigned int start, val;

	start = read_c0(CP0_COUNT, 0);
	val = *p;
	shared->peek_ticks = read_c0(CP0_COUNT, 0) - start;
	shared->peek_val = val;
}

static void rx_publish(void)
{
	shared->rx.frames = rx.comp_prod;
	shared->rx.idle_free = rx.idle_free;
	shared->rx.idle_ring = rx.idle_ring;
	shared->rx.busy_ticks = rx.busy;
}

static void rx_start(void)
{
	rx.window = KSEG1(shared->rx.ram_window);
	rx.comp = (volatile unsigned int *)(rx.window + shared->rx.comp_ring);
	rx.len = shared->rx.frame_len;
	rx.period = shared->rx.period;
	rx.next = read_c0(CP0_COUNT, 0);
	rx.busy = 0;
	rx.free_cons = 0;
	rx.comp_prod = 0;
	rx.idle_free = 0;
	rx.idle_ring = 0;
	shared->rx.notifies = 0;
	rx_publish();
	rx.active = 1;
}

static void rx_stop(void)
{
	rx.active = 0;
	rx_publish();
}

#define RX_BATCH	16

/* fills free buffers, as the iuDMA will, and hands them to the host */
static void rx_fill(void)
{
	unsigned int n, i, t0;

	t0 = read_c0(CP0_COUNT, 0);
	for (n = 0; n < RX_BATCH; n++) {
		volatile unsigned int *buf;
		unsigned int addr;

		if (rx.period) {
			if ((int)(read_c0(CP0_COUNT, 0) - rx.next) < 0)
				break;
			rx.next += rx.period;
		}
		if (rx.free_cons == rxq->free_prod) {
			rx.idle_free++;
			break;
		}
		if (rx.comp_prod - rxq->comp_cons >= FAP_RX_COMP_ENTRIES) {
			rx.idle_ring++;
			break;
		}

		addr = rxq->free_ring[rx.free_cons % FAP_RX_FREE_ENTRIES];
		rx.free_cons++;

		buf = (volatile unsigned int *)(rx.window + addr);
		for (i = 0; i < FAP_RX_PATTERN_WORDS; i++)
			buf[i] = FAP_RX_PATTERN(rx.comp_prod, i);

		/* the frame is in memory before its entry */
		__asm__ volatile("sync" ::: "memory");
		rx.comp[rx.comp_prod % FAP_RX_COMP_ENTRIES] =
			FAP_RX_PHASE(rx.comp_prod, FAP_RX_COMP_ENTRIES) |
			addr | rx.len;
		rx.comp_prod++;
	}

	if (n) {
		rx.busy += read_c0(CP0_COUNT, 0) - t0;
		rx_publish();
	}
}

/*
 * The receive doorbell carries no value, the host reads the ring. It goes
 * out only while the mailbox is free, so that it never overwrites a message
 * the host has not taken yet, such as a pong, and rx_step() tries again on
 * its next pass. Interrupts are off so that a pong cannot slip in between
 * the check and the write; irq_armed is cleared before the message, since
 * the host arms it again once it has drained the ring.
 */
static void rx_notify(void)
{
	unsigned int status = irq_save();

	if (!(reg_rd(FAP_REG_GP_STATUS) & FAP_GP_MBOX_IN)) {
		rxq->irq_armed = 0;
		shared->rx.notifies++;
		__asm__ volatile("sync" ::: "memory");
		reg_wr(FAP_REG_MBOX_IN, FAP_MSG(FAP_MSG_RX, 0));
	}

	irq_restore(status);
}

static void rx_step(void)
{
	if (!rx.period || (int)(read_c0(CP0_COUNT, 0) - rx.next) >= 0)
		rx_fill();

	/* the host waits for a doorbell and has entries it has not seen */
	if (rxq->irq_armed && rxq->comp_cons != rx.comp_prod)
		rx_notify();
}

/*
 * Drives one iuDMA TX channel the host does not use, set up as the enetsw
 * driver sets up its own, with a single descriptor in PSM. The host built
 * the frame; here we only check the engine took the descriptor.
 */
static void tx_test(void)
{
	volatile unsigned int *dma = (volatile unsigned int *)KSEG1(ENETSW_DMA);
	volatile struct iudma_desc *desc = (volatile struct iudma_desc *)
		(FAP_4KE_PSM + FAP_TX_DESC_PSM_OFFSET);
	unsigned int ch = shared->tx.channel;
	volatile unsigned int *cfg, *sram;
	unsigned int start, waited, i;

	cfg = &dma[(IUDMA_CHANNELS + ch * IUDMA_CH_STRIDE) / 4];
	sram = &dma[(IUDMA_SRAM + ch * IUDMA_CH_STRIDE) / 4];

	shared->tx.kick_ticks = 0;
	shared->tx.done_ticks = 0;
	shared->tx.status = 0;
	shared->tx.irq_status = 0;

	/* the enetsw driver enables the controller when its interface opens */
	shared->tx.controller_cfg = dma[0];
	if (!(shared->tx.controller_cfg & DMA_MASTER_EN))
		return;

	desc->addr = shared->tx.buf;
	desc->len_status = (shared->tx.len << DMA_LEN_SHIFT) | DMA_OWN |
			   DMA_SOP | DMA_EOP | DMA_WRAP | shared->tx.append;

	sram[IUDMA_SRAM_RSTART / 4] = FAP_PSM_HOST_BASE + FAP_TX_DESC_PSM_OFFSET;
	for (i = 1; i < IUDMA_SRAM_WORDS; i++)
		sram[i] = 0;
	cfg[IUDMA_CH_MAXBURST / 4] = IUDMA_MAXBURST;
	cfg[IUDMA_CH_INTSTAT / 4] = cfg[IUDMA_CH_INTSTAT / 4];
	__asm__ volatile("sync" ::: "memory");

	start = read_c0(CP0_COUNT, 0);
	cfg[IUDMA_CH_FLOWCTL / 4] = IUDMA_EN;
	shared->tx.kick_ticks = read_c0(CP0_COUNT, 0) - start;

	start = read_c0(CP0_COUNT, 0);
	do {
		waited = read_c0(CP0_COUNT, 0) - start;
		if (waited > TX_DONE_TIMEOUT)
			break;
	} while (desc->len_status & DMA_OWN);

	if (desc->len_status & DMA_OWN) {
		/* stop the channel and take the descriptor back */
		cfg[IUDMA_CH_FLOWCTL / 4] = 0;
		start = read_c0(CP0_COUNT, 0);
		while ((cfg[IUDMA_CH_FLOWCTL / 4] & IUDMA_EN) &&
		       read_c0(CP0_COUNT, 0) - start <= TX_DONE_TIMEOUT)
			;
		desc->len_status &= ~DMA_OWN;
	} else {
		shared->tx.done_ticks = waited;
	}

	shared->tx.status = desc->len_status & 0xffff;
	shared->tx.irq_status = cfg[IUDMA_CH_INTSTAT / 4];
	cfg[IUDMA_CH_INTSTAT / 4] = shared->tx.irq_status;
}

static void run_command(unsigned int cmd)
{
	switch (FAP_MSG_TYPE(cmd)) {
	case FAP_CMD_PROBE_IUDMA:
		iudma_probe();
		break;
	case FAP_CMD_PROBE_DDR:
		ddr_probe(FAP_MSG_VALUE(cmd));
		break;
	case FAP_CMD_PEEK:
		peek();
		break;
	case FAP_CMD_RX_START:
		rx_start();
		break;
	case FAP_CMD_RX_STOP:
		rx_stop();
		break;
	case FAP_CMD_TX_TEST:
		tx_test();
		break;
	}

	mbox_pong(cmd);
}

static void irq_init(void)
{
	shared->timer_lat_min = ~0u;
	write_c0(CP0_COMPARE, 0, read_c0(CP0_COUNT, 0) + FAP_TIMER_PERIOD);

	reg_wr(FAP_REG_GP_MASK, reg_rd(FAP_REG_GP_MASK) | FAP_GP_MBOX_OUT);
	reg_wr(FAP_REG_IRQ_4KE_MASK, FAP_IRQ_4KE_MBOX_OUT);

	write_c0(CP0_STATUS, 0, ST0_IM_FAP | ST0_IM_TIMER | ST0_IE);
}

void fap_exception(unsigned int cause, unsigned int epc,
		   unsigned int badvaddr)
{
	log_reg("exception cause", cause);
	log_reg("epc", epc);
	log_reg("badvaddr", badvaddr);
	mbox_send(FAP_MSG_EXCEPTION, (cause >> 2) & 0x1f);

	for (;;)
		;
}

void fap_main(void)
{
	unsigned int ready = 0;

	reg_wr(FAP_REG_GP_STATUS, FAP_GP_MBOX_OUT);
	main_loops = 0;
	main_loops_seen = 0;
	command = 0;
	rx.active = 0;
	shared->magic = FAP_SHARED_MAGIC;
	shared->count_res = count_res();

	log_puts("bcm6362 fap up\n");
	log_reg("prid", read_c0(CP0_PRID, 0));
	log_reg("config", read_c0(CP0_CONFIG, 0));
	log_reg("config1", read_c0(CP0_CONFIG, 1));
	log_reg("intctl", read_c0(CP0_STATUS, 1));
	shared->ebase = read_c0(CP0_EBASE, 1);
	log_reg("ebase", shared->ebase);
	log_reg("count res", shared->count_res);

	if (dspram_test())
		ready |= FAP_READY_DSPRAM;
	if (local_mem_test("psm", FAP_4KE_PSM, FAP_PSM_SIZE))
		ready |= FAP_READY_PSM;
	if (local_mem_test("qsm", FAP_4KE_QSM, FAP_QSM_SIZE))
		ready |= FAP_READY_QSM;

	bench();

	/* before the interrupts, which send pongs on the same mailbox */
	mbox_send(FAP_MSG_READY, ready);

	irq_init();
	log_reg("gp mask", reg_rd(FAP_REG_GP_MASK));

	for (;;) {
		unsigned int cmd = command;

		main_loops++;
		if (cmd) {
			command = 0;
			run_command(cmd);
		}
		if (rx.active)
			rx_step();
		else
			__asm__ volatile("wait");
	}
}
