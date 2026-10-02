// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 FAP firmware: log, local memory tests, access timings,
 * mailbox ping/pong and CP0 timer on interrupts
 */

#include "fap_shared.h"
#include "fap4ke.h"

/* CP0 Count ticks the 4KE waits for the host to take a message */
#define MBOX_ACK_TIMEOUT	(10 * FAP_TIMER_PERIOD)

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

extern char _dspram_end[], _stack_bottom[];

static volatile struct fap_shared *const shared =
	(volatile struct fap_shared *)KSEG1(FAP_4KE_BOOT_BASE + FAP_SHARED_OFFSET);

/* a DDR word for the access timings, the image block is in DDR */
static volatile unsigned int ddr_word;

/* main loop iterations, the timer turns them into the heartbeat */
static volatile unsigned int main_loops __attribute__((section(".dspram")));
static unsigned int main_loops_seen __attribute__((section(".dspram")));

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

/* the host clears FAP_GP_MBOX_IN once it has read the previous message */
static void mbox_send(unsigned int type, unsigned int val)
{
	unsigned int start = read_c0(CP0_COUNT, 0);

	while (reg_rd(FAP_REG_GP_STATUS) & FAP_GP_MBOX_IN) {
		if (read_c0(CP0_COUNT, 0) - start > MBOX_ACK_TIMEOUT) {
			shared->mbox_lost++;
			break;
		}
	}

	/* shared memory updates first, the host reads them on the message */
	__asm__ volatile("sync" ::: "memory");
	reg_wr(FAP_REG_MBOX_IN, FAP_MSG(type, val));
}

static void mbox_pong(void)
{
	unsigned int msg;

	reg_wr(FAP_REG_GP_STATUS, FAP_GP_MBOX_OUT);
	msg = reg_rd(FAP_REG_MBOX_OUT);
	shared->pong_count = read_c0(CP0_COUNT, 0);
	mbox_send(FAP_MSG_PONG, msg);
}

static void irq_mask_lines(unsigned int lines)
{
	write_c0(CP0_STATUS, 0, read_c0(CP0_STATUS, 0) & ~lines);
	shared->irq_masked |= lines;
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

	/* keeps the mailbox alive if its interrupt never shows up */
	if (reg_rd(FAP_REG_GP_STATUS) & FAP_GP_MBOX_OUT) {
		shared->mbox_polled++;
		mbox_pong();
	}
}

void fap_irq(unsigned int cause)
{
	unsigned int lines = cause & read_c0(CP0_STATUS, 0) & CAUSE_IP;

	if (cause & CAUSE_TI)
		irq_timer();

	if (reg_rd(FAP_REG_IRQ_4KE_STATUS) & FAP_IRQ_4KE_MBOX_OUT) {
		shared->mbox_irq++;
		shared->irq_lines |= lines;
		mbox_pong();

		if (reg_rd(FAP_REG_IRQ_4KE_STATUS) & FAP_IRQ_4KE_MBOX_OUT) {
			reg_wr(FAP_REG_IRQ_4KE_MASK, 0);
			irq_mask_lines(lines);
		}
	} else if (!(cause & CAUSE_TI)) {
		irq_mask_lines(lines);
	}
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
}

static void irq_init(void)
{
	shared->timer_lat_min = ~0u;
	write_c0(CP0_COMPARE, 0, read_c0(CP0_COUNT, 0) + FAP_TIMER_PERIOD);

	reg_wr(FAP_REG_GP_MASK, reg_rd(FAP_REG_GP_MASK) | FAP_GP_MBOX_OUT);
	reg_wr(FAP_REG_IRQ_4KE_MASK, FAP_IRQ_4KE_MBOX_OUT);

	write_c0(CP0_STATUS, 0, ST0_IM_HW | ST0_IE);
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
	shared->magic = FAP_SHARED_MAGIC;
	shared->count_res = count_res();

	log_puts("bcm6362 fap up\n");
	log_reg("prid", read_c0(CP0_PRID, 0));
	log_reg("config", read_c0(CP0_CONFIG, 0));
	log_reg("config1", read_c0(CP0_CONFIG, 1));
	log_reg("intctl", read_c0(CP0_STATUS, 1));
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

	for (;;)
		main_loops++;
}
