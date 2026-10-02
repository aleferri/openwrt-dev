/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Definitions shared by the BCM6362 FAP host driver and the 4KE firmware
 */

#ifndef __BCM6362_FAP_SHARED_H
#define __BCM6362_FAP_SHARED_H

/* memory block holding the firmware, naturally aligned to its size */
#define FAP_MEM_SIZE		0x10000
#define FAP_SHARED_OFFSET	0xf000
#define FAP_LOG_SIZE		0x800

/* 4KE physical address the block is mapped at to cover the reset vector */
#define FAP_4KE_BOOT_BASE	0x1fc00000

/* queue and packet shared memories, local to the FAP */
#define FAP_QSM_SIZE		0x4000
#define FAP_PSM_SIZE		0x8000

/* fill pattern of the memory tests, word index i */
#define FAP_MEM_PATTERN(i)	(((i) << 16) | (~(i) & 0xffff))

/* control registers used by both sides */
#define FAP_REG_IRQ_4KE_MASK	0x00
#define FAP_REG_IRQ_4KE_STATUS	0x04
#define  FAP_IRQ_4KE_MBOX_OUT	(1 << 5)	/* host wrote FAP_REG_MBOX_OUT */
#define FAP_REG_GP_MASK		0x10
#define FAP_REG_GP_STATUS	0x14
#define  FAP_GP_MBOX_IN		(1 << 2)	/* 4KE wrote FAP_REG_MBOX_IN */
#define  FAP_GP_MBOX_OUT	(1 << 3)	/* host wrote FAP_REG_MBOX_OUT */
#define FAP_REG_MBOX_IN		0x28		/* 4KE to host */
#define FAP_REG_MBOX_OUT	0x2c		/* host to 4KE */

/*
 * 4KE to host messages: type in the upper half, value in the lower one.
 * Every host write to FAP_REG_MBOX_OUT is a ping, answered by a pong that
 * carries the lower half of the written value.
 */
#define FAP_MSG(type, val)	(((type) << 16) | ((val) & 0xffff))
#define FAP_MSG_TYPE(msg)	((msg) >> 16)
#define FAP_MSG_VALUE(msg)	((msg) & 0xffff)

#define FAP_MSG_READY		1
#define  FAP_READY_DSPRAM	(1 << 0)	/* memory test passed */
#define  FAP_READY_PSM		(1 << 1)	/* memory test passed */
#define  FAP_READY_QSM		(1 << 2)	/* memory test passed */
#define FAP_MSG_PONG		2
#define FAP_MSG_EXCEPTION	3

#define FAP_SHARED_MAGIC	0x46415034

/* CP0 Count increments between two timer interrupts */
#define FAP_TIMER_PERIOD	100000

/*
 * Access benchmark: FAP_BENCH_ITERS iterations of FAP_BENCH_UNROLL
 * accesses to the same word of each target.
 */
#define FAP_BENCH_ITERS		1024
#define FAP_BENCH_UNROLL	8

#ifndef __ASSEMBLER__
enum fap_bench_target {
	FAP_BENCH_DSPRAM,
	FAP_BENCH_PSM,
	FAP_BENCH_QSM,
	FAP_BENCH_REGS,
	FAP_BENCH_DDR,
	FAP_BENCH_NR
};

/* CP0 Count ticks taken by a whole benchmark run */
struct fap_bench {
	unsigned int load;		/* load, then add the loaded value */
	unsigned int store;		/* stores, then sync */
};

struct fap_shared {
	unsigned int magic;
	unsigned int heartbeat;		/* timer interrupts */
	unsigned int log_head;
	unsigned int count_res;		/* cycles per CP0 Count increment */
	unsigned int pong_count;	/* CP0 Count when the last pong left */

	unsigned int mbox_irq;		/* pings taken by the FAP interrupt */
	unsigned int mbox_polled;	/* pings found by the timer interrupt */
	unsigned int mbox_lost;		/* messages overwritten before the ack */
	unsigned int irq_lines;		/* Cause.IP raised with a ping pending */
	unsigned int irq_masked;	/* Cause.IP masked as unknown or stuck */
	unsigned int timer_lat_min;	/* CP0 Count from Compare to handler */
	unsigned int timer_lat_max;

	unsigned int bench_alu;		/* the load loop, an add for each load */
	struct fap_bench bench[FAP_BENCH_NR];

	char log[FAP_LOG_SIZE];
};

_Static_assert(sizeof(struct fap_shared) <= FAP_MEM_SIZE - FAP_SHARED_OFFSET,
	       "shared area larger than its slot");
#endif

#endif /* __BCM6362_FAP_SHARED_H */
