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

/* exception base set at boot onto the block, kseg0 */
#define FAP_4KE_EBASE		(0x80000000 | FAP_4KE_BOOT_BASE)
#define FAP_EBASE_BASE_MASK	0xfffff000	/* the rest is CPUNum */

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
#define FAP_REG_SCRATCH		0x84		/* free for any use */

/*
 * Mailbox messages, both ways: type in the upper half, value in the lower
 * one. The 4KE answers each host command with a pong carrying the lower
 * half of the command, once the command is done.
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
#define FAP_MSG_RX		4	/* completions posted, the host had armed */

#define FAP_CMD_PING		0
#define FAP_CMD_PROBE_IUDMA	1	/* fills fap_shared.iudma */
#define FAP_CMD_PROBE_DDR	2	/* value: host address of the block >> 16 */
#define FAP_CMD_PEEK		3	/* reads the word at fap_shared.peek_addr */
#define FAP_CMD_RX_START	4	/* setup in fap_shared.rx and fap_rx_qsm */
#define FAP_CMD_RX_STOP		5
#define FAP_CMD_TX_TEST		6	/* setup in fap_shared.tx */

/* ENETSW iuDMA channels sampled by FAP_CMD_PROBE_IUDMA */
#define FAP_IUDMA_PROBE_CHANNELS	2

#define FAP_SHARED_MAGIC	0x46415034

/* CP0 Count increments between two timer interrupts */
#define FAP_TIMER_PERIOD	100000

/*
 * Access benchmark: FAP_BENCH_ITERS iterations of FAP_BENCH_UNROLL
 * accesses to the same word of each target.
 */
#define FAP_BENCH_ITERS		1024
#define FAP_BENCH_UNROLL	8

/* DDR loads alternating between two words this far apart, i < 2 */
#define FAP_BENCH_DDR_FAR(i)	(0x1000 << (3 * (i)))

/*
 * Receive path test. The host owns a pool of buffers in DDR and hands
 * their addresses to the 4KE through a free ring in QSM; the 4KE writes a
 * frame header into each and gives it back through a completion ring in
 * DDR. Each side writes across the bus and reads locally. Indices are
 * free running counters, the rings are powers of two.
 */
#define FAP_RX_BUF_SIZE		0x800
#define FAP_RX_FREE_ENTRIES	256
#define FAP_RX_COMP_ENTRIES	256
#define FAP_RX_QSM_OFFSET	0

/*
 * Completion entry: the phase bit flips at every pass over the ring, so
 * the host tells new entries apart without a producer index. Buffers are
 * FAP_RX_BUF_SIZE aligned and below 256 MB.
 */
#define FAP_RX_COMP_PHASE	0x80000000
#define FAP_RX_COMP_ADDR	0x0ffff800
#define FAP_RX_COMP_LEN		0x000007ff
#define FAP_RX_PHASE(count, entries) \
	((((count) / (entries)) & 1) ? 0 : FAP_RX_COMP_PHASE)

/* the header the 4KE writes in each buffer, numbered frame after frame */
#define FAP_RX_PATTERN_WORDS	16
#define FAP_RX_PATTERN(seq, i)	((seq) * FAP_RX_PATTERN_WORDS + (i))

/*
 * Transmit test: the 4KE drives one iuDMA TX channel the host does not
 * use, with a single descriptor in PSM, and reports whether the engine
 * took the frame (OWN cleared). The host fills the buffer and reads the
 * result; it owns nothing else of that channel.
 */
#define FAP_TX_DESC_PSM_OFFSET	0x100	/* past the receive rings */

/* ENETSW iuDMA controller_cfg */
#define DMA_MASTER_EN		0x1

/* iuDMA descriptor, big-endian: length in the high half, status in the low */
#define DMA_LEN_SHIFT		16
#define DMA_OWN			0x8000
#define DMA_EOP			0x4000
#define DMA_SOP			0x2000
#define DMA_WRAP		0x1000
#define DMA_APPEND_BRCM_TAG	0x0200
#define DMA_APPEND_CRC		0x0100

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

/* ENETSW iuDMA as read by the 4KE */
struct fap_iudma {
	unsigned int controller_cfg;
	struct {
		unsigned int cfg;
		unsigned int ring;	/* state RAM: descriptor ring start */
	} ch[FAP_IUDMA_PROBE_CHANNELS];
	unsigned int load_regs;		/* benchmark ticks, controller_cfg */
	unsigned int load_sram;		/* benchmark ticks, state RAM */
};

struct fap_shared {
	unsigned int magic;
	unsigned int heartbeat;		/* timer interrupts */
	unsigned int log_head;
	unsigned int count_res;		/* cycles per CP0 Count increment */
	unsigned int pong_count;	/* CP0 Count when the last pong left */

	unsigned int pings;
	unsigned int mbox_lost;		/* messages overwritten before the ack */
	unsigned int timer_lat_min;	/* CP0 Count from Compare to handler */
	unsigned int timer_lat_max;

	unsigned int bench_alu;		/* the load loop, an add for each load */
	struct fap_bench bench[FAP_BENCH_NR];
	unsigned int bench_ddr_far[2];	/* load ticks, see FAP_BENCH_DDR_FAR */
	struct fap_iudma iudma;

	/* the shared area read at its host physical address, untranslated */
	unsigned int ddr_direct_magic;
	unsigned int ddr_direct_load;	/* benchmark ticks */

	unsigned int ebase;		/* read back at boot */

	unsigned int peek_addr;		/* 4KE virtual address, from the host */
	unsigned int peek_val;
	unsigned int peek_ticks;	/* CP0 Count ticks around the load */

	struct fap_rx {
		/* set by the host before FAP_CMD_RX_START */
		unsigned int ram_window;	/* 4KE address of host address 0 */
		unsigned int comp_ring;		/* host address */
		unsigned int frame_len;
		unsigned int period;		/* CP0 Count ticks between frames, 0 = as fast as possible */
		/* written by the 4KE */
		unsigned int frames;
		unsigned int notifies;
		unsigned int idle_free;		/* passes without free buffers */
		unsigned int idle_ring;		/* passes with the ring full */
		unsigned int busy_ticks;	/* CP0 Count spent filling frames */
	} rx;

	struct fap_tx {
		/* set by the host before FAP_CMD_TX_TEST */
		unsigned int channel;		/* iuDMA channel index */
		unsigned int buf;		/* host address of the frame */
		unsigned int len;
		unsigned int append;		/* DMA_APPEND_* flags */
		/* written by the 4KE */
		unsigned int controller_cfg;	/* sent only with DMA_MASTER_EN */
		unsigned int kick_ticks;	/* CP0 Count to start the engine */
		unsigned int done_ticks;	/* until OWN cleared, 0 if stuck */
		unsigned int status;		/* descriptor status read back */
		unsigned int irq_status;	/* channel interrupt status, then cleared */
	} tx;

	char log[FAP_LOG_SIZE];
};

/* in QSM: written by the host and read by the 4KE, but irq_armed */
struct fap_rx_qsm {
	unsigned int free_prod;
	unsigned int comp_cons;
	unsigned int irq_armed;		/* the 4KE clears it when it notifies */
	unsigned int reserved;
	unsigned int free_ring[FAP_RX_FREE_ENTRIES];
};

_Static_assert(FAP_RX_QSM_OFFSET + sizeof(struct fap_rx_qsm) <= FAP_QSM_SIZE,
	       "receive queue larger than the QSM");

_Static_assert(sizeof(struct fap_shared) <= FAP_MEM_SIZE - FAP_SHARED_OFFSET,
	       "shared area larger than its slot");
#endif

#endif /* __BCM6362_FAP_SHARED_H */
