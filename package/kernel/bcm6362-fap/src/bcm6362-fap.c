// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 FAP coprocessor loader
 *
 * Boots a firmware image on the MIPS 4KE core of the FAP block: the image is
 * placed in a host memory block that the FAP address translators map both at
 * the 4KE reset vector and at 4KE physical address 0 (exception vectors).
 *
 * The firmware is requested asynchronously, so the driver does not need it
 * at probe time: the 4KE boots once it shows up in /lib/firmware and stays
 * in reset if it never does. Once running, it stays idle; a watchdog holds
 * it in reset if its heartbeat stalls or it reports an exception.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include "fap_shared.h"

#define FAP_REG_IRQ_MIPS_MASK	0x08
#define FAP_REG_IRQ_MIPS_STATUS	0x0c
#define  FAP_IRQ_MIPS_MBOX_IN	BIT(4)

/* address translators: mask, 4KE base and host base registers in a row */
#define FAP_REG_ADDR1		0x6c
#define FAP_REG_ADDR2		0x78
#define  FAP_ADDR_MASK		0x0
#define  FAP_ADDR_BASE_IN	0x4
#define  FAP_ADDR_BASE_OUT	0x8
/* 4KE address bits the translators ignore, making kseg0/kseg1 aliases */
#define FAP_ADDR_IGNORE		0xa0000000

#define FAP_REG_SOFT_RESETS	0x8c
#define  FAP_SOFT_RESET_4KE	BIT(0)
#define FAP_REG_CORE_STATUS	0x94

#define FAP_PING_TIMEOUT_MS	100
#define FAP_CLOCK_SAMPLE_MS	1000
#define FAP_WATCHDOG_MS		1000
#define FAP_HOST_BENCH_ACCESSES	4096

/* receive test: buffers in flight, frame length, entries per poll */
#define FAP_RX_POOL_BUFS	128
#define FAP_RX_TEST_LEN		1514
#define FAP_RX_BUDGET		64

/* transmit test: an iuDMA channel the host does not use */
#define FAP_TX_TEST_CHANNEL	3	/* logical channel 1, TX */

/*
 * Broadcom tags after the source address, as DSA builds them
 * (net/dsa/tag_brcm.c): the internal switch takes the legacy 6-byte tag on
 * its CPU port; a b53 switch cascaded behind one of its ports takes the
 * 4-byte tag after it. The switches check the minimum length once the tags
 * are stripped, so frames are padded as the taggers pad them.
 */
#define BRCM_LEG_TAG_TYPE	0x8874
#define BRCM_LEG_EGRESS		(2 << 5)
#define BRCM_LEG_TAG_LEN	6
#define BRCM_IG_OPCODE		(1 << 5)
#define BRCM_TAG_LEN		4
#define FAP_SWITCH_CPU_PORT	8
#define FAP_TX_TEST_LEN		(ETH_ZLEN + 2 * (BRCM_LEG_TAG_LEN + BRCM_TAG_LEN))

/* the RAM window must sit below the peripherals in the 4KE space */
#define FAP_4KE_PERIPH		0x10000000

static char *firmware = "bcm6362-fap.bin";
module_param(firmware, charp, 0444);
MODULE_PARM_DESC(firmware, "4KE firmware image in /lib/firmware");

enum bcm6362_fap_state {
	FAP_OFFLINE,	/* held in reset by the host */
	FAP_IDLE,	/* running, waiting for work */
	FAP_CRASHED,	/* held in reset by the watchdog, log kept */
};

static const char *const bcm6362_fap_state_names[] = {
	[FAP_OFFLINE] = "offline",
	[FAP_IDLE] = "idle",
	[FAP_CRASHED] = "crashed",
};

struct bcm6362_fap {
	struct device *dev;
	void __iomem *regs;
	void __iomem *qsm;
	void __iomem *psm;

	void *mem;
	dma_addr_t mem_dma;
	struct fap_shared *shared;

	u32 mbox_last;
	unsigned int mbox_count;

	/* serializes state changes and pings */
	struct mutex lock;
	enum bcm6362_fap_state state;
	struct delayed_work watchdog;
	u32 heartbeat;
	bool exception;

	u16 ping_seq;
	u32 count_hz;		/* measured by "clock", 0 until then */
	struct completion pong;
	/* completed once the firmware request has been answered */
	struct completion fw_done;
	u32 pong_msg;
	ktime_t pong_time;

	/* receive test, all but active touched only by the test and its work */
	struct {
		bool active;
		struct work_struct work;
		void *pool;
		dma_addr_t pool_dma;
		u32 *comp;
		dma_addr_t comp_dma;
		u32 comp_cons;
		u32 free_prod;
		u32 seq;
		u64 frames;
		u64 errors;
		u64 gaps;
		u64 bad;
		u64 busy_ns;
		ktime_t start;
		ktime_t stop;
	} rx;

	/* last transmit test frame */
	struct {
		bool sent;
		u8 port;
		s8 ext;		/* port of the cascaded switch, -1 if none */
		u16 seq;
	} tx;

	struct dentry *debugfs;
};

static inline u32 fap_rd(struct bcm6362_fap *fap, unsigned int off)
{
	return __raw_readl(fap->regs + off);
}

static inline void fap_wr(struct bcm6362_fap *fap, u32 val, unsigned int off)
{
	__raw_writel(val, fap->regs + off);
}

/* a power of two sized, naturally aligned window on both sides */
static void bcm6362_fap_map(struct bcm6362_fap *fap, unsigned int reg,
			    u32 fap_base, dma_addr_t host_base, u32 size)
{
	fap_wr(fap, ~((size - 1) | FAP_ADDR_IGNORE), reg + FAP_ADDR_MASK);
	fap_wr(fap, fap_base, reg + FAP_ADDR_BASE_IN);
	fap_wr(fap, host_base, reg + FAP_ADDR_BASE_OUT);
}

static irqreturn_t bcm6362_fap_isr(int irq, void *data)
{
	struct bcm6362_fap *fap = data;
	u32 msg;

	if (!(fap_rd(fap, FAP_REG_IRQ_MIPS_STATUS) & FAP_IRQ_MIPS_MBOX_IN))
		return IRQ_NONE;

	/*
	 * Read the message, then clear the interrupt: clearing first could
	 * drop a notification the 4KE raises in between. A fresh message
	 * right after is normal, not a stuck interrupt.
	 */
	msg = fap_rd(fap, FAP_REG_MBOX_IN);
	fap_wr(fap, FAP_GP_MBOX_IN, FAP_REG_GP_STATUS);

	fap->mbox_last = msg;
	fap->mbox_count++;

	switch (FAP_MSG_TYPE(msg)) {
	case FAP_MSG_PONG:
		fap->pong_time = ktime_get();
		fap->pong_msg = msg;
		complete(&fap->pong);
		break;
	case FAP_MSG_READY:
		dev_info(fap->dev, "4KE ready, dspram %s, psm %s, qsm %s\n",
			 msg & FAP_READY_DSPRAM ? "ok" : "failed",
			 msg & FAP_READY_PSM ? "ok" : "failed",
			 msg & FAP_READY_QSM ? "ok" : "failed");
		break;
	case FAP_MSG_RX:
		if (READ_ONCE(fap->rx.active))
			queue_work(system_highpri_wq, &fap->rx.work);
		break;
	case FAP_MSG_EXCEPTION:
		dev_err(fap->dev, "4KE exception, ExcCode %u\n",
			FAP_MSG_VALUE(msg));
		WRITE_ONCE(fap->exception, true);
		mod_delayed_work(system_wq, &fap->watchdog, 0);
		break;
	default:
		dev_dbg(fap->dev, "mailbox 0x%08x\n", msg);
		break;
	}

	return IRQ_HANDLED;
}

static void bcm6362_fap_hold(struct bcm6362_fap *fap)
{
	fap_wr(fap, fap_rd(fap, FAP_REG_SOFT_RESETS) | FAP_SOFT_RESET_4KE,
	       FAP_REG_SOFT_RESETS);
	fap_wr(fap, 0, FAP_REG_IRQ_4KE_MASK);
	fap_wr(fap, 0, FAP_REG_IRQ_MIPS_MASK);
	fap_wr(fap, 0, FAP_REG_GP_MASK);
}

static void bcm6362_fap_watchdog(struct work_struct *work)
{
	struct bcm6362_fap *fap = container_of(to_delayed_work(work),
					       struct bcm6362_fap, watchdog);
	u32 heartbeat;

	guard(mutex)(&fap->lock);

	if (fap->state != FAP_IDLE)
		return;

	heartbeat = READ_ONCE(fap->shared->heartbeat);

	if (READ_ONCE(fap->exception)) {
		dev_err(fap->dev, "4KE held in reset after an exception\n");
	} else if (heartbeat == fap->heartbeat) {
		dev_err(fap->dev, "4KE held in reset, heartbeat stuck at %u\n",
			heartbeat);
	} else {
		fap->heartbeat = heartbeat;
		schedule_delayed_work(&fap->watchdog,
				      msecs_to_jiffies(FAP_WATCHDOG_MS));
		return;
	}

	bcm6362_fap_hold(fap);
	fap->state = FAP_CRASHED;
}

/* the 4KE answers with the low half of val and the CP0 Count it sent at */
static int bcm6362_fap_ping(struct bcm6362_fap *fap, u32 val,
			    ktime_t *time, u32 *count)
{
	lockdep_assert_held(&fap->lock);

	if (fap->state != FAP_IDLE)
		return -ENODEV;

	reinit_completion(&fap->pong);
	fap_wr(fap, val, FAP_REG_MBOX_OUT);

	if (!wait_for_completion_timeout(&fap->pong,
					 msecs_to_jiffies(FAP_PING_TIMEOUT_MS)))
		return -ETIMEDOUT;

	if (FAP_MSG_VALUE(fap->pong_msg) != FAP_MSG_VALUE(val))
		return -EIO;

	if (time)
		*time = fap->pong_time;
	if (count) {
		/* pairs with the sync the firmware issues before the pong */
		rmb();
		*count = READ_ONCE(fap->shared->pong_count);
	}

	return 0;
}

static int regs_show(struct seq_file *s, void *unused)
{
	static const struct {
		const char *name;
		unsigned int off;
	} regs[] = {
		{ "irq_4ke_mask", FAP_REG_IRQ_4KE_MASK },
		{ "irq_4ke_status", FAP_REG_IRQ_4KE_STATUS },
		{ "irq_mips_mask", FAP_REG_IRQ_MIPS_MASK },
		{ "irq_mips_status", FAP_REG_IRQ_MIPS_STATUS },
		{ "gp_mask", FAP_REG_GP_MASK },
		{ "gp_status", FAP_REG_GP_STATUS },
		{ "mbox_in", FAP_REG_MBOX_IN },
		{ "mbox_out", FAP_REG_MBOX_OUT },
		{ "addr1_mask", FAP_REG_ADDR1 + FAP_ADDR_MASK },
		{ "addr1_base_in", FAP_REG_ADDR1 + FAP_ADDR_BASE_IN },
		{ "addr1_base_out", FAP_REG_ADDR1 + FAP_ADDR_BASE_OUT },
		{ "addr2_mask", FAP_REG_ADDR2 + FAP_ADDR_MASK },
		{ "addr2_base_in", FAP_REG_ADDR2 + FAP_ADDR_BASE_IN },
		{ "addr2_base_out", FAP_REG_ADDR2 + FAP_ADDR_BASE_OUT },
		{ "soft_resets", FAP_REG_SOFT_RESETS },
		{ "core_status", FAP_REG_CORE_STATUS },
	};
	struct bcm6362_fap *fap = s->private;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		seq_printf(s, "%-16s 0x%08x\n", regs[i].name,
			   fap_rd(fap, regs[i].off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(regs);

static int log_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	u32 head, i;

	head = READ_ONCE(fap->shared->log_head);
	/* pairs with the sync the firmware issues before moving log_head */
	rmb();

	for (i = head > FAP_LOG_SIZE ? head - FAP_LOG_SIZE : 0; i != head; i++)
		seq_putc(s, fap->shared->log[i % FAP_LOG_SIZE]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(log);

static void mem_pattern_show(struct seq_file *s, void __iomem *mem,
			     unsigned int size)
{
	unsigned int errors = 0;
	unsigned int i;
	u32 val;

	for (i = 0; i < size / 4; i++) {
		val = __raw_readl(mem + i * 4);
		if (val == FAP_MEM_PATTERN(i))
			continue;

		if (!errors)
			seq_printf(s, "first mismatch at 0x%04x: 0x%08x\n",
				   i * 4, val);
		errors++;
	}

	seq_printf(s, "%u of %u words differ from the test pattern\n",
		   errors, size / 4);
}

static int qsm_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;

	mem_pattern_show(s, fap->qsm, FAP_QSM_SIZE);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(qsm);

static int psm_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;

	mem_pattern_show(s, fap->psm, FAP_PSM_SIZE);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(psm);

static int mbox_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;

	seq_printf(s, "last 0x%08x count %u magic 0x%08x heartbeat %u\n",
		   fap->mbox_last, fap->mbox_count,
		   READ_ONCE(fap->shared->magic),
		   READ_ONCE(fap->shared->heartbeat));

	return 0;
}

static ssize_t mbox_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct bcm6362_fap *fap = file_inode(file)->i_private;
	u32 val;
	int ret;

	ret = kstrtou32_from_user(buf, count, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&fap->lock);
	ret = bcm6362_fap_ping(fap, val, NULL, NULL);
	mutex_unlock(&fap->lock);

	return ret ?: count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(mbox);

static int state_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;

	seq_printf(s, "%s\n", bcm6362_fap_state_names[READ_ONCE(fap->state)]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(state);

/* CP0 Count rate, against the host clock across two pings */
static int clock_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	ktime_t t0, t1;
	u32 c0, c1;
	u64 hz;
	int ret;

	mutex_lock(&fap->lock);
	ret = bcm6362_fap_ping(fap, ++fap->ping_seq, &t0, &c0);
	if (!ret) {
		msleep(FAP_CLOCK_SAMPLE_MS);
		ret = bcm6362_fap_ping(fap, ++fap->ping_seq, &t1, &c1);
	}
	mutex_unlock(&fap->lock);
	if (ret)
		return ret;

	hz = div64_u64((u64)(c1 - c0) * NSEC_PER_SEC,
		       ktime_to_ns(ktime_sub(t1, t0)));
	fap->count_hz = hz;

	seq_printf(s, "count %llu Hz, core %llu Hz\n", hz,
		   hz * READ_ONCE(fap->shared->count_res));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(clock);

/* hundredths of a core cycle per access */
static u32 bench_centicycles(struct bcm6362_fap *fap, u32 ticks)
{
	u64 cycles = (u64)ticks * READ_ONCE(fap->shared->count_res) * 100;

	return div_u64(cycles, FAP_BENCH_ITERS * FAP_BENCH_UNROLL);
}

static int stats_show(struct seq_file *s, void *unused)
{
	static const char *const names[FAP_BENCH_NR] = {
		[FAP_BENCH_DSPRAM] = "dspram",
		[FAP_BENCH_PSM] = "psm",
		[FAP_BENCH_QSM] = "qsm",
		[FAP_BENCH_REGS] = "regs",
		[FAP_BENCH_DDR] = "ddr",
	};
	struct bcm6362_fap *fap = s->private;
	struct fap_shared *sh = fap->shared;
	u32 load, store;
	int i;

	seq_printf(s, "count_res       %u\n", READ_ONCE(sh->count_res));
	seq_printf(s, "heartbeat       %u\n", READ_ONCE(sh->heartbeat));
	seq_printf(s, "timer latency   %u..%u counts\n",
		   READ_ONCE(sh->timer_lat_min), READ_ONCE(sh->timer_lat_max));
	seq_printf(s, "mbox            pings %u lost %u\n",
		   READ_ONCE(sh->pings), READ_ONCE(sh->mbox_lost));

	load = bench_centicycles(fap, READ_ONCE(sh->bench_alu));
	seq_puts(s, "\ncycles per access    load   store\n");
	seq_printf(s, "%-16s %4u.%02u\n", "alu", load / 100, load % 100);
	for (i = 0; i < FAP_BENCH_NR; i++) {
		load = bench_centicycles(fap, READ_ONCE(sh->bench[i].load));
		store = bench_centicycles(fap, READ_ONCE(sh->bench[i].store));
		seq_printf(s, "%-16s %4u.%02u %4u.%02u\n", names[i],
			   load / 100, load % 100, store / 100, store % 100);
	}
	for (i = 0; i < ARRAY_SIZE(sh->bench_ddr_far); i++) {
		load = bench_centicycles(fap,
					 READ_ONCE(sh->bench_ddr_far[i]));
		seq_printf(s, "ddr %5u apart %4u.%02u\n", FAP_BENCH_DDR_FAR(i),
			   load / 100, load % 100);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stats);

/* how the 4KE sees the ENETSW iuDMA; compare with the host side values */
static int iudma_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	struct fap_iudma *iudma = &fap->shared->iudma;
	u32 cmd, regs, sram;
	int ch, ret;

	mutex_lock(&fap->lock);
	cmd = FAP_MSG(FAP_CMD_PROBE_IUDMA, ++fap->ping_seq);
	ret = bcm6362_fap_ping(fap, cmd, NULL, NULL);
	mutex_unlock(&fap->lock);
	if (ret)
		return ret;

	/* pairs with the sync the firmware issues before the pong */
	rmb();

	seq_printf(s, "controller_cfg  0x%08x\n",
		   READ_ONCE(iudma->controller_cfg));
	for (ch = 0; ch < FAP_IUDMA_PROBE_CHANNELS; ch++)
		seq_printf(s, "channel %d       cfg 0x%08x ring 0x%08x\n", ch,
			   READ_ONCE(iudma->ch[ch].cfg),
			   READ_ONCE(iudma->ch[ch].ring));

	regs = bench_centicycles(fap, READ_ONCE(iudma->load_regs));
	sram = bench_centicycles(fap, READ_ONCE(iudma->load_sram));
	seq_printf(s, "cycles per load regs %u.%02u state ram %u.%02u\n",
		   regs / 100, regs % 100, sram / 100, sram % 100);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(iudma);

/* can the 4KE read DDR at host physical addresses, outside its windows? */
static int ddr_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	u32 load;
	int ret;

	mutex_lock(&fap->lock);
	ret = bcm6362_fap_ping(fap, FAP_MSG(FAP_CMD_PROBE_DDR,
					    fap->mem_dma >> 16), NULL, NULL);
	mutex_unlock(&fap->lock);
	if (ret)
		return ret;

	/* pairs with the sync the firmware issues before the pong */
	rmb();

	load = bench_centicycles(fap, READ_ONCE(fap->shared->ddr_direct_load));
	seq_printf(s, "magic at %pad: 0x%08x, expected 0x%08x\n",
		   &fap->mem_dma, READ_ONCE(fap->shared->ddr_direct_magic),
		   FAP_SHARED_MAGIC);
	seq_printf(s, "cycles per load %u.%02u\n", load / 100, load % 100);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ddr);

static int peek_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;

	seq_printf(s, "0x%08x: 0x%08x, %u counts\n",
		   READ_ONCE(fap->shared->peek_addr),
		   READ_ONCE(fap->shared->peek_val),
		   READ_ONCE(fap->shared->peek_ticks));

	return 0;
}

/* a 4KE virtual address for the 4KE to load once */
static ssize_t peek_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct bcm6362_fap *fap = file_inode(file)->i_private;
	u32 addr;
	int ret;

	ret = kstrtou32_from_user(buf, count, 0, &addr);
	if (ret)
		return ret;
	if (!IS_ALIGNED(addr, 4))
		return -EINVAL;

	guard(mutex)(&fap->lock);

	WRITE_ONCE(fap->shared->peek_addr, addr);
	WRITE_ONCE(fap->shared->peek_val, 0);
	WRITE_ONCE(fap->shared->peek_ticks, 0);
	/* the address is in memory before the command reaches the 4KE */
	wmb();
	ret = bcm6362_fap_ping(fap, FAP_MSG(FAP_CMD_PEEK, ++fap->ping_seq),
			       NULL, NULL);

	return ret ?: count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(peek);

/*
 * Receive test. The QSM is written with plain stores: on MIPS the MMIO
 * accessors issue a sync per access, and the ordering that matters is
 * enforced with explicit barriers.
 */
static void fap_rxq_wr(struct bcm6362_fap *fap, unsigned int off, u32 val)
{
	WRITE_ONCE(*(u32 __force *)(fap->qsm + FAP_RX_QSM_OFFSET + off), val);
}

#define FAP_RXQ(field)		offsetof(struct fap_rx_qsm, field)

static u32 bcm6362_fap_rx_entry(struct bcm6362_fap *fap)
{
	unsigned int slot = fap->rx.comp_cons % FAP_RX_COMP_ENTRIES;
	dma_addr_t dma = fap->rx.comp_dma + slot * 4;
	u32 entry;

	dma_sync_single_for_cpu(fap->dev, dma, 4, DMA_FROM_DEVICE);
	entry = READ_ONCE(fap->rx.comp[slot]);
	/*
	 * The 4KE keeps writing this line, and on most non-coherent MIPS the
	 * sync above does nothing: drop the line again so that the next
	 * read reaches memory.
	 */
	dma_sync_single_for_device(fap->dev, dma, 4, DMA_FROM_DEVICE);

	return entry;
}

static bool bcm6362_fap_rx_ready(struct bcm6362_fap *fap, u32 entry)
{
	return (entry & FAP_RX_COMP_PHASE) ==
	       FAP_RX_PHASE(fap->rx.comp_cons, FAP_RX_COMP_ENTRIES);
}

/* checks the header the 4KE wrote and gives the buffer back to it */
static void bcm6362_fap_rx_frame(struct bcm6362_fap *fap, u32 entry)
{
	u32 addr = entry & FAP_RX_COMP_ADDR;
	u32 off = addr - fap->rx.pool_dma;
	u32 *buf, seq;
	int i;

	if (off >= FAP_RX_POOL_BUFS * FAP_RX_BUF_SIZE ||
	    (entry & FAP_RX_COMP_LEN) != FAP_RX_TEST_LEN) {
		fap->rx.bad++;
		return;
	}

	buf = fap->rx.pool + off;
	dma_sync_single_for_cpu(fap->dev, addr, FAP_RX_PATTERN_WORDS * 4,
				DMA_FROM_DEVICE);

	seq = buf[0] / FAP_RX_PATTERN_WORDS;
	for (i = 0; i < FAP_RX_PATTERN_WORDS; i++) {
		if (buf[i] != FAP_RX_PATTERN(seq, i)) {
			fap->rx.errors++;
			break;
		}
	}
	if (seq != fap->rx.seq)
		fap->rx.gaps++;
	fap->rx.seq = seq + 1;
	fap->rx.frames++;

	dma_sync_single_for_device(fap->dev, addr, FAP_RX_PATTERN_WORDS * 4,
				   DMA_FROM_DEVICE);
	fap_rxq_wr(fap, FAP_RXQ(free_ring) +
		   (fap->rx.free_prod % FAP_RX_FREE_ENTRIES) * 4, addr);
	fap->rx.free_prod++;
}

static int bcm6362_fap_rx_poll(struct bcm6362_fap *fap, int budget)
{
	int n;

	for (n = 0; n < budget; n++) {
		u32 entry = bcm6362_fap_rx_entry(fap);

		if (!bcm6362_fap_rx_ready(fap, entry))
			break;
		bcm6362_fap_rx_frame(fap, entry);
		fap->rx.comp_cons++;
	}

	if (n) {
		/* the free ring entries before the index publishing them */
		wmb();
		fap_rxq_wr(fap, FAP_RXQ(free_prod), fap->rx.free_prod);
		fap_rxq_wr(fap, FAP_RXQ(comp_cons), fap->rx.comp_cons);
	}

	return n;
}

static void bcm6362_fap_rx_work(struct work_struct *work)
{
	struct bcm6362_fap *fap = container_of(work, struct bcm6362_fap,
					       rx.work);
	ktime_t start = ktime_get();

	do {
		while (bcm6362_fap_rx_poll(fap, FAP_RX_BUDGET) == FAP_RX_BUDGET)
			cond_resched();

		fap_rxq_wr(fap, FAP_RXQ(irq_armed), 1);
		/* armed before looking again, or a notification is missed */
		mb();
	} while (bcm6362_fap_rx_ready(fap, bcm6362_fap_rx_entry(fap)));

	fap->rx.busy_ns += ktime_to_ns(ktime_sub(ktime_get(), start));
}

/*
 * The low translator maps the RAM right above its own range in the 4KE
 * space, where nothing else is: host address a is 4KE address size + a.
 */
static int bcm6362_fap_ram_window(struct bcm6362_fap *fap, u32 *base)
{
	struct device_node *np;
	struct resource res;
	u32 size;
	int ret;

	np = of_find_node_by_type(NULL, "memory");
	if (!np)
		return -ENODEV;
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret)
		return ret;

	size = roundup_pow_of_two(res.end + 1);
	if (res.start || 2ULL * size > FAP_4KE_PERIPH) {
		dev_err(fap->dev, "RAM %pR does not fit the 4KE space\n", &res);
		return -ERANGE;
	}

	bcm6362_fap_map(fap, FAP_REG_ADDR1, size, 0, size);
	*base = size;

	return 0;
}

static void bcm6362_fap_rx_free(struct bcm6362_fap *fap)
{
	if (fap->rx.comp)
		dma_free_noncoherent(fap->dev, FAP_RX_COMP_ENTRIES * 4,
				     fap->rx.comp, fap->rx.comp_dma,
				     DMA_FROM_DEVICE);
	if (fap->rx.pool)
		dma_free_noncoherent(fap->dev,
				     FAP_RX_POOL_BUFS * FAP_RX_BUF_SIZE,
				     fap->rx.pool, fap->rx.pool_dma,
				     DMA_FROM_DEVICE);
	fap->rx.comp = NULL;
	fap->rx.pool = NULL;
}

static void bcm6362_fap_rx_stop(struct bcm6362_fap *fap)
{
	lockdep_assert_held(&fap->lock);

	if (!fap->rx.active)
		return;

	/* a held 4KE writes nothing anymore */
	if (fap->state == FAP_IDLE &&
	    bcm6362_fap_ping(fap, FAP_MSG(FAP_CMD_RX_STOP, ++fap->ping_seq),
			     NULL, NULL))
		dev_warn(fap->dev, "4KE did not acknowledge the receive stop\n");

	WRITE_ONCE(fap->rx.active, false);
	cancel_work_sync(&fap->rx.work);
	fap->rx.stop = ktime_get();
	bcm6362_fap_rx_free(fap);
}

static int bcm6362_fap_rx_start(struct bcm6362_fap *fap, u32 rate)
{
	u32 window;
	int i, ret;

	lockdep_assert_held(&fap->lock);

	if (fap->rx.active)
		return -EBUSY;
	if (fap->state != FAP_IDLE || !fap->qsm)
		return -ENODEV;

	if (rate && !fap->count_hz)
		return -EAGAIN;	/* read "clock" first */

	/* the vectors must not need the low window, which the RAM takes */
	if ((READ_ONCE(fap->shared->ebase) & FAP_EBASE_BASE_MASK) !=
	    FAP_4KE_EBASE) {
		dev_err(fap->dev, "4KE EBase 0x%08x, low window still needed\n",
			READ_ONCE(fap->shared->ebase));
		return -EBUSY;
	}

	fap->rx.pool = dma_alloc_noncoherent(fap->dev,
					     FAP_RX_POOL_BUFS * FAP_RX_BUF_SIZE,
					     &fap->rx.pool_dma,
					     DMA_FROM_DEVICE, GFP_KERNEL);
	fap->rx.comp = dma_alloc_noncoherent(fap->dev,
					     FAP_RX_COMP_ENTRIES * 4,
					     &fap->rx.comp_dma,
					     DMA_FROM_DEVICE, GFP_KERNEL);
	if (!fap->rx.pool || !fap->rx.comp) {
		ret = -ENOMEM;
		goto err_free;
	}

	if (fap->rx.pool_dma + FAP_RX_POOL_BUFS * FAP_RX_BUF_SIZE >
	    FAP_RX_COMP_ADDR + 1) {
		ret = -ERANGE;
		goto err_free;
	}

	ret = bcm6362_fap_ram_window(fap, &window);
	if (ret)
		goto err_free;

	/* no dirty line of ours may land over what the 4KE writes */
	memset(fap->rx.comp, 0, FAP_RX_COMP_ENTRIES * 4);
	dma_sync_single_for_device(fap->dev, fap->rx.comp_dma,
				   FAP_RX_COMP_ENTRIES * 4, DMA_FROM_DEVICE);
	dma_sync_single_for_device(fap->dev, fap->rx.pool_dma,
				   FAP_RX_POOL_BUFS * FAP_RX_BUF_SIZE,
				   DMA_FROM_DEVICE);

	fap->rx.comp_cons = 0;
	fap->rx.free_prod = 0;
	fap->rx.seq = 0;
	fap->rx.frames = 0;
	fap->rx.errors = 0;
	fap->rx.gaps = 0;
	fap->rx.bad = 0;
	fap->rx.busy_ns = 0;

	for (i = 0; i < FAP_RX_POOL_BUFS; i++)
		fap_rxq_wr(fap, FAP_RXQ(free_ring) + i * 4,
			   fap->rx.pool_dma + i * FAP_RX_BUF_SIZE);
	fap->rx.free_prod = FAP_RX_POOL_BUFS;
	fap_rxq_wr(fap, FAP_RXQ(comp_cons), 0);
	fap_rxq_wr(fap, FAP_RXQ(irq_armed), 1);
	/* the ring before the index publishing it */
	wmb();
	fap_rxq_wr(fap, FAP_RXQ(free_prod), fap->rx.free_prod);

	WRITE_ONCE(fap->shared->rx.ram_window, window);
	WRITE_ONCE(fap->shared->rx.comp_ring, fap->rx.comp_dma);
	WRITE_ONCE(fap->shared->rx.frame_len, FAP_RX_TEST_LEN);
	WRITE_ONCE(fap->shared->rx.period,
		   rate ? DIV_ROUND_UP(fap->count_hz, rate) : 0);
	/* the setup is in memory before the command reaches the 4KE */
	wmb();

	WRITE_ONCE(fap->rx.active, true);
	fap->rx.start = ktime_get();
	ret = bcm6362_fap_ping(fap, FAP_MSG(FAP_CMD_RX_START, ++fap->ping_seq),
			       NULL, NULL);
	if (ret) {
		WRITE_ONCE(fap->rx.active, false);
		cancel_work_sync(&fap->rx.work);
		goto err_free;
	}

	return 0;

err_free:
	bcm6362_fap_rx_free(fap);
	return ret;
}

/* on removal, after the 4KE is held */
static void bcm6362_fap_rx_release(void *data)
{
	struct bcm6362_fap *fap = data;

	guard(mutex)(&fap->lock);
	bcm6362_fap_rx_stop(fap);
}

static int rxtest_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	struct fap_rx *frx = &fap->shared->rx;
	u64 ns, rate, busy;

	guard(mutex)(&fap->lock);

	ns = ktime_to_ns(ktime_sub(fap->rx.active ? ktime_get() :
				   fap->rx.stop, fap->rx.start));
	rate = ns ? div64_u64(fap->rx.frames * NSEC_PER_SEC, ns) : 0;
	busy = ns ? div64_u64(fap->rx.busy_ns * 10000, ns) : 0;

	seq_printf(s, "active  %s\n", fap->rx.active ? "yes" : "no");
	seq_printf(s, "host    frames %llu errors %llu gaps %llu bad %llu\n",
		   fap->rx.frames, fap->rx.errors, fap->rx.gaps, fap->rx.bad);
	seq_printf(s, "        %llu frames/s, cpu %llu.%02llu%%\n", rate,
		   busy / 100, busy % 100);
	seq_printf(s, "4ke     frames %u notifies %u idle free %u ring %u\n",
		   READ_ONCE(frx->frames), READ_ONCE(frx->notifies),
		   READ_ONCE(frx->idle_free), READ_ONCE(frx->idle_ring));
	if (READ_ONCE(frx->frames))
		seq_printf(s, "        %u cycles/frame on the 4KE\n",
			   READ_ONCE(frx->busy_ticks) / READ_ONCE(frx->frames) *
			   READ_ONCE(fap->shared->count_res));

	return 0;
}

static ssize_t rxtest_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct bcm6362_fap *fap = file_inode(file)->i_private;
	char cmd[24];
	int ret = 0;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, count))
		return -EFAULT;
	cmd[count] = '\0';

	guard(mutex)(&fap->lock);

	if (!strncmp(cmd, "start", 5)) {
		u32 rate = 0;

		if (cmd[5] == ' ' && kstrtou32(cmd + 6, 0, &rate))
			return -EINVAL;
		ret = bcm6362_fap_rx_start(fap, rate);
	} else if (sysfs_streq(cmd, "stop")) {
		bcm6362_fap_rx_stop(fap);
	} else {
		return -EINVAL;
	}

	return ret ?: count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(rxtest);

/*
 * Transmit test. The 4KE drives one iuDMA TX channel the host does not use,
 * with a single descriptor in PSM. Writing "port" sends a numbered broadcast
 * out of that port of the internal switch, "port ext" out of port ext of
 * the switch cascaded behind it; reading shows what the engine did with the
 * last one.
 */

/* locally administered, \x02FAP */
static const u8 fap_tx_test_source[ETH_ALEN] = { 0x02, 0x46, 0x41, 0x50 };

static void bcm6362_fap_tx_frame(u8 *p, unsigned int port, int ext,
				 u16 seq)
{
	memset(p, 0, FAP_TX_TEST_LEN);
	eth_broadcast_addr(p);
	memcpy(p + ETH_ALEN, fap_tx_test_source, ETH_ALEN);
	p += 2 * ETH_ALEN;

	put_unaligned_be16(BRCM_LEG_TAG_TYPE, p);
	p[2] = BRCM_LEG_EGRESS;
	p[5] = port;
	p += BRCM_LEG_TAG_LEN;

	if (ext >= 0) {
		p[0] = BRCM_IG_OPCODE;
		p[3] = BIT(ext);
		p += BRCM_TAG_LEN;
	}

	put_unaligned_be16(ETH_P_802_EX1, p);
	put_unaligned_be16(seq, p + 2);
}

static int txtest_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	struct fap_tx *tx = &fap->shared->tx;
	u32 done;

	guard(mutex)(&fap->lock);

	if (!fap->tx.sent) {
		seq_puts(s, "no frame sent yet\n");
		return 0;
	}

	seq_printf(s, "frame %u, port %u", fap->tx.seq, fap->tx.port);
	if (fap->tx.ext >= 0)
		seq_printf(s, " then %d", fap->tx.ext);
	seq_printf(s, ", channel %u\n", FAP_TX_TEST_CHANNEL);
	if (!(READ_ONCE(tx->controller_cfg) & DMA_MASTER_EN)) {
		seq_puts(s, "iuDMA disabled, not sent: is the conduit up?\n");
		return 0;
	}

	done = READ_ONCE(tx->done_ticks);
	seq_printf(s, "status 0x%04x, irq 0x%x, kick %u counts\n",
		   READ_ONCE(tx->status), READ_ONCE(tx->irq_status),
		   READ_ONCE(tx->kick_ticks));
	if (done)
		seq_printf(s, "engine took the frame in %u counts\n", done);
	else
		seq_puts(s, "engine did not take the frame, channel stopped\n");

	return 0;
}

static ssize_t txtest_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct bcm6362_fap *fap = file_inode(file)->i_private;
	struct fap_tx *tx = &fap->shared->tx;
	unsigned int port, ext;
	char cmd[16];
	dma_addr_t dma;
	int n, ret;
	u8 *frame;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, buf, count))
		return -EFAULT;
	cmd[count] = '\0';

	n = sscanf(cmd, "%u %u", &port, &ext);
	if (n < 1 || port >= FAP_SWITCH_CPU_PORT ||
	    (n == 2 && ext >= FAP_SWITCH_CPU_PORT))
		return -EINVAL;

	frame = dma_alloc_noncoherent(fap->dev, FAP_TX_TEST_LEN, &dma,
				      DMA_TO_DEVICE, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;

	scoped_guard(mutex, &fap->lock) {
		bcm6362_fap_tx_frame(frame, port, n == 2 ? ext : -1,
				     ++fap->ping_seq);
		dma_sync_single_for_device(fap->dev, dma, FAP_TX_TEST_LEN,
					   DMA_TO_DEVICE);

		WRITE_ONCE(tx->channel, FAP_TX_TEST_CHANNEL);
		WRITE_ONCE(tx->buf, dma);
		WRITE_ONCE(tx->len, FAP_TX_TEST_LEN);
		WRITE_ONCE(tx->append, DMA_APPEND_CRC);
		/* the setup is in memory before the command reaches the 4KE */
		wmb();
		ret = bcm6362_fap_ping(fap, FAP_MSG(FAP_CMD_TX_TEST,
						    fap->ping_seq), NULL, NULL);
		if (!ret) {
			fap->tx.sent = true;
			fap->tx.port = port;
			fap->tx.ext = n == 2 ? ext : -1;
			fap->tx.seq = fap->ping_seq;
		}
	}

	dma_free_noncoherent(fap->dev, FAP_TX_TEST_LEN, frame, dma,
			     DMA_TO_DEVICE);

	return ret ?: count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(txtest);

/*
 * Host access timings on one word of each target, in hundredths of a ns
 * per access, both with plain loads and stores and with the MMIO
 * accessors, which on MIPS issue a sync around every access. Loads add
 * the loaded value; stores write back the value the word already holds
 * and end with a sync and a read back, so that posted writes are counted.
 */
struct host_bench {
	u32 load, store, readl, writel;
};

static u32 host_bench_ns(u64 t0, u64 t1)
{
	return div_u64((t1 - t0) * 100, FAP_HOST_BENCH_ACCESSES);
}

static void host_bench(void __iomem *p, struct host_bench *b)
{
	u32 *q = (u32 __force *)p;
	unsigned long flags;
	u64 t0, t1, t2, t3, t4;
	u32 sum = 0, val;
	int i;

	local_irq_save(flags);
	val = READ_ONCE(*q);

	t0 = ktime_get_ns();
	for (i = 0; i < FAP_HOST_BENCH_ACCESSES; i++)
		sum += READ_ONCE(*q);
	t1 = ktime_get_ns();
	for (i = 0; i < FAP_HOST_BENCH_ACCESSES; i++)
		WRITE_ONCE(*q, val);
	/* drain the write buffer, so that the stores are all timed */
	mb();
	sum += READ_ONCE(*q);
	t2 = ktime_get_ns();
	for (i = 0; i < FAP_HOST_BENCH_ACCESSES; i++)
		sum += __raw_readl(p);
	t3 = ktime_get_ns();
	for (i = 0; i < FAP_HOST_BENCH_ACCESSES; i++)
		__raw_writel(val, p);
	/* as above */
	mb();
	sum += READ_ONCE(*q);
	t4 = ktime_get_ns();

	local_irq_restore(flags);

	barrier_data(&sum);
	b->load = host_bench_ns(t0, t1);
	b->store = host_bench_ns(t1, t2);
	b->readl = host_bench_ns(t2, t3);
	b->writel = host_bench_ns(t3, t4);
}

static void host_bench_show(struct seq_file *s, const char *name,
			    void __iomem *p)
{
	struct host_bench b;

	host_bench(p, &b);
	seq_printf(s, "%-14s %4u.%02u %4u.%02u %4u.%02u %4u.%02u\n", name,
		   b.load / 100, b.load % 100, b.store / 100, b.store % 100,
		   b.readl / 100, b.readl % 100, b.writel / 100, b.writel % 100);
}

static int hostbench_show(struct seq_file *s, void *unused)
{
	struct bcm6362_fap *fap = s->private;
	u32 *cached;

	cached = kzalloc(sizeof(*cached), GFP_KERNEL);
	if (!cached)
		return -ENOMEM;

	seq_puts(s, "ns per access     load   store   readl  writel\n");
	host_bench_show(s, "regs", fap->regs + FAP_REG_SCRATCH);
	if (fap->qsm)
		host_bench_show(s, "qsm", fap->qsm);
	if (fap->psm)
		host_bench_show(s, "psm", fap->psm);
	/* the last word of the block, past the shared area */
	host_bench_show(s, "ddr uncached",
			(void __force __iomem *)fap->mem + FAP_MEM_SIZE - 4);
	host_bench_show(s, "ddr cached", (void __force __iomem *)cached);

	kfree(cached);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hostbench);

static void bcm6362_fap_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static void __iomem *bcm6362_fap_ioremap_optional(struct platform_device *pdev,
						  const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;

	return devm_ioremap_resource(&pdev->dev, res);
}

static int bcm6362_fap_boot(struct bcm6362_fap *fap, const struct firmware *fw)
{
	lockdep_assert_held(&fap->lock);

	if (fw->size > FAP_SHARED_OFFSET) {
		dev_err(fap->dev, "%s: %zu bytes, at most %u fit\n",
			firmware, fw->size, FAP_SHARED_OFFSET);
		return -EFBIG;
	}
	memcpy(fap->mem, fw->data, fw->size);

	bcm6362_fap_map(fap, FAP_REG_ADDR1, 0, fap->mem_dma, FAP_MEM_SIZE);
	bcm6362_fap_map(fap, FAP_REG_ADDR2, FAP_4KE_BOOT_BASE, fap->mem_dma,
			FAP_MEM_SIZE);

	fap_wr(fap, FAP_GP_MBOX_IN | FAP_GP_MBOX_OUT, FAP_REG_GP_STATUS);
	fap_wr(fap, FAP_GP_MBOX_IN, FAP_REG_GP_MASK);
	fap_wr(fap, FAP_IRQ_MIPS_MBOX_IN, FAP_REG_IRQ_MIPS_MASK);

	fap->state = FAP_IDLE;

	/* the image must be in memory before the 4KE fetches it */
	wmb();
	fap_wr(fap, fap_rd(fap, FAP_REG_SOFT_RESETS) & ~FAP_SOFT_RESET_4KE,
	       FAP_REG_SOFT_RESETS);

	schedule_delayed_work(&fap->watchdog, msecs_to_jiffies(FAP_WATCHDOG_MS));

	dev_info(fap->dev, "4KE started from %s, memory at %pad\n", firmware,
		 &fap->mem_dma);

	return 0;
}

/* without a firmware the 4KE simply stays in reset */
static void bcm6362_fap_fw_ready(const struct firmware *fw, void *context)
{
	struct bcm6362_fap *fap = context;

	if (fw) {
		scoped_guard(mutex, &fap->lock)
			bcm6362_fap_boot(fap, fw);
		release_firmware(fw);
	} else {
		dev_info(fap->dev, "%s not found, 4KE left in reset\n",
			 firmware);
	}

	complete(&fap->fw_done);
}

/* on removal, before the halt: the request may still be pending */
static void bcm6362_fap_fw_wait(void *data)
{
	struct bcm6362_fap *fap = data;

	wait_for_completion(&fap->fw_done);
}

static void bcm6362_fap_halt(void *data)
{
	struct bcm6362_fap *fap = data;

	guard(mutex)(&fap->lock);
	bcm6362_fap_hold(fap);
	fap->state = FAP_OFFLINE;
}

static int bcm6362_fap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	struct bcm6362_fap *fap;
	struct clk *clk;
	int irq, ret;

	fap = devm_kzalloc(dev, sizeof(*fap), GFP_KERNEL);
	if (!fap)
		return -ENOMEM;
	fap->dev = dev;
	init_completion(&fap->pong);
	init_completion(&fap->fw_done);
	INIT_WORK(&fap->rx.work, bcm6362_fap_rx_work);

	ret = devm_mutex_init(dev, &fap->lock);
	if (ret)
		return ret;

	fap->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fap->regs))
		return PTR_ERR(fap->regs);

	fap->qsm = bcm6362_fap_ioremap_optional(pdev, "qsm");
	if (IS_ERR(fap->qsm))
		return PTR_ERR(fap->qsm);

	fap->psm = bcm6362_fap_ioremap_optional(pdev, "psm");
	if (IS_ERR(fap->psm))
		return PTR_ERR(fap->psm);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "clock\n");

	rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "reset\n");

	ret = reset_control_reset(rst);
	if (ret)
		return ret;

	bcm6362_fap_hold(fap);

	fap->mem = dmam_alloc_coherent(dev, FAP_MEM_SIZE, &fap->mem_dma,
				       GFP_KERNEL);
	if (!fap->mem)
		return -ENOMEM;

	if (!IS_ALIGNED(fap->mem_dma, FAP_MEM_SIZE))
		return dev_err_probe(dev, -EINVAL, "unaligned memory %pad\n",
				     &fap->mem_dma);

	fap->shared = fap->mem + FAP_SHARED_OFFSET;

	/* the interrupt handler kicks the watchdog on exceptions */
	ret = devm_delayed_work_autocancel(dev, &fap->watchdog,
					   bcm6362_fap_watchdog);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, irq, bcm6362_fap_isr, 0, dev_name(dev),
			       fap);
	if (ret)
		return ret;

	fap->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("state", 0400, fap->debugfs, fap, &state_fops);
	debugfs_create_file("regs", 0400, fap->debugfs, fap, &regs_fops);
	debugfs_create_file("log", 0400, fap->debugfs, fap, &log_fops);
	debugfs_create_file("mbox", 0600, fap->debugfs, fap, &mbox_fops);
	debugfs_create_file("clock", 0400, fap->debugfs, fap, &clock_fops);
	debugfs_create_file("stats", 0400, fap->debugfs, fap, &stats_fops);
	debugfs_create_file("iudma", 0400, fap->debugfs, fap, &iudma_fops);
	debugfs_create_file("ddr", 0400, fap->debugfs, fap, &ddr_fops);
	debugfs_create_file("peek", 0600, fap->debugfs, fap, &peek_fops);
	debugfs_create_file("rxtest", 0600, fap->debugfs, fap, &rxtest_fops);
	if (fap->psm)
		debugfs_create_file("txtest", 0600, fap->debugfs, fap,
				    &txtest_fops);
	debugfs_create_file("hostbench", 0400, fap->debugfs, fap,
			    &hostbench_fops);
	if (fap->qsm)
		debugfs_create_file("qsm", 0400, fap->debugfs, fap, &qsm_fops);
	if (fap->psm)
		debugfs_create_file("psm", 0400, fap->debugfs, fap, &psm_fops);
	ret = devm_add_action_or_reset(dev, bcm6362_fap_debugfs_remove,
				       fap->debugfs);
	if (ret)
		return ret;

	/* released after the halt below, once the 4KE writes nothing */
	ret = devm_add_action_or_reset(dev, bcm6362_fap_rx_release, fap);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bcm6362_fap_halt, fap);
	if (ret)
		return ret;

	/* not _or_reset: waiting here before the request would never end */
	ret = devm_add_action(dev, bcm6362_fap_fw_wait, fap);
	if (ret)
		return ret;

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT, firmware,
				      dev, GFP_KERNEL, fap,
				      bcm6362_fap_fw_ready);
	if (ret)
		complete(&fap->fw_done);

	return ret;
}

static const struct of_device_id bcm6362_fap_of_match[] = {
	{ .compatible = "brcm,bcm6362-fap", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bcm6362_fap_of_match);

static struct platform_driver bcm6362_fap_driver = {
	.driver = {
		.name = "bcm6362-fap",
		.of_match_table = bcm6362_fap_of_match,
	},
	.probe = bcm6362_fap_probe,
};
module_platform_driver(bcm6362_fap_driver);

MODULE_DESCRIPTION("BCM6362 FAP coprocessor loader");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("bcm6362-fap.bin");
