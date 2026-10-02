// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * BCM6362 FAP coprocessor loader
 *
 * Boots a firmware image on the MIPS 4KE core of the FAP block: the image is
 * placed in a host memory block that the FAP address translators map both at
 * the 4KE reset vector and at 4KE physical address 0 (exception vectors).
 *
 * The firmware boots with the driver and stays idle. A watchdog holds the
 * 4KE in reset if its heartbeat stalls or it reports an exception.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
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
	struct completion pong;
	u32 pong_msg;
	ktime_t pong_time;

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

static void bcm6362_fap_map(struct bcm6362_fap *fap, unsigned int reg,
			    u32 fap_base)
{
	fap_wr(fap, ~((FAP_MEM_SIZE - 1) | FAP_ADDR_IGNORE),
	       reg + FAP_ADDR_MASK);
	fap_wr(fap, fap_base, reg + FAP_ADDR_BASE_IN);
	fap_wr(fap, fap->mem_dma, reg + FAP_ADDR_BASE_OUT);
}

static int bcm6362_fap_load(struct bcm6362_fap *fap)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, firmware, fap->dev);
	if (ret)
		return ret;

	if (fw->size > FAP_SHARED_OFFSET) {
		dev_err(fap->dev, "%s: %zu bytes, at most %u fit\n",
			firmware, fw->size, FAP_SHARED_OFFSET);
		ret = -EFBIG;
	} else {
		memcpy(fap->mem, fw->data, fw->size);
	}

	release_firmware(fw);

	return ret;
}

static irqreturn_t bcm6362_fap_isr(int irq, void *data)
{
	struct bcm6362_fap *fap = data;
	u32 msg;

	if (!(fap_rd(fap, FAP_REG_IRQ_MIPS_STATUS) & FAP_IRQ_MIPS_MBOX_IN))
		return IRQ_NONE;

	fap_wr(fap, FAP_GP_MBOX_IN, FAP_REG_GP_STATUS);
	msg = fap_rd(fap, FAP_REG_MBOX_IN);

	if (fap_rd(fap, FAP_REG_IRQ_MIPS_STATUS) & FAP_IRQ_MIPS_MBOX_IN) {
		fap_wr(fap, 0, FAP_REG_IRQ_MIPS_MASK);
		dev_err(fap->dev, "mailbox irq stuck, masked\n");
	}

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
	seq_printf(s, "mbox            irq %u polled %u lost %u\n",
		   READ_ONCE(sh->mbox_irq), READ_ONCE(sh->mbox_polled),
		   READ_ONCE(sh->mbox_lost));
	seq_printf(s, "irq lines       0x%04x masked 0x%04x\n",
		   READ_ONCE(sh->irq_lines), READ_ONCE(sh->irq_masked));

	load = bench_centicycles(fap, READ_ONCE(sh->bench_alu));
	seq_puts(s, "\ncycles per access    load   store\n");
	seq_printf(s, "%-16s %4u.%02u\n", "alu", load / 100, load % 100);
	for (i = 0; i < FAP_BENCH_NR; i++) {
		load = bench_centicycles(fap, READ_ONCE(sh->bench[i].load));
		store = bench_centicycles(fap, READ_ONCE(sh->bench[i].store));
		seq_printf(s, "%-16s %4u.%02u %4u.%02u\n", names[i],
			   load / 100, load % 100, store / 100, store % 100);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stats);

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

	ret = bcm6362_fap_load(fap);
	if (ret)
		return dev_err_probe(dev, ret, "loading %s\n", firmware);

	bcm6362_fap_map(fap, FAP_REG_ADDR1, 0);
	bcm6362_fap_map(fap, FAP_REG_ADDR2, FAP_4KE_BOOT_BASE);

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
	if (fap->qsm)
		debugfs_create_file("qsm", 0400, fap->debugfs, fap, &qsm_fops);
	if (fap->psm)
		debugfs_create_file("psm", 0400, fap->debugfs, fap, &psm_fops);
	ret = devm_add_action_or_reset(dev, bcm6362_fap_debugfs_remove,
				       fap->debugfs);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bcm6362_fap_halt, fap);
	if (ret)
		return ret;

	fap_wr(fap, FAP_GP_MBOX_IN | FAP_GP_MBOX_OUT, FAP_REG_GP_STATUS);
	fap_wr(fap, FAP_GP_MBOX_IN, FAP_REG_GP_MASK);
	fap_wr(fap, FAP_IRQ_MIPS_MBOX_IN, FAP_REG_IRQ_MIPS_MASK);

	fap->state = FAP_IDLE;

	/* the image must be in memory before the 4KE fetches it */
	wmb();
	fap_wr(fap, fap_rd(fap, FAP_REG_SOFT_RESETS) & ~FAP_SOFT_RESET_4KE,
	       FAP_REG_SOFT_RESETS);

	schedule_delayed_work(&fap->watchdog, msecs_to_jiffies(FAP_WATCHDOG_MS));

	dev_info(dev, "4KE started from %s, memory at %pad\n", firmware,
		 &fap->mem_dma);

	return 0;
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
