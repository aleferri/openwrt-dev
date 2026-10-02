// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * jand/jnand polarity test for Broadcom 802.11 (d11) cores.
 *
 * For each operand pair: reset the core, upload b43/jandtest{5,15}.fw,
 * store the operands in SHM, run the PSM and report what the ucode wrote.
 * Core reset, MAC control and ucode upload follow b43 on the b43-ac branch,
 * AC cores included. The probe always fails, so the core is left unbound
 * once the results are printed.
 */

#include <linux/bcma/bcma.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/module.h>

#define JT_MMIO_MACCTL			0x120
#define JT_MMIO_SHM_CONTROL		0x160
#define JT_MMIO_SHM_DATA		0x164
#define JT_MMIO_SHM_DATA_UNALIGNED	0x166

#define JT_MACCTL_PSM_RUN		0x00000002
#define JT_MACCTL_PSM_JMP0		0x00000004
#define JT_MACCTL_SHM_ENABLED		0x00000100
#define JT_MACCTL_IHR_ENABLED		0x00000400
#define JT_MACCTL_INFRA			0x00020000
#define JT_MACCTL_AWAKE			0x04000000
#define JT_MACCTL_GMODE			0x80000000

#define JT_SHM_UCODE			0x0000
#define JT_SHM_SHARED			0x0001
#define JT_SHM_SCRATCH			0x0002
#define JT_SHM_AUTOINC_W		0x0100

#define JT_IOCTL_PHY_CLKEN		0x00000004
#define JT_IOCTL_PHY_RESET		0x00000008
#define JT_IOCTL_PHY_BW_20MHZ		0x00000040
#define JT_IOCTL_DAC			0x00000300
#define JT_IOCTL_GMODE			0x00002000
#define JT_IOST_2G_PHY			0x00000001

#define JT_CLKCTLST_PLL_REQ		0x00000300
#define JT_CLKCTLST_PLL_ST		0x03000000

/* SHM word addresses, as in jandtest.asm */
#define JT_A				0x100
#define JT_B				0x101
#define JT_RES_JAND			0x102
#define JT_RES_JNAND			0x103
#define JT_DONE				0x104
#define JT_DONE_MAGIC			0xA5A5

struct jt_fw_header {
	u8 type;
	u8 ver;
	u8 __padding[2];
	__be32 size;
} __packed;

struct jt_core {
	struct bcma_device *core;
	bool ac;
	bool gmode;
};

static void jt_shm_control(struct bcma_device *core, u16 routing, u16 offset)
{
	bcma_write32(core, JT_MMIO_SHM_CONTROL, ((u32)routing << 16) | offset);
}

static u16 jt_shared_read(struct bcma_device *core, u16 word)
{
	jt_shm_control(core, JT_SHM_SHARED, word >> 1);
	return bcma_read16(core, word & 1 ? JT_MMIO_SHM_DATA_UNALIGNED
					  : JT_MMIO_SHM_DATA);
}

static void jt_shared_write(struct bcma_device *core, u16 word, u16 value)
{
	jt_shm_control(core, JT_SHM_SHARED, word >> 1);
	bcma_write16(core, word & 1 ? JT_MMIO_SHM_DATA_UNALIGNED
				    : JT_MMIO_SHM_DATA, value);
}

static void jt_ioctl_maskset(struct bcma_device *core, u32 mask, u32 set)
{
	bcma_awrite32(core, BCMA_IOCTL,
		      (bcma_aread32(core, BCMA_IOCTL) & mask) | set);
}

/* b43_bcma_wireless_core_reset() and b43_wireless_core_reset() */
static void jt_core_reset(const struct jt_core *jt)
{
	struct bcma_device *core = jt->core;
	u32 macctl;

	bcma_core_enable(core, JT_IOCTL_PHY_CLKEN |
			       (jt->gmode ? JT_IOCTL_GMODE : 0));

	if (jt->ac) {
		jt_ioctl_maskset(core, ~JT_IOCTL_DAC, 0x100);
		jt_ioctl_maskset(core, ~JT_IOCTL_PHY_CLKEN, 0);
		jt_ioctl_maskset(core, ~0, JT_IOCTL_PHY_CLKEN);
	}

	bcma_core_set_clockmode(core, BCMA_CLKMODE_FAST);

	jt_ioctl_maskset(core, ~0, JT_IOCTL_PHY_RESET | JT_IOCTL_PHY_BW_20MHZ |
				   (jt->ac ? BCMA_IOCTL_FGC : 0));
	udelay(2);
	jt_ioctl_maskset(core, ~(JT_IOCTL_PHY_RESET | JT_IOCTL_PHY_CLKEN |
				 BCMA_IOCTL_FGC),
			 jt->ac ? 0 : BCMA_IOCTL_FGC);
	udelay(1);
	jt_ioctl_maskset(core, ~BCMA_IOCTL_FGC, JT_IOCTL_PHY_CLKEN);
	udelay(1);

	if (!jt->ac || (bcma_read32(core, BCMA_CLKCTLST) & JT_CLKCTLST_PLL_ST) !=
		       JT_CLKCTLST_PLL_ST)
		bcma_core_pll_ctl(core, JT_CLKCTLST_PLL_REQ, JT_CLKCTLST_PLL_ST,
				  true);

	macctl = bcma_read32(core, JT_MMIO_MACCTL) & ~JT_MACCTL_GMODE;
	if (jt->gmode)
		macctl |= JT_MACCTL_GMODE;
	macctl |= JT_MACCTL_IHR_ENABLED;
	if (jt->ac)
		macctl |= JT_MACCTL_AWAKE;
	bcma_write32(core, JT_MMIO_MACCTL, macctl);
}

static void jt_run(const struct jt_core *jt, const __be32 *ucode, size_t len,
		   u16 a, u16 b)
{
	struct bcma_device *core = jt->core;
	unsigned int i;
	u32 macctl;
	u16 done;

	jt_core_reset(jt);

	/* b43_chip_init() */
	macctl = JT_MACCTL_IHR_ENABLED | JT_MACCTL_INFRA;
	macctl |= jt->ac ? JT_MACCTL_AWAKE : JT_MACCTL_SHM_ENABLED;
	if (jt->gmode)
		macctl |= JT_MACCTL_GMODE;
	bcma_write32(core, JT_MMIO_MACCTL, macctl);

	/* b43_upload_microcode() */
	bcma_write32(core, JT_MMIO_MACCTL, macctl | JT_MACCTL_PSM_JMP0);
	for (i = 0; i < 64; i++) {
		jt_shm_control(core, JT_SHM_SCRATCH, i);
		bcma_write16(core, JT_MMIO_SHM_DATA, 0);
	}
	for (i = 0; i < 2048; i++)
		jt_shared_write(core, i, 0);

	jt_shm_control(core, JT_SHM_UCODE | JT_SHM_AUTOINC_W, 0);
	for (i = 0; i < len; i++) {
		bcma_write32(core, JT_MMIO_SHM_DATA, be32_to_cpu(ucode[i]));
		udelay(10);
	}

	jt_shared_write(core, JT_A, a);
	jt_shared_write(core, JT_B, b);

	bcma_write32(core, JT_MMIO_MACCTL, macctl | JT_MACCTL_PSM_RUN);
	for (i = 0; i < 1000; i++) {
		done = jt_shared_read(core, JT_DONE);
		if (done == JT_DONE_MAGIC)
			break;
		udelay(10);
	}
	bcma_write32(core, JT_MMIO_MACCTL, macctl | JT_MACCTL_PSM_JMP0);

	dev_info(&core->dev, "jandtest: A=%u B=%u jand=%u jnand=%u done=0x%04X%s\n",
		 a, b, jt_shared_read(core, JT_RES_JAND),
		 jt_shared_read(core, JT_RES_JNAND), done,
		 done == JT_DONE_MAGIC ? "" : " (TIMEOUT)");
}

static int jt_probe(struct bcma_device *core)
{
	static const u16 vectors[][2] = {
		{ 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 },
	};
	const char *name = core->id.rev >= 15 ? "b43/jandtest15.fw"
					      : "b43/jandtest5.fw";
	struct jt_core jt = {
		.core = core,
		.ac = core->id.rev == 40 || core->id.rev == 42,
		.gmode = bcma_aread32(core, BCMA_IOST) & JT_IOST_2G_PHY,
	};
	const struct jt_fw_header *hdr;
	const struct firmware *fw;
	unsigned int i;
	size_t len;
	int err;

	err = request_firmware(&fw, name, &core->dev);
	if (err)
		return err;

	hdr = (const struct jt_fw_header *)fw->data;
	if (fw->size < sizeof(*hdr) || hdr->type != 'u' || hdr->ver != 1 ||
	    be32_to_cpu(hdr->size) != fw->size - sizeof(*hdr) ||
	    (fw->size - sizeof(*hdr)) % sizeof(__be32)) {
		dev_err(&core->dev, "jandtest: %s: bad format\n", name);
		err = -EINVAL;
		goto out;
	}
	len = (fw->size - sizeof(*hdr)) / sizeof(__be32);

	dev_info(&core->dev, "jandtest: core rev %u%s%s, %s\n", core->id.rev,
		 jt.ac ? ", AC" : "", jt.gmode ? ", 2.4 GHz" : "", name);

	for (i = 0; i < ARRAY_SIZE(vectors); i++)
		jt_run(&jt, (const __be32 *)(fw->data + sizeof(*hdr)), len,
		       vectors[i][0], vectors[i][1]);

	bcma_core_disable(core, 0);
	err = -ENODEV;
out:
	release_firmware(fw);
	return err;
}

static const struct bcma_device_id jt_bcma_tbl[] = {
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_80211, BCMA_ANY_REV, BCMA_ANY_CLASS),
	{},
};

static struct bcma_driver jt_bcma_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= jt_bcma_tbl,
	.probe		= jt_probe,
};

static int __init jt_init(void)
{
	return bcma_driver_register(&jt_bcma_driver);
}

static void __exit jt_exit(void)
{
	bcma_driver_unregister(&jt_bcma_driver);
}

module_init(jt_init);
module_exit(jt_exit);

MODULE_DESCRIPTION("jand/jnand polarity test for Broadcom 802.11 cores");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("b43/jandtest5.fw");
MODULE_FIRMWARE("b43/jandtest15.fw");
