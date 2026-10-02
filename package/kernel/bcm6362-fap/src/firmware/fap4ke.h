/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * BCM6362 FAP: 4KE side definitions, private to the firmware
 */

#ifndef __BCM6362_FAP4KE_H
#define __BCM6362_FAP4KE_H

#define KSEG0_BASE		0x80000000
#define KSEG1_BASE		0xa0000000
#define KSEG0(x)		(KSEG0_BASE | (x))
#define KSEG1(x)		(KSEG1_BASE | (x))

/* FAP blocks as seen by the 4KE */
#define FAP_4KE_REGS		0xe0001000
#define FAP_4KE_QSM		0xe0004000
#define FAP_4KE_PSM		0xe0010000
/* the same PSM as the iuDMA (and the host) address it */
#define FAP_PSM_HOST_BASE	0x11010000

/* ENETSW iuDMA, at its host physical address */
#define ENETSW_DMA		0x1000d800
#define IUDMA_CHANNELS		0x200
#define IUDMA_SRAM		0x400
#define IUDMA_CH_STRIDE		0x10
#define IUDMA_CH_FLOWCTL	0x0	/* DmaChannelCfg: enable, flowc */
#define  IUDMA_EN		0x1
#define  IUDMA_PKTHALT		0x2
#define  IUDMA_BURSTHALT	0x4
#define IUDMA_CH_INTSTAT	0x4	/* write one to clear */
#define IUDMA_CH_MAXBURST	0xc
#define  IUDMA_MAXBURST		8	/* 64-bit words, as the SDK and enetsw */
#define IUDMA_SRAM_RSTART	0x0	/* DmaStateRam: ring start */
#define IUDMA_SRAM_WORDS	4	/* ring start, then the engine state */

/* internal switch, CPU port queue to iuDMA channel map */
#define ENETSW_SWITCH		0x10e00000
#define SWITCH_IUDMA_QCTRL	0x000000ac

/* data scratchpad, at the physical address the SDK firmware uses */
#define DSPRAM_BASE		0x4000
#define DSPRAM_SIZE		0x2000

/* the stack sits at the top of the DSPRAM */
#define FAP_STACK_SIZE		0x800

/* CP0 registers */
#define CP0_BADVADDR		$8
#define CP0_COUNT		$9
#define CP0_COMPARE		$11
#define CP0_STATUS		$12
#define CP0_CAUSE		$13
#define CP0_EPC			$14
#define CP0_PRID		$15
#define CP0_EBASE		$15		/* select 1 */
#define CP0_CONFIG		$16
#define CP0_ERRCTL		$26
#define CP0_TAGLO		$28
#define CP0_TAGHI		$29

#define ST0_IE			(1 << 0)
#define ST0_IM_FAP		(1 << 10)	/* IP2: irq_4ke_status */
#define ST0_IM_TIMER		(1 << 15)	/* IP7: CP0 Count/Compare */

#define CAUSE_TI		(1 << 30)
#define CAUSE_EXCCODE		0x7c

#define CONFIG_K0_MASK		0x7
#define CONFIG_K0_WT		0x0

#define ERRCTL_SPR		(1 << 28)
#define SPRAM_TAG_ENABLE	0x80
#define SPRAM_TAG_STRIDE	8

#define INDEX_LOAD_TAG_D	0x05
#define INDEX_STORE_TAG_I	0x08
#define INDEX_STORE_TAG_D	0x09

#endif /* __BCM6362_FAP4KE_H */
