/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "fap4ke.h"

/* exception frame: o32 argument area, then one slot per GPR, then HI/LO */
#define FRAME_GPR(n)	(16 + 4 * (n))
#define FRAME_HI	FRAME_GPR(32)
#define FRAME_LO	FRAME_GPR(33)
#define FRAME_SIZE	FRAME_GPR(34)

	.set	noreorder

/* 0xbfc00000: still uncached, continue from the kseg1 alias of _start */
	.section .vec.reset, "ax"
	.globl	_reset
_reset:
	la	$t0, _start
	li	$t1, KSEG1_BASE
	or	$t0, $t0, $t1
	jr	$t0
	 nop

/* 0x80000180 (BEV=0) through the low address translator */
	.section .vec.general, "ax"
	la	$k0, _exception
	jr	$k0
	 nop

/* 0xbfc00380 (BEV=1) */
	.section .vec.boot, "ax"
	la	$k0, _exception
	jr	$k0
	 nop

/* 0xbfc00480: EJTAG debug exception */
	.section .vec.ejtag, "ax"
	deret

	.text
_start:
	mtc0	$zero, CP0_CAUSE
	mtc0	$zero, CP0_STATUS

	/* invalidate the I-cache, geometry from Config1 */
	mfc0	$t0, CP0_CONFIG, 1
	srl	$t1, $t0, 19
	andi	$t1, $t1, 7		/* IL */
	beqz	$t1, 2f
	 li	$t2, 2
	sllv	$t2, $t2, $t1		/* line size */
	srl	$t3, $t0, 22
	andi	$t3, $t3, 7		/* IS */
	li	$t4, 64
	sllv	$t4, $t4, $t3		/* sets per way */
	srl	$t3, $t0, 16
	andi	$t3, $t3, 7		/* IA */
	addiu	$t3, $t3, 1
	mul	$t4, $t4, $t3
	mul	$t4, $t4, $t2		/* cache size */
	mtc0	$zero, CP0_TAGLO
	mtc0	$zero, CP0_TAGHI
	li	$t3, KSEG0_BASE
	addu	$t4, $t4, $t3
1:	cache	INDEX_STORE_TAG_I, 0($t3)
	addu	$t3, $t3, $t2
	bne	$t3, $t4, 1b
	 nop
2:
	mfc0	$t0, CP0_CONFIG
	li	$t1, ~CONFIG_K0_MASK
	and	$t0, $t0, $t1
	ori	$t0, $t0, CONFIG_K0_WT
	mtc0	$t0, CP0_CONFIG

	/* map the DSPRAM, it holds the stack */
	mfc0	$t0, CP0_ERRCTL
	li	$t1, ERRCTL_SPR
	or	$t1, $t0, $t1
	mtc0	$t1, CP0_ERRCTL
	ehb
	li	$t1, DSPRAM_BASE | SPRAM_TAG_ENABLE
	mtc0	$t1, CP0_TAGLO
	ehb
	li	$t1, KSEG0_BASE
	cache	INDEX_STORE_TAG_D, 0($t1)
	ehb
	mtc0	$t0, CP0_ERRCTL
	ehb

	la	$sp, _stack_top
	la	$t0, 3f
	jr.hb	$t0			/* to kseg0, with the new K0 */
	 nop

3:	la	$t0, _bss_start
	la	$t1, _bss_end
4:	beq	$t0, $t1, 5f
	 nop
	sw	$zero, 0($t0)
	b	4b
	 addiu	$t0, $t0, 4

5:	jal	fap_main
	 nop
6:	b	6b
	 nop

_exception:
	mfc0	$k0, CP0_CAUSE
	andi	$k0, $k0, CAUSE_EXCCODE
	bnez	$k0, _fatal
	 nop

	addiu	$sp, $sp, -FRAME_SIZE
	.set	noat
	.irp	reg, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 24, 25, 31
	sw	$\reg, FRAME_GPR(\reg)($sp)
	.endr
	.set	at
	mfhi	$t0
	sw	$t0, FRAME_HI($sp)
	mflo	$t0
	sw	$t0, FRAME_LO($sp)

	jal	fap_irq
	 mfc0	$a0, CP0_CAUSE

	lw	$t0, FRAME_HI($sp)
	mthi	$t0
	lw	$t0, FRAME_LO($sp)
	mtlo	$t0
	.set	noat
	.irp	reg, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 24, 25, 31
	lw	$\reg, FRAME_GPR(\reg)($sp)
	.endr
	.set	at
	addiu	$sp, $sp, FRAME_SIZE
	eret

_fatal:
	mfc0	$a0, CP0_CAUSE
	mfc0	$a1, CP0_EPC
	mfc0	$a2, CP0_BADVADDR
	la	$sp, _stack_top
	jal	fap_exception
	 nop
7:	b	7b
	 nop
