/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Access timing loops. Each one runs FAP_BENCH_UNROLL accesses per
 * iteration on the same word and returns the CP0 Count ticks spent.
 *
 * unsigned int bench_alu(volatile unsigned int *p, unsigned int iters);
 * unsigned int bench_load(volatile unsigned int *p, unsigned int iters);
 * unsigned int bench_store(volatile unsigned int *p, unsigned int iters);
 * unsigned int bench_load_pair(volatile unsigned int *p,
 *				volatile unsigned int *q, unsigned int iters);
 */

#include "fap_shared.h"
#include "fap4ke.h"

	.set	noreorder
	.text

	.globl	bench_alu
bench_alu:
	mfc0	$t9, CP0_COUNT
1:	.rept	FAP_BENCH_UNROLL
	addu	$v1, $v1, $t0
	addu	$v1, $v1, $t0
	.endr
	addiu	$a1, $a1, -1
	bnez	$a1, 1b
	 nop
	mfc0	$v0, CP0_COUNT
	jr	$ra
	 subu	$v0, $v0, $t9

	.globl	bench_load
bench_load:
	mfc0	$t9, CP0_COUNT
1:	.rept	FAP_BENCH_UNROLL
	lw	$t0, 0($a0)
	addu	$v1, $v1, $t0
	.endr
	addiu	$a1, $a1, -1
	bnez	$a1, 1b
	 nop
	mfc0	$v0, CP0_COUNT
	jr	$ra
	 subu	$v0, $v0, $t9

/* as bench_load, alternating between two words */
	.globl	bench_load_pair
bench_load_pair:
	mfc0	$t9, CP0_COUNT
1:	.rept	FAP_BENCH_UNROLL / 2
	lw	$t0, 0($a0)
	addu	$v1, $v1, $t0
	lw	$t0, 0($a1)
	addu	$v1, $v1, $t0
	.endr
	addiu	$a2, $a2, -1
	bnez	$a2, 1b
	 nop
	mfc0	$v0, CP0_COUNT
	jr	$ra
	 subu	$v0, $v0, $t9

/* the word keeps its value: it is read once and written back */
	.globl	bench_store
bench_store:
	lw	$t0, 0($a0)
	mfc0	$t9, CP0_COUNT
1:	.rept	FAP_BENCH_UNROLL
	sw	$t0, 0($a0)
	.endr
	addiu	$a1, $a1, -1
	bnez	$a1, 1b
	 nop
	sync
	mfc0	$v0, CP0_COUNT
	jr	$ra
	 subu	$v0, $v0, $t9
