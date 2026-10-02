/*
 * jand/jnand polarity test.
 *
 * The host zeroes SHM, writes the operands to JT_A and JT_B and starts the
 * PSM. JT_RES_JAND / JT_RES_JNAND are set to 1 only on the taken path of the
 * respective jump, JT_DONE is set to JT_DONE_MAGIC in all cases.
 *
 * Build: b43-asm jandtest.asm jandtest5.fw --cpp-args -DARCH=5 --    (core rev < 15)
 *        b43-asm jandtest.asm jandtest15.fw --cpp-args -DARCH=15 --  (core rev >= 15)
 */

#define JT_A		0x100
#define JT_B		0x101
#define JT_RES_JAND	0x102
#define JT_RES_JNAND	0x103
#define JT_DONE		0x104
#define JT_DONE_MAGIC	0xA5A5

#define COND_TRUE	0x7F

%arch	ARCH
%start	entry

entry:
	or	[JT_A], 0x000, r0
	or	[JT_B], 0x000, r1
	jand	r0, r1, jand_taken
	jext	COND_TRUE, test_jnand
jand_taken:
	mov	1, [JT_RES_JAND]
test_jnand:
	jnand	r0, r1, jnand_taken
	jext	COND_TRUE, done
jnand_taken:
	mov	1, [JT_RES_JNAND]
done:
	mov	JT_DONE_MAGIC, [JT_DONE]
halt:
	jext	COND_TRUE, halt
