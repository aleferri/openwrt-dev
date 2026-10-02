/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "fap_shared.h"
#include "fap4ke.h"

/*
 * The memory block is seen by the 4KE at FAP_4KE_BOOT_BASE and the image
 * runs from its kseg0 alias. The core has no D-cache, so data in kseg0 is
 * not cached either.
 */

OUTPUT_ARCH(mips)
ENTRY(_reset)

_stack_top = KSEG0(DSPRAM_BASE + DSPRAM_SIZE);
_stack_bottom = _stack_top - FAP_STACK_SIZE;

SECTIONS
{
	/* variables kept in the DSPRAM, not initialized by the startup */
	.dspram KSEG0(DSPRAM_BASE) (NOLOAD) : {
		*(.dspram)
		. = ALIGN(4);
		_dspram_end = .;
	}

	.vectors KSEG0(FAP_4KE_BOOT_BASE) : {
		KEEP(*(.vec.reset))
		. = 0x180;
		KEEP(*(.vec.general))
		. = 0x380;
		KEEP(*(.vec.boot))
		. = 0x480;
		KEEP(*(.vec.ejtag))
	}

	.text : {
		*(.text .text.*)
	}

	.data : {
		*(.rodata .rodata.*)
		*(.data .data.*)
	}

	.bss (NOLOAD) : {
		. = ALIGN(4);
		_bss_start = .;
		*(.bss .bss.* COMMON)
		. = ALIGN(4);
		_bss_end = .;
	}

	ASSERT(_bss_end <= KSEG0(FAP_4KE_BOOT_BASE + FAP_SHARED_OFFSET),
	       "firmware overlaps the shared area")
	ASSERT(_dspram_end <= _stack_bottom, "DSPRAM variables overlap the stack")

	/DISCARD/ : {
		*(.reginfo .MIPS.abiflags .MIPS.options .pdr .comment .note.*)
		*(.gnu.attributes .mdebug.*)
	}
}
