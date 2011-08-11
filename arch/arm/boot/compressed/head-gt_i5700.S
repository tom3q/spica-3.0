/*
 * linux/arch/arm/boot/compressed/head-gt_i5700.S
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * GT-i5700 specific tweaks.  This is merged into head.S by the linker.
 *
 */

#include <linux/linkage.h>
#include <asm/mach-types.h>

		.section        ".start", "ax"

__GT_I5700_start:
		@ Set the UART switch to PDA to allow serial debugging
		mov	r0, #0x7f000000
		orr	r0, r0, #0x8800
		@ Set GPK(4) to output
		ldr	r1, [r0]
		mvn	r2, #0x000f0000
		and	r1, r1, r2
		orr	r1, r1, #0x00010000
		str	r1, [r0]
		@ Set GPK(4) to 1
		orr	r0, r0, #0x08
		ldr	r1, [r0]
		orr	r1, r1, #0x0010
		str	r1, [r0]

		@ Our bootloader identifies us as an SMDK6410 board,
		@ which we aren't... This is a hack to put the correct
		@ mach type into r7 if it contains the mach type of SMDK6410
		adr	r0, __smdk6410
		ldr	r0, [r0]
		cmp	r7, r0
		bne	1f
		adr	r0, __gt_i5700
		ldr	r7, [r0]
		b	1f

		@ Mach types we operate on
__smdk6410:	.word	MACH_TYPE_SMDK6410
__gt_i5700:	.word	MACH_TYPE_GT_I5700

		@ We continue here after some utility data
1:
