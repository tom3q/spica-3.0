/*
 * Register definition file for Samsung Camera Interface (FIMC) driver
 *
 * Copyright (c) 2010 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef REGS_FIMC_H_
#define REGS_FIMC_H_

/*
 * Common registers
 */

/* Input source format */
#define S3C_CISRCFMT			0x00
#define S3C_CISRCFMT_ITU601_8BIT	(1 << 31)
#define S3C_CISRCFMT_ORDER422_YCBYCR	(0 << 14)
#define S3C_CISRCFMT_ORDER422_YCRYCB	(1 << 14)
#define S3C_CISRCFMT_ORDER422_CBYCRY	(2 << 14)
#define S3C_CISRCFMT_ORDER422_CRYCBY	(3 << 14)
#define S3C_CISRCFMT_HSIZE(x)		((x) << 16)
#define S3C_CISRCFMT_VSIZE(x)		((x) << 0)

/* Window offset */
#define S3C_CIWDOFST			0x04
#define S3C_CIWDOFST_OFF_EN		(1 << 31)
#define S3C_CIWDOFST_CLROVFIY		(1 << 30)
#define S3C_CIWDOFST_HOROFF_MASK	(0x7ff << 16)
#define S3C_CIWDOFST_CLROVFICB		(1 << 15)
#define S3C_CIWDOFST_CLROVFICR		(1 << 14)
#define S3C_CIWDOFST_HOROFF(x)		((x) << 16)
#define S3C_CIWDOFST_VEROFF(x)		((x) << 0)
#define S3C_CIWDOFST_VEROFF_MASK	(0xfff << 0)

/* Global control */
#define S3C_CIGCTRL			0x08
#define S3C_CIGCTRL_SWRST		(1 << 31)
#define S3C_CIGCTRL_CAMRST_A		(1 << 30)
#define S3C_CIGCTRL_TESTPAT_NORMAL	(0 << 27)
#define S3C_CIGCTRL_TESTPAT_COLOR_BAR	(1 << 27)
#define S3C_CIGCTRL_TESTPAT_HOR_INC	(2 << 27)
#define S3C_CIGCTRL_TESTPAT_VER_INC	(3 << 27)
#define S3C_CIGCTRL_TESTPAT_MASK	(3 << 27)
#define S3C_CIGCTRL_TESTPAT_SHIFT	(27)
#define S3C_CIGCTRL_INVPOLPCLK		(1 << 26)
#define S3C_CIGCTRL_INVPOLVSYNC		(1 << 25)
#define S3C_CIGCTRL_INVPOLHREF		(1 << 24)
#define S3C_CIGCTRL_IRQ_OVFEN		(1 << 22)
#define S3C_CIGCTRL_HREF_MASK		(1 << 21)
#define S3C_CIGCTRL_IRQ_LEVEL		(1 << 20)
#define S3C_CIGCTRL_IRQ_CLR		(1 << 19)
#define S3C_CIGCTRL_FIELDMODE		(1 << 2)
#define S3C_CIGCTRL_INTERLACE		(1 << 0)

/* Window offset 2 */
#define S3C_CIWDOFST2			0x14
#define S3C_CIWDOFST2_HOROFF_MASK	(0xfff << 16)
#define S3C_CIWDOFST2_VEROFF_MASK	(0xfff << 0)
#define S3C_CIWDOFST2_HOROFF(x)		((x) << 16)
#define S3C_CIWDOFST2_VEROFF(x)		((x) << 0)

/*
 * Registers separate for each image path
 */

/* Output DMA Y/Cb/Cr plane start addresses */
#define S3C_CIOYSA(n)			(0x18 + (n) * 4)
#define S3C_CIOCBSA(n)			(0x28 + (n) * 4)
#define S3C_CIOCRSA(n)			(0x38 + (n) * 4)

/* Target image format */
#define S3C_CITRGFMT			0x48
#define S3C_CITRGFMT_YCBCR420		(0 << 29)
#define S3C_CITRGFMT_YCBCR422		(1 << 29)
#define S3C_CITRGFMT_YCBCR422_1P	(2 << 29)
#define S3C_CITRGFMT_RGB		(3 << 29)
#define S3C_CITRGFMT_FMT_MASK		(3 << 29)
#define S3C_CITRGFMT_HSIZE_MASK		(0xfff << 16)
#define S3C_CITRGFMT_FLIP_SHIFT		(14)
#define S3C_CITRGFMT_FLIP_NORMAL	(0 << 14)
#define S3C_CITRGFMT_FLIP_X_MIRROR	(1 << 14)
#define S3C_CITRGFMT_FLIP_Y_MIRROR	(2 << 14)
#define S3C_CITRGFMT_FLIP_180		(3 << 14)
#define S3C_CITRGFMT_FLIP_MASK		(3 << 14)
#define S3C_CITRGFMT_VSIZE_MASK		(0xfff << 0)
#define S3C_CITRGFMT_HSIZE(x)		((x) << 16)
#define S3C_CITRGFMT_VSIZE(x)		((x) << 0)

/* Output DMA control */
#define S3C_CIOCTRL			0x4c
#define S3C_CIOCTRL_BURST_MASK		(0xfffff << 4)
#define S3C_CIOCTRL_Y1BURST(x)		((x) << 19)
#define S3C_CIOCTRL_Y2BURST(x)		((x) << 14)
#define S3C_CIOCTRL_C1BURST(x)		((x) << 9)
#define S3C_CIOCTRL_C2BURST(x)		((x) << 4)
#define S3C_CIOCTRL_ORDER422_MASK	(3 << 0)
#define S3C_CIOCTRL_ORDER422_CRYCBY	(0 << 0)
#define S3C_CIOCTRL_ORDER422_CBYCRY	(1 << 0)
#define S3C_CIOCTRL_ORDER422_YCRYCB	(2 << 0)
#define S3C_CIOCTRL_ORDER422_YCBYCR	(3 << 0)
#define S3C_CIOCTRL_LASTIRQ_ENABLE	(1 << 2)

/* Pre-scaler control 1 */
#define S3C_CISCPRERATIO		0x50
#define S3C_CISCPRERATIO_SHFACTOR(x)	((x) << 28)
#define S3C_CISCPRERATIO_HOR(x)		((x) << 16)
#define S3C_CISCPRERATIO_VER(x)		((x) << 0)

#define S3C_CISCPREDST			0x54
#define S3C_CISCPREDST_WIDTH(x)		((x) << 16)
#define S3C_CISCPREDST_HEIGHT(x)	((x) << 0)

/* Main scaler control */
#define S3C_CISCCTRL			0x58
#define S3C_CISCCTRL_SCALERBYPASS	(1 << 31)
#define S3C_CISCCTRL_SCALEUP_H		(1 << 30)
#define S3C_CISCCTRL_SCALEUP_V		(1 << 29)
#define S3C_CISCCTRL_CSCR2Y_WIDE	(1 << 28)
#define S3C_CISCCTRL_CSCY2R_WIDE	(1 << 27)
#define S3C_CISCCTRL_LCDPATHEN_FIFO	(1 << 26)
#define S3C_CISCCTRL_INTERLACE		(1 << 25)
#define S3C_CISCCTRL_SCALERSTART	(1 << 15)
#define S3C_CISCCTRL_INRGB_FMT_RGB565	(0 << 13)
#define S3C_CISCCTRL_INRGB_FMT_RGB666	(1 << 13)
#define S3C_CISCCTRL_INRGB_FMT_RGB888	(2 << 13)
#define S3C_CISCCTRL_INRGB_FMT_MASK	(3 << 13)
#define S3C_CISCCTRL_OUTRGB_FMT_RGB565	(0 << 11)
#define S3C_CISCCTRL_OUTRGB_FMT_RGB666	(1 << 11)
#define S3C_CISCCTRL_OUTRGB_FMT_RGB888	(2 << 11)
#define S3C_CISCCTRL_OUTRGB_FMT_MASK	(3 << 11)
#define S3C_CISCCTRL_RGB_EXT		(1 << 10)
#define S3C_CISCCTRL_ONE2ONE		(1 << 9)
#define S3C_CISCCTRL_MHRATIO(x)		((x) << 16)
#define S3C_CISCCTRL_MVRATIO(x)		((x) << 0)
#define S3C_CISCCTRL_MHRATIO_MASK	(0x1ff << 16)
#define S3C_CISCCTRL_MVRATIO_MASK	(0x1ff << 0)
#define S3C_CISCCTRL_MHRATIO_EXT(x)	(((x) >> 6) << 16)
#define S3C_CISCCTRL_MVRATIO_EXT(x)	(((x) >> 6) << 0)

/* Target area */
#define S3C_CITAREA			0x5c
#define S3C_CITAREA_MASK		0x0fffffff

/* General status */
#define S3C_CISTATUS			0x64
#define S3C_CISTATUS_OVFIY		(1 << 31)
#define S3C_CISTATUS_OVFICB		(1 << 30)
#define S3C_CISTATUS_OVFICR		(1 << 29)
#define S3C_CISTATUS_VSYNC		(1 << 28)
#define S3C_CISTATUS_FRAMECNT_MASK	(3 << 26)
#define S3C_CISTATUS_FRAMECNT_SHIFT	26
#define S3C_CISTATUS_WINOFF_EN		(1 << 25)
#define S3C_CISTATUS_IMGCPT_EN		(1 << 22)
#define S3C_CISTATUS_IMGCPT_SCEN	(1 << 21)
#define S3C_CISTATUS_VSYNC_A		(1 << 20)
#define S3C_CISTATUS_FRAME_END		(1 << 17)

/*
 * Common registers
 */

/* Image capture control */
#define S3C_CIIMGCPT			0xc0
#define S3C_CIIMGCPT_IMGCPTEN		(1 << 31)
#define S3C_CIIMGCPT_IMGCPTEN_SC	(1 << 30)
#define S3C_CIIMGCPT_CPT_FREN_ENABLE	(1 << 25)
#define S3C_CIIMGCPT_CPT_FRMOD_CNT	(1 << 18)

/* Frame capture sequence */
#define S3C_CICPTSEQ			0xc4

/* Image effect */
#define S3C_CIIMGEFF			0xd0
#define S3C_CIIMGEFF_IE_DISABLE		(0 << 30)
#define S3C_CIIMGEFF_IE_ENABLE		(1 << 30)
#define S3C_CIIMGEFF_IE_SC_BEFORE	(0 << 29)
#define S3C_CIIMGEFF_IE_SC_AFTER	(1 << 29)
#define S3C_CIIMGEFF_FIN_BYPASS		(0 << 26)
#define S3C_CIIMGEFF_FIN_ARBITRARY	(1 << 26)
#define S3C_CIIMGEFF_FIN_NEGATIVE	(2 << 26)
#define S3C_CIIMGEFF_FIN_ARTFREEZE	(3 << 26)
#define S3C_CIIMGEFF_FIN_EMBOSSING	(4 << 26)
#define S3C_CIIMGEFF_FIN_SILHOUETTE	(5 << 26)
#define S3C_CIIMGEFF_FIN_MASK		(7 << 26)
#define S3C_CIIMGEFF_PAT_CBCR_MASK	((0xff < 13) | (0xff < 0))
#define S3C_CIIMGEFF_PAT_CB(x)		((x) << 13)
#define S3C_CIIMGEFF_PAT_CR(x)		((x) << 0)

/* Input DMA Y/Cb/Cr plane start address */
#define S3C_CIIYSA			0xd4
#define S3C_CIICBSA			0xd8
#define S3C_CIICRSA			0xdc

/* Input DMA Y/Cb/Cr plane end address */
#define S3C_CIIYEND			0xe0
#define S3C_CIICBEND			0xe4
#define S3C_CIICREND			0xe8

/* Input DMA Y/Cb/Cr plane end address */
#define S3C_CIIYOFF			0xec
#define S3C_CIICBOFF			0xf0
#define S3C_CIICROFF			0xf4

/* Real input DMA image size */
#define S3C_CIREAL_ISIZE		0xf8
#define S3C_CIREAL_ISIZE_AUTOLOAD_EN	(1 << 31)
#define S3C_CIREAL_ISIZE_ADDR_CH_DIS	(1 << 30)
#define S3C_CIREAL_ISIZE_HEIGHT(x)	((x) << 16)
#define S3C_CIREAL_ISIZE_WIDTH(x)	((x) << 0)

/* Input DMA control */
#define S3C_MSCTRL			0xfc
#define S3C_MSCTRL_ORDER422_SHIFT	4
#define S3C_MSCTRL_ORDER422_YCBYCR	(0 << 4)
#define S3C_MSCTRL_ORDER422_CBYCRY	(1 << 4)
#define S3C_MSCTRL_ORDER422_YCRYCB	(2 << 4)
#define S3C_MSCTRL_ORDER422_CRYCBY	(3 << 4)
#define S3C_MSCTRL_ORDER422_MASK	(3 << 4)
#define S3C_MSCTRL_INPUT_EXTCAM		(0 << 3)
#define S3C_MSCTRL_INPUT_MEMORY		(1 << 3)
#define S3C_MSCTRL_INPUT_MASK		(1 << 3)
#define S3C_MSCTRL_INFORMAT_YCBCR420	(0 << 1)
#define S3C_MSCTRL_INFORMAT_YCBCR422	(1 << 1)
#define S3C_MSCTRL_INFORMAT_YCBCR422_1P	(2 << 1)
#define S3C_MSCTRL_INFORMAT_RGB		(3 << 1)
#define S3C_MSCTRL_INFORMAT_MASK	(3 << 1)
#define S3C_MSCTRL_ENVID		(1 << 0)

/* Output DMA Y/Cb/Cr offset */
#define S3C_CIOYOFF			0x12c
#define S3C_CIOCBOFF			0x130
#define S3C_CIOCROFF			0x134

#define S3C_CIO_OFFS_VER(x)		((x) << 16)
#define S3C_CIO_OFFS_HOR(x)		((x) << 0)

#endif /* REGS_FIMC_H_ */
