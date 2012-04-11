/* linux/drivers/media/video/s3c-jpeg/jpeg-hw.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef JPEG_HW_H_
#define JPEG_HW_H_

#include <linux/io.h>
#include <linux/videodev2.h>
#include <linux/delay.h>

#include "jpeg-hw.h"
#include "jpeg-regs.h"

#define S3C_JPEG_MIN_WIDTH		32
#define S3C_JPEG_MIN_HEIGHT		32
#define S3C_JPEG_MAX_WIDTH		8192
#define S3C_JPEG_MAX_HEIGHT		8192
#define S3C_JPEG_ENCODE			0
#define S3C_JPEG_DECODE			1
#define S3C_JPEG_RAW_IN_565		0
#define S3C_JPEG_RAW_IN_422		1
#define S3C_JPEG_RAW_OUT_422		0
#define S3C_JPEG_RAW_OUT_420		1

static inline void jpeg_reset(void __iomem *regs)
{
	writel(0, regs + S3C_JPG_SW_RESET);
}

static inline void jpeg_input_raw_mode(void __iomem *regs, unsigned long mode)
{
	unsigned long reg, m;

	m = S3C_MOD_SEL_565;
	if (mode == S3C_JPEG_RAW_IN_565)
		m = S3C_MOD_SEL_565;
	else if (mode == S3C_JPEG_RAW_IN_422)
		m = S3C_MOD_SEL_422;

	reg = readl(regs + S3C_JPGCMOD);
	reg &= ~S3C_MOD_SEL_MASK;
	reg |= m;
	writel(reg, regs + S3C_JPGCMOD);
}

static inline void jpeg_input_raw_y16(void __iomem *regs, bool y16)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPGCMOD);
	if (y16)
		reg |= S3C_MODE_Y16;
	else
		reg &= ~S3C_MODE_Y16_MASK;
	writel(reg, regs + S3C_JPGCMOD);
}

static inline void jpeg_proc_mode(void __iomem *regs, unsigned long mode)
{
	unsigned long reg, m;

	m = S3C_PROC_MODE_DECOMPR;
	if (mode == S3C_JPEG_ENCODE)
		m = S3C_PROC_MODE_COMPR;
	else
		m = S3C_PROC_MODE_DECOMPR;
	reg = readl(regs + S3C_JPGMOD);
	reg &= ~S3C_PROC_MODE_MASK;
	reg |= m;
	writel(reg, regs + S3C_JPGMOD);
}

static inline void jpeg_subsampling_mode(void __iomem *regs, unsigned int mode)
{
	unsigned long reg, m;

	if (mode == V4L2_JPEG_CHROMA_SUBSAMPLING_420)
		m = S3C_SUBSAMPLING_MODE_420;
	else
		m = S3C_SUBSAMPLING_MODE_422;

	reg = readl(regs + S3C_JPGMOD);
	reg &= ~S3C_SUBSAMPLING_MODE_MASK;
	reg |= m;
	writel(reg, regs + S3C_JPGMOD);
}

static inline unsigned int jpeg_get_subsampling_mode(void __iomem *regs)
{
	return readl(regs + S3C_JPGMOD) & S3C_SUBSAMPLING_MODE_MASK;
}

static inline void jpeg_dri(void __iomem *regs, unsigned int dri)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPGDRI);
	reg &= ~0xffff;
	reg |= dri & 0xffff;
	writel(reg, regs + S3C_JPGDRI);
}

static inline void jpeg_qtbl(void __iomem *regs, unsigned int t, unsigned int n)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPG_QHNO);
	reg &= ~S3C_QT_NUMt_MASK(t);
	reg |= (n << S3C_QT_NUMt_SHIFT(t)) & S3C_QT_NUMt_MASK(t);
	writel(reg, regs + S3C_JPG_QHNO);
}

static inline void jpeg_htbl_ac(void __iomem *regs, unsigned int t)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPG_QHNO);
	reg &= ~S3C_HT_NUMt_AC_MASK(t);
	/* this driver uses table 0 for all color components */
	reg |= (0 << S3C_HT_NUMt_AC_SHIFT(t)) & S3C_HT_NUMt_AC_MASK(t);
	writel(reg, regs + S3C_JPG_QHNO);
}

static inline void jpeg_htbl_dc(void __iomem *regs, unsigned int t)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPG_QHNO);
	reg &= ~S3C_HT_NUMt_DC_MASK(t);
	/* this driver uses table 0 for all color components */
	reg |= (0 << S3C_HT_NUMt_DC_SHIFT(t)) & S3C_HT_NUMt_DC_MASK(t);
	writel(reg, regs + S3C_JPG_QHNO);
}

static inline void jpeg_y(void __iomem *regs, unsigned int y)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPGY);
	reg &= ~0xffff;
	reg |= y & 0xffff;
	writel(reg, regs + S3C_JPGY);
}

static inline void jpeg_x(void __iomem *regs, unsigned int x)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPGX);
	reg &= ~0xffff;
	reg |= x & 0xffff;
	writel(reg, regs + S3C_JPGX);
}

static inline void jpeg_rst_int_enable(void __iomem *regs, bool enable)
{
	writel(0x78, regs + S3C_JPGINTSE);
}

static inline void jpeg_jpgadr(void __iomem *regs, unsigned long addr)
{
	writel(addr, regs + S3C_JPG_JPGADR0);
	writel(addr, regs + S3C_JPG_JPGADR1);
}

static inline void jpeg_imgadr(void __iomem *regs, unsigned long addr)
{
	writel(addr, regs + S3C_JPG_IMGADR0);
	writel(addr, regs + S3C_JPG_IMGADR1);
}

static inline void jpeg_coef(void __iomem *regs, unsigned int i,
			     unsigned int j, unsigned int coef)
{
	unsigned long reg;

	reg = readl(regs + S3C_JPG_COEF(i));
	reg &= ~S3C_COEFn_MASK(j);
	reg |= (coef << S3C_COEFn_SHIFT(j)) & S3C_COEFn_MASK(j);
	writel(reg, regs + S3C_JPG_COEF(i));
}

static inline void jpeg_start_dec(void __iomem *regs)
{
	writel(0, regs + S3C_JPGCTL);
	writel(1, regs + S3C_JSTART);
}

static inline void jpeg_restart_dec(void __iomem *regs)
{
	writel(1, regs + S3C_JRSTART);
}

static inline void jpeg_start_enc(void __iomem *regs)
{
	writel(0, regs + S3C_JPGCTL);
	writel(0, regs + S3C_JSTART);
}

static inline int jpeg_int_stat(void __iomem *regs)
{
	readl(regs + S3C_JPGOPR);
	return readl(regs + S3C_JPGINTST);
}

static inline int jpeg_result_stat_ok(int stat)
{
	return (int)((stat & S3C_RESULT_STAT_MASK) >> S3C_RESULT_STAT_SHIFT);
}

static inline int jpeg_stream_stat_ok(int stat)
{
	return !(int)((stat & S3C_STREAM_STAT_MASK) >> S3C_STREAM_STAT_SHIFT);
}

static inline int jpeg_hdr_stat_ok(int stat)
{
	return (int)((stat & S3C_HDR_STAT_MASK) >> S3C_HDR_STAT_SHIFT);
}

static inline unsigned int jpeg_compressed_size(void __iomem *regs)
{
	return readl(regs + S3C_JPGCNT) & 0xffffff;
}

#endif /* JPEG_HW_H_ */
