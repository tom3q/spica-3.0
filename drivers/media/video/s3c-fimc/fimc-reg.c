/*
 * Register interface file for Samsung Camera Interface (FIMC) driver
 *
 * Copyright (c) 2010 Samsung Electronics
 *
 * Sylwester Nawrocki, s.nawrocki@samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>
#include <linux/delay.h>
#include <mach/map.h>
#include <media/s3c_fimc.h>

#include "fimc-core.h"


void fimc_hw_reset(struct fimc_dev *dev)
{
	u32 cfg;

	cfg = readl(dev->regs + S3C_CISRCFMT);
	cfg |= S3C_CISRCFMT_ITU601_8BIT;
	writel(cfg, dev->regs + S3C_CISRCFMT);

	/* Software reset. */
	cfg = readl(dev->regs + S3C_CIGCTRL);
	cfg |= (S3C_CIGCTRL_SWRST | S3C_CIGCTRL_IRQ_LEVEL);
	writel(cfg, dev->regs + S3C_CIGCTRL);
	udelay(1000);

	cfg = readl(dev->regs + S3C_CIGCTRL);
	cfg &= ~S3C_CIGCTRL_SWRST;
	writel(cfg, dev->regs + S3C_CIGCTRL);
}

static u32 fimc_hw_get_target_flip(struct fimc_ctx *ctx)
{
	u32 flip = S3C_CITRGFMT_FLIP_NORMAL;

	switch (ctx->flip) {
	case FLIP_X_AXIS:
		flip = S3C_CITRGFMT_FLIP_X_MIRROR;
		break;
	case FLIP_Y_AXIS:
		flip = S3C_CITRGFMT_FLIP_Y_MIRROR;
		break;
	case FLIP_XY_AXIS:
		flip = S3C_CITRGFMT_FLIP_180;
		break;
	default:
		break;
	}
	if (ctx->rotation <= 90)
		return flip;

	return (flip ^ S3C_CITRGFMT_FLIP_180) & S3C_CITRGFMT_FLIP_180;
}

void fimc_hw_set_rotation(struct fimc_ctx *ctx)
{
	u32 cfg;
	struct fimc_dev *dev = ctx->fimc_dev;

	cfg = readl(dev->regs + S3C_CITRGFMT);

	if (ctx->out_path == FIMC_DMA) {
		cfg |= fimc_hw_get_target_flip(ctx);
		writel(cfg, dev->regs + S3C_CITRGFMT);
	}
}

void fimc_hw_set_target_format(struct fimc_ctx *ctx)
{
	u32 cfg;
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_frame *frame = &ctx->d_frame;

	dbg("w= %d, h= %d color: %d", frame->width,
		frame->height, frame->fmt->color);

	cfg = readl(dev->regs + S3C_CITRGFMT);
	cfg &= ~(S3C_CITRGFMT_FMT_MASK | S3C_CITRGFMT_HSIZE_MASK |
		  S3C_CITRGFMT_VSIZE_MASK);

	switch (frame->fmt->color) {
	case S3C_FIMC_RGB565...S3C_FIMC_RGB888:
		cfg |= S3C_CITRGFMT_RGB;
		break;
	case S3C_FIMC_YCBCR420:
		cfg |= S3C_CITRGFMT_YCBCR420;
		break;
	case S3C_FIMC_YCBYCR422...S3C_FIMC_CRYCBY422:
		if (frame->fmt->colplanes == 1)
			cfg |= S3C_CITRGFMT_YCBCR422_1P;
		else
			cfg |= S3C_CITRGFMT_YCBCR422;
		break;
	default:
		break;
	}

	if (ctx->rotation == 90 || ctx->rotation == 270) {
		cfg |= S3C_CITRGFMT_HSIZE(frame->height);
		cfg |= S3C_CITRGFMT_VSIZE(frame->width);
	} else {

		cfg |= S3C_CITRGFMT_HSIZE(frame->width);
		cfg |= S3C_CITRGFMT_VSIZE(frame->height);
	}

	writel(cfg, dev->regs + S3C_CITRGFMT);

	cfg = readl(dev->regs + S3C_CITAREA) & ~S3C_CITAREA_MASK;
	cfg |= (frame->width * frame->height);
	writel(cfg, dev->regs + S3C_CITAREA);
}

void fimc_hw_set_out_dma(struct fimc_ctx *ctx)
{
	u32 cfg;
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_frame *frame = &ctx->d_frame;
	struct fimc_dma_offset *offset = &frame->dma_offset;

	/* Set the input dma offsets. */
	cfg = 0;
	cfg |= S3C_CIO_OFFS_HOR(offset->y_h);
	cfg |= S3C_CIO_OFFS_VER(offset->y_v);
	writel(cfg, dev->regs + S3C_CIOYOFF);

	cfg = 0;
	cfg |= S3C_CIO_OFFS_HOR(offset->cb_h);
	cfg |= S3C_CIO_OFFS_VER(offset->cb_v);
	writel(cfg, dev->regs + S3C_CIOCBOFF);

	cfg = 0;
	cfg |= S3C_CIO_OFFS_HOR(offset->cr_h);
	cfg |= S3C_CIO_OFFS_VER(offset->cr_v);
	writel(cfg, dev->regs + S3C_CIOCROFF);

	/* Configure chroma components order. */
	cfg = readl(dev->regs + S3C_CIOCTRL);

	cfg &= ~(S3C_CIOCTRL_ORDER422_MASK);

	if (frame->fmt->colplanes == 1)
		cfg |= ctx->out_order_1p;

	writel(cfg, dev->regs + S3C_CIOCTRL);
}

void fimc_hw_en_lastirq(struct fimc_dev *dev, int enable)
{
	u32 cfg = readl(dev->regs + S3C_CIOCTRL);
	if (enable)
		cfg |= S3C_CIOCTRL_LASTIRQ_ENABLE;
	else
		cfg &= ~S3C_CIOCTRL_LASTIRQ_ENABLE;
	writel(cfg, dev->regs + S3C_CIOCTRL);
}

void fimc_hw_set_prescaler(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev =  ctx->fimc_dev;
	struct fimc_scaler *sc = &ctx->scaler;
	u32 cfg, shfactor;

	shfactor = 10 - (sc->hfactor + sc->vfactor);

	cfg = S3C_CISCPRERATIO_SHFACTOR(shfactor);
	cfg |= S3C_CISCPRERATIO_HOR(sc->pre_hratio);
	cfg |= S3C_CISCPRERATIO_VER(sc->pre_vratio);
	writel(cfg, dev->regs + S3C_CISCPRERATIO);

	cfg = S3C_CISCPREDST_WIDTH(sc->pre_dst_width);
	cfg |= S3C_CISCPREDST_HEIGHT(sc->pre_dst_height);
	writel(cfg, dev->regs + S3C_CISCPREDST);
}

static void fimc_hw_set_scaler(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_scaler *sc = &ctx->scaler;
	struct fimc_frame *src_frame = &ctx->s_frame;
	struct fimc_frame *dst_frame = &ctx->d_frame;
	u32 cfg = 0;

	if (!(ctx->flags & FIMC_COLOR_RANGE_NARROW))
		cfg |= (S3C_CISCCTRL_CSCR2Y_WIDE | S3C_CISCCTRL_CSCY2R_WIDE);

	if (!sc->enabled)
		cfg |= S3C_CISCCTRL_SCALERBYPASS;

	if (sc->scaleup_h)
		cfg |= S3C_CISCCTRL_SCALEUP_H;

	if (sc->scaleup_v)
		cfg |= S3C_CISCCTRL_SCALEUP_V;

	if (sc->copy_mode)
		cfg |= S3C_CISCCTRL_ONE2ONE;


	if (ctx->in_path == FIMC_DMA) {
		if (src_frame->fmt->color == S3C_FIMC_RGB565)
			cfg |= S3C_CISCCTRL_INRGB_FMT_RGB565;
		else if (src_frame->fmt->color == S3C_FIMC_RGB666)
			cfg |= S3C_CISCCTRL_INRGB_FMT_RGB666;
		else if (src_frame->fmt->color == S3C_FIMC_RGB888)
			cfg |= S3C_CISCCTRL_INRGB_FMT_RGB888;
	}

	if (ctx->out_path == FIMC_DMA) {
		if (dst_frame->fmt->color == S3C_FIMC_RGB565)
			cfg |= S3C_CISCCTRL_OUTRGB_FMT_RGB565;
		else if (dst_frame->fmt->color == S3C_FIMC_RGB666)
			cfg |= S3C_CISCCTRL_OUTRGB_FMT_RGB666;
		else if (dst_frame->fmt->color == S3C_FIMC_RGB888)
			cfg |= S3C_CISCCTRL_OUTRGB_FMT_RGB888;
	} else {
		cfg |= S3C_CISCCTRL_OUTRGB_FMT_RGB888;

		if (ctx->flags & FIMC_SCAN_MODE_INTERLACED)
			cfg |= S3C_CISCCTRL_INTERLACE;
	}

	writel(cfg, dev->regs + S3C_CISCCTRL);
}

void fimc_hw_set_mainscaler(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_scaler *sc = &ctx->scaler;
	u32 cfg;

	dbg("main_hratio= 0x%X  main_vratio= 0x%X",
		sc->main_hratio, sc->main_vratio);

	fimc_hw_set_scaler(ctx);

	cfg = readl(dev->regs + S3C_CISCCTRL);
	cfg &= ~(S3C_CISCCTRL_MHRATIO_MASK | S3C_CISCCTRL_MVRATIO_MASK);
	cfg |= S3C_CISCCTRL_MHRATIO(sc->main_hratio);
	cfg |= S3C_CISCCTRL_MVRATIO(sc->main_vratio);
	writel(cfg, dev->regs + S3C_CISCCTRL);
}

void fimc_hw_en_capture(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;

	u32 cfg = readl(dev->regs + S3C_CIIMGCPT);

	if (ctx->out_path == FIMC_DMA) {
		/* one shot mode */
		cfg |= S3C_CIIMGCPT_CPT_FREN_ENABLE | S3C_CIIMGCPT_IMGCPTEN;
	} else {
		/* Continuous frame capture mode (freerun). */
		cfg &= ~(S3C_CIIMGCPT_CPT_FREN_ENABLE |
			 S3C_CIIMGCPT_CPT_FRMOD_CNT);
		cfg |= S3C_CIIMGCPT_IMGCPTEN;
	}

	if (ctx->scaler.enabled)
		cfg |= S3C_CIIMGCPT_IMGCPTEN_SC;

	writel(cfg | S3C_CIIMGCPT_IMGCPTEN, dev->regs + S3C_CIIMGCPT);
}

void fimc_hw_set_effect(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_effect *effect = &ctx->effect;
	u32 cfg = (S3C_CIIMGEFF_IE_ENABLE | S3C_CIIMGEFF_IE_SC_AFTER);

	cfg |= effect->type;

	if (effect->type == S3C_FIMC_EFFECT_ARBITRARY) {
		cfg |= S3C_CIIMGEFF_PAT_CB(effect->pat_cb);
		cfg |= S3C_CIIMGEFF_PAT_CR(effect->pat_cr);
	}

	writel(cfg, dev->regs + S3C_CIIMGEFF);
}

static void fimc_hw_set_in_dma_size(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_frame *frame = &ctx->s_frame;
	u32 cfg_r = 0;

	cfg_r |= S3C_CIREAL_ISIZE_WIDTH(frame->width);
	cfg_r |= S3C_CIREAL_ISIZE_HEIGHT(frame->height);

	writel(cfg_r, dev->regs + S3C_CIREAL_ISIZE);
}

void fimc_hw_set_in_dma(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;
	struct fimc_frame *frame = &ctx->s_frame;
	struct fimc_dma_offset *offset = &frame->dma_offset;
	u32 cfg;

	/* Set the pixel offsets. */
	cfg = S3C_CIO_OFFS_HOR(offset->y_h);
	cfg |= S3C_CIO_OFFS_VER(offset->y_v);
	writel(cfg, dev->regs + S3C_CIIYOFF);

	cfg = S3C_CIO_OFFS_HOR(offset->cb_h);
	cfg |= S3C_CIO_OFFS_VER(offset->cb_v);
	writel(cfg, dev->regs + S3C_CIICBOFF);

	cfg = S3C_CIO_OFFS_HOR(offset->cr_h);
	cfg |= S3C_CIO_OFFS_VER(offset->cr_v);
	writel(cfg, dev->regs + S3C_CIICROFF);

	/* Input original and real size. */
	fimc_hw_set_in_dma_size(ctx);

	/* Set the input DMA to process single frame only. */
	cfg = readl(dev->regs + S3C_MSCTRL);
	cfg &= ~(S3C_MSCTRL_INFORMAT_MASK
		| S3C_MSCTRL_INPUT_MASK);

	cfg |= (S3C_MSCTRL_INPUT_MEMORY);

	switch (frame->fmt->color) {
	case S3C_FIMC_RGB565...S3C_FIMC_RGB888:
		cfg |= S3C_MSCTRL_INFORMAT_RGB;
		break;
	case S3C_FIMC_YCBCR420:
		cfg |= S3C_MSCTRL_INFORMAT_YCBCR420;
		break;
	case S3C_FIMC_YCBYCR422...S3C_FIMC_CRYCBY422:
		if (frame->fmt->colplanes == 1)
			cfg |= ctx->in_order_1p
				| S3C_MSCTRL_INFORMAT_YCBCR422_1P;
		else
			cfg |= S3C_MSCTRL_INFORMAT_YCBCR422;
		break;
	default:
		break;
	}

	writel(cfg, dev->regs + S3C_MSCTRL);
}


void fimc_hw_set_input_path(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;

	u32 cfg = readl(dev->regs + S3C_MSCTRL);
	cfg &= ~S3C_MSCTRL_INPUT_MASK;

	if (ctx->in_path == FIMC_DMA)
		cfg |= S3C_MSCTRL_INPUT_MEMORY;
	else
		cfg |= S3C_MSCTRL_INPUT_EXTCAM;

	writel(cfg, dev->regs + S3C_MSCTRL);
}

void fimc_hw_set_output_path(struct fimc_ctx *ctx)
{
	struct fimc_dev *dev = ctx->fimc_dev;

	u32 cfg = readl(dev->regs + S3C_CISCCTRL);
	cfg &= ~S3C_CISCCTRL_LCDPATHEN_FIFO;
	writel(cfg, dev->regs + S3C_CISCCTRL);
}

void fimc_hw_set_input_addr(struct fimc_dev *dev, struct fimc_addr *paddr)
{
	u32 cfg = readl(dev->regs + S3C_CIREAL_ISIZE);
	cfg |= S3C_CIREAL_ISIZE_ADDR_CH_DIS;
	writel(cfg, dev->regs + S3C_CIREAL_ISIZE);

	writel(paddr->y, dev->regs + S3C_CIIYSA);
	writel(paddr->cb, dev->regs + S3C_CIICBSA);
	writel(paddr->cr, dev->regs + S3C_CIICRSA);

	cfg &= ~S3C_CIREAL_ISIZE_ADDR_CH_DIS;
	writel(cfg, dev->regs + S3C_CIREAL_ISIZE);
}

void fimc_hw_set_output_addr(struct fimc_dev *dev,
			     struct fimc_addr *paddr, int index)
{
	int i = (index == -1) ? 0 : index;
	do {
		writel(paddr->y, dev->regs + S3C_CIOYSA(i));
		writel(paddr->cb, dev->regs + S3C_CIOCBSA(i));
		writel(paddr->cr, dev->regs + S3C_CIOCRSA(i));
		dbg("dst_buf[%d]: 0x%X, cb: 0x%X, cr: 0x%X",
		    i, paddr->y, paddr->cb, paddr->cr);
	} while (index == -1 && ++i < FIMC_MAX_OUT_BUFS);
}

int fimc_hw_set_camera_polarity(struct fimc_dev *fimc,
				struct s3c_fimc_isp_info *cam)
{
	u32 cfg = readl(fimc->regs + S3C_CIGCTRL);

	cfg &= ~(S3C_CIGCTRL_INVPOLPCLK | S3C_CIGCTRL_INVPOLVSYNC |
		 S3C_CIGCTRL_INVPOLHREF);

	if (cam->flags & FIMC_CLK_INV_PCLK)
		cfg |= S3C_CIGCTRL_INVPOLPCLK;

	if (cam->flags & FIMC_CLK_INV_VSYNC)
		cfg |= S3C_CIGCTRL_INVPOLVSYNC;

	if (cam->flags & FIMC_CLK_INV_HREF)
		cfg |= S3C_CIGCTRL_INVPOLHREF;

	writel(cfg, fimc->regs + S3C_CIGCTRL);

	return 0;
}

int fimc_hw_set_camera_source(struct fimc_dev *fimc,
			      struct s3c_fimc_isp_info *cam)
{
	struct fimc_frame *f = &fimc->vid_cap.ctx->s_frame;
	u32 cfg = 0;
	u32 bus_width;
	int i;

	static const struct {
		u32 pixelcode;
		u32 cisrcfmt;
		u16 bus_width;
	} pix_desc[] = {
		{ V4L2_MBUS_FMT_YUYV8_2X8, S3C_CISRCFMT_ORDER422_YCBYCR, 8 },
		{ V4L2_MBUS_FMT_YVYU8_2X8, S3C_CISRCFMT_ORDER422_YCRYCB, 8 },
		{ V4L2_MBUS_FMT_VYUY8_2X8, S3C_CISRCFMT_ORDER422_CRYCBY, 8 },
		{ V4L2_MBUS_FMT_UYVY8_2X8, S3C_CISRCFMT_ORDER422_CBYCRY, 8 },
		/* TODO: Add pixel codes for 16-bit bus width */
	};

	if (cam->bus_type == FIMC_ITU_601 || cam->bus_type == FIMC_ITU_656) {
		for (i = 0; i < ARRAY_SIZE(pix_desc); i++) {
			if (fimc->vid_cap.fmt.code == pix_desc[i].pixelcode) {
				cfg = pix_desc[i].cisrcfmt;
				bus_width = pix_desc[i].bus_width;
				break;
			}
		}

		if (i == ARRAY_SIZE(pix_desc)) {
			v4l2_err(&fimc->vid_cap.v4l2_dev,
				 "Camera color format not supported: %d\n",
				 fimc->vid_cap.fmt.code);
			return -EINVAL;
		}

		if (cam->bus_type == FIMC_ITU_601) {
			if (bus_width == 8)
				cfg |= S3C_CISRCFMT_ITU601_8BIT;
		} /* else defaults to ITU-R BT.656 8-bit */
	}

	cfg |= S3C_CISRCFMT_HSIZE(f->o_width) | S3C_CISRCFMT_VSIZE(f->o_height);
	writel(cfg, fimc->regs + S3C_CISRCFMT);
	return 0;
}


int fimc_hw_set_camera_offset(struct fimc_dev *fimc, struct fimc_frame *f)
{
	u32 hoff2, voff2;

	u32 cfg = readl(fimc->regs + S3C_CIWDOFST);

	cfg &= ~(S3C_CIWDOFST_HOROFF_MASK | S3C_CIWDOFST_VEROFF_MASK);
	cfg |=  S3C_CIWDOFST_OFF_EN |
		S3C_CIWDOFST_HOROFF(f->offs_h) |
		S3C_CIWDOFST_VEROFF(f->offs_v);

	writel(cfg, fimc->regs + S3C_CIWDOFST);

	/* See CIWDOFSTn register description in the datasheet for details. */
	hoff2 = f->o_width - f->width - f->offs_h;
	voff2 = f->o_height - f->height - f->offs_v;
	cfg = S3C_CIWDOFST2_HOROFF(hoff2) | S3C_CIWDOFST2_VEROFF(voff2);

	writel(cfg, fimc->regs + S3C_CIWDOFST2);
	return 0;
}

int fimc_hw_set_camera_type(struct fimc_dev *fimc,
			    struct s3c_fimc_isp_info *cam)
{
	u32 cfg;

	cfg = readl(fimc->regs + S3C_CIGCTRL);

	/* Disable test pattern. */
	cfg &= ~(S3C_CIGCTRL_TESTPAT_MASK);

	writel(cfg, fimc->regs + S3C_CIGCTRL);

	return 0;
}
