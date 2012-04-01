/*
 * Samsung FIMG-2D device driver
 *
 * Copyright 2010-2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ioport.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif

#include <linux/s3c_g2d.h>

/*
 * Registers
 */
#define G2D_CONTROL_REG		(0x00)
#define G2D_INTEN_REG		(0x04)
#define G2D_FIFO_INTC_REG	(0x08)
#define G2D_INTC_PEND_REG	(0x0c)
#define G2D_FIFO_STAT_REG	(0x10)
#define G2D_CMD0_REG		(0x100)
#define G2D_CMD1_REG		(0x104)
#define G2D_CMD2_REG		(0x108)
#define G2D_CMD3_REG		(0x10c)
#define G2D_CMD4_REG		(0x110)
#define G2D_CMD5_REG		(0x114)
#define G2D_CMD6_REG		(0x118)
#define G2D_CMD7_REG		(0x11c)
#define G2D_SRC_RES_REG		(0x200)
#define G2D_SRC_HORI_REG	(0x204)
#define G2D_SRC_VERT_REG	(0x208)
#define G2D_DST_RES_REG		(0x210)
#define G2D_DST_HORI_REG	(0x214)
#define G2D_DST_VERT_REG	(0x218)
#define G2D_CW_LT_REG		(0x220)
#define G2D_CW_LT_X_REG		(0x224)
#define G2D_CW_LT_Y_REG		(0x228)
#define G2D_CW_RB_REG		(0x230)
#define G2D_CW_RB_X_REG		(0x234)
#define G2D_CW_RB_Y_REG		(0x238)
#define G2D_COORD0_REG		(0x300)
#define G2D_COORD0_X_REG	(0x304)
#define G2D_COORD0_Y_REG	(0x308)
#define G2D_COORD1_REG		(0x310)
#define G2D_COORD1_X_REG	(0x314)
#define G2D_COORD1_Y_REG	(0x318)
#define G2D_COORD2_REG		(0x320)
#define G2D_COORD2_X_REG	(0x324)
#define G2D_COORD2_Y_REG	(0x328)
#define G2D_COORD3_REG		(0x330)
#define G2D_COORD3_X_REG	(0x334)
#define G2D_COORD3_Y_REG	(0x338)
#define G2D_ROT_OC_REG		(0x340)
#define G2D_ROT_OC_X_REG	(0x344)
#define G2D_ROT_OC_Y_REG	(0x348)
#define G2D_ROTATE_REG		(0x34c)
#define G2D_END_RDSIZE_REG	(0x350)
#define G2D_X_INCR_REG		(0x400)
#define G2D_Y_INCR_REG		(0x404)
#define G2D_ROP_REG		(0x410)
#define G2D_ALPHA_REG		(0x420)
#define G2D_FG_COLOR_REG	(0x500)
#define G2D_BG_COLOR_REG	(0x504)
#define G2D_BS_COLOR_REG	(0x508)
#define G2D_SRC_FORMAT_REG	(0x510)
#define G2D_DST_FORMAT_REG	(0x514)
#define G2D_PATTERN_REG		(0x600)
#define G2D_PATOFF_REG		(0x700)
#define G2D_PATOFF_X_REG	(0x704)
#define G2D_PATOFF_Y_REG	(0x708)
#define G2D_STENCIL_CNTL_REG	(0x720)
#define G2D_STENCIL_DR_MIN_REG	(0x724)
#define G2D_STENCIL_DR_MAX_REG	(0x728)
#define G2D_SRC_BASE_ADDR	(0x730)
#define G2D_DST_BASE_ADDR	(0x734)

/*
 * Register bits
 */
#define G2D_INTC_PEND_REG_CLRSEL_LEVEL		(1 << 31)
#define G2D_INTC_PEND_REG_CLRSEL_PULSE		(0 << 31)

#define G2D_INTEN_REG_FIFO_INT_E		(1 << 0)
#define G2D_INTEN_REG_ACF			(1 << 9)
#define G2D_INTEN_REG_CCF			(1 << 10)

#define G2D_PEND_REG_INTP_ALL_FIN		(1 << 9)
#define G2D_PEND_REG_INTP_CMD_FIN		(1 << 10)

#define G2D_CMD0_REG_D_LAST			(0 << 9)
#define G2D_CMD0_REG_D_NO_LAST			(1 << 9)
#define G2D_CMD0_REG_M_Y			(0 << 8)
#define G2D_CMD0_REG_M_X			(1 << 8)
#define G2D_CMD0_REG_L				(1 << 1)
#define G2D_CMD0_REG_P				(1 << 0)

#define G2D_CMD1_REG_S				(1 << 1)
#define G2D_CMD1_REG_N				(1 << 0)

#define G2D_COLOR_MODE_REG_C3_32BPP		(1 << 3)
#define G2D_COLOR_MODE_REG_C3_24BPP		(1 << 3)
#define G2D_COLOR_MODE_REG_C2_18BPP		(1 << 2)
#define G2D_COLOR_MODE_REG_C1_16BPP		(1 << 1)
#define G2D_COLOR_MODE_REG_C0_15BPP		(1 << 0)

#define G2D_COLOR_RGB_565			(0 << 0)
#define G2D_COLOR_RGBA_5551			(1 << 0)
#define G2D_COLOR_ARGB_1555			(2 << 0)
#define G2D_COLOR_RGBA_8888			(3 << 0)
#define G2D_COLOR_ARGB_8888			(4 << 0)
#define G2D_COLOR_XRGB_8888			(5 << 0)
#define G2D_COLOR_RGBX_8888			(6 << 0)

#define G2D_ROTATE_REG_FY			(1 << 5)
#define G2D_ROTATE_REG_FX			(1 << 4)
#define G2D_ROTATE_REG_R3_270			(1 << 3)
#define G2D_ROTATE_REG_R2_180			(1 << 2)
#define G2D_ROTATE_REG_R1_90			(1 << 1)
#define G2D_ROTATE_REG_R0_0			(1 << 0)

#define G2D_ENDIAN_READSIZE_BIG_ENDIAN		(1 << 4)
#define G2D_ENDIAN_READSIZE_SIZE_HW		(1 << 2)
#define G2D_ENDIAN_READSIZE_READ_SIZE_1		(0 << 0)
#define G2D_ENDIAN_READSIZE_READ_SIZE_4		(1 << 0)
#define G2D_ENDIAN_READSIZE_READ_SIZE_8		(2 << 0)
#define G2D_ENDIAN_READSIZE_READ_SIZE_16	(3 << 0)

#define G2D_ROP_REG_OS_PATTERN			(0 << 13)
#define G2D_ROP_REG_OS_FG_COLOR			(1 << 13)

#define G2D_ROP_REG_ABM_NO_BLENDING		(0 << 10)
#define G2D_ROP_REG_ABM_SRC_BITMAP		(1 << 10)
#define G2D_ROP_REG_ABM_REGISTER		(2 << 10)
#define G2D_ROP_REG_ABM_FADING 			(4 << 10)

#define G2D_ROP_REG_T_OPAQUE_MODE		(0 << 9)
#define G2D_ROP_REG_T_TRANSP_MODE		(1 << 9)
#define G2D_ROP_REG_B_BS_MODE_OFF		(0 << 8)
#define G2D_ROP_REG_B_BS_MODE_ON		(1 << 8)

#define G2D_STENCIL_CNTL_REG_STENCIL_ON_ON	(1 << 31)
#define G2D_STENCIL_CNTL_REG_STENCIL_ON_OFF	(0 << 31)
#define G2D_STENCIL_CNTL_REG_STENCIL_INVERSE	(1 << 23)
#define G2D_STENCIL_CNTL_REG_STENCIL_SWAP	(1 << 0)

/*
 * Various definitions
 */
#define G2D_FIFO_SIZE		32
#define G2D_TIMEOUT		100
#define G2D_RESET_TIMEOUT	1000
#define G2D_AUTOSUSPEND_DELAY	1000

/*
 * Internal data structures
 */
struct g2d_drvdata {
	void __iomem		*base;
	struct miscdevice	mdev;
	struct mutex		mutex;
	struct completion	completion;
	struct workqueue_struct	*workqueue;
	wait_queue_head_t	waitq;
	int			irq;
	struct resource 	*mem;
	struct clk		*clock;
	struct device		*dev;
};

struct g2d_context
{
	struct g2d_drvdata	*data;
	struct work_struct	work;
	struct s3c_g2d_req	*blit;
	struct s3c_g2d_fillrect	*fill;
	uint32_t		blend;
	uint32_t		alpha;
	uint32_t		rot;
	uint32_t		rop;
	struct file		*srcf;
	struct file		*dstf;
};

/*
 * Register accessors
 */
static inline void g2d_write(struct g2d_drvdata *d, uint32_t b, uint32_t r)
{
	writel(b, d->base + r);
}

static inline uint32_t g2d_read(struct g2d_drvdata *d, uint32_t r)
{
	return readl(d->base + r);
}

/*
 * Hardware operations
 */
static inline uint32_t g2d_check_fifo(struct g2d_drvdata *data, uint32_t needed)
{
	int i;
	u32 used;

	for (i = 0; i < G2D_TIMEOUT; i++) {
		used = (g2d_read(data, G2D_FIFO_STAT_REG) >> 1) & 0x3f;

		if ((G2D_FIFO_SIZE - used) >= needed)
			return 0;
	}

	return 1;
}

static void g2d_soft_reset(struct g2d_drvdata *data)
{
	int i;
	u32 reg;

	g2d_write(data, 1, G2D_CONTROL_REG);

	for(i = 0; i < G2D_RESET_TIMEOUT; i++) {
		reg = g2d_read(data, G2D_CONTROL_REG) & 1;

		if(reg == 0)
			break;

		udelay(1);
	}

	if(i == G2D_RESET_TIMEOUT)
		dev_err(data->dev, "soft reset timeout.\n");
}

static inline uint32_t g2d_pack_xy(uint32_t x, uint32_t y)
{
	return (y << 16) | x;
}

static int g2d_do_blit(struct g2d_context *ctx)
{
	struct g2d_drvdata *data = ctx->data;
	struct s3c_g2d_req *req = ctx->blit;
	uint32_t srcw, srch, dstw, dsth;
	uint32_t xincr = 1, yincr = 1;
	uint32_t stretch;
	uint32_t blend;
	uint32_t vdx1, vdy1, vdx2, vdy2, vsw, vsh;

	srcw = req->src.r - req->src.l + 1;
	srch = req->src.b - req->src.t + 1;
	dstw = req->dst.r - req->dst.l + 1;
	dsth = req->dst.b - req->dst.t + 1;

	switch (ctx->rot) {
	case G2D_ROT_90:
		// origin : (dst_x2, dst_y1)
		vdx1 = req->dst.r;
		vdy1 = req->dst.t;
		vdx2 = req->dst.r + dsth - 1;
		vdy2 = req->dst.t + dstw - 1;
		vsw = srch;
		vsh = srcw;
		break;
	case G2D_ROT_180:
		// origin : (dst_x2, dst_y2)
		vdx1 = req->dst.r;
		vdy1 = req->dst.b;
		vdx2 = req->dst.r + dstw - 1;
		vdy2 = req->dst.b + dsth - 1;
		vsw = srcw;
		vsh = srch;
		break;
	case G2D_ROT_270:
		// origin : (dst_x1, dst_y2)
		vdx1 = req->dst.l;
		vdy1 = req->dst.b;
		vdx2 = req->dst.l + dsth - 1;
		vdy2 = req->dst.b + dstw - 1;
		vsw = srch;
		vsh = srcw;
		break;
	case G2D_ROT_FLIP_X:
		// origin : (dst_x1, dst_y2)
		vdx1 = req->dst.l;
		vdy1 = req->dst.b;
		vdx2 = req->dst.l + dstw - 1;
		vdy2 = req->dst.b + dsth - 1;
		vsw = srcw;
		vsh = srch;
		break;
	case G2D_ROT_FLIP_Y:
		// origin : (dst_x2, dst_y1)
		vdx1 = req->dst.r;
		vdy1 = req->dst.t;
		vdx2 = req->dst.r + dstw - 1;
		vdy2 = req->dst.t + dsth - 1;
		vsw = srcw;
		vsh = srch;
		break;
	default:
		vdx1 = req->dst.l;
		vdy1 = req->dst.t;
		vdx2 = req->dst.r;
		vdy2 = req->dst.b;
		vsw = srcw;
		vsh = srch;
		break;
	}

	stretch = (vsw != dstw) || (vsh != dsth);

	/* NOTE: Theoretically this could by skipped */
	if (g2d_check_fifo(data, 21)) {
		dev_err(data->dev, "timeout while waiting for FIFO\n");
		return -EBUSY;
	}

	if(stretch) {
		/* Compute X scaling factor */
		/* (division takes time, so do it now for pipelining */
		xincr = (vsw << 11) / dstw;
	}

	/* Configure source image */
	dev_dbg(data->dev, "SRC %08x + %08x, %dx%d, fmt = %d\n", req->src.base, req->src.offs,
					req->src.w, req->src.h, req->src.fmt);

	g2d_write(data, req->src.base + req->src.offs, G2D_SRC_BASE_ADDR);
	g2d_write(data, g2d_pack_xy(req->src.w, req->src.h), G2D_SRC_RES_REG);
	g2d_write(data, req->src.fmt, G2D_SRC_FORMAT_REG);

	if (req->src.fmt == G2D_RGBA32 || req->src.fmt == G2D_RGBX32)
		g2d_write(data, 1, G2D_END_RDSIZE_REG);
	else
		g2d_write(data, 0, G2D_END_RDSIZE_REG);

	/* Configure destination image */
	dev_dbg(data->dev, "DST %08x + %08x, %dx%d, fmt = %d\n", req->dst.base, req->dst.offs,
					req->dst.w, req->dst.h, req->dst.fmt);

	g2d_write(data, req->dst.base + req->dst.offs, G2D_DST_BASE_ADDR);
	g2d_write(data, g2d_pack_xy(req->dst.w, req->dst.h), G2D_DST_RES_REG);
	g2d_write(data, req->dst.fmt, G2D_DST_FORMAT_REG);

	/* Configure clipping window to destination size */
	dev_dbg(data->dev, "CLIP (%d,%d) (%d,%d)\n", 0, 0, req->dst.w - 1, req->dst.h - 1);

	g2d_write(data, g2d_pack_xy(0, 0), G2D_CW_LT_REG);
	g2d_write(data, g2d_pack_xy(req->dst.w - 1, req->dst.h - 1),
							G2D_CW_RB_REG);

	if(stretch) {
		/* Compute Y scaling factor */
		/* (division takes time, so do it now for pipelining */
		yincr = (vsh << 11) / dsth;
	}

	/* Configure ROP and alpha blending */
	if(ctx->blend == G2D_PIXEL_ALPHA) {
		switch(req->src.fmt) {
		case G2D_ARGB16:
		case G2D_ARGB32:
		case G2D_RGBA32:
			blend = G2D_ROP_REG_ABM_SRC_BITMAP;
			break;
		default:
			blend = G2D_ROP_REG_ABM_REGISTER;
		}
	} else {
		blend = ctx->blend << 10;
	}
	blend |= ctx->rop;
	g2d_write(data, blend, G2D_ROP_REG);
	g2d_write(data, ctx->alpha, G2D_ALPHA_REG);

	/* Configure rotation */
	g2d_write(data, ctx->rot, G2D_ROTATE_REG);
	g2d_write(data, g2d_pack_xy(vdx1, vdy1), G2D_ROT_OC_REG);

	dev_dbg(data->dev, "BLEND %08x ROTATE %08x REF=(%d, %d)\n",
			blend, ctx->rot, vdx1, vdy1);

	/* Configure coordinates */
	dev_dbg(data->dev, "BLIT (%d,%d) (%d,%d) => (%d,%d) (%d,%d)\n",
		req->src.l, req->src.t, req->src.r, req->src.b,
		vdx1, vdy1, vdx2, vdy2);

	g2d_write(data, g2d_pack_xy(req->src.l, req->src.t), G2D_COORD0_REG);
	g2d_write(data, g2d_pack_xy(req->src.r, req->src.b), G2D_COORD1_REG);

	g2d_write(data, g2d_pack_xy(vdx1, vdy1), G2D_COORD2_REG);
	g2d_write(data, g2d_pack_xy(vdx2, vdy2), G2D_COORD3_REG);

	/* Configure scaling factors */
	dev_dbg(data->dev, "SCALE X_INCR = %08x, Y_INCR = %08x\n", xincr, yincr);
	g2d_write(data, xincr, G2D_X_INCR_REG);
	g2d_write(data, yincr, G2D_Y_INCR_REG);

	g2d_write(data, G2D_INTEN_REG_ACF, G2D_INTEN_REG);

	/* Start the operation */
	if(stretch)
		g2d_write(data, G2D_CMD1_REG_S, G2D_CMD1_REG);
	else
		g2d_write(data, G2D_CMD1_REG_N, G2D_CMD1_REG);

	return 0;
}

static int g2d_do_fillrect(struct g2d_context *ctx)
{
	struct g2d_drvdata *data = ctx->data;
	struct s3c_g2d_fillrect *req = ctx->fill;

	/* NOTE: Theoretically this could by skipped */
	if (g2d_check_fifo(data, 19)) {
		dev_err(data->dev, "timeout while waiting for FIFO\n");
		return -EBUSY;
	}

	/* Configure images */
	dev_dbg(data->dev, "DST %08x + %08x, %dx%d, fmt = %d\n", req->dst.base, req->dst.offs,
					req->dst.w, req->dst.h, req->dst.fmt);

	g2d_write(data, req->dst.base + req->dst.offs, G2D_SRC_BASE_ADDR);
	g2d_write(data, req->dst.base + req->dst.offs, G2D_DST_BASE_ADDR);
	g2d_write(data, g2d_pack_xy(req->dst.w, req->dst.h), G2D_SRC_RES_REG);
	g2d_write(data, g2d_pack_xy(req->dst.w, req->dst.h), G2D_DST_RES_REG);
	g2d_write(data, req->dst.fmt, G2D_SRC_FORMAT_REG);
	g2d_write(data, req->dst.fmt, G2D_DST_FORMAT_REG);
	g2d_write(data, (req->dst.fmt == G2D_RGBA32), G2D_END_RDSIZE_REG);

	/* Configure clipping window to destination size */
	dev_dbg(data->dev, "CLIP (%d,%d) (%d,%d)\n", 0, 0, req->dst.w - 1, req->dst.h - 1);

	g2d_write(data, g2d_pack_xy(0, 0), G2D_CW_LT_REG);
	g2d_write(data, g2d_pack_xy(req->dst.w - 1, req->dst.h - 1),
							G2D_CW_RB_REG);

	/* Configure ROP and alpha blending */
	g2d_write(data, G2D_ROP_REG_OS_FG_COLOR | G2D_ROP_3RD_OPRND_ONLY,
								G2D_ROP_REG);
	g2d_write(data, req->alpha, G2D_ALPHA_REG);

	/* Configure fill color */
	g2d_write(data, req->color, G2D_FG_COLOR_REG);

	/* Configure rotation */
	g2d_write(data, G2D_ROTATE_REG_R0_0, G2D_ROTATE_REG);

	/* Configure coordinates */
	dev_dbg(data->dev, "FILL %08x => (%d,%d) (%d,%d)\n", req->color, req->dst.l,
					req->dst.t, req->dst.r, req->dst.b);

	g2d_write(data, g2d_pack_xy(req->dst.l, req->dst.t), G2D_COORD0_REG);
	g2d_write(data, g2d_pack_xy(req->dst.r, req->dst.b), G2D_COORD1_REG);

	g2d_write(data, g2d_pack_xy(req->dst.l, req->dst.t), G2D_COORD2_REG);
	g2d_write(data, g2d_pack_xy(req->dst.r, req->dst.b), G2D_COORD3_REG);

	g2d_write(data, G2D_INTEN_REG_ACF, G2D_INTEN_REG);

	/* Start the operation */
	g2d_write(data, G2D_CMD1_REG_N, G2D_CMD1_REG);

	return 0;
}

/*
 * PMEM support
 */
static inline int get_img(struct s3c_g2d_image *img, struct file** filep)
{
#ifdef CONFIG_ANDROID_PMEM
	unsigned long vstart, start, len;

	if(!get_pmem_file(img->fd, &start, &vstart, &len, filep)) {
		img->base = start;
		return 0;
	}
#endif
	return -1;
}

static inline void put_img(struct file *file)
{
#ifdef CONFIG_ANDROID_PMEM
	if (file)
		put_pmem_file(file);
#endif
}

static inline u32 get_pixel_size(uint32_t fmt)
{
	switch(fmt) {
	case G2D_RGBA32:
	case G2D_ARGB32:
	case G2D_XRGB32:
	case G2D_RGBX32:
		return 4;
	default:
		return 2;
	}
}

static inline void flush_img(struct s3c_g2d_image *img, struct file *file)
{
#ifdef CONFIG_ANDROID_PMEM
	u32 len;

	if(file) {
		/* flush image to memory before blit operation */
		len = img->w * img->h * get_pixel_size(img->fmt);
		flush_pmem_file(file, img->offs, len);
	}
#endif
}

/*
 * State processing
 */
static inline int wait_for_g2d(struct g2d_drvdata *data)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(
						&data->completion, G2D_TIMEOUT);
	if (!ret) {
		dev_err(data->dev,
			"timeout while waiting for interrupt, resetting\n");
		g2d_soft_reset(data);
		return -ETIMEDOUT;
	}

	return 0;
}

static irqreturn_t g2d_handle_irq(int irq, void *dev_id)
{
	struct g2d_drvdata *data = (struct g2d_drvdata *)dev_id;
	u32 stat;

	stat = g2d_read(data, G2D_INTC_PEND_REG);
	if (stat & G2D_PEND_REG_INTP_ALL_FIN) {
		g2d_write(data, G2D_PEND_REG_INTP_ALL_FIN, G2D_INTC_PEND_REG);
		complete(&data->completion);
	}

	return IRQ_HANDLED;
}

static void g2d_workfunc(struct work_struct *work)
{
	struct g2d_context *ctx =
				container_of(work, struct g2d_context, work);
	struct g2d_drvdata *data = ctx->data;

	wait_for_g2d(data);

	pm_runtime_mark_last_busy(data->dev);
	pm_runtime_put_autosuspend(data->dev);

	put_img(ctx->srcf);
	put_img(ctx->dstf);

	mutex_unlock(&data->mutex);

	wake_up_interruptible(&data->waitq);
}

/*
 * Graphics operations
 */
static int s3c_g2d_fill(struct g2d_context *ctx, unsigned long arg, int nblock)
{
	struct g2d_drvdata *data = ctx->data;
	struct file *dstf = 0;
	struct s3c_g2d_fillrect req;
	int ret = 0;

	if(!mutex_trylock(&data->mutex)) {
		if(nblock)
			return -EWOULDBLOCK;

		ret = mutex_lock_interruptible(&data->mutex);
		if (ret)
			return ret;
	}

	if (unlikely(copy_from_user(&req, (struct s3c_g2d_fillrect*)arg,
					sizeof(struct s3c_g2d_fillrect)))) {
		dev_err(data->dev, "copy_from_user failed\n");
		ret = -EFAULT;
		goto err_noput;
	}

	if (unlikely((req.dst.w <= 1) || (req.dst.h <= 1))) {
		dev_err(data->dev, "invalid destination resolution\n");
		ret = -EINVAL;
		goto err_noput;
	}

	if (unlikely(req.dst.base == 0) && unlikely(get_img(&req.dst, &dstf))) {
		dev_err(data->dev, "could not retrieve dst image from memory\n");
		ret = -EINVAL;
		goto err_noput;
	}

	ctx->srcf = 0;
	ctx->dstf = dstf;
	ctx->fill = &req; // becomes invalid after leaving this function!

	ret = pm_runtime_get_sync(data->dev);
	if (ret < 0) {
		dev_err(data->dev, "G2D power up failed\n");
		goto err_cmd;
	}

	flush_img(&req.dst, dstf);

	ret = g2d_do_fillrect(ctx);
	if(ret != 0) {
		dev_err(data->dev, "Failed to start G2D operation (%d)\n", ret);
		g2d_soft_reset(data);
		goto err_cmd;
	}

	// block mode
	if (nblock) {
		queue_work(data->workqueue, &ctx->work);
		return 0;
	}

	ret = wait_for_g2d(data);

err_cmd:
	pm_runtime_mark_last_busy(data->dev);
	pm_runtime_put_autosuspend(data->dev);
	put_img(dstf);
err_noput:
	mutex_unlock(&data->mutex);
	wake_up_interruptible(&data->waitq);

	return ret;
}

static int s3c_g2d_blit(struct g2d_context *ctx, unsigned long arg, int nblock)
{
	struct g2d_drvdata *data = ctx->data;
	struct file *srcf = 0, *dstf = 0;
	struct s3c_g2d_req req;
	int ret = 0;

	if(!mutex_trylock(&data->mutex)) {
		if(nblock)
			return -EWOULDBLOCK;

		ret = mutex_lock_interruptible(&data->mutex);
		if (ret)
			return ret;
	}

	if (unlikely(copy_from_user(&req, (struct s3c_g2d_req*)arg,
					sizeof(struct s3c_g2d_req)))) {
		dev_err(data->dev, "copy_from_user failed\n");
		ret = -EFAULT;
		goto err_noput;
	}

	if (unlikely((req.src.w <= 1) || (req.src.h <= 1))) {
		dev_err(data->dev, "invalid source resolution\n");
		ret = -EINVAL;
		goto err_noput;
	}

	if (unlikely((req.dst.w <= 1) || (req.dst.h <= 1))) {
		dev_err(data->dev, "invalid destination resolution\n");
		ret = -EINVAL;
		goto err_noput;
	}

	/* do this first so that if this fails, the caller can always
	 * safely call put_img */
	if (likely(req.src.base == 0) && unlikely(get_img(&req.src, &srcf))) {
		dev_err(data->dev, "could not retrieve src image from memory\n");
		ret = -EINVAL;
		goto err_noput;
	}

	if (unlikely(req.dst.base == 0) && unlikely(get_img(&req.dst, &dstf))) {
		dev_err(data->dev, "could not retrieve dst image from memory\n");
		put_img(srcf);
		ret = -EINVAL;
		goto err_noput;
	}

	ctx->srcf = srcf;
	ctx->dstf = dstf;
	ctx->blit = &req; // becomes invalid after leaving this function!

	ret = pm_runtime_get_sync(data->dev);
	if (ret < 0) {
		dev_err(data->dev, "G2D power up failed\n");
		goto err_pwr;
	}

	flush_img(&req.src, srcf);
	flush_img(&req.dst, dstf);

	ret = g2d_do_blit(ctx);
	if(ret != 0) {
		dev_err(data->dev, "Failed to start G2D operation (%d)\n", ret);
		g2d_soft_reset(data);
		goto err_cmd;
	}

	// block mode
	if (nblock) {
		queue_work(data->workqueue, &ctx->work);
		return 0;
	}

	ret = wait_for_g2d(data);

err_cmd:
	pm_runtime_mark_last_busy(data->dev);
	pm_runtime_put_autosuspend(data->dev);
err_pwr:
	put_img(srcf);
	put_img(dstf);
err_noput:
	mutex_unlock(&data->mutex);
	wake_up_interruptible(&data->waitq);

	return ret;
}

/*
 * File operations
 */
static long s3c_g2d_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct g2d_context *ctx = (struct g2d_context *)file->private_data;
	int nblock = file->f_flags & O_NONBLOCK;

	switch (cmd) {
	/* Proceed with the operation */
	case S3C_G2D_BITBLT:
		return s3c_g2d_blit(ctx, arg, nblock);
	case S3C_G2D_FILLRECT:
		return s3c_g2d_fill(ctx, arg, nblock);
	/* Set the parameter and return */
	case S3C_G2D_SET_TRANSFORM:
		ctx->rot = arg;
		return 0;
	case S3C_G2D_SET_ALPHA_VAL:
		ctx->alpha = (arg > ALPHA_VALUE_MAX) ? 255 : arg;
		return 0;
	case S3C_G2D_SET_RASTER_OP:
		ctx->rop = arg & 0xff;
		return 0;
	case S3C_G2D_SET_BLENDING:
		ctx->blend = arg;
		return 0;
	/* Invalid IOCTL call */
	default:
		return -EINVAL;
	}
}

static unsigned int s3c_g2d_poll(struct file *file, poll_table *wait)
{
	struct g2d_context *ctx = (struct g2d_context *)file->private_data;
	struct g2d_drvdata *data = ctx->data;
	unsigned int mask = 0;

	poll_wait(file, &data->waitq, wait);

	if(!mutex_is_locked(&data->mutex))
		mask = POLLOUT|POLLWRNORM;

	return mask;
}

static int s3c_g2d_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct g2d_drvdata *data = container_of(mdev, struct g2d_drvdata, mdev);
	struct g2d_context *ctx;

	ctx = kmalloc(sizeof(struct g2d_context), GFP_KERNEL);
	if (ctx == NULL) {
		dev_err(data->dev, "Context allocation failed\n");
		return -ENOMEM;
	}

	memset(ctx, 0, sizeof(struct g2d_context));
	ctx->data	= data;
	ctx->rot	= G2D_ROT_0;
	ctx->alpha	= ALPHA_VALUE_MAX;
	ctx->rop	= G2D_ROP_SRC_ONLY;
	INIT_WORK(&ctx->work, g2d_workfunc);

	file->private_data = ctx;

	dev_dbg(data->dev, "device opened\n");

	return 0;
}

static int s3c_g2d_release(struct inode *inode, struct file *file)
{
	struct g2d_context *ctx = (struct g2d_context *)file->private_data;
	struct g2d_drvdata *data = ctx->data;

	cancel_work_sync(&ctx->work);
	kfree(ctx);

	dev_dbg(data->dev, "device released\n");

	return 0;
}

static struct file_operations s3c_g2d_fops = {
	.owner		= THIS_MODULE,
	.open		= s3c_g2d_open,
	.release	= s3c_g2d_release,
	.unlocked_ioctl	= s3c_g2d_ioctl,
	.poll		= s3c_g2d_poll,
};

/*
 * Platform device operations
 */
static int __init s3c_g2d_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct g2d_drvdata *data;
	int ret;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "only a single instance (id=-1) is allowed.\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct g2d_drvdata), GFP_KERNEL);
	if (data == NULL) {
		dev_err(data->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	data->workqueue = create_singlethread_workqueue("s3c-g2d");
	if (data->workqueue == NULL) {
		dev_err(data->dev, "failed to create workqueue.\n");
		ret = -ENOMEM;
		goto err_workqueue;
	}

	/* get the clock */
	data->clock = clk_get(&pdev->dev, "2d");
	if (data->clock == NULL) {
		dev_err(data->dev, "failed to find g2d clock source\n");
		ret = -ENOENT;
		goto err_clock;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, G2D_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	/* get the memory region */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(data->dev, "failed to get memory region resource.\n");
		ret = -ENOENT;
		goto err_mem;
	}

	/* reserve the memory */
	data->mem = request_mem_region(res->start, resource_size(res),
								pdev->name);
	if (data->mem == NULL) {
		dev_err(data->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_mem;
	}

	/* map the memory */
	data->base = ioremap(data->mem->start, resource_size(data->mem));
	if (data->base == NULL) {
		dev_err(data->dev, "ioremap failed\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	/* get the IRQ */
	data->irq = platform_get_irq(pdev, 0);
	if (data->irq <= 0) {
		dev_err(data->dev, "failed to get irq resource (%d).\n", data->irq);
		ret = data->irq;
		goto err_irq;
	}

	/* request the IRQ */
	ret = request_irq(data->irq, g2d_handle_irq, 0, pdev->name, data);
	if (ret) {
		dev_err(data->dev, "request_irq failed (%d).\n", ret);
		goto err_irq;
	}

	data->dev = &pdev->dev;
	mutex_init(&data->mutex);
	init_completion(&data->completion);
	init_waitqueue_head(&data->waitq);

	platform_set_drvdata(pdev, data);

	pm_runtime_get_sync(&pdev->dev);

	g2d_soft_reset(data);

	data->mdev.minor = MISC_DYNAMIC_MINOR;
	data->mdev.name = "s3c-g2d";
	data->mdev.fops = &s3c_g2d_fops;

	ret = misc_register(&data->mdev);
	if (ret) {
		dev_err(data->dev, "cannot register miscdev (%d)\n", ret);
		goto err_misc_register;
	}

	pm_runtime_put_sync(&pdev->dev);

	dev_info(data->dev, "driver loaded succesfully.\n");

	return 0;

err_misc_register:
	pm_runtime_put_sync(&pdev->dev);
	free_irq(data->irq, pdev);
err_irq:
	iounmap(data->base);
err_ioremap:
	release_resource(data->mem);
err_mem:
	pm_runtime_disable(&pdev->dev);
	clk_put(data->clock);
err_clock:
	destroy_workqueue(data->workqueue);
err_workqueue:
	kfree(data);

	return ret;
}

static int __devexit s3c_g2d_remove(struct platform_device *pdev)
{
	struct g2d_drvdata *data = platform_get_drvdata(pdev);

	misc_deregister(&data->mdev);

	destroy_workqueue(data->workqueue);

	pm_runtime_suspend(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	free_irq(data->irq, data);
	iounmap(data->base);
	release_resource(data->mem);

	kfree(data);

	return 0;
}

static int s3c_g2d_runtime_suspend(struct device *dev)
{
	struct g2d_drvdata *data = dev_get_drvdata(dev);

	clk_disable(data->clock);

	return 0;
}

static int s3c_g2d_runtime_resume(struct device *dev)
{
	struct g2d_drvdata *data = dev_get_drvdata(dev);

	clk_enable(data->clock);
	g2d_soft_reset(data);

	return 0;
}

static int s3c_g2d_suspend(struct device *dev)
{
	struct g2d_drvdata *data = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev))
		return 0;

	clk_disable(data->clock);

	return 0;
}

static int s3c_g2d_resume(struct device *dev)
{
	struct g2d_drvdata *data = dev_get_drvdata(dev);

	clk_enable(data->clock);
	g2d_soft_reset(data);

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_request_idle(dev);

	return 0;
}

static struct dev_pm_ops s3c_g2d_pm_ops = {
	.suspend		= s3c_g2d_suspend,
	.resume			= s3c_g2d_resume,
	.runtime_suspend	= s3c_g2d_runtime_suspend,
	.runtime_resume		= s3c_g2d_runtime_resume,
};

static struct platform_driver s3c_g2d_driver = {
	.remove         = __devexit_p(s3c_g2d_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g2d",
		.pm	= &s3c_g2d_pm_ops,
	},
};

/*
 * Module operations
 */
int __init  s3c_g2d_init(void)
{
	return platform_driver_probe(&s3c_g2d_driver, s3c_g2d_probe);
}

void __exit s3c_g2d_exit(void)
{
	platform_driver_unregister(&s3c_g2d_driver);
}

module_init(s3c_g2d_init);
module_exit(s3c_g2d_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("Samsung FIMG-2D device driver");
MODULE_LICENSE("GPL");
