/* linux/drivers/media/video/s3c-jpeg/jpeg-regs.h
 *
 * Register definition file for Samsung JPEG codec driver
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

#ifndef JPEG_REGS_H_
#define JPEG_REGS_H_

/* JPEG mode register */
#define S3C_JPGMOD			0x00
#define S3C_PROC_MODE_MASK		(0x1 << 3)
#define S3C_PROC_MODE_DECOMPR		(0x1 << 3)
#define S3C_PROC_MODE_COMPR		(0x0 << 3)
#define S3C_SUBSAMPLING_MODE_MASK	0x7
#define S3C_SUBSAMPLING_MODE_444	(0x0 << 0)
#define S3C_SUBSAMPLING_MODE_422	(0x1 << 0)
#define S3C_SUBSAMPLING_MODE_420	(0x2 << 0)
#define S3C_SUBSAMPLING_MODE_GRAY	(0x3 << 0)

/* JPEG operation status register */
#define S3C_JPGOPR			0x04

/* Quantization and Huffman tables*/
#define S3C_JPG_QHNO			0x08
#define S3C_QT_NUMt_SHIFT(t)		(8 + 2*((t) - 1))
#define S3C_QT_NUMt_MASK(t)		(0x3 << S3C_QT_NUMt_SHIFT(t))
#define S3C_HT_NUMt_AC_SHIFT(t)		(1 + 2*((t) - 1))
#define S3C_HT_NUMt_AC_MASK(t)		(0x1 << S3C_HT_NUMt_AC_SHIFT(t))
#define S3C_HT_NUMt_DC_SHIFT(t)		(2*((t) - 1))
#define S3C_HT_NUMt_DC_MASK(t)		(0x1 << S3C_HT_NUMt_DC_SHIFT(t))

/* JPEG restart interval register */
#define S3C_JPGDRI			0x0c

/* JPEG vertical resolution register */
#define S3C_JPGY			0x10

/* JPEG horizontal resolution register */
#define S3C_JPGX			0x14

/* JPEG byte count register */
#define S3C_JPGCNT			0x18

/* JPEG interrupt setting register */
#define S3C_JPGINTSE			0x1c

/* JPEG interrupt status register */
#define S3C_JPGINTST			0x20
#define S3C_RESULT_STAT_SHIFT		6
#define S3C_RESULT_STAT_MASK		(0x1 << S3C_RESULT_STAT_SHIFT)
#define S3C_STREAM_STAT_SHIFT		4
#define S3C_STREAM_STAT_MASK		(0x1 << S3C_STREAM_STAT_SHIFT)
#define S3C_HDR_STAT_SHIFT		3
#define S3C_HDR_STAT_MASK		(0x1 << S3C_HDR_STAT_SHIFT)

/* Raw image data r/w address register */
#define S3C_JPG_IMGADR0			0x1000
#define S3C_JPG_IMGADR1			0x1004

/* JPEG file r/w address register */
#define S3C_JPG_JPGADR0			0x1008
#define S3C_JPG_JPGADR1			0x100c

/* Coefficient for RGB-to-YCbCr converter register */
#define S3C_JPG_COEF(n)			(0x1020 + 4*((n) - 1))
#define S3C_COEFn_SHIFT(j)		(8*(3 - (j)))
#define S3C_COEFn_MASK(j)		(0xff << S3C_COEFn_SHIFT(j))

/* JPEG color mode register */
#define S3C_JPGCMOD			0x102c
#define S3C_MOD_SEL_MASK		(0x7 << 5)
#define S3C_MOD_SEL_422			(0x1 << 5)
#define S3C_MOD_SEL_565			(0x2 << 5)
#define S3C_MODE_Y16_MASK		(0x1 << 1)
#define S3C_MODE_Y16			(0x1 << 1)

/* JPEG start register */
#define S3C_JSTART			0x1010

/* JPEG restart register */
#define S3C_JRSTART			0x1014

/* JPEG SW reset register */
#define S3C_JPG_SW_RESET		0x1018

/* JPEG control register */
#define S3C_JPGCTL			0x101c

/* JPEG quantizer table register */
#define S3C_JPG_QTBL_CONTENT(n)		(0x400 + (n) * 0x100)

/* JPEG DC Huffman table register */
#define S3C_JPG_HDCTBL(n)		(0x800 + (n) * 0x400)

/* JPEG DC Huffman table register */
#define S3C_JPG_HDCTBLG(n)		(0x840 + (n) * 0x400)

/* JPEG AC Huffman table register */
#define S3C_JPG_HACTBL(n)		(0x880 + (n) * 0x400)

/* JPEG AC Huffman table register */
#define S3C_JPG_HACTBLG(n)		(0x8c0 + (n) * 0x400)

#endif /* JPEG_REGS_H_ */

