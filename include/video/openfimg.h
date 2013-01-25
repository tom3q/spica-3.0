/* video/openfimg.h
 *
 * Copyright (c) 2012 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * OpenFIMG kernel driver for Samsung FIMG-3DSE GPU
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
 */

#ifndef _VIDEO_G3D_H_
#define _VIDEO_G3D_H_

#include <linux/ioctl.h>

/*
 * Data types
 */

enum g3d_register {
	/*
	 * Protected registers
	 */

	FGPF_FRONTST,
	FGPF_DEPTHT,
	FGPF_DBMSK,
	FGPF_FBCTL,

	G3D_NUM_PROT_REGISTERS,

	/*
	 * Public registers
	 */

	/* Host interface */
	FGHI_ATTRIB0,
	FGHI_ATTRIB1,
	FGHI_ATTRIB2,
	FGHI_ATTRIB3,
	FGHI_ATTRIB4,
	FGHI_ATTRIB5,
	FGHI_ATTRIB6,
	FGHI_ATTRIB7,
	FGHI_ATTRIB8,
	FGHI_ATTRIB9,
	FGHI_ATTRIB_VBCTRL0,
	FGHI_ATTRIB_VBCTRL1,
	FGHI_ATTRIB_VBCTRL2,
	FGHI_ATTRIB_VBCTRL3,
	FGHI_ATTRIB_VBCTRL4,
	FGHI_ATTRIB_VBCTRL5,
	FGHI_ATTRIB_VBCTRL6,
	FGHI_ATTRIB_VBCTRL7,
	FGHI_ATTRIB_VBCTRL8,
	FGHI_ATTRIB_VBCTRL9,
	FGHI_ATTRIB_VBBASE0,
	FGHI_ATTRIB_VBBASE1,
	FGHI_ATTRIB_VBBASE2,
	FGHI_ATTRIB_VBBASE3,
	FGHI_ATTRIB_VBBASE4,
	FGHI_ATTRIB_VBBASE5,
	FGHI_ATTRIB_VBBASE6,
	FGHI_ATTRIB_VBBASE7,
	FGHI_ATTRIB_VBBASE8,
	FGHI_ATTRIB_VBBASE9,

	/* Primitive engine */
	FGPE_VERTEX_CONTEXT,
	FGPE_VIEWPORT_OX,
	FGPE_VIEWPORT_OY,
	FGPE_VIEWPORT_HALF_PX,
	FGPE_VIEWPORT_HALF_PY,
	FGPE_DEPTHRANGE_HALF_F_SUB_N,
	FGPE_DEPTHRANGE_HALF_F_ADD_N,

	/* Raster engine */
	FGRA_PIX_SAMP,
	FGRA_D_OFF_EN,
	FGRA_D_OFF_FACTOR,
	FGRA_D_OFF_UNITS,
	FGRA_D_OFF_R_IN,
	FGRA_BFCULL,
	FGRA_YCLIP,
	FGRA_LODCTL,
	FGRA_XCLIP,
	FGRA_PWIDTH,
	FGRA_PSIZE_MIN,
	FGRA_PSIZE_MAX,
	FGRA_COORDREPLACE,
	FGRA_LWIDTH,

	/* Per-fragment unit */
	FGPF_ALPHAT,
	FGPF_BACKST,
	FGPF_CCLR,
	FGPF_BLEND,
	FGPF_LOGOP,

	G3D_NUM_REGISTERS
};

enum g3d_request_id {
	G3D_REQUEST_STATE_BUFFER,
	G3D_REQUEST_COMMAND_BUFFER,
	G3D_REQUEST_FENCE,
	G3D_REQUEST_SHADER_PROGRAM,
	G3D_REQUEST_SHADER_DATA,
	G3D_REQUEST_TEXTURE,
	G3D_REQUEST_FRAMEBUFFER,

	G3D_NUM_REQUESTS
};

struct g3d_state_entry {
	enum g3d_register reg;
	u32 val;
};

#define G3D_FENCE_FLUSH		(1 << 0)
#define G3D_FENCE_TIMED_OUT	(1 << 1)

enum g3d_shader_type {
	G3D_SHADER_VERTEX,
	G3D_SHADER_PIXEL,

	G3D_NUM_SHADERS
};

enum g3d_shader_data_type {
	G3D_SHADER_DATA_FLOAT,
	G3D_SHADER_DATA_INT,
	G3D_SHADER_DATA_BOOL,

	G3D_NUM_SHADER_DATA
};

enum g3d_buffer_type {
	G3D_BUFFER_DMA,
	G3D_BUFFER_PMEM,
	G3D_BUFFER_DMA_BUF,

	G3D_NUM_BUFFER_TYPES
};

struct g3d_user_buffer {
	enum g3d_buffer_type type;
	off_t offset;
	size_t length;
	long handle;
};

struct g3d_user_request {
	enum g3d_request_id type;
	union {
		struct {
			const struct g3d_state_entry *regs;
			size_t nr_regs;
			size_t nr_prot_regs;
			unsigned long flags;
		} state;
		struct {
			const u32 *buf;
			unsigned int count;
			size_t len;
		} command;
		struct {
			unsigned long flags;
			unsigned long priv;
		} fence;
		struct {
			enum g3d_shader_type type;
			const u32 *code;
			size_t len;
		} shader;
		struct {
			enum g3d_shader_type shader;
			enum g3d_shader_data_type type;
			const u32 *buf;
			off_t offset;
			size_t len;
		} shader_data;
		struct {
			unsigned int unit;
			unsigned int id;
		} texture;
		struct {
			unsigned int id;
		} framebuffer;
	};
	struct g3d_user_request __user *next;
};

/*
 * IOCTLs
 */

#define G3D_IOCTL_MAGIC		0x3d

enum {
	_REQUEST_SUBMIT_NR,
	_FENCE_WAIT_NR,
	_CREATE_BUFFER_NR
};

#define G3D_REQUEST_SUBMIT	_IOW(G3D_IOCTL_MAGIC, _REQUEST_SUBMIT_NR, \
						struct g3d_user_request)
#define G3D_FENCE_WAIT		_IOR(G3D_IOCTL_MAGIC, _FENCE_WAIT_NR, \
						struct g3d_user_request)
#define G3D_CREATE_BUFFER	_IOWR(G3D_IOCTL_MAGIC, _CREATE_BUFFER_NR, \
						struct g3d_user_buffer)

#endif /* _VIDEO_G3D_H_ */
