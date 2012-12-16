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

#ifndef _VIDEO_OPENFIMG_H_
#define _VIDEO_OPENFIMG_H_

#include <linux/ioctl.h>

/*
 * Data types
 */

enum g3d_request_id {
	OPENFIMG_REQUEST_STATE_BUFFER,
	OPENFIMG_REQUEST_COMMAND_BUFFER,
	OPENFIMG_REQUEST_FENCE
};

struct g3d_user_request {
	enum g3d_request_id type;
	unsigned long priv;
};

/*
 * IOCTLs
 */

#define OPENFIMG_IOCTL_MAGIC	0x3d

enum {
	_ALLOC_NR,
	_SUBMIT_NR,
	_FENCE_WAIT_NR,
};

#define OPENFIMG_ALLOC		_IOWR(OPENFIMG_IOCTL_MAGIC, _ALLOC_NR, \
						struct g3d_user_request)
#define OPENFIMG_SUBMIT		_IOW(OPENFIMG_IOCTL_MAGIC, _SUBMIT_NR, \
						struct g3d_user_request)
#define OPENFIMG_FENCE_WAIT	_IOR(OPENFIMG_IOCTL_MAGIC, _FENCE_WAIT_NR, \
						struct g3d_user_request)

#endif /* _VIDEO_OPENFIMG_H_ */
