/*
 * max9877.h  --  amp driver for max9877
 *
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _MAX9877_H
#define _MAX9877_H

#define MAX9877_INPUT_MODE		0x00
#define MAX9877_SPK_VOLUME		0x01
#define MAX9877_HPL_VOLUME		0x02
#define MAX9877_HPR_VOLUME		0x03
#define MAX9877_OUTPUT_MODE		0x04
#define MAX9877_CACHEREGNUM		5

/* MAX9877_INPUT_MODE */
#define MAX9877_INB			4
#define MAX9877_INA			5
#define MAX9877_ZCD			6

/* MAX9877_OUTPUT_MODE */
#define MAX9877_OUTMODE_SHIFT		0
#define MAX9877_OUTMODE_MASK		(0xf << MAX9877_OUTMODE_SHIFT)
#define MAX9877_OSC_SHIFT		4
#define MAX9877_OSC_MASK		(0x3 << MAX9877_OSC_SHIFT)
#define MAX9877_BYPASS_SHIFT		6
#define MAX9877_BYPASS			(1 << MAX9877_BYPASS_SHIFT)
#define MAX9877_SHDN_SHIFT		7
#define MAX9877_SHDN			(1 << MAX9877_SHDN_SHIFT)

#endif
