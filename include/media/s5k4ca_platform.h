/*
 * Driver for S5K4CA (QXGA camera) from Samsung Electronics
 * 
 * 1/4" 3.2Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MEDIA_S5K4CA_PLATFORM_H_
#define _MEDIA_S5K4CA_PLATFORM_H_

struct s5k4ca_platform_data {
	void (*set_power)(int);
};

#endif /* _MEDIA_S5K4CA_PLATFORM_H_ */
