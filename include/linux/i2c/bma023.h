/*
 * bma023.h - BMA023 Tri-axis accelerometer driver
 *
 * Copyright (c) 2010 Samsung Eletronics
 * Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _BMA023_H_
#define _BMA023_H_

#define	BMA023_RANGE_2G		0
#define	BMA023_RANGE_4G		1
#define	BMA023_RANGE_8G		2

/* Used to setup the digital filtering bandwidth of ADC output */
#define	BMA023_BW_25HZ		0
#define	BMA023_BW_50HZ		1
#define	BMA023_BW_100HZ		2
#define	BMA023_BW_190HZ		3
#define	BMA023_BW_375HZ		4
#define	BMA023_BW_750HZ		5
#define BMA023_BW_1500HZ	6

struct bma023_platform_data {
	u8 range;		/* BMA023_RANGE_xxx */
	u8 bandwidth;		/* BMA023_BW_xxx */
	u8 new_data_int;	/* Set to enable new data interrupt */
	u8 hg_int;		/* Set to enable high G interrupt */
	u8 lg_int;		/* Set to enable low G interrupt */
	u8 lg_dur;		/* Low G duration */
	u8 lg_thres;		/* Low G threshold */
	u8 lg_hyst;		/* Low G Hysterisis */
	u8 hg_dur;		/* Ditto for High G */
	u8 hg_thres;
	u8 hg_hyst;
	int auto_delay;		/* Runtime PM keeps active on use for mS */
};

#endif /* _BMA023_H_ */
