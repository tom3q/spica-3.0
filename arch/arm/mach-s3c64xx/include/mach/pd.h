/* linux/arch/arm/plat-samsung/include/plat/pd.h
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ARM_MACH_S3C64XX_PD_H
#define __ARM_MACH_S3C64XX_PD_H __FILE__

#include <plat/pd.h>

enum s3c64xx_pd_block {
	S3C64XX_DOMAIN_ETM = 0,
	S3C64XX_DOMAIN_S,
	S3C64XX_DOMAIN_F,
	S3C64XX_DOMAIN_P,
	S3C64XX_DOMAIN_I,
	S3C64XX_DOMAIN_V,
	S3C6410_DOMAIN_G
};

extern int s3c64xx_add_pd_devices(void);

#endif /* __ARM_MACH_S3C64XX_PD_H */
