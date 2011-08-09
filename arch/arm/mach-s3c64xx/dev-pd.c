/* linux/arch/arm/mach-s3c64xx/dev-pd.c
 *
 * Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS4 - Power Domain support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/map.h>
#include <mach/regs-sys.h>
#include <mach/regs-syscon-power.h>

#include <mach/pd.h>

struct s3c64xx_pd_data {
	unsigned int ctrl_bit;
	unsigned int stat_bit;
};

static int s3c64xx_pd_pwr_done(int ctrl)
{
	unsigned int cnt;
	cnt = 1000;

	do {
		if (__raw_readl(S3C64XX_BLK_PWR_STAT) & ctrl)
			return 0;
		udelay(1);
	} while (cnt-- > 0);

	return -ETIMEDOUT;
}

static int s3c64xx_pd_pwr_off(int ctrl)
{
	unsigned int cnt;
	cnt = 1000;

	do {
		if (!(__raw_readl(S3C64XX_BLK_PWR_STAT) & ctrl))
			return 0;
		udelay(1);
	} while (cnt-- > 0);

	return -ETIMEDOUT;
}

static int s3c64xx_pd_ctrl(int ctrlbit, int statbit, int enable)
{
	u32 pd_reg = __raw_readl(S3C64XX_NORMAL_CFG);

	if (enable) {
		__raw_writel((pd_reg | ctrlbit), S3C64XX_NORMAL_CFG);
		if (s3c64xx_pd_pwr_done(statbit))
			return -ETIME;
	} else {
		__raw_writel((pd_reg & ~(ctrlbit)), S3C64XX_NORMAL_CFG);
		if (s3c64xx_pd_pwr_off(statbit))
			return -ETIME;
	}
	return 0;
}

static int s3c64xx_pd_enable(struct device *dev)
{
	struct samsung_pd_info *pdata = dev->platform_data;
	struct s3c64xx_pd_data *pd = (struct s3c64xx_pd_data *)pdata->base;
	int ret;

	ret = s3c64xx_pd_ctrl(pd->ctrl_bit, pd->stat_bit, 1);
	if (ret < 0) {
		printk(KERN_ERR "failed to enable power domain\n");
		return ret;
	}

	return 0;
}

static int s3c64xx_pd_disable(struct device *dev)
{
	struct samsung_pd_info *pdata =  dev->platform_data;
	struct s3c64xx_pd_data *pd = (struct s3c64xx_pd_data *)pdata->base;
	int ret;

	ret = s3c64xx_pd_ctrl(pd->ctrl_bit, pd->stat_bit, 0);
	if (ret < 0) {
		printk(KERN_ERR "faild to disable power domain\n");
		return ret;
	}

	return 0;
}

struct platform_device s3c64xx_device_pd[] = {
	[S3C64XX_DOMAIN_ETM] = {
		.name		= "samsung-pd",
		.id		= 0,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_ETM_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_ETM,
				},
			},
		},
	},
	[S3C64XX_DOMAIN_S] = {
		.name		= "samsung-pd",
		.id		= 1,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_S_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_S,
				},
			},
		},
	},
	[S3C64XX_DOMAIN_F] = {
		.name		= "samsung-pd",
		.id		= 2,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_F_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_F,
				},
			},
		},
	},
	[S3C64XX_DOMAIN_P] = {
		.name		= "samsung-pd",
		.id		= 3,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_P_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_P,
				},
			},
		},
	},
	[S3C64XX_DOMAIN_I] = {
		.name		= "samsung-pd",
		.id		= 4,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_I_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_I,
				},
			},
		},
	},
	[S3C64XX_DOMAIN_V] = {
		.name		= "samsung-pd",
		.id		= 5,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_V_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_V,
				},
			},
		},
	},
	[S3C6410_DOMAIN_G] = {
		.name		= "samsung-pd",
		.id		= 6,
		.dev = {
			.platform_data = &(struct samsung_pd_info) {
				.enable		= s3c64xx_pd_enable,
				.disable	= s3c64xx_pd_disable,
				.base		= &(struct s3c64xx_pd_data) {
					.ctrl_bit = S3C64XX_NORMALCFG_DOMAIN_G_ON,
					.stat_bit = S3C64XX_BLKPWRSTAT_G,
				},
			},
		},
	},
};

int s3c64xx_add_pd_devices(void)
{
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(s3c64xx_device_pd) && !ret; ++i)
		ret = platform_device_register(&s3c64xx_device_pd[i]);

	return ret;
}
