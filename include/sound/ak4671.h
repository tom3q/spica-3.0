/*
 * AK4671 ALSA SoC Codec driver
 *
 * Copyright 2011 Tomasz Figa
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AK4671_H
#define __AK4671_H

/**
 * struct ak4671_platform_data - platform specific AK4641 configuration
 * @gpio_pdn:	GPIO connected to AK4671 nPDN pin (optional)
 */
struct ak4671_platform_data {
	int gpio_npdn;
};

#endif /* __AK4671_H */
