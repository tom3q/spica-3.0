/*
 * Platform data structure for spica-dpram
 * Copyright 2012 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _PLATFORM_DATA_SPICA_DPRAM_H
#define _PLATFORM_DATA_SPICA_DPRAM_H

struct dpram_platform_data {
	unsigned int gpio_phone_on;
	unsigned int gpio_phone_rst_n;
	unsigned int gpio_phone_active;
	unsigned int gpio_cp_boot_sel;
	unsigned int gpio_usim_boot;
	unsigned int gpio_pda_active;
	unsigned int gpio_onedram_int_n;
	unsigned int gpio_sim_detect_n;
};

#endif /* _PLATFORM_DATA_SPICA_DPRAM_H */
