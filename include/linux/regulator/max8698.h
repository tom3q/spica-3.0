/*
 * max8698.h - Public definitions for Maxim MAX8698 voltage regulator driver
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _MAX8698_H_
#define _MAX8698_H_

enum {
	MAX8698_LDO2,
	MAX8698_LDO3,
	MAX8698_LDO4,
	MAX8698_LDO5,
	MAX8698_LDO6,
	MAX8698_LDO7,
	MAX8698_LDO8,
	MAX8698_LDO9,
	MAX8698_BUCK1,
	MAX8698_BUCK2,
	MAX8698_BUCK3
};

/**
 * max8698_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (contraints, supplies, ...)
 */
struct max8698_regulator_data {
	int				id;
	struct regulator_init_data	*initdata;
};

/**
 * struct max8698_board - packages regulator init data
 * @num_regulators: number of regultors used
 * @regulators: array of defined regulators
 */

struct max8698_platform_data {
	int				num_regulators;
	struct max8698_regulator_data	*regulators;
};

#endif /* _MAX8698_H_ */
