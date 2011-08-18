/*
 * include/linux/tiny6410_1wire.h
 *
 * Definition of platform data structure for Tiny6410 1-wire bus driver.
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef TINY6410_1WIRE_H
#define TINY6410_1WIRE_H

struct tiny6410_1wire_platform_data {
	int	gpio_pin;
	void 	(*set_pullup)(int state);
};

#endif	/* TINY6410_1WIRE_H */
