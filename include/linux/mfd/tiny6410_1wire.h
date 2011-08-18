/*
 * include/linux/mfd/tiny6410_1wire.h
 *
 * Some definitions for drivers of devices running on the 1-wire bus
 * of the Tiny6410 board.
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

#ifndef MFD_TINY6410_1WIRE_H
#define MFD_TINY6410_1WIRE_H

#include <linux/mfd/core.h>

struct tiny6410_1wire;

extern int tiny6410_1wire_transfer(struct tiny6410_1wire *bus,
						u8 tx_data, u32 *rx_data);

#endif	/* MFD_TINY6410_1WIRE_H */
