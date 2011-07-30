/* include/video/s6d05a.h
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * Platform data for S6D05A LCD controller.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#ifndef _S6D05A_H_
#define _S6D05A_H_

#define S6D05A_TYPE_CMD		(1 << 8)
#define S6D05A_COMMAND(x)	(S6D05A_TYPE_CMD | (x))
#define S6D05A_TYPE_SLP		(2 << 8)
#define S6D05A_SLEEP(x)		(S6D05A_TYPE_SLP | (x))
#define S6D05A_END		(4 << 8)

#define S6D05A_TYPE(x)		((x) & ~0xff)
#define S6D05A_DATA(x)		((x) & 0xff)

/* SPI command codes */
#define PWRCTL			S6D05A_COMMAND(0xF3)
#define SLPIN			S6D05A_COMMAND(0x10)
#define SLPOUT			S6D05A_COMMAND(0x11)
#define DISCTL			S6D05A_COMMAND(0xF2)
#define VCMCTL			S6D05A_COMMAND(0xF4)
#define SRCCTL			S6D05A_COMMAND(0xF5)
#define GATECTL			S6D05A_COMMAND(0xFD)
#define MADCTL			S6D05A_COMMAND(0x36)
#define COLMOD			S6D05A_COMMAND(0x3A)
#define GAMCTL1			S6D05A_COMMAND(0xF7)
#define GAMCTL2			S6D05A_COMMAND(0xF8)
#define GAMCTL3			S6D05A_COMMAND(0xF9)
#define GAMCTL4			S6D05A_COMMAND(0xFA)
#define GAMCTL5			S6D05A_COMMAND(0xFB)
#define GAMCTL6			S6D05A_COMMAND(0xFC)
#define BCMODE			S6D05A_COMMAND(0xCB)
#define WRCABC			S6D05A_COMMAND(0x55)
#define DCON			S6D05A_COMMAND(0xEF)
#define WRCTRLD			S6D05A_COMMAND(0x53)
#define WRDISBV			S6D05A_COMMAND(0x51)
#define WRCABCMB		S6D05A_COMMAND(0x5E)
#define MIECTL1			S6D05A_COMMAND(0xCA)
#define MIECTL2			S6D05A_COMMAND(0xCC)
#define MIECTL3			S6D05A_COMMAND(0xCD)

/* Platform data */
struct s6d05a_platform_data {
	/* Required: GPIOs */
	unsigned cs_gpio;
	unsigned sck_gpio;
	unsigned sda_gpio;
	unsigned reset_gpio;

	/* Optional: override of power on/off command sequences */
	const u16 *power_on_seq;
	const u16 *power_off_seq;
};

#endif /* _S6D05A_H_ */
