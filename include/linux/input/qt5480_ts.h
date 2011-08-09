/*
 * include/linux/input/qt5480_ts.h
 *
 * Platform data structure for AT42QT5480 touchscreen.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __LINUX_INPUT_QT5480_H__
#define __LINUX_INPUT_QT5480_H__

struct qt5480_platform_data {
	int rst_gpio;
	int rst_inverted;

	int en_gpio;
	int en_inverted;
};

#endif
