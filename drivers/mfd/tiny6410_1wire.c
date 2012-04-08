/*
 * mini6410_1wire_host.c
 *
 * LCD-CPU one wire communication for Tiny6410
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/completion.h>
#include <linux/gpio.h>

#include <linux/tiny6410_1wire.h>
#include <linux/mfd/tiny6410_1wire.h>

#define TINY6410_BUS_CLOCK	(9600)
#define TINY6410_1WIRE_DELAY	((NSEC_PER_SEC / TINY6410_BUS_CLOCK) - 100)

/*
 * Driver data
 */

enum tiny6410_1wire_state {
	TINY6410_1WIRE_STOP = 0,
	TINY6410_1WIRE_PREPARE,
	TINY6410_1WIRE_RESET,
	TINY6410_1WIRE_TX,
	TINY6410_1WIRE_WAIT1,
	TINY6410_1WIRE_WAIT2,
	TINY6410_1WIRE_RX,
};

struct tiny6410_1wire {
	struct mutex			lock;
	struct completion		completion;
	struct hrtimer			timer;
	enum tiny6410_1wire_state	state;
	struct device			*dev;

	u16		tx_data;
	volatile u32	rx_data;
	volatile int	error;
	int		bits_left;

	struct tiny6410_1wire_platform_data	*pdata;
};

/*
 * CRC 8
 */

static const unsigned char crc8_tab[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
};

/*
 * 1-wire I/O
 */

int tiny6410_1wire_transfer(struct tiny6410_1wire *bus,
						u8 tx_data, u32 *rx_data)
{
	int ret;
	u32 rx;
	u8 crc;
	int retry = 0;
	ktime_t time;
	unsigned long flags;

	/* Calculate CRC8 checksum */
	crc = crc8_tab[0xac ^ tx_data];

restart:
	/* Lock the bus */
	mutex_lock(&bus->lock);

	/* Prepare the transfer */
	bus->error = 0;
	bus->tx_data = (tx_data << 8) | crc;
	bus->state = TINY6410_1WIRE_PREPARE;

	local_irq_save(flags);

	/* Schedule the timer */
	time = ktime_add_ns(ktime_get(), TINY6410_1WIRE_DELAY);
	hrtimer_start(&bus->timer, time, HRTIMER_MODE_ABS);

	local_irq_restore(flags);

	/* Wait for the transfer to finish */
	ret = wait_for_completion_interruptible(&bus->completion);
	if (!ret)
		ret = bus->error;
	rx = bus->rx_data;

	/* Unlock the bus */
	mutex_unlock(&bus->lock);

	/* Check CRC8 checksum of received data */
	if (!ret && rx_data) {
		*rx_data = rx >> 8;
		crc = crc8_tab[0xac ^ ((rx >> 24) & 0xff)];
		crc = crc8_tab[crc ^ ((rx >> 16) & 0xff)];
		crc = crc8_tab[crc ^ ((rx >> 8) & 0xff)];
		if (crc != (rx & 0xff)) {
			ret = -EBADMSG;
			dev_err(bus->dev, "CRC error in received data (%02x != %02x)\n",
							crc, rx & 0xff);
		}
	}

	if (ret && ++retry <= 3)
		goto restart;

	return ret;
}
EXPORT_SYMBOL(tiny6410_1wire_transfer);

static enum hrtimer_restart tiny6410_1wire_timer(struct hrtimer *timer)
{
	struct tiny6410_1wire *bus = container_of(timer,
						struct tiny6410_1wire, timer);
	struct tiny6410_1wire_platform_data *pdata = bus->pdata;
	ktime_t delay = ktime_set(0, TINY6410_1WIRE_DELAY);
	int ret;

	switch(bus->state) {
	case TINY6410_1WIRE_PREPARE:
		/* Sync with the clock */
		delay = ktime_add(hrtimer_cb_get_time(&bus->timer), delay);
		gpio_direction_output(bus->pdata->gpio_pin, 0);
		hrtimer_set_expires(&bus->timer, delay);
		bus->state = TINY6410_1WIRE_RESET;
		return HRTIMER_RESTART;

	case TINY6410_1WIRE_RESET:
		/* Start transfer */
		bus->bits_left = 16;
		bus->state = TINY6410_1WIRE_TX;
		break;

	case TINY6410_1WIRE_TX:
		/* Send a bit */
		gpio_set_value(pdata->gpio_pin, bus->tx_data & 0x8000);
		bus->tx_data <<= 1;
		if (--bus->bits_left == 0)
			bus->state = TINY6410_1WIRE_WAIT1;
		break;

	case TINY6410_1WIRE_WAIT1:
		/* Wait state */
		bus->state = TINY6410_1WIRE_WAIT2;
		break;

	case TINY6410_1WIRE_WAIT2:
		/* Wait state */
		gpio_direction_input(pdata->gpio_pin);
		bus->bits_left = 32;
		bus->state = TINY6410_1WIRE_RX;
		break;

	case TINY6410_1WIRE_RX:
		/* Receive a bit */
		bus->rx_data <<= 1;
		bus->rx_data |= gpio_get_value(pdata->gpio_pin);
		if (--bus->bits_left == 0) {
			bus->bits_left = 2;
			bus->state = TINY6410_1WIRE_STOP;
		}
		break;

	case TINY6410_1WIRE_STOP:
		/* Stop condition */
		gpio_direction_output(pdata->gpio_pin, 1);
		if (--bus->bits_left == 0) {
			complete(&bus->completion);
			return HRTIMER_NORESTART;
		}
		break;
	}

	ret = hrtimer_forward_now(&bus->timer, delay);
	if (ret > 1) {
		gpio_direction_output(pdata->gpio_pin, 1);
		bus->state = TINY6410_1WIRE_STOP;
		bus->error = -ETIMEDOUT;
		complete(&bus->completion);
		return HRTIMER_NORESTART;
	}

	return HRTIMER_RESTART;
}

/*
 * Platform bus
 */

static struct mfd_cell tiny6410_1wire_cells[] = {
	{
		.name = "tiny6410-1wire-backlight",
	},
	{
		.name = "tiny6410-1wire-touchscreen",
	},
};

static int tiny6410_1wire_probe(struct platform_device *pdev)
{
	struct tiny6410_1wire_platform_data *pdata = pdev->dev.platform_data;
	struct tiny6410_1wire *data;
	int ret;

	if (!pdata || !pdata->set_pullup || !gpio_is_valid(pdata->gpio_pin)) {
		dev_err(&pdev->dev, "Invalid platform data.\n");
		return -EINVAL;
	}

	ret = gpio_request(pdata->gpio_pin, "Tiny6410 1-wire");
	if (ret) {
		dev_err(&pdev->dev, "Could not request 1-wire GPIO.\n");
		return ret;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Could not allocate driver data.\n");
		gpio_free(pdata->gpio_pin);
		return -ENOMEM;
	}

	data->pdata = pdata;
	data->dev = &pdev->dev;
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	mutex_init(&data->lock);
	init_completion(&data->completion);
	data->timer.function = tiny6410_1wire_timer;
	platform_set_drvdata(pdev, data);

	gpio_direction_output(pdata->gpio_pin, 1);
	pdata->set_pullup(1);

	ret =  mfd_add_devices(&pdev->dev, -1, tiny6410_1wire_cells,
					ARRAY_SIZE(tiny6410_1wire_cells), 0, 0);

	dev_info(&pdev->dev, "Tiny6410 1-wire host initialized.\n");

	return ret;
}


static int tiny6410_1wire_remove(struct platform_device *pdev)
{
	struct tiny6410_1wire *data = platform_get_drvdata(pdev);

	hrtimer_cancel(&data->timer);

	mfd_remove_devices(&pdev->dev);

	gpio_set_value(data->pdata->gpio_pin, 1);
	data->pdata->set_pullup(0);
	gpio_set_value(data->pdata->gpio_pin, 0);

	gpio_free(data->pdata->gpio_pin);
	kfree(data);

	return 0;
}

static struct platform_driver tiny6410_1wire_driver = {
	.driver = {
		.name	= "tiny6410-1wire",
		.owner = THIS_MODULE,
	},
	.probe	= tiny6410_1wire_probe,
	.remove	= tiny6410_1wire_remove,
};

/*
 * Module init
 */

static int tiny6410_1wire_init(void)
{
	return platform_driver_register(&tiny6410_1wire_driver);
}
module_init(tiny6410_1wire_init);

static void tiny6410_1wire_exit(void)
{
	platform_driver_unregister(&tiny6410_1wire_driver);
}
module_exit(tiny6410_1wire_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("Tiny6410 virtual 1-wire bus");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tiny6410-1wire");
