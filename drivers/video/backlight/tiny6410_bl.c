/*
 *  1-wire backlight controller driver for Tiny6410 board.
 *
 *  Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/slab.h>

#include <linux/mfd/tiny6410_1wire.h>

#define TINY6410_BL_REQ		(0x80)

struct tiny6410_1wire_bl {
	struct tiny6410_1wire	*bus;
	struct backlight_device	*bl;
};

static int tiny6410_1wire_bl_update_status(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;
	struct tiny6410_1wire_bl *data = dev_get_drvdata(&dev->dev);
	int power = max(props->power, props->fb_blank);
	int brightness = props->brightness;

	if (power)
		brightness = 0;

	brightness >>= 1;
	brightness &= 0xff;

	tiny6410_1wire_transfer(data->bus, TINY6410_BL_REQ | brightness, 0);

	return 0;
}

static int tiny6410_1wire_bl_get_brightness(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;

	return props->brightness;
}

static const struct backlight_ops bl_ops = {
	.get_brightness		= tiny6410_1wire_bl_get_brightness,
	.update_status		= tiny6410_1wire_bl_update_status,
};

static int __devinit tiny6410_1wire_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct tiny6410_1wire_bl *data;
	int ret = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bus = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, data);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 255;
	props.type = BACKLIGHT_PLATFORM;
	data->bl = backlight_device_register("tiny6410-1wire-bl",
					&pdev->dev, data, &bl_ops, &props);
	if (IS_ERR(data->bl)) {
		ret = PTR_ERR(data->bl);
		goto err_reg;
	}

	data->bl->props.brightness = 255;
	data->bl->props.power = FB_BLANK_UNBLANK;

	backlight_update_status(data->bl);

	return 0;

err_reg:
	data->bl = NULL;
	kfree(data);
	return ret;
}

static int __devexit tiny6410_1wire_bl_remove(struct platform_device *pdev)
{
	struct tiny6410_1wire_bl *data = platform_get_drvdata(pdev);

	backlight_device_unregister(data->bl);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int tiny6410_1wire_bl_suspend(struct platform_device *pdev, pm_message_t pm)
{
	struct tiny6410_1wire_bl *data = platform_get_drvdata(pdev);

	tiny6410_1wire_transfer(data->bus, TINY6410_BL_REQ, 0);

	return 0;
}

static int tiny6410_1wire_bl_resume(struct platform_device *pdev)
{
	struct tiny6410_1wire_bl *data = platform_get_drvdata(pdev);

	backlight_update_status(data->bl);
	return 0;
}
#else
#define tiny6410_1wire_bl_suspend NULL
#define tiny6410_1wire_bl_resume NULL
#endif

static struct platform_driver tiny6410_1wire_bl_driver = {
	.driver = {
		.name	= "tiny6410-1wire-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= tiny6410_1wire_bl_probe,
	.remove		= tiny6410_1wire_bl_remove,
	.suspend	= tiny6410_1wire_bl_suspend,
	.resume		= tiny6410_1wire_bl_resume,
};

static int __init tiny6410_1wire_bl_init(void)
{
	return platform_driver_register(&tiny6410_1wire_bl_driver);
}

static void __exit tiny6410_1wire_bl_exit(void)
{
	platform_driver_unregister(&tiny6410_1wire_bl_driver);
}

module_init(tiny6410_1wire_bl_init);
module_exit(tiny6410_1wire_bl_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Backlight control for Tiny6410");
