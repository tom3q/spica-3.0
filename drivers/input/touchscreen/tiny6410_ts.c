/*
 * 1-wire touchscreen driver for Tiny6410 board.
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/mfd/tiny6410_1wire.h>

#define TINY6410_1WIRE_TS_DELAY		(HZ / 50)
#define TINY6410_1WIRE_TS_REQ		(0x40)

struct tiny6410_1wire_ts {
	struct delayed_work	work;
	struct workqueue_struct	*workqueue;
	struct tiny6410_1wire	*bus;
	struct input_dev	*input_dev;
	struct device		*dev;

	/* Signed fixed point 16:16 offset */
	s16 base_x;
	s16 base_y;

	/* Signed fixed point 16:16 scale */
	s32 scale_x;
	s32 scale_y;

	/* Screen parameters */
	u16 width;
	u16 height;

	u16 x;
	u16 y;
	bool down;
};

static void tiny6410_1wire_ts_report(struct tiny6410_1wire_ts *ts, u32 data)
{
	u16 x, y;
	bool down;

	y  = (data & 0xf00000) >> 12;
	y |= (data & 0x00ff00) >> 8;

	x  = (data & 0x0f0000) >> 8;
	x |= (data & 0x0000ff) >> 0;

	down = (x != 0xfff) || (y != 0xfff);

	if (x != ts->x || y != ts->y || down != ts->down) {
		ts->x = x;
		ts->y = y;
		ts->down = down;

		x = ((x + ts->base_x) * ts->scale_x) >> 16;
		y = ((y + ts->base_y) * ts->scale_y) >> 16;

		if (x >= ts->width)
			x = ts->width - 1;

		if (y >= ts->height)
			y = ts->height - 1;

		input_report_key(ts->input_dev, BTN_TOUCH, down);
		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
		input_sync(ts->input_dev);
	}
}

static void tiny6410_1wire_ts_workfunc(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct tiny6410_1wire_ts *ts =
		container_of(delayed_work, struct tiny6410_1wire_ts, work);
	u32 rx_data;
	u8 tx_data = TINY6410_1WIRE_TS_REQ;
	int ret;

	ret = tiny6410_1wire_transfer(ts->bus, tx_data, &rx_data);
	if (!ret)
		tiny6410_1wire_ts_report(ts, rx_data);

	schedule_delayed_work(delayed_work, TINY6410_1WIRE_TS_DELAY);
}

static ssize_t calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tiny6410_1wire_ts *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u %d %d %u %d %d\n",
					ts->width, ts->base_x, ts->scale_x,
					ts->height, ts->base_y, ts->scale_y);
}

static ssize_t calibration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tiny6410_1wire_ts *ts = dev_get_drvdata(dev);
	int ret;
	s32 base_x, base_y, scale_x, scale_y;
	u32 width, height;

	ret = sscanf(buf, "%u %d %d %u %d %d",
			&width, &base_x, &scale_x, &height, &base_y, &scale_y);

	if (ret != 6)
		return -EINVAL;

	cancel_delayed_work_sync(&ts->work);

	ts->base_x = base_x;
	ts->base_y = base_y;
	ts->scale_x = scale_x;
	ts->scale_y = scale_y;
	ts->width = width;
	ts->height = height;

	input_set_abs_params(ts->input_dev, ABS_X, 0, width - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, height - 1, 0, 0);

	schedule_delayed_work(&ts->work, TINY6410_1WIRE_TS_DELAY);

	return size;
}

static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR,
					calibration_show, calibration_store);

static struct attribute *tiny6410_ts_attrs[] = {
	&dev_attr_calibration.attr,
	NULL
};

static const struct attribute_group tiny6410_ts_attr_group = {
	.attrs = tiny6410_ts_attrs,
};

static int __devinit tiny6410_1wire_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct tiny6410_1wire_ts *ts;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->bus = dev_get_drvdata(pdev->dev.parent);
	ts->dev = &pdev->dev;
	ts->workqueue = create_singlethread_workqueue("tiny6410_1wire_ts");
	if (!ts->workqueue) {
		ret = -ENOMEM;
		goto err_workqueue;
	}
	INIT_DELAYED_WORK(&ts->work, tiny6410_1wire_ts_workfunc);

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	ts->input_dev->name = "Tiny6410 Touchscreen";
	ts->input_dev->dev.parent = &pdev->dev;
	ts->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* Set calibration data for 1:1 operation */
	ts->base_x = 0;
	ts->base_y = 0;
	ts->scale_x = 1 << 16;
	ts->scale_y = 1 << 16;
	ts->width = 0x1000;
	ts->height = 0x1000;

	/* Configure absolute value ranges appropriately */
	input_set_abs_params(ts->input_dev, ABS_X, 0, 0xfff, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 0xfff, 0, 0);

	platform_set_drvdata(pdev, ts);

	ret = input_register_device(ts->input_dev);
	if (ret)
		goto err_input_register;

	schedule_delayed_work(&ts->work, TINY6410_1WIRE_TS_DELAY);

	if(sysfs_create_group(&pdev->dev.kobj, &tiny6410_ts_attr_group))
		dev_warn(&pdev->dev, "Failed to create sysfs group.\n");

	return 0;

err_input_register:
	input_free_device(ts->input_dev);
err_input_alloc:
	destroy_workqueue(ts->workqueue);
err_workqueue:
	kfree(ts);
	return ret;
}

static int __devexit tiny6410_1wire_ts_remove(struct platform_device *pdev)
{
	struct tiny6410_1wire_ts *ts = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &tiny6410_ts_attr_group);

	destroy_workqueue(ts->workqueue);

	input_get_device(ts->input_dev);
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);

	kfree(ts);

	return 0;
}

static struct platform_driver tiny6410_1wire_ts_driver = {
	.probe = tiny6410_1wire_ts_probe,
	.remove = __devexit_p(tiny6410_1wire_ts_remove),
	.driver = {
		.name = "tiny6410-1wire-touchscreen",
	},
};

static int __init tiny6410_1wire_ts_init(void)
{
	return platform_driver_register(&tiny6410_1wire_ts_driver);
}
module_init(tiny6410_1wire_ts_init);

static void __exit tiny6410_1wire_ts_exit(void)
{
	platform_driver_unregister(&tiny6410_1wire_ts_driver);
}
module_exit(tiny6410_1wire_ts_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("Tiny6410 1-wire Touchscreen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tiny6410-1wire-touchscreen");
