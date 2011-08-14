/*
 * Bluetooth power control for Samsung GT-i5700 (Spica)
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spica_bt.h>
#include <linux/slab.h>

struct spica_bt {
	struct rfkill		*rfk;
	struct spica_bt_pdata	*pdata;
	struct device		*dev;
	struct wake_lock	wake_lock;
	unsigned int		irq;
};

irqreturn_t spica_bt_host_wake_irq(int irq, void *dev_id)
{
	struct spica_bt *bt = dev_id;

	if (gpio_get_value(bt->pdata->gpio_host_wake))
		wake_lock(&bt->wake_lock);
	else
		wake_lock_timeout(&bt->wake_lock, HZ);

	return IRQ_HANDLED;
}

static int spica_bt_set_block(void *data, bool blocked)
{
	struct spica_bt *bt = data;
	struct spica_bt_pdata *pdata = bt->pdata;

	if (blocked) {
		dev_dbg(bt->dev, "power off\n");

		disable_irq(bt->irq);
		WARN_ON(disable_irq_wake(bt->irq) < 0);

		if (pdata->set_power)
			pdata->set_power(0);

		wake_unlock(&bt->wake_lock);
	} else {
		dev_dbg(bt->dev, "power on\n");
		
		if (pdata->set_power)
			pdata->set_power(1);

		WARN_ON(enable_irq_wake(bt->irq) < 0);
		enable_irq(bt->irq);
	}

	return 0;
}

static const struct rfkill_ops spica_bt_ops = {
	.set_block = spica_bt_set_block,
};

static int spica_bt_probe(struct platform_device *pdev)
{
	struct spica_bt_pdata *pdata = pdev->dev.platform_data;
	struct spica_bt *bt;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data specified\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_host_wake)) {
		dev_err(&pdev->dev, "invalid BT host wake GPIO\n");
		return -EINVAL;
	}

	bt = kzalloc(sizeof(struct spica_bt), GFP_KERNEL);
	if (!bt) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	bt->pdata = pdata;
	bt->dev = &pdev->dev;
	wake_lock_init(&bt->wake_lock, WAKE_LOCK_SUSPEND, "bt_host_wake");

	WARN_ON(gpio_request(pdata->gpio_host_wake, "BT host wake") < 0);

	ret = gpio_to_irq(pdata->gpio_host_wake);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get host wake IRQ number\n");
		goto err_gpio_irq;
	}
	bt->irq = ret;

	ret = request_irq(bt->irq, spica_bt_host_wake_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"BT host wake", bt);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request bt host wake IRQ\n");
		goto err_irq;
	}

	disable_irq(bt->irq);

	bt->rfk = rfkill_alloc("bcm4329", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &spica_bt_ops, bt);
	if (IS_ERR(bt->rfk)) {
		dev_err(&pdev->dev, "rfkill_alloc failed\n");
		ret = PTR_ERR(bt->rfk);
		goto err_alloc;
	}

	rfkill_init_sw_state(bt->rfk, true);

	ret = rfkill_register(bt->rfk);
	if (ret) {
		dev_err(&pdev->dev, "rfkill_register failed (%d)\n", ret);
		goto err_register;
	}

	return 0;

err_register:
	rfkill_destroy(bt->rfk);
err_alloc:
	free_irq(bt->irq, bt);
err_irq:
	gpio_free(pdata->gpio_host_wake);
err_gpio_irq:
	kfree(bt);
	return ret;
}

static struct platform_driver spica_device_rfkill = {
	.probe = spica_bt_probe,
	.driver = {
		.name = "spica_bt",
		.owner = THIS_MODULE,
	},
};

static int __init spica_bt_init(void)
{
	return platform_driver_probe(&spica_device_rfkill, spica_bt_probe);
}

module_init(spica_bt_init);
MODULE_DESCRIPTION("Spica Bluetooth");
MODULE_LICENSE("GPL");
