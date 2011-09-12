/*
 * Vibetonz control interface
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vibetonz.h>

#include "../staging/android/timed_output.h"

#define VIBRATOR_DEF_HZ		22222	// 128 * actuator resonant frequency
#define VIBRATOR_DEF_DUTY	1	// strong vibrations (use 33 for weak)
#define VIBRATOR_MAX_TIMEOUT	5000

/*
 * Data structures
 */
struct vibetonz {
	int period;
	int duty;
	struct pwm_device *pwm;
	struct hrtimer timer;
	struct timed_output_dev timed_output;
	struct mutex mutex;
	struct vibetonz_platform_data *pdata;
};

/*
 * Hardware control
 */
static void vibetonz_start(struct vibetonz *vib, int duty)
{
	pwm_config(vib->pwm, (duty*vib->period) / 100, vib->period);
	pwm_enable(vib->pwm);
	gpio_set_value(vib->pdata->gpio_en, 1);
}

static void vibetonz_stop(struct vibetonz *vib)
{
	gpio_set_value(vib->pdata->gpio_en, 0);
	pwm_disable(vib->pwm);
}

static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{
	struct vibetonz *vib = container_of(timer, struct vibetonz, timer);

	vibetonz_stop(vib);

	return HRTIMER_NORESTART;
}

/*
 * Timed output
 */
static int get_time_for_vibetonz(struct timed_output_dev *tdev)
{
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);

	if (!hrtimer_active(&vib->timer))
		return 0;

	return ktime_to_ms(hrtimer_get_remaining(&vib->timer));
}

static void enable_vibetonz_from_user(struct timed_output_dev *tdev, int value)
{
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);
	int timeout, duty;

	duty = value >> 16;
	timeout = value & 0xffff;

	mutex_lock(&vib->mutex);

	if (duty <= 0 || duty > 100)
		duty = vib->duty;

	if (timeout > VIBRATOR_MAX_TIMEOUT)
		timeout = VIBRATOR_MAX_TIMEOUT;

	hrtimer_cancel(&vib->timer);

	if (!timeout) {
		vibetonz_stop(vib);
	} else {
		ktime_t time = ktime_set(timeout / MSEC_PER_SEC,
				(timeout % MSEC_PER_SEC) * NSEC_PER_MSEC);
		vibetonz_start(vib, duty);
		hrtimer_start(&vib->timer, time, HRTIMER_MODE_REL);
	}

	mutex_unlock(&vib->mutex);
}

/*
 * Frequency
 */
static ssize_t freq_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t len)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);
	int pwm_period;

	if(sscanf(buf, "%d", &pwm_period) <= 0)
		return -EINVAL;

	vib->period = NSEC_PER_SEC / pwm_period;

	enable_vibetonz_from_user(&vib->timed_output, 1000);

	return len;
}

static ssize_t freq_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);

	return sprintf(buf, "%ld\n", NSEC_PER_SEC / vib->period);
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, freq_show, freq_store);

/*
 * Duty
 */
static ssize_t duty_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t len)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);
	int pwm_duty;

	if(sscanf(buf, "%d", &pwm_duty) <= 0)
		return -EINVAL;

	if (pwm_duty < 1)
		pwm_duty = 1;
	if (pwm_duty > 100)
		pwm_duty = 100;

	vib->duty = pwm_duty;

	return len;
}

static ssize_t duty_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct vibetonz *vib =
			container_of(tdev, struct vibetonz, timed_output);

	return sprintf(buf, "%d\n", vib->duty);
}

static DEVICE_ATTR(duty, S_IRUGO | S_IWUSR, duty_show, duty_store);

/*
 * Platform driver
 */
static int __init vibetonz_probe(struct platform_device *pdev)
{
	struct vibetonz *vib;
	struct vibetonz_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "Only a single instance is allowed.\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data specified.\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_en)) {
		dev_err(&pdev->dev, "Invalid GPIO pin specified.\n");
		return -EINVAL;
	}

	vib = kzalloc(sizeof(struct vibetonz), GFP_KERNEL);
	if (!vib) {
		dev_err(&pdev->dev, "Failed to allocate driver data.\n");
		return -ENOMEM;
	}

	mutex_init(&vib->mutex);
	hrtimer_init(&vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->timer.function		= vibetonz_timer_func;
	vib->duty			= VIBRATOR_DEF_DUTY;
	vib->period 			= (NSEC_PER_SEC / VIBRATOR_DEF_HZ);
	vib->timed_output.name		= "vibrator",
	vib->timed_output.get_time	= get_time_for_vibetonz,
	vib->timed_output.enable	= enable_vibetonz_from_user,
	vib->pdata			= pdata;

	vib->pwm = pwm_request(pdata->pwm_chan, "Vibetonz");
	if (IS_ERR(vib->pwm)) {
		dev_err(&pdev->dev, "Failed to request PWM timer %d (%ld).\n",
					pdata->pwm_chan, PTR_ERR(vib->pwm));
		ret = PTR_ERR(vib->pwm);
		goto err_pwm;
	}

	ret = gpio_request(pdata->gpio_en, "Vibetonz");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request GPIO_VIB_EN.\n");
		goto err_gpio;
	}
	gpio_direction_output(pdata->gpio_en, 0);

	vibetonz_start(vib, vib->duty);
	msleep(50);
	vibetonz_stop(vib);

	platform_set_drvdata(pdev, vib);

	ret = timed_output_dev_register(&vib->timed_output);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register timed output device.\n");
		goto err_timed;
	}

	ret = device_create_file(vib->timed_output.dev, &dev_attr_freq);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create freq attribute.\n");
		goto err_freq;
	}

	ret = device_create_file(vib->timed_output.dev, &dev_attr_duty);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create duty attribute.\n");
		goto err_duty;
	}

	return 0;

err_duty:
	device_remove_file(vib->timed_output.dev, &dev_attr_freq);
err_freq:
	timed_output_dev_unregister(&vib->timed_output);
err_timed:
	gpio_free(pdata->gpio_en);
err_gpio:
	pwm_free(vib->pwm);
err_pwm:
	kfree(vib);

	return ret;
}

static int __devexit vibetonz_remove(struct platform_device *pdev)
{
	struct vibetonz *vib = platform_get_drvdata(pdev);

	device_remove_file(vib->timed_output.dev, &dev_attr_duty);
	device_remove_file(vib->timed_output.dev, &dev_attr_freq);
	timed_output_dev_unregister(&vib->timed_output);
	vibetonz_stop(vib);
	gpio_free(vib->pdata->gpio_en);
	pwm_free(vib->pwm);
	kfree(vib);

	return 0;
}

static void vibetonz_shutdown(struct platform_device *pdev)
{
	struct vibetonz *vib = platform_get_drvdata(pdev);

	vibetonz_start(vib, vib->duty);
	msleep(50);
	vibetonz_stop(vib);
}

static int vibetonz_suspend(struct device *dev)
{
	struct vibetonz *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->timer);
	vibetonz_stop(vib);

	return 0;
}

static struct dev_pm_ops vibetonz_pm_ops = {
	.suspend	= vibetonz_suspend,
};

static struct platform_driver vibetonz_driver = {
	.remove		= __devexit_p(vibetonz_remove),
	.shutdown	= vibetonz_shutdown,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= "vibetonz",
		.pm	= &vibetonz_pm_ops,
	},
};

/*
 * Kernel module
 */
static int __init vibetonz_init(void)
{
	return platform_driver_probe(&vibetonz_driver, vibetonz_probe);
}

static void __exit vibetonz_exit(void)
{
	platform_driver_unregister(&vibetonz_driver);
}

module_init(vibetonz_init);
module_exit(vibetonz_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("vibetonz control interface");
