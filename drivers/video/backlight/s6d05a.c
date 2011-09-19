/*
 * drivers/video/backlight/s6d05a.c
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>

#include <video/s6d05a.h>

#define S6D05A_SPI_DELAY_USECS		5

/*
 * Driver data
 */
struct s6d05a_data {
	struct backlight_device	*bl;
	struct device *dev;

	unsigned cs_gpio;
	unsigned sck_gpio;
	unsigned sda_gpio;
	unsigned reset_gpio;

	struct regulator *vdd3;
	struct regulator *vci;

	int state;
	int brightness;

	const u16 *power_on_seq;
	const u16 *power_off_seq;
};

/*
 * Power on command sequence
 */
static const u16 s6d05a_power_on_seq[] = {
	PWRCTL,		0x00, 0x00, 0x2A, 0x00, 0x00, 0x33, 0x29, 0x29,
	SLPOUT,
	S6D05A_SLEEP(15),

	DISCTL,		0x16, 0x16, 0x0F, 0x0A, 0x05, 0x0A, 0x05, 0x10, 0x00,
			0x16, 0x16,
	PWRCTL,		0x00, 0x01, 0x2A, 0x00, 0x00, 0x33, 0x29, 0x29, 0x00,
	VCMCTL,		0x1A, 0x1A, 0x18, 0x18, 0x00,
	SRCCTL,		0x00, 0x00, 0x0A, 0x01, 0x01, 0x1D,
	GATECTL,	0x44, 0x3B, 0x00,
	S6D05A_SLEEP(15),

	PWRCTL,		0x00, 0x03, 0x2A, 0x00, 0x00, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(15),

	PWRCTL,		0x00, 0x07, 0x2A, 0x00, 0x00, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(15),

	PWRCTL,		0x00, 0x0F, 0x2A, 0x00, 0x02, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(15),

	PWRCTL,		0x00, 0x1F, 0x2A, 0x00, 0x02, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(15),

	PWRCTL,		0x00, 0x3F, 0x2A, 0x00, 0x08, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(25),

	PWRCTL,		0x00, 0x7F, 0x2A, 0x00, 0x08, 0x33, 0x29, 0x29, 0x00,
	S6D05A_SLEEP(35),

	MADCTL,		0x98,
	COLMOD,		0x66,
	GAMCTL1,	0x00, 0x00, 0x00, 0x14, 0x27, 0x2D, 0x2C, 0x2D, 0x10,
			0x11, 0x10, 0x16, 0x04, 0x22, 0x22,
	GAMCTL2,	0x00, 0x00, 0x00, 0x14, 0x27, 0x2D, 0x2C, 0x2D, 0x10,
			0x11, 0x10, 0x16, 0x04, 0x22, 0x22,
	GAMCTL3,	0x96, 0x00, 0x00, 0x00, 0x00, 0x15, 0x1E, 0x23, 0x16,
			0x0D, 0x07, 0x10, 0x00, 0x81, 0x42,
	GAMCTL4,	0x80, 0x16, 0x00, 0x00, 0x00, 0x15, 0x1E, 0x23, 0x16,
			0x0D, 0x07, 0x10, 0x00, 0x81, 0x42,
	GAMCTL5,	0x00, 0x00, 0x34, 0x30, 0x2F, 0x2F, 0x2E, 0x2F, 0x0E,
			0x0D, 0x09, 0x0E, 0x00, 0x22, 0x12,
	GAMCTL6,	0x00, 0x00, 0x34, 0x30, 0x2F, 0x2F, 0x2E, 0x2F, 0x0E,
			0x0D, 0x09, 0x0E, 0x00, 0x22, 0x12,
	BCMODE,		0x01,
	MIECTL3,	0x7C, 0x01,
	WRDISBV,	0x00,
	DCON,		0x06,
	S6D05A_SLEEP(40),

	DCON,		0x07,
	WRCTRLD,	0x2C,

	S6D05A_END,
};

/*
 * Power off command sequence
 */
static const u16 s6d05a_power_off_seq[] = {
	WRDISBV,	0x00,
	S6D05A_SLEEP(250),

	DCON,		0x06,
	S6D05A_SLEEP(40),

	DCON,		0x00,
	S6D05A_SLEEP(25),

	PWRCTL,		0x00, 0x00, 0x2A, 0x00, 0x00, 0x33, 0x29, 0x29, 0x00,
	SLPIN,
	S6D05A_SLEEP(200),

	S6D05A_END,
};

/*
 * Hardware interface
 */
static inline void s6d05a_send_word(struct s6d05a_data *data, u16 word)
{
	unsigned mask = 1 << 8;

	gpio_set_value(data->sck_gpio, 1);
	udelay(S6D05A_SPI_DELAY_USECS);

	gpio_set_value(data->cs_gpio, 0);
	udelay(S6D05A_SPI_DELAY_USECS);

	do {
		gpio_set_value(data->sck_gpio, 0);
		udelay(S6D05A_SPI_DELAY_USECS);

		gpio_set_value(data->sda_gpio, word & mask);
		udelay(S6D05A_SPI_DELAY_USECS);

		gpio_set_value(data->sck_gpio, 1);
		udelay(S6D05A_SPI_DELAY_USECS);

		mask >>= 1;
	} while (mask);

	gpio_set_value(data->cs_gpio, 1);
	udelay(S6D05A_SPI_DELAY_USECS);

	gpio_set_value(data->sck_gpio, 0);
}

static void s6d05a_send_command_seq(struct s6d05a_data *data, const u16 *cmd)
{
	while (*cmd != S6D05A_END) {
		u16 word = *cmd++;
		switch (S6D05A_TYPE(word)) {
		case S6D05A_TYPE_CMD:
			s6d05a_send_word(data, S6D05A_DATA(word));
			break;
		case S6D05A_TYPE_SLP:
			msleep(S6D05A_DATA(word));
			break;
		default:
			s6d05a_send_word(data, 0x100 | S6D05A_DATA(word));
		}
	};
}

/*
 * Backlight interface
 */
static void s6d05a_set_power(struct s6d05a_data *data, int power)
{
	if (data->state == power)
		return;

	if (power) {
		pm_runtime_get_sync(data->dev);

		gpio_set_value(data->cs_gpio, 1);

		gpio_set_value(data->reset_gpio, 0);
		udelay(15);

		regulator_enable(data->vdd3);
		udelay(15);

		regulator_enable(data->vci);
		udelay(15);

		gpio_set_value(data->reset_gpio, 1);

		msleep(10);

		s6d05a_send_command_seq(data, data->power_on_seq);
	} else {
		s6d05a_send_command_seq(data, data->power_off_seq);

		gpio_set_value(data->reset_gpio, 0);

		regulator_disable(data->vci);

		regulator_disable(data->vdd3);

		gpio_set_value(data->cs_gpio, 0);
		gpio_set_value(data->sck_gpio, 0);

		pm_runtime_put_sync(data->dev);
	}

	data->state = power;
}

static void s6d05a_set_backlight(struct s6d05a_data *data, u8 value)
{
	const u16 cmd[] = { WRDISBV, value, S6D05A_END };

	s6d05a_send_command_seq(data, cmd);
}

static int s6d05a_bl_update_status(struct backlight_device *bl)
{
	struct s6d05a_data *data = bl_get_data(bl);
	int new_state = 1;

	if (!bl->props.brightness || bl->props.power != FB_BLANK_UNBLANK
					|| bl->props.state & BL_CORE_FBBLANK
					|| bl->props.state & BL_CORE_SUSPENDED)
		new_state = 0;

	data->brightness = bl->props.brightness;

	s6d05a_set_power(data, new_state);

	if (new_state)
		s6d05a_set_backlight(data, data->brightness);

	return 0;
}

static int s6d05a_bl_get_brightness(struct backlight_device *bl)
{
	struct s6d05a_data *data = bl_get_data(bl);

	if (!data->state)
		return 0;

	return data->brightness;
}

static int s6d05a_bl_check_fb(struct backlight_device *bl, struct fb_info *info)
{
	struct s6d05a_data *data = bl_get_data(bl);

	if (data->dev->parent == NULL)
		return 1;

	return data->dev->parent == info->device;
}

static struct backlight_ops s6d05a_bl_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= s6d05a_bl_update_status,
	.get_brightness	= s6d05a_bl_get_brightness,
	.check_fb	= s6d05a_bl_check_fb,
};

/*
 * Platform driver
 */
static int __devinit s6d05a_probe(struct platform_device *pdev)
{
	struct s6d05a_data *data;
	struct s6d05a_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	if (!pdata)
		return -ENOENT;

	ret = gpio_request(pdata->reset_gpio, "s6d05a reset");
	if (ret)
		return ret;

	ret = gpio_direction_output(pdata->reset_gpio, 0);
	if (ret)
		goto err_reset;

	ret = gpio_request(pdata->cs_gpio, "s6d05a cs");
	if (ret)
		goto err_reset;

	ret = gpio_direction_output(pdata->cs_gpio, 0);
	if (ret)
		goto err_cs;

	ret = gpio_request(pdata->sck_gpio, "s6d05a sck");
	if (ret)
		goto err_cs;

	ret = gpio_direction_output(pdata->sck_gpio, 0);
	if (ret)
		goto err_sck;

	ret = gpio_request(pdata->sda_gpio, "s6d05a sda");
	if (ret)
		goto err_sck;

	ret = gpio_direction_output(pdata->sda_gpio, 0);
	if (ret)
		goto err_sda;

	data = kzalloc(sizeof(struct s6d05a_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&pdev->dev, "No memory for device state\n");
		ret = -ENOMEM;
		goto err_sda;
	}

	data->dev = &pdev->dev;
	data->reset_gpio = pdata->reset_gpio;
	data->cs_gpio = pdata->cs_gpio;
	data->sck_gpio = pdata->sck_gpio;
	data->sda_gpio = pdata->sda_gpio;

	if (pdata->power_on_seq)
		data->power_on_seq = pdata->power_on_seq;
	else
		data->power_on_seq = s6d05a_power_on_seq;

	if (pdata->power_off_seq)
		data->power_off_seq = pdata->power_off_seq;
	else
		data->power_off_seq = s6d05a_power_off_seq;

	data->vci = regulator_get(&pdev->dev, "vci");
	if (IS_ERR(data->vci)) {
		dev_err(&pdev->dev, "Failed to get vci regulator\n");
		ret = PTR_ERR(data->vci);
		goto err_free;
	}

	data->vdd3 = regulator_get(&pdev->dev, "vdd3");
	if (IS_ERR(data->vdd3)) {
		dev_err(&pdev->dev, "Failed to get vdd3 regulator\n");
		ret = PTR_ERR(data->vdd3);
		goto err_vci;
	}

	platform_set_drvdata(pdev, data);

	pm_runtime_enable(data->dev);
	pm_runtime_no_callbacks(data->dev);

	data->bl = backlight_device_register(dev_driver_string(&pdev->dev),
			&pdev->dev, data, &s6d05a_bl_ops, NULL);
	if (IS_ERR(data->bl)) {
		dev_err(&pdev->dev, "Failed to register backlight device\n");
		ret = PTR_ERR(data->bl);
		goto err_vdd3;
	}

	data->bl->props.max_brightness	= 255;
	data->bl->props.brightness	= 128;
	data->bl->props.type		= BACKLIGHT_RAW;
	data->bl->props.power		= FB_BLANK_UNBLANK;
	backlight_update_status(data->bl);

	return 0;

err_vdd3:
	regulator_put(data->vdd3);
err_vci:
	regulator_put(data->vci);
err_free:
	kfree(data);
err_sda:
	gpio_free(pdata->sda_gpio);
err_sck:
	gpio_free(pdata->sck_gpio);
err_cs:
	gpio_free(pdata->cs_gpio);
err_reset:
	gpio_free(pdata->reset_gpio);

	return ret;
}

static int __devexit s6d05a_remove(struct platform_device *pdev)
{
	struct s6d05a_data *data = platform_get_drvdata(pdev);

	s6d05a_set_power(data, 0);

	backlight_device_unregister(data->bl);

	gpio_free(data->reset_gpio);
	gpio_free(data->cs_gpio);
	gpio_free(data->sck_gpio);
	gpio_free(data->sda_gpio);

	regulator_put(data->vci);
	regulator_put(data->vdd3);

	kfree(data);

	return 0;
}

static int s6d05a_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s6d05a_data *data = platform_get_drvdata(pdev);

	s6d05a_set_power(data, 0);

	return 0;
}

static int s6d05a_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct s6d05a_data *data = platform_get_drvdata(pdev);

	s6d05a_bl_update_status(data->bl);

	return 0;
}

static struct dev_pm_ops s6d05a_pm_ops = {
	.suspend	= s6d05a_suspend,
	.resume		= s6d05a_resume,
};

static void s6d05a_shutdown(struct platform_device *pdev)
{
	struct s6d05a_data *data = platform_get_drvdata(pdev);

	s6d05a_set_power(data, 0);
}

static struct platform_driver s6d05a_driver = {
	.driver = {
		.name	= "s6d05a-lcd",
		.owner	= THIS_MODULE,
		.pm	= &s6d05a_pm_ops,
	},
	.probe		= s6d05a_probe,
	.remove		= s6d05a_remove,
	.shutdown	= s6d05a_shutdown,
};

/*
 * Module
 */
static int __init s6d05a_init(void)
{
	return platform_driver_register(&s6d05a_driver);
}
module_init(s6d05a_init);

static void __exit s6d05a_exit(void)
{
	platform_driver_unregister(&s6d05a_driver);
}
module_exit(s6d05a_exit);

MODULE_DESCRIPTION("S6D05A LCD Controller Driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPL");
