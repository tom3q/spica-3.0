/* drivers/input/touchscreen/qt5480_ts.c
 *
 * Input driver for AT42QT5480 touchscreen.
 *
 * Original driver:
 * 	Copyright (C) 2009 Samsung Electronics Co. Ltd.
 * Multitouch:
 * 	Copyright (C) 2010 Lambertus Gorter <l.gorter@gmail.com>
 * Driver cleanup and optimization:
 * 	Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>

#include <linux/input/qt5480_ts.h>

#include "qt5480_ts.h"

/*
 * Debugging macros
 */

//#define QT5480_DEBUG
//#define QT5480_DEBUG_I2C
//#define QT5480_DEBUG_DEV

/* General debug */
#ifdef QT5480_DEBUG
	#define DBG(p, ...)	\
		printk("[QT5480]:[%s] " p, __func__, ##__VA_ARGS__)
#else
	#define DBG(p, ...)
#endif

/* I2C debug */
#ifdef QT5480_DEBUG_I2C
	#define DBG_I2C(p, ...)	\
		printk("[QT5480]:I2C:[%s] " p, __func__, ##__VA_ARGS__)
#else
	#define DBG_I2C(p, ...)
#endif

/* Input device debug */
#ifdef QT5480_DEBUG_DEV
	#define DBG_DEV(p, ...)	\
		printk("[QT5480]:DEV:[%s] " p, __func__, ##__VA_ARGS__)
#else
	#define DBG_DEV(p, ...)
#endif

/*
 * Helper definitions
 */

#define QT5480_I2C_ADDR			0x60
#define QT5480_BL_I2C_ADDR		0x4A
#define QT5480_CHIP_ID			0x0F
#define QT5480_MAX_XC			1023
#define QT5480_MAX_YC			1023
#define QT5480_MAX_ZC			63
#define QT5480_MAX_WIDTH		63
#define QT5480_MAX_TOUCH_ID		1
#define QT5480_RESET_TIME		100
#define QT5480_RESET_ASSERT_TIME	10
#define QT5480_MIN_FW_VERSION		0x5502

/*
 * QT5480 only be able to send a data packet consists of five bytes
 */

#define BASIC_READ_COUNT	5

/*
 * Driver data
 */

#define SINGLETOUCH_FLAG	0x01
#define MULTITOUCH_FLAG		0x02

struct qt5480_report {
	int contact;	//finger on screen
	int pos_x;	//x coordinate
	int pos_y;	//y coordinate
	int pressure;	//pressure
	int width;	//touch width
};

struct qt5480_touch {
	int ready;		//new data ready
	struct qt5480_report curr;	//current status (to be reported)
	struct qt5480_report prev;	//last reported status
};

struct qt5480 {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	struct qt5480_touch		touch[2];
	struct qt5480_platform_data	*pdata;
	struct device			*dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */

	int irq;

	int first;
	int ignore;
	int palm_touch;
	int reset_gpio;
	int charger_gpio;

	u8 *config;
};

struct qt5480_ctrl_word {
	u8 class;
	u8 data[4];
};

/*
 * Helper routines
 */

static u16 qt5480_calc_crc16(u8 val, u16 prev)
{
	const u32 crc_poly = 0x00008005;
	int i;
	u32 result;

	result = (prev << 8) | val;

	for(i = 0; i < 8; ++i) {
		if((result <<= 1) & 0x1000000) {
			result ^= (crc_poly << 8);
		}
	}

	return (result >> 8) & 0xffff;
}

static u16 qt5480_calc_cfg_chksum(u8 *data)
{
	u32 crc_val = 0;
	int length = QT5480_CONFIG_REGS;

	do {
		crc_val = qt5480_calc_crc16(*(data++), crc_val);
	} while (--length);

	return crc_val & 0xffff;
}

/*
 * I2C device access
 */

static inline int qt5480_i2c_write(struct qt5480 *qt, u16 reg, u8 data)
{
	u8 wbuf[3] = { reg & 0xFF, reg >> 8, data };

	DBG_I2C("Wrote %02x to register %04x\n", data, reg);

	return i2c_master_send(qt->client, wbuf, sizeof(wbuf));
}

static inline int qt5480_i2c_read(struct qt5480 *qt,
						struct qt5480_ctrl_word *buf)
{
	int ret;

	ret = i2c_master_recv(qt->client, (u8 *)buf, sizeof(*buf));

	DBG_I2C("Read class %02x, data %02x %02x %02x %02x\n", buf->class,
			buf->data[0], buf->data[1], buf->data[2], buf->data[3]);

	return ret;
}

static inline int qt5480_i2c_read_regs(struct qt5480 *qt,
						struct qt5480_ctrl_word *buf)
{
	int ret, timeout = 32;
	u8 reg = buf->class;

	DBG_I2C("Reading from register block %02x (%04x - %04x)\n",
							reg, reg*4, reg*4 + 3);

	ret = qt5480_i2c_write(qt, REG_ADDRESS_POINTER, reg);
	if (ret < 0)
		return ret;

	do {
		ret = qt5480_i2c_read(qt, buf);
		if (ret < 0)
			return ret;
	} while (--timeout && buf->class != reg);

	if (buf->class != reg) {
		buf->class = reg;
		return -EIO;
	}

	return 0;
}

/*
 * Hardware configuration
 */

static void qt5480_power_up(struct qt5480 *qt)
{
	int timeout = 100, ret;
	struct qt5480_ctrl_word ctrl;

	gpio_set_value(qt->pdata->rst_gpio, qt->pdata->rst_inverted);
	gpio_set_value(qt->pdata->en_gpio, !qt->pdata->en_inverted);

	msleep(QT5480_RESET_ASSERT_TIME);

	gpio_set_value(qt->pdata->rst_gpio, !qt->pdata->rst_inverted);

	msleep(QT5480_RESET_TIME);

	ctrl.class = REG_CLASS(REG_CHIP_ID);
	do {
		msleep(1);
		ret = qt5480_i2c_read_regs(qt, &ctrl);
	} while(--timeout && (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID));

	if (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID)
		dev_err(qt->dev, "Device power up timed out\n");
}

#if 0 /* Unused yet */
static void qt5480_power_down(struct qt5480 *qt)
{
	gpio_set_value(qt->pdata->en_gpio, qt->pdata->en_inverted);
}
#endif

static int qt5480_check_chip_id(struct qt5480 *qt)
{
	struct qt5480_ctrl_word buf;
	int ret;

	ret = qt5480_i2c_write(qt, REG_LP_MODE, 32); //Write LPmode
	if(ret < 0) {
		dev_err(qt->dev,
			"write LP Register address failed(%d)!\n", ret);
		return ret;
	}

	buf.class = REG_CLASS(REG_CHIP_ID);
	ret = qt5480_i2c_read_regs(qt, &buf);
	if(ret < 0) {
		dev_err(qt->dev, "Chip ID read failed(%d)!\n", ret);
		return ret;
	}

	if(buf.data[REG_OFFS(REG_CHIP_ID)] != QT5480_CHIP_ID) {
		dev_err(qt->dev,
			"Chip ID does not match (got %d, expected %d).\n",
			buf.data[REG_OFFS(REG_CHIP_ID)], QT5480_CHIP_ID);
		return -ENODEV;
	}

	return 0;
}

static int qt5480_verify_checksum(struct qt5480 *qt)
{
	u16 checksum;
	struct qt5480_ctrl_word buf;
	int ret;

	checksum = qt5480_calc_cfg_chksum(qt->config);
	DBG("setup code Checksum: 0x%08x\n", checksum);

	buf.class = REG_CLASS(REG_EEPROM_CHKSUM_H);
	ret = qt5480_i2c_read_regs(qt, &buf);
	if(ret < 0) {
		dev_err(qt->dev, "checksum read failed(%d)\n", ret);
		return ret;
	}

	DBG("read Checksum: 0x%02x, 0x%02x\n",
				buf.data[REG_OFFS(REG_EEPROM_CHKSUM_H)],
				buf.data[REG_OFFS(REG_EEPROM_CHKSUM_L)]);

	checksum ^= buf.data[REG_OFFS(REG_EEPROM_CHKSUM_H)] << 8;
	checksum ^= buf.data[REG_OFFS(REG_EEPROM_CHKSUM_L)];

	if(!checksum)
		return 0;

	return -1;
}

static int qt5480_write_config(struct qt5480 *qt)
{
	u16 addr = REG_CHANNEL_CONTROL;
	u8 *reg = qt->config;
	int len = QT5480_CONFIG_REGS;
	int ret = 0;

	DBG("setup code size: %d\n", len);

	do {
		ret = qt5480_i2c_write(qt, addr, *reg);
		if(ret < 0) {
			dev_err(qt->dev,
					"setup code(0x%x) write failed (%d)\n",
					addr, ret);
			return ret;
		}

		DBG("setup[%4d] = 0x%02x\n", addr, *reg);

		++reg; ++addr;
	} while (--len);

	ret = qt5480_i2c_write(qt, REG_BACKUP_REQUEST, 0x55);
	if(ret < 0) {
		dev_err(qt->dev, "backup write failed(%d).\n", ret);
		return ret;
	}

	return 0;
}


static int qt5480_reset(struct qt5480 *qt)
{
	int ret = 0, timeout = 100;
	struct qt5480_ctrl_word ctrl;

	// H/W reset
	gpio_set_value(qt->pdata->rst_gpio, qt->pdata->rst_inverted);
	msleep(QT5480_RESET_ASSERT_TIME);

	gpio_set_value(qt->pdata->rst_gpio, !qt->pdata->rst_inverted);
	msleep(QT5480_RESET_TIME);

	// check if the device responds
	ctrl.class = REG_CLASS(REG_CHIP_ID);
	do {
		msleep(1);
		ret = qt5480_i2c_read_regs(qt, &ctrl);
	} while(--timeout && (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID));

	if (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID)
		return -EIO;

	return 0;
}

static int qt5480_update_firmware(struct qt5480 *qt)
{
	u8 wbuf[4] = { 0x00, 0xDC, 0xAA, 0x55 };
	int ret, size;
	u8 status;
	struct i2c_msg rmsg = {
		.addr = QT5480_BL_I2C_ADDR, .flags = I2C_M_RD,
		.len = 1, .buf = &status
	};
	struct i2c_msg wmsg = {
		.addr = QT5480_BL_I2C_ADDR, .flags = 0
	};
	u8 *data = qt5480_firmware;

	dev_info(qt->dev, "Updating firmware...\n");

	// Enter the bootloader mode
	ret = i2c_master_send(qt->client, wbuf, sizeof(wbuf));
	if (ret < 0) {
		dev_err(qt->dev, "Failed to enter bootloader mode.\n");
		return ret;
	}

	// Wait for the chip to enter bootloader mode
	msleep(5);

	// Write the firmware data
	do {
		ret = i2c_transfer(qt->client->adapter, &rmsg, 1);
		if(ret < 0) {
			dev_err(qt->dev, "Status read failed, aborting.\n");
			qt5480_reset(qt);
			return ret;
		}

		// do something according to the current status
		switch(status) {
		case 0x0:
			// POWER ON
			// do nothing
			break;

		case 0x1:
			// WAITING FRAME DATA
			// send the firmware data

			size = 16*data[0] + data[1] + 2;
			if(size == 2) {
				// reset TSP
				qt5480_reset(qt);
				return 0;
			}

			wmsg.len = size;
			wmsg.buf = data;

			ret = i2c_transfer(qt->client->adapter, &wmsg, 1);
			if(ret < 0) {
				dev_err(qt->dev,
					"Firmware data write failed.\n");
				return ret;
			}

			data += size;
			break;

		case 0x2:
			// FRAME CRC CHECK
			// do nothing
			break;

		case 0x3:
			// FRAME CRC ERROR
			// resend

			data -= size;
			dev_warn(qt->dev,
					"CRC error at offset %d, resending.\n",
					data - qt5480_firmware);
			break;

		case 0x4:
			// FRAME CRC PASS
			// update finished if last frame

			if(data[0] == 0 && data[1] == 0) {
				dev_info(qt->dev,
					"Firmware updated successfully.\n");

				// wait for the chip to finish
				// processing the firmware
				msleep(100);

				// reset TSP
				qt5480_reset(qt);
				return 0;
			}
			break;

		default:
			// APP CRC FAIL
			// firmware CRC check failed

			dev_err(qt->dev, "Firmware CRC check failed.\n");
			qt5480_reset(qt);
			return -EIO;
		}
	} while(1);

	return -EIO;
}

static int qt5480_get_fw_version(struct qt5480 *qt)
{
	int ret;
	struct qt5480_ctrl_word buf;
	u8 major, minor;

	buf.class = REG_CLASS(REG_CODE_VERSION);
	ret = qt5480_i2c_read_regs(qt, &buf);
	if(ret < 0) {
		dev_err(qt->dev,
			"Major version register read failed(%d).\n", ret);
		return ret;
	}
	major = buf.data[REG_OFFS(REG_CODE_VERSION)];

	msleep(5);

	buf.class = REG_CLASS(REG_MINOR_VERSION);
	ret = qt5480_i2c_read_regs(qt, &buf);
	if(ret < 0) {
		dev_err(qt->dev,
			"Minor version register read failed(%d).\n", ret);
		return ret;
	}
	minor = buf.data[REG_OFFS(REG_MINOR_VERSION)];

	return (major << 8) | minor;
}

static int qt5480_init_hw(struct qt5480 *qt)
{
	int ret = 0;

	if ((ret = gpio_request(qt->pdata->rst_gpio, "qt5480 reset")) != 0) {
		dev_err(qt->dev, "Failed to request reset GPIO\n");
		return ret;
	}
	gpio_direction_output(qt->pdata->rst_gpio, qt->pdata->rst_inverted);

	if ((ret = gpio_request(qt->pdata->en_gpio, "qt5480 enable")) != 0) {
		dev_err(qt->dev, "Failed to request enable GPIO\n");
		return ret;
	}
	gpio_direction_output(qt->pdata->en_gpio, qt->pdata->en_inverted);

	qt5480_power_up(qt);

	if(qt5480_check_chip_id(qt) < 0) {
		dev_err(qt->dev, "Device is not a AT42QT5480\n");
		return -ENODEV;
	}

	if ((ret = qt5480_get_fw_version(qt)) < 0) {
		dev_err(qt->dev, "Failed to get firmware version\n");
		return ret;
	}

	if (ret < QT5480_MIN_FW_VERSION) {
		dev_info(qt->dev, "Firmware too old, upgrading...\n");
		if ((ret = qt5480_update_firmware(qt)) != 0) {
			dev_err(qt->dev, "Failed to upgrade firmware\n");
			return ret;
		}
	}

	if ((ret = qt5480_write_config(qt)) != 0) {
		dev_err(qt->dev, "Failed to write setup code\n");
		return ret;
	}

	if ((ret = qt5480_verify_checksum(qt)) != 0) {
		dev_err(qt->dev, "EEPROM checksum verification failed\n");
		return ret;
	}

	if((ret = qt5480_i2c_write(qt, REG_CALIBRATE, 0x55)) < 0) {
		dev_err(qt->dev, "Calibration request failed\n");
		return ret;
	}

	return 0;
}

/*
 * Input events processing
 */

static void qt5480_handle_data(struct qt5480 *qt, struct qt5480_ctrl_word *ctrl)
{
	struct qt5480_touch *touch = qt->touch;

	switch (ctrl->class) {
	case 3:
		qt->ignore = 0;

		/* palm touch */
		if (ctrl->data[2] & 0x20) {
			qt->ignore = 1;
			qt->palm_touch = 1;
			return;
		}

		if (qt->palm_touch) {
			qt->palm_touch = 0;
			qt5480_i2c_write(qt, REG_CALIBRATE, 0x55);
		}

		/* Tracking lost or calibrating*/
		if ((ctrl->data[3] & 0x10) || (ctrl->data[3] & 0x40)) {
			qt->ignore = 1;
			return;
		}

		/* contacts */
		touch[0].curr.contact = !!(ctrl->data[3] & 0x01);
		touch[1].curr.contact = !!(ctrl->data[3] & 0x02);

		if(touch[0].curr.contact == 0) {
			touch[0].ready = 1;
			if(touch[1].prev.contact)
				touch[1].ready = 1;
		}

		if(touch[1].curr.contact == 0) {
			touch[1].ready = 1;
			if(touch[0].prev.contact)
				touch[0].ready = 1;
		}

		/* Error bit */
		if(ctrl->data[3] & 0x20) {
			touch[0].curr.contact = 0;
			touch[1].curr.contact = 0;
			touch[0].ready = 1;
			touch[1].ready = 1;
		}

		break;

	case 4:
		if(qt->ignore)
			return;

		touch[0].curr.pos_y = ctrl->data[0] * 4 + ctrl->data[1] / 64;
		touch[0].curr.pos_x = ctrl->data[2] * 4 + ctrl->data[3] / 64;
		touch[0].curr.width = (ctrl->data[1] & 63);
		touch[0].curr.pressure = (ctrl->data[3] & 63);
		touch[0].ready = 1;

		if (!touch[1].curr.contact)
			touch[1].ready = 1;

		break;

	case 5:
		if(qt->ignore)
			return;

		touch[1].curr.pos_y = ctrl->data[0] * 4 + ctrl->data[1] / 64;
		touch[1].curr.pos_x = ctrl->data[2] * 4 + ctrl->data[3] / 64;
		touch[1].curr.width = (ctrl->data[1] & 63);
		touch[1].curr.pressure = (ctrl->data[3] & 63);
		touch[1].ready = 1;

		if (!touch[0].curr.contact)
			touch[0].ready = 1;

		break;

	case 255: //ERROR
		touch[0].curr.contact = 0;
		touch[1].curr.contact = 0;
		touch[0].ready = 1;
		touch[1].ready = 1;

		break;
	}
}

static void qt5480_report_input(struct qt5480 *qt)
{
	struct qt5480_touch *touch = qt->touch;
	struct input_dev* dev = qt->input_dev;
	int changed = 0;

	/* leave if both touches are not ready yet*/
	if(!touch[0].ready || !touch[1].ready)
		return;

	/* leave if there are no changes*/
	changed |= (touch[0].curr.contact != touch[0].prev.contact);
	changed |= (touch[1].curr.contact != touch[1].prev.contact);

	if(touch[0].curr.contact) {
		changed |= (touch[0].curr.pos_x != touch[0].prev.pos_x);
		changed |= (touch[0].curr.pos_y != touch[0].prev.pos_y);
		changed |= (touch[0].curr.width != touch[0].prev.width);
		changed |= (touch[0].curr.pressure != touch[0].prev.pressure);
	}

	if(touch[1].curr.contact) {
		changed |= (touch[1].curr.pos_x != touch[1].prev.pos_x);
		changed |= (touch[1].curr.pos_y != touch[1].prev.pos_y);
		changed |= (touch[1].curr.width != touch[1].prev.width);
		changed |= (touch[1].curr.pressure != touch[1].prev.pressure);
	}

	if(!changed)
		return;

	/* report the last contact first */
	if(!qt->first && (touch[1].curr.contact || touch[1].prev.contact)) {
		/* report touch[1] only on contact or on leaving */
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
					(touch[1].curr.contact)
					? touch[1].curr.width : 0);
		input_report_abs(dev, ABS_MT_POSITION_X,
						touch[1].curr.pos_x);
		input_report_abs(dev, ABS_MT_POSITION_Y,
						touch[1].curr.pos_y);

		DBG_DEV("TOUCH_1:%d @%d:%d (w=%d,p=%d)\n",
			touch[1].curr.contact, touch[1].curr.pos_x,
			touch[1].curr.pos_y, touch[1].curr.width,
			touch[1].curr.pressure);

		input_mt_sync(dev);

		DBG_DEV("MT_SYNC\n");
	}

	/* report touch[0] only on contact or on leaving */
	if(touch[0].curr.contact || touch[0].prev.contact) {
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
					(touch[0].curr.contact)
					? touch[0].curr.width : 0);
		input_report_abs(dev, ABS_MT_POSITION_X, touch[0].curr.pos_x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch[0].curr.pos_y);

		DBG_DEV("TOUCH_0:%d @%d:%d (w=%d,p=%d)\n",
			touch[0].curr.contact, touch[0].curr.pos_x,
			touch[0].curr.pos_y, touch[0].curr.width,
			touch[0].curr.pressure);

		input_mt_sync(dev);

		DBG_DEV("MT_SYNC\n");
	}

	/* report the first contact last */
	if(qt->first && (touch[1].curr.contact || touch[1].prev.contact)) {
		/* report touch[1] only on contact or on leaving */
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
						(touch[1].curr.contact)
						? touch[1].curr.width : 0);
		input_report_abs(dev, ABS_MT_POSITION_X, touch[1].curr.pos_x);
		input_report_abs(dev, ABS_MT_POSITION_Y, touch[1].curr.pos_y);

		DBG_DEV("TOUCH_1:%d @%d:%d (w=%d,p=%d)\n",
			touch[1].curr.contact, touch[1].curr.pos_x,
			touch[1].curr.pos_y, touch[1].curr.width,
			touch[1].curr.pressure);

		input_mt_sync(dev);

		DBG_DEV("MT_SYNC\n");
	}

	input_sync(dev);

	DBG_DEV("SYNC\n");

	if(!touch[1].curr.contact)
		qt->first = 0;
	else if (!touch[0].curr.contact)
		qt->first = 1;

	touch[0].ready = 0;
	touch[0].prev = touch[0].curr;
	touch[1].ready = 0;
	touch[1].prev = touch[1].curr;
}

static irqreturn_t qt5480_irq_handler(int irq, void *dev_id)
{
	struct qt5480 *qt = (struct qt5480 *)dev_id;
	struct qt5480_ctrl_word ctrl;
	int ret;

	do {
		/* receive i2c data*/
		ret = qt5480_i2c_read(qt, &ctrl);
		if(ret < 0 ) {
			dev_err(qt->dev, "i2c_read failed.");
			return IRQ_HANDLED;
		}

		if (ctrl.class == 0xff)
			break;

		/* handle received data */
		DBG("Read control word: class %d, data "
				"{%02x, %02x, %02x, %02x}\n", ctrl.class,
				ctrl.data[0], ctrl.data[1],
				ctrl.data[2], ctrl.data[3]);

		qt5480_handle_data(qt, &ctrl);

		/* handle input device */
		qt5480_report_input(qt);
	} while (1);

	return IRQ_HANDLED;
}

/*
 * Sysfs entries
 */

static ssize_t config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct qt5480 *qt = dev_get_drvdata(dev);
	int i;
	char *ptr = buf;
	u8 *reg = qt->config;

	for(i = 0; i < QT5480_CONFIG_REGS; ++i) {
		ptr += sprintf(ptr, "%02x ", *(reg++));
		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}

	if (i % 8)
		ptr += sprintf(ptr, "\n");

	return ptr - buf;
}

static ssize_t config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct qt5480 *qt = dev_get_drvdata(dev);
	unsigned long reg, val;
	char *next = 0;

	reg = simple_strtoul(buf, &next, 0);
	if (reg >= QT5480_CONFIG_REGS)
		return -EINVAL;

	val = simple_strtoul(next, 0, 0);
	if (val > 255)
		return -EINVAL;

	qt->config[512 + reg] = val;
	qt5480_i2c_write(qt, 512 + reg, val);

	return size;
}

static DEVICE_ATTR(config, S_IRUGO | S_IWUSR, config_show, config_store);

static struct attribute *qt5480_attrs[] = {
	&dev_attr_config.attr,
	NULL
};

static const struct attribute_group qt5480_attr_group = {
	.attrs = qt5480_attrs,
};

/*
 * Power management
 */

#ifdef CONFIG_PM
static int qt5480_do_suspend(struct qt5480 *qt)
{
	int ret = 0;
#ifdef DEBUG_REGISTER_WRITES
	struct qt5480_ctrl_word ctrl;
#endif

	// Disable interrupts
	ret = qt5480_i2c_write(qt, REG_STATUS_MASK, 0);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write status mask register\n");
		return ret;
	}

	// Set Awake Timeout
	ret = qt5480_i2c_write(qt, REG_AWAKE_TIMEOUT, 5);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write awake timeout register\n");
		return ret;
	}

#ifdef DEBUG_REGISTER_WRITES
	// Verify Awake Timeout register
	ctrl.class = REG_CLASS(REG_AWAKE_TIMEOUT);
	ret = qt5480_i2c_read_regs(qt, &ctrl);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to read awake timeout register\n");
		return ret;
	}

	DBG("Awake Timeout read: (%x)\n", read_buf[3]);

	if (ctrl.data[REG_OFFS(REG_AWAKE_TIMEOUT)] != 5) {
		dev_err(qt->dev, "Failed to write awake timeout register\n");
		return -EIO;
	}
#endif

	// Enter LP mode
	ret = qt5480_i2c_write(qt, REG_LP_MODE, 0); // LP Mode
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write LP mode register\n");
		return ret;
	}


#ifdef DEBUG_REGISTER_WRITES
	// Verify Awake Timeout register
	ctrl.class = REG_CLASS(REG_LP_MODE);
	ret = qt5480_i2c_read_regs(qt, &ctrl);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to read LP mode register\n");
		return ret;
	}

	DBG("Awake Timeout read: (%x)\n", read_buf[3]);

	if (ctrl.data[REG_OFFS(REG_LP_MODE)] != 0) {
		dev_err(qt->dev, "Failed to write LP mode register\n");
		return -EIO;
	}
#endif

	return 0;
}

static int qt5480_do_resume(struct qt5480 *qt)
{
	int ret;

	// Exit LP mode
	ret = qt5480_i2c_write(qt, REG_LP_MODE, qt->config[REG_LP_MODE - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write LP mode register\n");
		return ret;
	}

	// Reset Awake Timeout register
	ret = qt5480_i2c_write(qt, REG_AWAKE_TIMEOUT,
					qt->config[REG_AWAKE_TIMEOUT - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write awake timeout register\n");
		return ret;
	}

	// Restore interrupt mask
	ret = qt5480_i2c_write(qt, REG_STATUS_MASK,
					qt->config[REG_STATUS_MASK - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write status mask register\n");
		return ret;
	}

	// Request calibration
	ret = qt5480_i2c_write(qt, REG_CALIBRATE, 0x55);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to issue calibration request\n");
		return ret;
	}

	msleep(10);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt5480_early_suspend(struct early_suspend *h)
{
	struct qt5480 *qt = container_of(h, struct qt5480, early_suspend);

	qt5480_do_suspend(qt);
}

static void qt5480_late_resume(struct early_suspend *h)
{
	struct qt5480 *qt = container_of(h, struct qt5480, early_suspend);

	qt5480_do_resume(qt);
}

static const struct dev_pm_ops qt5480_pm_ops = {
	.suspend	= 0,
	.resume		= 0,
};
#else /* CONFIG_HAS_EARLYSUSPEND */
static int qt5480_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qt5480 *qt = i2c_get_clientdata(client);

	return qt5480_do_suspend(qt);
}

static int qt5480_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qt5480 *qt = i2c_get_clientdata(client);

	return qt5480_do_resume(qt);
}

static const struct dev_pm_ops qt5480_pm_ops = {
	.suspend	= qt5480_suspend,
	.resume		= qt5480_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM */

/*
 * I2C Driver
 */

static int __devinit qt5480_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	struct qt5480 *qt;
	int ret = 0;

	if (!client->dev.platform_data)
		return -EINVAL;

	if((qt = kzalloc(sizeof(*qt), GFP_KERNEL)) == 0) {
		dev_err(qt->dev, "Failed to allocate driver data\n");
		return -ENOMEM;
	}

	if((input_dev = input_allocate_device()) == 0) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc_input;
	}

	input_dev->name = "AT42QT5480 Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
						QT5480_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
						QT5480_MAX_YC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,
						QT5480_MAX_ZC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
						QT5480_MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
						QT5480_MAX_TOUCH_ID, 0, 0);

	input_set_drvdata(input_dev, qt);

	qt->dev = &client->dev;
	qt->client = client;
	qt->input_dev = input_dev;
	qt->pdata = client->dev.platform_data;
	qt->irq = client->irq;

	qt->config = kmemdup(qt5480_default_config,
						QT5480_CONFIG_REGS, GFP_KERNEL);
	if (!qt->config) {
		dev_err(qt->dev,
			"Failed to allocate memory for device config\n");
		ret = -ENOMEM;
		goto err_config_alloc;
	}

	i2c_set_clientdata(client, qt);

	if ((ret = qt5480_init_hw(qt)) != 0) {
		dev_err(qt->dev, "Hardware init failed\n");
		goto err_hw_init;
	}
	
	if ((ret = input_register_device(input_dev)) != 0) {
		dev_err(qt->dev, "Failed to register input device\n");
		goto err_input_register;
	}

	ret = request_threaded_irq(qt->irq, 0, qt5480_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					client->dev.driver->name, qt);
	if (ret) {
		dev_err(qt->dev, "Failed to request interrupt\n");
		goto err_irq;
	}

	ret = sysfs_create_group(&client->dev.kobj, &qt5480_attr_group);
	if (ret)
		goto err_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	qt->early_suspend.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	qt->early_suspend.suspend	= qt5480_early_suspend;
	qt->early_suspend.resume	= qt5480_late_resume;
	register_early_suspend(&qt->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */

	return 0;

err_sysfs:
	free_irq(client->irq, qt);
err_irq:
	input_unregister_device(input_dev);
err_input_register:
err_hw_init:
	i2c_set_clientdata(client, 0);
	kfree(qt->config);
err_config_alloc:
	input_free_device(input_dev);
err_alloc_input:
	kfree(qt);

	return ret;
}

static int __devexit qt5480_remove(struct i2c_client *client)
{
	struct qt5480 *qt = i2c_get_clientdata(client);

	i2c_set_clientdata(client, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&qt->early_suspend);
#endif  // End of CONFIG_HAS_EARLYSUSPEND

	free_irq(qt->irq, qt);
	input_unregister_device(qt->input_dev);
	input_free_device(qt->input_dev);
	kfree(qt->config);
	kfree(qt);

	return 0;
}

static const struct i2c_device_id qt5480_id[] = {
	{ "qt5480_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qt5480_id);

static struct i2c_driver qt5480_driver = {
	.driver = {
		.name	= "qt5480_ts",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &qt5480_pm_ops,
#endif
	},
	.probe		= qt5480_probe,
	.remove		= __devexit_p(qt5480_remove),
	.id_table	= qt5480_id,
};

/*
 * Module
 */

static int __init qt5480_init(void)
{
	return i2c_add_driver(&qt5480_driver);
}

static void __exit qt5480_exit(void)
{
	i2c_del_driver(&qt5480_driver);
}

module_init(qt5480_init);
module_exit(qt5480_exit);

/* Module information */
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("AT42QT5480 Touchscreen driver");
MODULE_LICENSE("GPL");
