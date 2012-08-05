/* drivers/input/touchscreen/qt5480_ts.c
 *
 * Input driver for AT42QT5480 touchscreen.
 *
 * Original driver:
 * 	Copyright (C) 2009 Samsung Electronics Co. Ltd.
 * Multitouch:
 * 	Copyright (C) 2010 Lambertus Gorter <l.gorter@gmail.com>
 * Complete rewrite:
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

/*
 * QT5480 register definitions
 */
#define REG_CLASS(reg)		((reg) >> 2)
#define REG_OFFS(reg)		((reg) & 3)

/* Control registers */
#define	REG_CHIP_ID				0	/* byte */
#define	REG_CODE_VERSION			1	/* byte */
#define	REG_CALIBRATE				2	/* byte */
#define	REG_RESET				3	/* byte */
#define	REG_BACKUP_REQUEST			4	/* byte */
#define	REG_ADDRESS_POINTER			5	/* byte */
#define	REG_EEPROM_CHKSUM_L			6	/* byte */
#define REG_EEPROM_CHKSUM_H			7	/* byte */
#define	REG_KEY_STATUS_1			8	/* byte */
/* Unknown 9 - 13 */
#define	REG_GENERAL_STATUS_1			14	/* byte */
#define	REG_GENERAL_STATUS_2			15	/* byte */
#define	REG_TOUCHSCR_0_X			16	/* word */
#define	REG_TOUCHSCR_0_Y			18	/* word */
#define	REG_TOUCHSCR_1_X			20	/* word */
#define	REG_TOUCHSCR_1_Y			22	/* word */
#define	REG_SLIDER_POS				20	/* 6 bytes */
#define	REG_FORCE_MESURE			26	/* byte */
#define	REG_CHANNEL_GATING_INPUT_STATUS		27	/* byte */
/* Unknown 28 - 36 */
#define	REG_MINOR_VERSION			37	/* byte */
/* Unknown 38 - 255 */
#define	REG_CHANNEL_0_DELTA			256	/* 48 words */
#define	REG_CHANNEL_0_REFERENCE			352	/* 48 words */
/* Unknown 448 - 511 */

/* Setup data */
#define	REG_CHANNEL_CONTROL			512	/* 48 bytes */
#define	REG_CHANNEL_NEGATIVE_THRESHHOLD		560	/* 48 bytes */
#define	REG_CHANNEL_BURST_LENGTH		608	/* 48 bytes */
#define	REG_LP_MODE				656	/* byte */
#define	REG_MIN_CYCLE_TIME			657	/* byte */
#define	REG_AWAKE_TIMEOUT			658	/* byte */
#define	REG_TRIGGER_CONTROL			659	/* byte */
#define	REG_GUARD_CHANNEL_ENABLE		660	/* byte */
#define	REG_TOUCHSCREEN_SETUP			661	/* byte */
#define	REG_TOUCHSCREEN_LENGTH			662	/* byte */
#define	REG_SLIDER_SETUP			662	/* 6 bytes */
#define	REG_TOUCHSCREEN_HYSTERESIS		668	/* 6 bytes */
#define	REG_SLIDER_HYSTERESIS			668	/* 6 bytes */
#define	REG_GPO_CONTROL				674	/* byte */
#define	REG_NDRIFT_SETTING			675	/* byte */
#define	REG_PDRIFT_SETTING			676	/* byte */
#define	REG_NDIL_SETTING			677	/* byte */
#define	REG_SDIL_SETTING			678	/* byte */
#define	REG_NEGATIVE_RECALIBRATION_DELAY	679	/* byte */
#define	REG_DRIFT_HOLD_TIME			680	/* byte */
#define	REG_FORCE_SENSOR_THRESHHOLD		681	/* byte */
#define	REG_POSITION_CLIPPING_LIMITS		682	/* 2 bytes */
#define	REG_LINEAR_X_OFFSET			684	/* 2 bytes */
#define	REG_LINEAR_X_SEGMENTS			686	/* 16 bytes */
#define	REG_LINEAR_Y_OFFSET			702	/* 2 bytes */
#define	REG_LINEAR_Y_SEGMENTS			704	/* 16 bytes */
#define	REG_BURST_CONTROL			720	/* byte */
#define	REG_STATUS_MASK				721	/* byte */
#define	REG_POSITION_FILTER_CONTROL		722	/* byte */
#define	REG_TOUCH_SIZE_RESOLUTION_CONTROL	723	/* byte */
#define	REG_TOUCHSCREEN_PLATEAU_CONTROL		724	/* byte */
#define	REG_SLEW_RATE_FILTER_CONTROL		725	/* byte */
#define	REG_MEDIAN_FILTER_LENGTH		726	/* byte */
#define	REG_IIR_FILTER_CONTROL			727	/* byte */
#define	REG_TOUCHDOWN_HYSTERESIS		728	/* byte */
#define	REG_GESTURE_CONFIG_REGISTERS		734	/* 14 bytes */

/*
 * QT5480 register configuration
 */
static u8 qt5480_default_config[] = {
	/* REG_CHANNEL_CONTROL */
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,
	0,	0,	0,	0,

	/* REG_CHANNEL_NEGATIVE_THRESHHOLD */
	48,	48,	48,	48,
	48,	48,	49,	49,
	48,	48,	48,	48,
	48,	48,	49,	49,
	48,	48,	48,	48,
	48,	48,	48,	48,
	48,	48,	48,	48,
	48,	48,	48,	48,
	48,	48,	48,	48,
	48,	48,	49,	49,
	48,	48,	48,	48,
	48,	48,	49,	49,

	/* REG_CHANNEL_BURST_LENGTH */
	56,	44,	48,	48,
	48,	48,	48,	60,
	56,	36,	40,	40,
	40,	40,	40,	60,
	56,	40,	44,	44,
	44,	44,	44,	60,
	56,	40,	44,	44,
	44,	44,	44,	60,
	56,	40,	44,	44,
	44,	44,	44,	60,
	60,	52,	56,	56,
	56,	56,	56,	72,

	20,	/* REG_LP_MODE */
	255,	/* REG_MIN_CYCLE_TIME */
	50,	/* REG_AWAKE_TIMEOUT */
	0,	/* REG_TRIGGER_CONTROL */
	48,	/* REG_GUARD_CHANNEL_ENABLE */
	22,	/* REG_TOUCHSCREEN_SETUP */

	/* REG_TOUCHSCREEN_LENGTH / REG_SLIDER_SETUP */
	8,	0,	0,	0,	0,	0,

	/* REG_TOUCHSCREEN_HYSTERESIS / REG_SLIDER_HYSTERESIS */
	0,	0,	0,	0,	0,	0,

	0,	/* REG_GPO_CONTROL */
	12,	/* REG_NDRIFT_SETTING */
	1,	/* REG_PDRIFT_SETTING */
	2,	/* REG_NDIL_SETTING */
	0,	/* REG_SDIL_SETTING */
	150,	/* REG_NEGATIVE_RECALIBRATION_DELAY */
	5,	/* REG_DRIFT_HOLD_TIME */
	255,	/* REG_FORCE_SENSOR_THRESHHOLD */

	/* REG_POSITION_CLIPPING_LIMITS */
	0,	0,

	/* REG_LINEAR_X_OFFSET */
	0,	0,

	/* REG_LINEAR_X_SEGMENTS */
	64,	64,	64,	64,
	64,	64,	64,	64,
	64,	64,	64,	64,
	64,	64,	64,	64,

	/* REG_LINEAR_Y_OFFSET */
	0,	0,

	/* REG_LINEAR_Y_SEGMENTS */
	64,	64,	64,	64,
	64,	64,	64,	64,
	64,	64,	64,	64,
	64,	64,	64,	64,

	2,	/* REG_BURST_CONTROL */
	14,	/* REG_STATUS_MASK */
	8,	/* REG_POSITION_FILTER_CONTROL */
	3,	/* REG_TOUCH_SIZE_RESOLUTION_CONTROL */
	0,	/* REG_TOUCHSCREEN_PLATEAU_CONTROL */
	0,	/* REG_SLEW_RATE_FILTER_CONTROL */
	1,	/* REG_MEDIAN_FILTER_LENGTH */
	0,	/* REG_IIR_FILTER_CONTROL */
	0,	/* REG_TOUCHDOWN_HYSTERESIS */

	/* 729 - 733 (Unknown) */
	0,	25,	0,	0,	26,

	/* REG_GESTURE_CONFIG_REGISTERS */
	0,	6,	16,	6,	16,	16,	16,
	0,	75,	0,	50,	0,	0,	0,

	/* 748++ (Unknown) */
	13,	10,	0,	0,	0
};

#define QT5480_CONFIG_REGS	(sizeof(qt5480_default_config))

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
#define QT5480_CHIP_ID			0x0F
#define QT5480_MAX_XC			1023
#define QT5480_MAX_YC			1023
#define QT5480_MAX_ZC			63
#define QT5480_MAX_WIDTH		63
#define QT5480_MAX_FINGER		2
#define QT5480_HARD_RESET_TIMEOUT	10
#define QT5480_HARD_RESET_DELAY		20
#define QT5480_SOFT_RESET_TIMEOUT	10
#define QT5480_SOFT_RESET_DELAY		50
#define QT5480_REGISTER_READ_TIMEOUT	20
#define QT5480_RESET_ASSERT_TIME	10
#define QT5480_MIN_FW_VERSION		0x5502

/*
 * Touch states
 */
#define QT5480_NONE		0
#define QT5480_MOVE		1
#define QT5480_RELEASE		2

/*
 * Structures used by the driver
 */
struct qt5480_touch {
	int status;
	int pos_x;
	int pos_y;
	int pressure;
	int width;
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
static int qt5480_power_up(struct qt5480 *qt)
{
	int timeout, ret;
	struct qt5480_ctrl_word ctrl;

	gpio_set_value(qt->pdata->rst_gpio, 0);

	msleep(QT5480_RESET_ASSERT_TIME);

	gpio_set_value(qt->pdata->rst_gpio, 1);

	timeout = QT5480_HARD_RESET_TIMEOUT;
	do {
		msleep(QT5480_HARD_RESET_DELAY);
	} while (--timeout && gpio_get_value(qt->pdata->change_gpio));

	if (!timeout && gpio_get_value(qt->pdata->change_gpio)) {
		dev_err(qt->dev, "Device power up reset timed out\n");
		return -EFAULT;
	}

	dev_dbg(qt->dev, "timeout = %d\n", timeout);

	timeout = QT5480_REGISTER_READ_TIMEOUT;
	while (--timeout && !gpio_get_value(qt->pdata->change_gpio)) {
		/* receive i2c data*/
		ret = qt5480_i2c_read(qt, &ctrl);
		if (ret < 0) {
			dev_err(qt->dev, "i2c_read failed.");
			return -EFAULT;
		}
	}

	if (!timeout && !gpio_get_value(qt->pdata->change_gpio)) {
		dev_err(qt->dev, "Device power up reset timed out\n");
		return -EFAULT;
	}

	dev_dbg(qt->dev, "timeout = %d\n", timeout);

	ret = qt5480_i2c_write(qt, REG_RESET, 1);
	if (ret < 0) {
		dev_err(qt->dev, "i2c_read failed.");
		return -EFAULT;
	}

	timeout = QT5480_SOFT_RESET_TIMEOUT;
	do {
		msleep(QT5480_SOFT_RESET_DELAY);
	} while (--timeout && gpio_get_value(qt->pdata->change_gpio));

	if (!timeout && gpio_get_value(qt->pdata->change_gpio)) {
		dev_err(qt->dev, "Device power up reset timed out\n");
		return -EFAULT;
	}

	dev_dbg(qt->dev, "timeout = %d\n", timeout);

	timeout = QT5480_REGISTER_READ_TIMEOUT;
	while (--timeout && !gpio_get_value(qt->pdata->change_gpio)) {
		/* receive i2c data*/
		ret = qt5480_i2c_read(qt, &ctrl);
		if (ret < 0) {
			dev_err(qt->dev, "i2c_read failed.");
			return -EFAULT;
		}
	}

	if (!timeout && !gpio_get_value(qt->pdata->change_gpio)) {
		dev_err(qt->dev, "Device power up reset timed out\n");
		return -EFAULT;
	}

	dev_dbg(qt->dev, "timeout = %d\n", timeout);

	timeout = QT5480_REGISTER_READ_TIMEOUT;
	ctrl.class = REG_CLASS(REG_CHIP_ID);
	do {
		msleep(1);
		ret = qt5480_i2c_read_regs(qt, &ctrl);
	} while(--timeout && (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID));

	if (ret < 0 || ctrl.data[0] != QT5480_CHIP_ID) {
		dev_err(qt->dev, "Device power up timed out\n");
		return -EFAULT;
	}

	dev_dbg(qt->dev, "timeout = %d\n", timeout);

	return 0;
}

static int qt5480_check_chip_id(struct qt5480 *qt)
{
	struct qt5480_ctrl_word buf;
	int ret;

	ret = qt5480_i2c_write(qt, REG_LP_MODE, 32); /*Write LPmode */
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
	gpio_direction_output(qt->pdata->rst_gpio, 0);

	if ((ret = gpio_request(qt->pdata->change_gpio, "qt5480 chg")) != 0) {
		dev_err(qt->dev, "Failed to request change GPIO\n");
		return ret;
	}
	gpio_direction_input(qt->pdata->change_gpio);

	if ((ret = qt5480_power_up(qt)) != 0) {
		dev_err(qt->dev, "Failed to power up the chip\n");
		return ret;
	}

	if(qt5480_check_chip_id(qt) < 0) {
		dev_err(qt->dev, "Device is not a AT42QT5480\n");
		return -ENODEV;
	}

	if ((ret = qt5480_get_fw_version(qt)) < 0) {
		dev_err(qt->dev, "Failed to get firmware version\n");
		return ret;
	}

	if (ret < QT5480_MIN_FW_VERSION) {
		dev_err(qt->dev, "Incompatible firmware version found. "
					"Expected at least %04x, got %04x.\n",
					QT5480_MIN_FW_VERSION, ret);
		return -ENODEV;
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
static void qt5480_report_input(struct qt5480 *qt)
{
	struct qt5480_touch *touch = qt->touch;
	struct input_dev* dev = qt->input_dev;
	int id;

	for (id = 0; id < QT5480_MAX_FINGER; ++id) {
		if (!touch[id].status)
			continue;

		if (touch[id].status != QT5480_RELEASE) {
			input_report_abs(dev, ABS_MT_PRESSURE,
							touch[id].pressure);
			input_report_abs(dev, ABS_MT_WIDTH_MAJOR,
							touch[id].width);
			input_report_abs(dev, ABS_MT_POSITION_X,
							touch[id].pos_x);
			input_report_abs(dev, ABS_MT_POSITION_Y,
							touch[id].pos_y);
		} else {
			input_report_abs(dev, ABS_MT_PRESSURE, 0);
			touch[id].status = 0;
		}

		input_report_abs(dev, ABS_MT_TRACKING_ID, id);
		input_mt_sync(dev);
	}

	input_sync(dev);
}

static void qt5480_handle_data(struct qt5480 *qt, struct qt5480_ctrl_word *ctrl)
{
	struct qt5480_touch *touch = qt->touch;
	int id;
	int report = 0;

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
			qt->ignore = 1;
			qt->palm_touch = 0;
			qt5480_i2c_write(qt, REG_CALIBRATE, 0x55);
			return;
		}

		/* Tracking lost or calibrating*/
		if ((ctrl->data[3] & 0x10) || (ctrl->data[3] & 0x40)) {
			qt->ignore = 1;
			return;
		}

		/* contacts */
		if (!(ctrl->data[3] & 0x01)) {
			touch[0].status = QT5480_RELEASE;
			report = 1;
		}

		if (!(ctrl->data[3] & 0x02)) {
			touch[1].status = QT5480_RELEASE;
			report = 1;
		}

		/* Error bit */
		if (ctrl->data[3] & 0x20) {
			if (touch[0].status) {
				touch[0].status = QT5480_RELEASE;
				report = 1;
			}
			if (touch[1].status) {
				touch[1].status = QT5480_RELEASE;
				report = 1;
			}
		}

		if (report)
			qt5480_report_input(qt);

		break;

	case 4:
	case 5:
		if(qt->ignore)
			return;

		id = ctrl->class - 4;

		touch[id].status = QT5480_MOVE;
		touch[id].pos_y = (ctrl->data[0] << 2) | (ctrl->data[1] >> 6);
		touch[id].pos_x = (ctrl->data[2] << 2) | (ctrl->data[3] >> 6);
		touch[id].pressure = (ctrl->data[1] & 0x3f);
		touch[id].width = (ctrl->data[3] & 0x3f);

		qt5480_report_input(qt);

		break;
	}
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
	} while (1);

	return IRQ_HANDLED;
}

/*
 * Power management
 */
static int qt5480_do_suspend(struct qt5480 *qt)
{
	int ret = 0;

	/* Disable interrupts */
	ret = qt5480_i2c_write(qt, REG_STATUS_MASK, 0);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write status mask register\n");
		return ret;
	}

	/* Set Awake Timeout */
	ret = qt5480_i2c_write(qt, REG_AWAKE_TIMEOUT, 5);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write awake timeout register\n");
		return ret;
	}

	/* Enter LP mode */
	ret = qt5480_i2c_write(qt, REG_LP_MODE, 0); /* LP Mode */
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write LP mode register\n");
		return ret;
	}

	return 0;
}

static int qt5480_do_resume(struct qt5480 *qt)
{
	int ret;

	/* Exit LP mode */
	ret = qt5480_i2c_write(qt, REG_LP_MODE, qt->config[REG_LP_MODE - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write LP mode register\n");
		return ret;
	}

	/* Reset Awake Timeout register */
	ret = qt5480_i2c_write(qt, REG_AWAKE_TIMEOUT,
					qt->config[REG_AWAKE_TIMEOUT - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write awake timeout register\n");
		return ret;
	}

	/* Restore interrupt mask */
	ret = qt5480_i2c_write(qt, REG_STATUS_MASK,
					qt->config[REG_STATUS_MASK - 512]);
	if(ret < 0) {
		dev_err(qt->dev, "Failed to write status mask register\n");
		return ret;
	}

	/* Request calibration */
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

	input_dev->name = "qt5480_ts_input";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
						0, QT5480_MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
						0, QT5480_MAX_ZC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, QT5480_MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, QT5480_MAX_YC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
						QT5480_MAX_FINGER - 1, 0, 0);

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
#ifdef CONFIG_HAS_EARLYSUSPEND
	qt->early_suspend.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	qt->early_suspend.suspend	= qt5480_early_suspend;
	qt->early_suspend.resume	= qt5480_late_resume;
	register_early_suspend(&qt->early_suspend);
#endif
	return 0;

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

	free_irq(qt->irq, qt);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&qt->early_suspend);
#endif
	i2c_set_clientdata(client, 0);

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
		.pm	= &qt5480_pm_ops,
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
