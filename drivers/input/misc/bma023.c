/*
 * bma023.c - BMA023/150/SMB380 Tri-axis accelerometer driver
 *
 * Copyright (C) 2010 Samsung Eletronics Co.Ltd
 * Kim Kyuwon <q1.kim@samsung.com>
 * Kyungmin Park <kyungmin.park@samsung.com>
 * Donggeun Kim <dg77.kim@samsung.com>
 *
 * Copyright (C) 2011 Wistron Co.Ltd
 * Joseph Lai <joseph_lai@wistron.com>
 *
 * Copyright (c) 2011 Intel Corporation
 * Alan Cox <alan@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * TODO (After)
 *	Investigate if the IRQ disable is really needed (needs that data
 *		sheet reviewing in more detail)
 *
 * Data sheet:
 * http://www.bosch-sensortec.com/content/language4/downloads/BST-BMA150-DS000-06.pdf
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/i2c/bma023.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#define BMA023_DEFAULT_AUTO_DELAY	250	/* mS */

#define BMA023_CHIP_ID_REG	0x00
#define BMA023_X_LSB_REG	0x02
#define BMA023_X_MSB_REG	0x03
#define BMA023_Y_LSB_REG	0x04
#define BMA023_Y_MSB_REG	0x05
#define BMA023_Z_LSB_REG	0x06
#define BMA023_Z_MSB_REG	0x07
#define BMA023_TEMP_REG		0x08 /*  An output of 0 equals -30C,
					 1 LSB equals 0.5C */
#define BMA023_CTRL1_REG	0x0a
#define BMA023_CTRL2_REG	0x0b
#define BMA023_SETTINGS1_REG	0x0c
#define BMA023_SETTINGS2_REG	0x0d
#define BMA023_SETTINGS3_REG	0x0e
#define BMA023_SETTINGS4_REG	0x0f
#define BMA023_SETTINGS5_REG	0x10
#define BMA023_SETTINGS6_REG	0x11
#define BMA023_RANGE_BW_REG	0x14
#define BMA023_CONF2_REG	0x15

#define BMA023_CHIP_ID		0x02

#define BMA023_NEW_DATA_INT_SHIFT	5
#define BMA023_NEW_DATA_INT_MASK	(0x1 << 5)

#define BMA023_RANGE_SHIFT		3
#define BMA023_RANGE_MASK		(0x3 << 3)
#define BMA023_BANDWIDTH_SHIFT		0
#define BMA023_BANDWIDTH_MASK		(0x7)

#define BMA023_HG_HYST_SHIFT		3
#define BMA023_HG_HYST_MASK		(0x7 << 3)
#define BMA023_LG_HYST_SHIFT		0
#define BMA023_LG_HYST_MASK		(0x7)

#define BMA023_HG_DUR_SHIFT		(0x0)
#define BMA023_HG_DUR_MASK		(0xff)
#define BMA023_HG_THRES_SHIFT		(0x0)
#define BMA023_HG_THRES_MASK		(0xff)
#define BMA023_LG_DUR_SHIFT		(0x0)
#define BMA023_LG_DUR_MASK		(0xff)
#define BMA023_LG_THRES_SHIFT		(0x0)
#define BMA023_LG_THRES_MASK		(0xff)

#define BMA023_ENABLE_HG_SHIFT		1
#define BMA023_ENABLE_HG_MASK		(0x1 << 1)
#define BMA023_ENABLE_LG_SHIFT		0
#define BMA023_ENABLE_LG_MASK		(0x1)

#define BMA023_SLEEP_SHIFT		0
#define BMA023_SLEEP_MASK		(0x1)

#define BMA023_ACCEL_BITS		10
#define BMA023_MAX_VALUE		((1 << ((BMA023_ACCEL_BITS) - 1)) - 1)
#define BMA023_MIN_VALUE		(-(1 << ((BMA023_ACCEL_BITS) - 1)))

#define BMA023_DEFAULT_RANGE		BMA023_RANGE_2G
#define BMA023_DEFAULT_BANDWIDTH	BMA023_BW_190HZ
#define BMA023_DEFAULT_NEW_DATA_INT	0
#define BMA023_DEFAULT_HG_INT		0
#define BMA023_DEFAULT_LG_INT		0
#define BMA023_DEFAULT_HG_DURATION	0x05
#define BMA023_DEFAULT_HG_THRESHOLD	0xa0
#define BMA023_DEFAULT_HG_HYST		0
#define BMA023_DEFAULT_LG_DURATION	0x05
#define BMA023_DEFAULT_LG_THRESHOLD	0x14
#define BMA023_DEFAULT_LG_HYST		0

#define BMA023_SUSPEND	0
#define BMA023_RESUME	1

#define BMA023_CUSTOMER_REG	0x12
#define BMA023_INSPECT_VAL	0xBB
#define BMA023_CHECK_SHIFT	0
#define BMA023_CHECK_MASK	0xff

struct bma023_data {
	s16 x;
	s16 y;
	s16 z;
};

struct bma023_sensor {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *idev;
	struct mutex lock;
	struct bma023_data data;
	int soft_power;		/* Our sysfs requested power status as
				   opposed to the actual one which is
				   a combination of this and pm runtime */
	u8 range;
	u8 bandwidth;
	u8 new_data_int;
	u8 hg_int;
	u8 lg_int;
	u8 lg_dur;
	u8 lg_thres;
	u8 lg_hyst;
	u8 hg_dur;
	u8 hg_thres;
	u8 hg_hyst;
	u8 power_mode;
};

/**
 *	bma023_write_reg	-	write a BMA023 register via I2C
 *	@reg: register being updated
 *	@val: value to write
 *
 *	Updates the register or returns an error if the write fails.
 */
static int bma023_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	/*
	 * According to the datasheet, the interrupt should be deactivated
	 * on the host side when write sequences operate.
	 */
	disable_irq_nosync(client->irq);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	enable_irq(client->irq);
	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
			__func__, reg, val, ret);
	return ret;
}

/**
 *	bma023_read_reg	-	read a BMA023 register via I2C
 *	@reg: register being updated
 *	@val: value to write
 *
 *	Reads the register or returns a negative error code on failure.
 */
static int bma023_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, err %d\n",
			__func__, reg, ret);
	return ret;
}

/**
 *	bma023_xyz_read_reg	-	read the axes values
 *	@reg: register being updated
 *	@val: value to write
 *
 *	Reads the register values in one transaction or returns a negative
 *	error code on failure/
 */
static int bma023_xyz_read_reg(struct i2c_client *client,
			       u8 *buffer, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buffer,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};
	return i2c_transfer(client->adapter, msg, 2);
}

/**
 *	bma023_set_reg_bits	-	update a subset of register bits
 *	@client: i2c client for the sensor
 *	@val: value to write
 *	@shift: bit offset of field
 *	@mask: bit mask of field to update
 *	@reg: register number
 *
 *	Update one of the many bitfield registers on the BMA023 by reading
 *	and writing back bits. The caller *MUST* hold the sensor lock when
 *	using this function in order to prevent overlapping updates corrupting
 *	the register.
 *
 *	Returns success or an error code
 */
static int bma023_set_reg_bits(struct i2c_client *client,
					int val, int shift, u8 mask, u8 reg)
{
	int data = bma023_read_reg(client, reg);

	if (data < 0)
		return data;

	data = (data & ~mask) | ((val << shift) & mask);
	return bma023_write_reg(client, reg, data);
}

/**
 *	bma023_get_reg_bits	-	read register bitfield
 *	@client: i2c client for the sensor
 *	@shift: bit offset of field
 *	@mask: bit mask of field to update
 *	@reg: register number
 *
 *	Read a bit field from the sensor and return the value of the field. The
 *	lock is not needed in this case
 */
static int bma023_get_reg_bits(struct i2c_client *client, int shift,
					u8 mask, u8 reg)
{
	int data = bma023_read_reg(client, reg);
	if (data < 0)
		return data;
	data = (data & mask) >> shift;
	return data;
}

/*
 *	Functions for handling all the configurable fields
 */

/**
 *	bma023_set_check_reg	-	set an inspective value
 *	@client: i2c client for the sensor
 *	@check: update the inspective value
 *
 *	Set an inspective value to customer's register, which can record
 *	any value from user. The inspective value will keep until the chip
 *	off or shutdown. User can recognize the chip needs re-initialize
 *	or not by this register.
 */
static int bma023_set_check_reg(struct i2c_client *client, u8 check)
{
	return bma023_set_reg_bits(client, check, BMA023_CHECK_SHIFT,
				   BMA023_CHECK_MASK, BMA023_CUSTOMER_REG);
}

static int bma023_get_check_reg(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_CHECK_SHIFT,
				   BMA023_CHECK_MASK, BMA023_CUSTOMER_REG);
}

/**
 *	bma023_set_range	-	set the full scale acceleration range
 *	@client: i2c client for the sensor
 *	@range: new range setting
 *
 *	The range of bma023 will affect the chip to change the sensitivity 
 *	of gravitation. The value of range includes +/- 2g, 4g and 8g.
 *
 *	The change of range will respond to 3 axes directly.
 */
static int bma023_set_range(struct i2c_client *client, u8 range)
{
	return bma023_set_reg_bits(client, range, BMA023_RANGE_SHIFT,
				   BMA023_RANGE_MASK, BMA023_RANGE_BW_REG);
}

static int bma023_get_range(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_RANGE_SHIFT,
				   BMA023_RANGE_MASK, BMA023_RANGE_BW_REG);
}

/**
 *	bma023_set_bandwidth	-	set the digital filtering of ADC output data
 *	@client: i2c client for the sensor
 *	@bw: new bandwidth setting
 *
 *	The bandwidth affect the ability of noise filter. The value of bandwidth
 *	includes 25Hz, 50Hz, 100Hz, 190Hz, 375Hz, 750Hz and 1500Hz.
 *
 *	The change of bandwidth will respond to 3 axes directly.
 */
static int bma023_set_bandwidth(struct i2c_client *client, u8 bw)
{
	return bma023_set_reg_bits(client, bw, BMA023_BANDWIDTH_SHIFT,
				   BMA023_BANDWIDTH_MASK, BMA023_RANGE_BW_REG);
}

static int bma023_get_bandwidth(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_BANDWIDTH_SHIFT,
				   BMA023_BANDWIDTH_MASK, BMA023_RANGE_BW_REG);
}

/**
 *	bma023_set_new_data_int	-	switch on/off of new data interrupt
 *	@client: i2c client for the sensor
 *	@val: value to switch on/off, 1: on, 0: off
 *
 *	To turn on this interrupt will generate an interrupt when all three
 *	axes acceleration values are new.
 */
static int bma023_set_new_data_int(struct i2c_client *client, u8 val)
{
	return bma023_set_reg_bits(client, val, BMA023_NEW_DATA_INT_SHIFT,
				   BMA023_NEW_DATA_INT_MASK, BMA023_CONF2_REG);
}

static int bma023_get_new_data_int(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_NEW_DATA_INT_SHIFT,
				   BMA023_NEW_DATA_INT_MASK, BMA023_CONF2_REG);
}

/**
 *	bma023_set_hg_int	-	switch on/off of high-g interrupt
 *	@client: i2c client for the sensor
 *	@val: value to switch on/off, 1: on, 0: off
 *
 *	To turn on this interrupt will generate an interrupt when the
 *	oscillatory criteria is satisfied by hg_dur and hg_thres.
 *
 *	Usually, it is used to debounce the high-g criteria.
 */
static int bma023_set_hg_int(struct i2c_client *client, u8 val)
{
	return bma023_set_reg_bits(client, val, BMA023_ENABLE_HG_SHIFT,
				   BMA023_ENABLE_HG_MASK, BMA023_CTRL2_REG);
}

static int bma023_get_hg_int(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_ENABLE_HG_SHIFT,
				   BMA023_ENABLE_HG_MASK, BMA023_CTRL2_REG);
}

/**
 *	bma023_set_lg_int	-	switch on/off of low-g interrupt
 *	@client: i2c client for the sensor
 *	@val: value to switch on/off, 1: on, 0: off
 *
 *	To turn on this setting will generate an interrupt when the
 *	oscillatory criteria is satisfied by lg_dur and lg_thres.
 *
 *	Usually, it is used to detect a free fall.
 */
static int bma023_set_lg_int(struct i2c_client *client, u8 val)
{
	return bma023_set_reg_bits(client, val,	BMA023_ENABLE_LG_SHIFT,
				   BMA023_ENABLE_LG_MASK, BMA023_CTRL2_REG);
}

static int bma023_get_lg_int(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_ENABLE_LG_SHIFT,
				   BMA023_ENABLE_LG_MASK, BMA023_CTRL2_REG);
}

/**
 *	bma023_set_lg_dur	-	set low-g duration
 *	@client: i2c client for the sensor
 *	@dur: duration value, 0-255
 *	
 *	Generate an interrupt when the gravitation is detected in low-g
 *	criterion for long enough duration.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_lg_dur(struct i2c_client *client, u8 dur)
{
	return bma023_set_reg_bits(client, dur,	BMA023_LG_DUR_SHIFT,
				   BMA023_LG_DUR_MASK, BMA023_SETTINGS2_REG);
}

static int bma023_get_lg_dur(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_LG_DUR_SHIFT,
				   BMA023_LG_DUR_MASK, BMA023_SETTINGS2_REG);
}

/**
 *	bma023_set_lg_thres	-	set low-g threshold
 *	@client: i2c client for the sensor
 *	@thres: threshold value, 0-255
 *	
 *	Filter 3 axes detection and limit to trigger an interrupt signal
 *	when the gravitation is detected in low-g threshold.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_lg_thres(struct i2c_client *client, u8 thres)
{
	return bma023_set_reg_bits(client, thres, BMA023_LG_THRES_SHIFT,
				   BMA023_LG_THRES_MASK, BMA023_SETTINGS1_REG);
}

static int bma023_get_lg_thres(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_LG_THRES_SHIFT,
				   BMA023_LG_THRES_MASK, BMA023_SETTINGS1_REG);
}

/**
 *	bma023_set_lg_hyst	-	set low-g hysteresis
 *	@client: i2c client for the sensor
 *	@hyst: hysteresis value, 0-7
 *	
 *	Expand the range of detectable gravitation to reset an interrupt.
 *	An interrupt will be latched until outside of low-g criterion with 
 *	hysteresis.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_lg_hyst(struct i2c_client *client, u8 hyst)
{
	return bma023_set_reg_bits(client, hyst, BMA023_LG_HYST_SHIFT,
				   BMA023_LG_HYST_MASK, BMA023_SETTINGS6_REG);
}

static int bma023_get_lg_hyst(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_LG_HYST_SHIFT,
				   BMA023_LG_HYST_MASK,	BMA023_SETTINGS6_REG);
}

/**
 *	bma023_set_hg_dur	-	set high-g duration
 *	@client: i2c client for the sensor
 *	@dur: duration value, 0-255
 *	
 *	Generate an interrupt when the gravitation is detected in high-g
 *	criterion for long enough duration.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_hg_dur(struct i2c_client *client, u8 dur)
{
	return bma023_set_reg_bits(client, dur,	BMA023_HG_DUR_SHIFT,
				   BMA023_HG_DUR_MASK, BMA023_SETTINGS4_REG);
}

static int bma023_get_hg_dur(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_HG_DUR_SHIFT,
				   BMA023_HG_DUR_MASK, BMA023_SETTINGS4_REG);
}

/**
 *	bma023_set_hg_thres	-	set high-g threshold
 *	@client: i2c client for the sensor
 *	@thres: threshold value, 0-255
 *	
 *	Filter 3 axes detection and limit to trigger an interrupt signal
 *	when the gravitation is detected in high-g threshold.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_hg_thres(struct i2c_client *client, u8 thres)
{
	return bma023_set_reg_bits(client, thres, BMA023_HG_THRES_SHIFT,
				   BMA023_HG_THRES_MASK, BMA023_SETTINGS3_REG);
}

static int bma023_get_hg_thres(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_HG_THRES_SHIFT,
				   BMA023_HG_THRES_MASK, BMA023_SETTINGS3_REG);
}

/**
 *	bma023_set_hg_hyst	-	set low-g hysteresis
 *	@client: i2c client for the sensor
 *	@hyst: hysteresis value, 0-7
 *	
 *	Expand the range of detectable gravitation to reset an interrupt.
 *	An interrupt will be latched until outside of high-g criterion with 
 *	hysteresis.
 *
 *	This setting is used to interrupt detection.
 */
static int bma023_set_hg_hyst(struct i2c_client *client, u8 hyst)
{
	return bma023_set_reg_bits(client, hyst, BMA023_HG_HYST_SHIFT,
				   BMA023_HG_HYST_MASK,	BMA023_SETTINGS6_REG);
}

static int bma023_get_hg_hyst(struct i2c_client *client)
{
	return bma023_get_reg_bits(client, BMA023_HG_HYST_SHIFT,
				   BMA023_HG_HYST_MASK,	BMA023_SETTINGS6_REG);
}

/**
 *	bma023_power_mode	-	set the power mode
 *	@client: i2c client for the sensor
 *	@val: value to switch on/off of power, 1: normal power, 0: low power
 *
 *	Put device to normal-power mode or low-power mode.
 */
static int bma023_set_power_mode(struct i2c_client *client, u8 val)
{
	return bma023_set_reg_bits(client, ~val, BMA023_SLEEP_SHIFT,
				   BMA023_SLEEP_MASK, BMA023_CTRL1_REG);
}

static int bma023_get_power_mode(struct i2c_client *client)
{
	return !bma023_get_reg_bits(client, BMA023_SLEEP_SHIFT,
				   BMA023_SLEEP_MASK, BMA023_CTRL1_REG);
}

/**
 *	bma023_correct_accel_sign	-	convert to expected format
 *	@val: twos complement value
 *
 *	The description of the digital signals x, y and z is "2' complement".
 *	So we need to correct the sign of data read by i2c.
 */
static s16 bma023_correct_accel_sign(s16 val)
{
	val <<= (sizeof(s16) * BITS_PER_BYTE - BMA023_ACCEL_BITS);
	val >>= (sizeof(s16) * BITS_PER_BYTE - BMA023_ACCEL_BITS);
	return val;
}

/**
 *	bma023_merge_register_values	-	turn into co-ordinate
 *	@lsb: low bits from sensor
 *	@msb: high bits from sensor
 *
 *	Turns a sensor register reading into a Linux input layer
 *	value and returns it.
 */
static s16 bma023_merge_register_values(u8 lsb, u8 msb)
{
	s16 val = (msb << 2) | (lsb >> 6);
	return bma023_correct_accel_sign(val);
}

/**
 *	bma023_read_xyz		-	get co-ordinates from device
 *	@client: i2c address of sensor
 *	@coords: co-ordinates to update
 *
 *	Return the converted X Y and Z co-ordinates from the sensor device
 */
static void bma023_read_xyz(struct i2c_client *client,
				struct bma023_data *coords)
{
	u8 buffer[6];
	buffer[0] = BMA023_X_LSB_REG;
	bma023_xyz_read_reg(client, buffer, 6);
	coords->x = bma023_merge_register_values(buffer[0], buffer[1]);
	coords->y = bma023_merge_register_values(buffer[2], buffer[3]);
	coords->z = bma023_merge_register_values(buffer[4], buffer[5]);
	dev_dbg(&client->dev, "%s: x %d, y %d, z %d\n", __func__,
					coords->x, coords->y, coords->z);
}

/**
 *	bma023_show_xyz		-	show co-ordinate readings
 *	@dev: device of sensor
 *	@attr: device attributes of sysfs node
 *	@buf: buffer for output
 *
 *	Perform a one off read of the sensor data. In non-interrupt mode
 *	the sensor can be used for one off reads of the axes rather than
 *	as an input device.
 */
static ssize_t bma023_show_xyz(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);

	mutex_lock(&sensor->lock);
	bma023_read_xyz(sensor->client, &sensor->data);
	mutex_unlock(&sensor->lock);

	pm_runtime_put(dev);

	return sprintf(buf, "(%d,%d,%d)\n",
		sensor->data.x, sensor->data.y, sensor->data.z);
}
static DEVICE_ATTR(accel_data, S_IRUGO, bma023_show_xyz, NULL);

/**
 *	bma023_set_attr	-	set an attribute on the sensor
 *	@sensor sensor to update
 *	@buf: buffer to parse
 *	@count: length of buffer written
 *	@get: read function to use
 *	@set: write function to use
 *	@data: where to store cached result
 *
 *	Helper function for setting attributes of the sensor. This does
 *	all the parsing and locking in one place calling out to the helper
 *	functions provided.
 */
static ssize_t bma023_set_attr(struct bma023_sensor *sensor,
				const char *buf, size_t count,
				int (*get)(struct i2c_client *),
				int (*set)(struct i2c_client *, u8 val),
				u8 *data)
{
	unsigned long val;
	int ret;

	if (!count)
		return count;

	ret = strict_strtoul(buf, 10, &val);
	if (!ret) {
		/* We don't need the device to be fully powered up for this
		   but we do need to be sure that we force the i²c controller
		   to be awake */
		pm_runtime_get_sync(sensor->dev);
		mutex_lock(&sensor->lock);
		ret = set(sensor->client, val);
		if (ret >= 0) {
			ret = get(sensor->client);
			if (ret >= 0)
				*data = ret;
		}
		mutex_unlock(&sensor->lock);
		pm_runtime_put(sensor->dev);
		if (ret >= 0)
			return count;
	}
	return ret;
}

/* Methods for the sysfs attributes */
static ssize_t bma023_show_power_mode(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", bma023_get_power_mode(sensor->client));
}

static ssize_t bma023_store_power_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	unsigned long val;
	int ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&sensor->lock);
	if (val & !sensor->soft_power)
		pm_runtime_get(dev);
	else if (!val && sensor->soft_power)
		pm_runtime_put(dev);
	sensor->soft_power = val;
	mutex_unlock(&sensor->lock);

	return count;
}

static DEVICE_ATTR(power_mode, S_IRUGO | S_IWUSR,
		bma023_show_power_mode, bma023_store_power_mode);

static ssize_t bma023_show_range(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->range);
}

static ssize_t bma023_store_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_range,
			bma023_set_range,
			&sensor->range);
}

static DEVICE_ATTR(range, S_IRUGO | S_IWUSR,
		bma023_show_range, bma023_store_range);

static ssize_t bma023_show_bandwidth(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->bandwidth);
}

static ssize_t bma023_store_bandwidth(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_bandwidth,
			bma023_set_bandwidth,
			&sensor->bandwidth);
}

static DEVICE_ATTR(bandwidth, S_IRUGO | S_IWUSR,
		bma023_show_bandwidth, bma023_store_bandwidth);

static ssize_t bma023_show_new_data_int(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->new_data_int);
}

static ssize_t bma023_store_new_data_int(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_new_data_int,
			bma023_set_new_data_int,
			&sensor->new_data_int);
}

static DEVICE_ATTR(new_data_int, S_IRUGO | S_IWUSR,
		bma023_show_new_data_int, bma023_store_new_data_int);

static ssize_t bma023_show_hg_int(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->hg_int);
}

static ssize_t bma023_store_hg_int(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_hg_int,
			bma023_set_hg_int,
			&sensor->hg_int);
}

static DEVICE_ATTR(hg_int, S_IRUGO | S_IWUSR,
		bma023_show_hg_int, bma023_store_hg_int);

static ssize_t bma023_show_lg_int(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->lg_int);
}

static ssize_t bma023_store_lg_int(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_lg_int,
			bma023_set_lg_int,
			&sensor->lg_int);
}

static DEVICE_ATTR(lg_int, S_IRUGO | S_IWUSR,
		bma023_show_lg_int, bma023_store_lg_int);

static ssize_t bma023_show_lg_dur(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->lg_dur);
}

static ssize_t bma023_store_lg_dur(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_lg_dur,
			bma023_set_lg_dur,
			&sensor->lg_dur);
}

static DEVICE_ATTR(lg_dur, S_IRUGO | S_IWUSR,
		bma023_show_lg_dur, bma023_store_lg_dur);

static ssize_t bma023_show_lg_thres(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->lg_thres);
}

static ssize_t bma023_store_lg_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_lg_thres,
			bma023_set_lg_thres,
			&sensor->lg_thres);
}

static DEVICE_ATTR(lg_thres, S_IRUGO | S_IWUSR,
		bma023_show_lg_thres, bma023_store_lg_thres);

static ssize_t bma023_show_lg_hyst(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->lg_hyst);
}

static ssize_t bma023_store_lg_hyst(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_lg_hyst,
			bma023_set_lg_hyst,
			&sensor->lg_hyst);
}

static DEVICE_ATTR(lg_hyst, S_IRUGO | S_IWUSR,
		bma023_show_lg_hyst, bma023_store_lg_hyst);

static ssize_t bma023_show_hg_dur(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->hg_dur);
}

static ssize_t bma023_store_hg_dur(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_hg_dur,
			bma023_set_hg_dur,
			&sensor->hg_dur);
}

static DEVICE_ATTR(hg_dur, S_IRUGO | S_IWUSR,
		bma023_show_hg_dur, bma023_store_hg_dur);

static ssize_t bma023_show_hg_thres(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->hg_thres);
}

static ssize_t bma023_store_hg_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_hg_thres,
			bma023_set_hg_thres,
			&sensor->hg_thres);
}

static DEVICE_ATTR(hg_thres, S_IRUGO | S_IWUSR,
		bma023_show_hg_thres, bma023_store_hg_thres);

static ssize_t bma023_show_hg_hyst(struct device *dev,
		struct device_attribute *att, char *buf)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", sensor->hg_hyst);
}

static ssize_t bma023_store_hg_hyst(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	return bma023_set_attr(sensor, buf, count,
			bma023_get_hg_hyst,
			bma023_set_hg_hyst,
			&sensor->hg_hyst);
}

static DEVICE_ATTR(hg_hyst, S_IRUGO | S_IWUSR,
		bma023_show_hg_hyst, bma023_store_hg_hyst);

static struct attribute *bma023_attributes[] = {
	&dev_attr_accel_data.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_new_data_int.attr,
	&dev_attr_hg_int.attr,
	&dev_attr_lg_int.attr,
	&dev_attr_lg_dur.attr,
	&dev_attr_lg_thres.attr,
	&dev_attr_lg_hyst.attr,
	&dev_attr_hg_dur.attr,
	&dev_attr_hg_thres.attr,
	&dev_attr_hg_hyst.attr,
	NULL
};

static const struct attribute_group bma023_group = {
	.attrs	= bma023_attributes,
};

/**
 *	bma023_interrupt_thread	-	handle an IRQ
 *	@irq: interrupt numner
 *	@data: the sensor
 *
 *	Called by the kernel single threaded after an interrupt occurs. Read
 *	the sensor data and generate an input event for it.
 */
static irqreturn_t bma023_interrupt_thread(int irq, void *data)
{
	struct bma023_sensor *sensor = data;

	mutex_lock(&sensor->lock);
	bma023_read_xyz(sensor->client, &sensor->data);
	mutex_unlock(&sensor->lock);

	input_report_abs(sensor->idev, ABS_X, sensor->data.x);
	input_report_abs(sensor->idev, ABS_Y, sensor->data.y);
	input_report_abs(sensor->idev, ABS_Z, sensor->data.z);
	input_sync(sensor->idev);

	return IRQ_HANDLED;
}

/**
 *	bma023_initialize	-	set sensor parameters
 *	@sensor: sensor to configure
 *
 *	Load the actual hardware with the sensor values that were provided
 */
static void bma023_initialize(struct bma023_sensor *sensor)
{
	bma023_set_range(sensor->client, sensor->range);
	bma023_set_bandwidth(sensor->client, sensor->bandwidth);
	bma023_set_new_data_int(sensor->client, sensor->new_data_int);
	bma023_set_hg_dur(sensor->client, sensor->hg_dur);
	bma023_set_hg_thres(sensor->client, sensor->hg_thres);
	bma023_set_hg_hyst(sensor->client, sensor->hg_hyst);
	bma023_set_lg_dur(sensor->client, sensor->lg_dur);
	bma023_set_lg_thres(sensor->client, sensor->lg_thres);
	bma023_set_lg_hyst(sensor->client, sensor->lg_hyst);
	bma023_set_hg_int(sensor->client, sensor->hg_int);
	bma023_set_lg_int(sensor->client, sensor->lg_int);
	bma023_set_power_mode(sensor->client, sensor->power_mode);
	bma023_set_check_reg(sensor->client, BMA023_INSPECT_VAL);
}

/**
 *	bma023_input_open	-	called on input event open
 *	@input: input dev of opened device
 *
 *	The input layer calls this function when input event is opened. The
 *	function will push the device to resume. Then, the device is ready
 *	to provide data.
 */
static int bma023_input_open(struct input_dev *input)
{
	struct bma023_sensor *sensor = input_get_drvdata(input);
	pm_runtime_get(sensor->dev);
	return 0;
}

/**
 *	bma023_input_close	-	called on input event close
 *	@input: input dev of closed device
 *
 *	The input layer calls this function when input event is closed. The
 *	function will push the device to suspend.
 */
static void bma023_input_close(struct input_dev *input)
{
	struct bma023_sensor *sensor = input_get_drvdata(input);
	pm_runtime_put(sensor->dev);
}

/**
 *	bma023_unregister_input_device	-	remove input dev
 *	@sensor: sensor to remove from input
 *
 *	Free the interrupt and input device for the sensor. We must free
 *	the interrupt first
 */
static void bma023_unregister_input_device(struct bma023_sensor *sensor)
{
	struct i2c_client *client = sensor->client;
	if (client->irq > 0)
		free_irq(client->irq, sensor);
	input_unregister_device(sensor->idev);
	sensor->idev = NULL;
}

/**
 *	bma023_register_input_device	-	remove input dev
 *	@sensor: sensor to remove from input
 *
 *	Add an input device to the sensor. This will be used to report
 *	events from the sensor itself.
 */
static int bma023_register_input_device(struct bma023_sensor *sensor)
{
	struct i2c_client *client = sensor->client;
	struct input_dev *idev;
	int ret;
	sensor->idev = input_allocate_device();
	idev = sensor->idev;
	if (!idev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed_alloc;
	}
	idev->name = "BMA023 Sensor";
	idev->open = bma023_input_open;
	idev->close = bma023_input_close;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_X, BMA023_MIN_VALUE,
			BMA023_MAX_VALUE, 0, 0);
	input_set_abs_params(idev, ABS_Y, BMA023_MIN_VALUE,
			BMA023_MAX_VALUE, 0, 0);
	input_set_abs_params(idev, ABS_Z, BMA023_MIN_VALUE,
			BMA023_MAX_VALUE, 0, 0);
	input_set_drvdata(idev, sensor);
	ret = input_register_device(idev);
	if (ret) {
		dev_err(&client->dev, "failed to register input device\n");
		goto failed_reg;
	}
	if (client->irq > 0) {
		ret = request_threaded_irq(client->irq, NULL,
				bma023_interrupt_thread, IRQF_TRIGGER_RISING,
					"bma023", sensor);
		if (ret) {
			dev_err(&client->dev, "can't get IRQ %d, ret %d\n",
					client->irq, ret);
			goto failed_irq;
		}
	}
	return 0;
failed_irq:
	input_unregister_device(idev);
	return ret;

failed_reg:
	if (idev)
		input_free_device(idev);
failed_alloc:
	return ret;
}

/**
 *	bma023_probe	-	device detection callback
 *	@client: i2c client of found device
 *	@id: id match information
 *
 *	The I2C layer calls us when it believes a sensor is present at this
 *	address. Probe to see if this is correct and to validate the device.
 *
 *	If present install the relevant sysfs interfaces and input device.
 */
static int __devinit bma023_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bma023_sensor *sensor;
	struct bma023_platform_data *pdata;
	int ret;
	int auto_delay;		/* PM timeout */

	sensor = kzalloc(sizeof(struct bma023_sensor), GFP_KERNEL);
	if (!sensor) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}
	pdata = client->dev.platform_data;
	sensor->dev = &client->dev;

	sensor->client = client;
	i2c_set_clientdata(client, sensor);

	ret = bma023_read_reg(client, BMA023_CHIP_ID_REG);
	if (ret < 0) {
		dev_err(&client->dev, "failed to detect device\n");
		goto failed_free;
	}
	if (ret != BMA023_CHIP_ID) {
		dev_err(&client->dev, "unsupported chip id\n");
		goto failed_free;
	}

	mutex_init(&sensor->lock);

	ret = sysfs_create_group(&client->dev.kobj, &bma023_group);
	if (ret) {
		dev_err(&client->dev, "failed to create attribute group\n");
		goto failed_free;
	}

	pm_runtime_set_active(&client->dev);

	ret = bma023_register_input_device(sensor);
	if (ret)
		dev_err(&client->dev, "only provide sysfs\n");

	if (pdata) {
		sensor->range = pdata->range;
		sensor->bandwidth = pdata->bandwidth;
		sensor->new_data_int = pdata->new_data_int;
		sensor->hg_int = pdata->hg_int;
		sensor->lg_int = pdata->lg_int;
		sensor->hg_dur = pdata->hg_dur;
		sensor->hg_thres = pdata->hg_thres;
		sensor->hg_hyst = pdata->hg_hyst;
		sensor->lg_dur = pdata->lg_dur;
		sensor->lg_thres = pdata->lg_thres;
		sensor->lg_hyst = pdata->lg_hyst;
		auto_delay = pdata->auto_delay;
	} else {
		sensor->range = BMA023_DEFAULT_RANGE;
		sensor->bandwidth = BMA023_DEFAULT_BANDWIDTH;
		sensor->new_data_int = BMA023_DEFAULT_NEW_DATA_INT;
		sensor->hg_int = BMA023_DEFAULT_HG_INT;
		sensor->lg_int = BMA023_DEFAULT_LG_INT;
		sensor->hg_dur = BMA023_DEFAULT_HG_DURATION;
		sensor->hg_thres = BMA023_DEFAULT_HG_THRESHOLD;
		sensor->hg_hyst = BMA023_DEFAULT_HG_HYST;
		sensor->lg_dur = BMA023_DEFAULT_LG_DURATION;
		sensor->lg_thres = BMA023_DEFAULT_LG_THRESHOLD;
		sensor->lg_hyst = BMA023_DEFAULT_LG_HYST;
		auto_delay = BMA023_DEFAULT_AUTO_DELAY;
	}
	sensor->power_mode = BMA023_SUSPEND;

	bma023_initialize(sensor);

	pm_runtime_enable(&client->dev);
	pm_runtime_set_autosuspend_delay(&client->dev, auto_delay);

	dev_info(&client->dev, "%s registered\n", id->name);
	return 0;

failed_free:
	kfree(sensor);
	return ret;
}

/**
 *	bam023_remove	-	remove a sensor
 *	@client: i2c client of sensor being removed
 *
 *	Our sensor is going away, clean up the resources.
 */
static int __devexit bma023_remove(struct i2c_client *client)
{
	struct bma023_sensor *sensor = i2c_get_clientdata(client);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	if (sensor->idev)
		bma023_unregister_input_device(sensor);
	sysfs_remove_group(&client->dev.kobj, &bma023_group);
	kfree(sensor);
	return 0;
}

#ifdef CONFIG_PM

/**
 *	bma023_suspend		-	called on device suspend
 *	@client: i2c client of sensor
 *	@mesg: actual suspend type
 *
 *	Put the device into sleep mode before we suspend the machine.
 */
static int bma023_suspend(struct i2c_client *client, pm_message_t mesg)
{
	bma023_set_power_mode(client, BMA023_SUSPEND);
	return 0;
}

/**
 *	bma023_resume		-	called on device resume
 *	@client: i2c client of sensor
 *
 *	Put the device into powered mode on resume.
 */
static int bma023_resume(struct i2c_client *client)
{
	struct bma023_sensor *sensor = dev_get_drvdata(&client->dev);
	if (bma023_get_check_reg(client) != BMA023_INSPECT_VAL)
		bma023_initialize(sensor);
	bma023_set_power_mode(client, sensor->power_mode);
	msleep(4);  /* wait for accel chip resume */
	return 0;
}

#else
#define bma023_suspend NULL
#define bma023_resume NULL
#endif

#ifdef CONFIG_PM_RUNTIME

static int bma023_runtime_suspend(struct device *dev)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	bma023_set_power_mode(sensor->client, BMA023_SUSPEND);
	return 0;
}

static int bma023_runtime_resume(struct device *dev)
{
	struct bma023_sensor *sensor = dev_get_drvdata(dev);
	int ret = bma023_set_power_mode(sensor->client, BMA023_RESUME);
	if (ret == 0)
		msleep(4);  /* wait for accel chip resume */
	return ret;
}

static const struct dev_pm_ops bma023_pm = {
	.runtime_suspend = bma023_runtime_suspend,
	.runtime_resume  = bma023_runtime_resume,
};
#endif

static const struct i2c_device_id bma023_ids[] = {
	{ "bma023", 0 },
	{ "smb380", 1 },
	{ "bma150", 2 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma023_ids);

static struct i2c_driver bma023_i2c_driver = {
	.driver	= {
		.name	= "bma023",
#ifdef CONFIG_PM_RUNTIME
		.pm		= &bma023_pm,
#endif		
	},
	.probe		= bma023_probe,
	.remove		= __devexit_p(bma023_remove),
	.suspend	= bma023_suspend,
	.resume		= bma023_resume,
	.id_table	= bma023_ids,
};

static int __init bma023_init(void)
{
	return i2c_add_driver(&bma023_i2c_driver);
}
module_init(bma023_init);

static void __exit bma023_exit(void)
{
	i2c_del_driver(&bma023_i2c_driver);
}
module_exit(bma023_exit);

MODULE_AUTHOR("Kim Kyuwon <q1.kim@samsung.com>, Wistron Corp.");
MODULE_DESCRIPTION("BMA023/SMB380 Tri-axis accelerometer driver");
MODULE_LICENSE("GPL");
