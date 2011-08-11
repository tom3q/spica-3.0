/*
 * drivers/i2c/chips/akm8973.c - akm8973 compass driver
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 * Author: viral wang <viralwang@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Revised by AKM 2010/11/08
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm8973.h>
#include <linux/earlysuspend.h>

#define AKM8973_DEBUG		1
#define AKM8973_DEBUG_MSG	1
#define AKM8973_DEBUG_FUNC	0
#define AKM8973_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define AKM8973_RETRY_COUNT	10
#define AKM8973_DEFAULT_DELAY	100

#if AKM8973_DEBUG_MSG
#define AKMDBG(format, ...)	\
		printk(KERN_INFO "AKM8973 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM8973_DEBUG_FUNC
#define AKMFUNC(func) \
		printk(KERN_INFO "AKM8973 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

static struct i2c_client *this_client;

struct akm8973_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend akm_early_suspend;
};

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;

static int failure_count;

static short akmd_delay = AKM8973_DEFAULT_DELAY;

/*AKM DEBUG Interface*/
static akm_s64 akmd_mdelay = 100000000;
static akm_s64 akmd_adelay = 100000000;
static akm_s64 akmd_mvdelay = 100000000;
/*DEBUG IF END*/

static atomic_t suspend_flag = ATOMIC_INIT(0);

static struct akm8973_platform_data *pdata;

static int AKI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if AKM8973_DEBUG_DATA
	int i;
	char addr = rxData[0];
#endif
#ifdef AKM8973_DEBUG
	/* Caller should check parameter validity.*/
	if ((rxData == NULL) || (length < 1)) {
		return -EINVAL;
	}
#endif
	for (loop_i = 0; loop_i < AKM8973_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= AKM8973_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
				__func__, AKM8973_RETRY_COUNT);
		return -EIO;
	}
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
	for (i = 0; i < length; i++) {
		printk(KERN_INFO " %02x", rxData[i]);
	}
    printk(KERN_INFO "\n");
#endif
	return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
#if AKM8973_DEBUG_DATA
	int i;
#endif
#ifdef AKM8973_DEBUG
	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2)) {
		return -EINVAL;
	}
#endif
	for (loop_i = 0; loop_i < AKM8973_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= AKM8973_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n",
				__func__, AKM8973_RETRY_COUNT);
		return -EIO;
	}
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=",
			length, txData[0]);
	for (i = 0; i < (length-1); i++) {
		printk(KERN_INFO " %02x", txData[i + 1]);
	}
	printk(KERN_INFO "\n");
#endif
	return 0;
}

static void AKECS_Reset(void)
{
	gpio_set_value(pdata->gpio_RST, 0);
	udelay(120);
	gpio_set_value(pdata->gpio_RST, 1);
}

static int AKECS_SetMode_Measure(void)
{
	char buffer[2];

	atomic_set(&data_ready, 0);

	/* Set measure mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_E2PRead(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_E2P_READ;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];
	int ret;

	/* Set powerdown mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_POWERDOWN;
	/* Set data */
	ret = AKI2C_TxData(buffer, 2);
	if (ret < 0) {
		return ret;
	}

	/* Dummy read for clearing INT pin */
	buffer[0] = AK8973_REG_TMPS;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	if (ret < 0) {
		return ret;
	}
	return ret;
}

static int AKECS_SetMode(char mode)
{
	int ret;

	switch (mode) {
	case AK8973_MODE_MEASURE:
		ret = AKECS_SetMode_Measure();
		break;
	case AK8973_MODE_E2P_READ:
		ret = AKECS_SetMode_E2PRead();
		break;
	case AK8973_MODE_POWERDOWN:
		ret = AKECS_SetMode_PowerDown();
		break;
	default:
		AKMDBG("%s: Unknown mode(%d)", __func__, mode);
		return -EINVAL;
	}
	/* wait at least 300us after changing mode */
	udelay(300);
	return ret;
}

static int AKECS_CheckDevice(void)
{
	int ret;
	ret = AKECS_SetMode_PowerDown();
	if (ret < 0) {
		printk(KERN_ERR "AKM8973 AKECS_CheckDevice: set power down mode error\n");
	}
	return ret;
}

static int AKECS_GetData(char *rbuf, int size)
{
#ifdef AKM8973_DEBUG
	/* This function is not exposed, so parameters
	   should be checked internally.*/
	if ((rbuf == NULL) || (size < SENSOR_DATA_SIZE)) {
		return -EINVAL;
	}
#endif
	wait_event_interruptible_timeout(
		data_ready_wq,
		atomic_read(&data_ready),
		1000);
	if (!atomic_read(&data_ready)) {
		AKMDBG("%s: data_ready is not set.", __func__);
		if (!atomic_read(&suspend_flag)) {
			AKMDBG("%s: suspend_flag is not set.", __func__);
			failure_count++;
			if (failure_count >= MAX_FAILURE_COUNT) {
				printk(KERN_ERR
					"AKM8973 AKECS_GetData: "
					"successive %d failure.\n",
					failure_count);
				atomic_set(&open_flag, -1);
				wake_up(&open_wq);
				failure_count = 0;
			}
		}
		return -1;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(rbuf, sense_data, size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);

	failure_count = 0;
	return 0;
}

static void AKECS_SetYPR(short *rbuf)
{
	struct akm8973_data *data = i2c_get_clientdata(this_client);
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "AKM8973 %s:\n", __func__);
	printk(KERN_INFO "  yaw =%6d, pitch =%6d, roll =%6d\n",
		   rbuf[0], rbuf[1], rbuf[2]);
	printk(KERN_INFO "  tmp =%6d, m_stat =%6d, g_stat =%6d\n",
		   rbuf[3], rbuf[4], rbuf[5]);
	printk(KERN_INFO "  Acceleration[LSB]: %6d,%6d,%6d\n",
	       rbuf[6], rbuf[7], rbuf[8]);
	printk(KERN_INFO "  Geomagnetism[LSB]: %6d,%6d,%6d\n",
	       rbuf[9], rbuf[10], rbuf[11]);
#endif
	/* Report magnetic sensor information */
	if (atomic_read(&m_flag)) {
		input_report_abs(data->input_dev, ABS_RX, rbuf[0]);
		input_report_abs(data->input_dev, ABS_RY, rbuf[1]);
		input_report_abs(data->input_dev, ABS_RZ, rbuf[2]);
		input_report_abs(data->input_dev, ABS_RUDDER, rbuf[4]);
	}

	/* Report acceleration sensor information */
	if (atomic_read(&a_flag)) {
		input_report_abs(data->input_dev, ABS_X, rbuf[6]);
		input_report_abs(data->input_dev, ABS_Y, rbuf[7]);
		input_report_abs(data->input_dev, ABS_Z, rbuf[8]);
		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]);
	}

	/* Report temperature information */
	if (atomic_read(&t_flag)) {
		input_report_abs(data->input_dev, ABS_THROTTLE, rbuf[3]);
	}

	if (atomic_read(&mv_flag)) {
		input_report_abs(data->input_dev, ABS_HAT0X, rbuf[9]);
		input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[10]);
		input_report_abs(data->input_dev, ABS_BRAKE, rbuf[11]);
	}

	input_sync(data->input_dev);
}

static void AKECS_SetYPR_Select(short *rbuf)
{
	struct akm8973_data *data = i2c_get_clientdata(this_client);
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "AKM8973 %s: flag =0x%X\n", __func__, rbuf[0]);
	printk(KERN_INFO "  yaw =%6d, pitch =%6d, roll =%6d, tmp =%6d\n",
		   rbuf[1], rbuf[2], rbuf[3], rbuf[4]);
	printk(KERN_INFO "  Acceleration[LSB]: %6d,%6d,%6d, g_stat =%6d\n",
	       rbuf[5], rbuf[6], rbuf[7], rbuf[8]);
	printk(KERN_INFO "  Geomagnetism[LSB]: %6d,%6d,%6d, m_stat =%6d\n",
	       rbuf[9], rbuf[10], rbuf[11], rbuf[12]);
#endif
	/* Report orientation sensor information */
	if (atomic_read(&m_flag) && (rbuf[0] & ORI_DATA_READY)) {
		input_report_abs(data->input_dev, ABS_RX, rbuf[1]);
		input_report_abs(data->input_dev, ABS_RY, rbuf[2]);
		input_report_abs(data->input_dev, ABS_RZ, rbuf[3]);
		input_report_abs(data->input_dev, ABS_RUDDER, rbuf[12]);
	}
	/* Report acceleration sensor information */
	if (atomic_read(&a_flag) && (rbuf[0] & ACC_DATA_READY)) {
		input_report_abs(data->input_dev, ABS_X, rbuf[5]);
		input_report_abs(data->input_dev, ABS_Y, rbuf[6]);
		input_report_abs(data->input_dev, ABS_Z, rbuf[7]);
		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[8]);
	}
	/* Report magnetic sensor information */
	if (atomic_read(&mv_flag) && (rbuf[0] & MAG_DATA_READY)) {
		input_report_abs(data->input_dev, ABS_HAT0X, rbuf[9]);
		input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[10]);
		input_report_abs(data->input_dev, ABS_BRAKE, rbuf[11]);
		input_report_abs(data->input_dev, ABS_PRESSURE, rbuf[12]);
	}
	/* Report temperature information */
	if (atomic_read(&t_flag) && (rbuf[0] & MAG_DATA_READY)) {
		input_report_abs(data->input_dev, ABS_THROTTLE, rbuf[4]);
	}

	input_sync(data->input_dev);
}

static int AKECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&t_flag, 1);
	atomic_set(&mv_flag, 1);
}

static void AKECS_GetDelay(akm_s64 delay[3])
{
	if (atomic_read(&m_flag)) {
		delay[0] = akmd_mdelay;
	} else {
		delay[0] = -1;
	}
	if (atomic_read(&a_flag)) {
		delay[1] = akmd_adelay;
	} else {
		delay[1] = -1;
	}
	if (atomic_read(&mv_flag)) {
		delay[2] = akmd_mvdelay;
	} else {
		delay[2] = -1;
	}
}


/***** akm_aot functions ***************************************/
static int akm_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;

	AKMFUNC("akm_aot_open");
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	AKMDBG("akm_aot_open (%d,%d,%d)",
		   atomic_read(&open_count),
		   atomic_read(&open_flag),
		   atomic_read(&reserve_open_flag));
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
	AKMFUNC("akm_aot_release");
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static long
akm_aot_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
	akm_s64 nano_delay;

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_TFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		if (flag < 0 || flag > 1) {
			return -EINVAL;
		}
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_APP_SET_MDELAY:
	case ECS_IOCTL_APP_SET_ADELAY:
	case ECS_IOCTL_APP_SET_MVDELAY:
		if (copy_from_user(&nano_delay, argp, sizeof(nano_delay))) {
			return -EFAULT;
		}
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		AKMDBG("MFLAG is set to %d", flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		AKMDBG("AFLAG is set to %d", flag);
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_TFLAG:
		atomic_set(&t_flag, flag);
		AKMDBG("TFLAG is set to %d", flag);
		break;
	case ECS_IOCTL_APP_GET_TFLAG:
		flag = atomic_read(&t_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		AKMDBG("MVFLAG is set to %d", flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		akmd_delay = flag;
		/*DEBUG IF*/
		akmd_mdelay = akmd_adelay = akmd_mvdelay = akmd_delay * 1000000;
		/*DEBUG IF END*/
		AKMDBG("Delay is set to %d", flag);
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = akmd_delay;
		break;
/*AKM DEBUG*/
	case ECS_IOCTL_APP_SET_MDELAY:
		akmd_mdelay = nano_delay;
		break;
	case ECS_IOCTL_APP_SET_ADELAY:
		akmd_adelay = nano_delay;
		break;
	case ECS_IOCTL_APP_SET_MVDELAY:
		akmd_mvdelay = nano_delay;
		break;
/*AKM DEBUG END*/
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag))) {
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

/***** akmd functions ********************************************/
static int akmd_open(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_open");
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_release");
	AKECS_CloseDone();
	return 0;
}

static long
akmd_ioctl(struct file *file, unsigned int cmd,
		   unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	char rwbuf[RWBUF_SIZE];	/* for READ/WRITE */
	char mode;			/* for SET_MODE*/
	short value[12];	/* for SET_YPR */
	short valueSel[13];	/* for SET_YPR_SELECT */
	short delay;		/* for GET_DELAY */
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = -1;		/* Return value. */
	akm_s64 nano_delay[3];	/* for GET_NANO_DELAY */

	switch (cmd) {
	case ECS_IOCTL_WRITE:
	case ECS_IOCTL_READ:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&value, argp, sizeof(value))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR_SELECT:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&valueSel, argp, sizeof(valueSel))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_WRITE:
		AKMFUNC("IOCTL_WRITE");
		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_READ:
		AKMFUNC("IOCTL_READ");
		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_RESET:
		AKMFUNC("IOCTL_RESET");
		AKECS_Reset();
		break;
	case ECS_IOCTL_SET_MODE:
		AKMFUNC("IOCTL_SET_MODE");
		ret = AKECS_SetMode(mode);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_GETDATA:
		AKMFUNC("IOCTL_GET_DATA");
		ret = AKECS_GetData(sData, SENSOR_DATA_SIZE);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		AKECS_SetYPR(value);
		break;
	case ECS_IOCTL_SET_YPR_SELECT:
		AKECS_SetYPR_Select(valueSel);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		AKMFUNC("IOCTL_GET_OPEN_STATUS");
		status = AKECS_GetOpenStatus();
		AKMDBG("AKECS_GetOpenStatus returned (%d)", status);
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		AKMFUNC("IOCTL_GET_CLOSE_STATUS");
		status = AKECS_GetCloseStatus();
		AKMDBG("AKECS_GetCloseStatus returned (%d)", status);
		break;
	case ECS_IOCTL_GET_DELAY:
		AKMFUNC("IOCTL_GET_DELAY");
		delay = akmd_delay;
		break;
	case ECS_IOCTL_GET_NANO_DELAY:
		AKMFUNC("IOCTL_GET_NANO_DELAY");
		AKECS_GetDelay(nano_delay);
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, rwbuf[0]+1)) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GETDATA:
		if (copy_to_user(argp, &sData, sizeof(sData))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_NANO_DELAY:
		if (copy_to_user(argp, &nano_delay, sizeof(nano_delay))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static void akm8973_work_func(struct work_struct *work)
{
	char buffer[SENSOR_DATA_SIZE];
	int ret;

	memset(buffer, 0, SENSOR_DATA_SIZE);
	buffer[0] = AK8973_REG_ST;
	ret = AKI2C_RxData(buffer, SENSOR_DATA_SIZE);
	if (ret < 0) {
		printk(KERN_ERR "AKM8973 akm8973_work_func: I2C failed\n");
		goto WORK_FUNC_END;
	}
	/* Check ST bit */
	if ((buffer[0] & 0x01) != 0x01) {
		printk(KERN_ERR "AKM8973 akm8973_work_func: ST is not set\n");
		goto WORK_FUNC_END;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, buffer, SENSOR_DATA_SIZE);
	atomic_set(&data_ready, 1);
	wake_up(&data_ready_wq);
	mutex_unlock(&sense_data_mutex);

WORK_FUNC_END:
	enable_irq(this_client->irq);

	AKMFUNC("akm8973_work_func");
}

static irqreturn_t akm8973_interrupt(int irq, void *dev_id)
{
	struct akm8973_data *data = dev_id;
	AKMFUNC("akm8973_interrupt");
	disable_irq(this_client->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLY_SUSPEND
static void akm8973_early_suspend(struct early_suspend *handler)
{
	AKMFUNC("akm8973_early_suspend");
	atomic_set(&suspend_flag, 1);
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
	disable_irq(this_client->irq);
	AKMDBG("suspended with flag=%d",
	       atomic_read(&reserve_open_flag));
}

static void akm8973_early_resume(struct early_suspend *handler)
{
	AKMFUNC("akm8973_early_resume");
	enable_irq(this_client->irq);
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);
	AKMDBG("resumed with flag=%d",
	       atomic_read(&reserve_open_flag));
}
#endif

/*********************************************/
static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.compat_ioctl = akmd_ioctl,
};

static struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.compat_ioctl = akm_aot_ioctl,
};

static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8973_dev",
	.fops = &akmd_fops,
};

static struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8973_aot",
	.fops = &akm_aot_fops,
};

/*********************************************/
int akm8973_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm8973_data *akm;
	int err = 0;

	AKMFUNC("akm8973_probe");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR
			"AKM8973 akm8973_probe: "
			"check_functionality failed.\n");
		err = -ENODEV;
		goto exit0;
	}

	/* Allocate memory for driver data */
	akm = kzalloc(sizeof(struct akm8973_data), GFP_KERNEL);
	if (!akm) {
		printk(KERN_ERR
			"AKM8973 akm8973_probe: "
			"memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}

	INIT_WORK(&akm->work, akm8973_work_func);
	i2c_set_clientdata(client, akm);

	/* Check platform data*/
	if (client->dev.platform_data == NULL) {
		printk(KERN_ERR
			"AKM8973 akm8973_probe: "
			"platform data is NULL\n");
		err = -ENOMEM;
		goto exit2;
	}
	/* Copy to global variable */
	pdata = client->dev.platform_data;
	this_client = client;

	/* Check connection */
	err = AKECS_CheckDevice();
	if (err < 0) {
		printk(KERN_ERR
			"AKM8973 akm8973_probe: "
			"set power down mode error\n");
		goto exit3;
	}

	/* IRQ */
	err = request_irq(client->irq, akm8973_interrupt, IRQ_TYPE_EDGE_RISING,
					  "akm8973_INT", akm);
	if (err < 0) {
		printk(KERN_ERR
			"AKM8973 akm8973_probe: "
			"request irq failed\n");
		goto exit4;
	}

	/* Declare input device */
	akm->input_dev = input_allocate_device();
	if (!akm->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "AKM8973 akm8973_probe: "
			   "Failed to allocate input device\n");
		goto exit5;
	}
	/* Setup input device */
	set_bit(EV_ABS, akm->input_dev->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(akm->input_dev, ABS_RX, 0, 23040, 0, 0);
	/* pitch (-180, 180) */
	input_set_abs_params(akm->input_dev, ABS_RY, -11520, 11520, 0, 0);
	/* roll (-90, 90) */
	input_set_abs_params(akm->input_dev, ABS_RZ, -5760, 5760, 0, 0);
	/* x-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_X, -5760, 5760, 0, 0);
	/* y-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Y, -5760, 5760, 0, 0);
	/* z-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Z, -5760, 5760, 0, 0);
	/* temparature */
	input_set_abs_params(akm->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(akm->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
	/* x-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev, ABS_HAT0X, -2048, 2032, 0, 0);
	/* y-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev, ABS_HAT0Y, -2048, 2032, 0, 0);
	/* z-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev, ABS_BRAKE, -2048, 2032, 0, 0);
	/* Set name */
	akm->input_dev->name = "compass";

	/* Register */
	err = input_register_device(akm->input_dev);
	if (err) {
		printk(KERN_ERR
		       "AKM8973 akm8973_probe: "
			   "Unable to register input device\n");
		goto exit6;
	}

	err = misc_register(&akmd_device);
	if (err) {
		printk(KERN_ERR
			   "AKM8973 akm8973_probe: "
			   "akmd_device register failed\n");
		goto exit7;
	}

	err = misc_register(&akm_aot_device);
	if (err) {
		printk(KERN_ERR
		       "AKM8973 akm8973_probe: "
			   "akm_aot_device register failed\n");
		goto exit8;
	}

	mutex_init(&sense_data_mutex);

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&t_flag, 1);
	atomic_set(&mv_flag, 1);

#ifdef CONFIG_HAS_EARLY_SUSPEND
	akm->akm_early_suspend.suspend = akm8973_early_suspend;
	akm->akm_early_suspend.resume = akm8973_early_resume;
	register_early_suspend(&akm->akm_early_suspend);
#endif

	AKMDBG("successfully probed.");
	return 0;

exit8:
	misc_deregister(&akmd_device);
exit7:
	input_unregister_device(akm->input_dev);
exit6:
	input_free_device(akm->input_dev);
exit5:
	free_irq(client->irq, akm);
exit4:
exit3:
exit2:
	kfree(akm);
exit1:
exit0:
	return err;
}

static int akm8973_remove(struct i2c_client *client)
{
	struct akm8973_data *akm = i2c_get_clientdata(client);
	AKMFUNC("akm8973_remove");
	unregister_early_suspend(&akm->akm_early_suspend);
	misc_deregister(&akm_aot_device);
	misc_deregister(&akmd_device);
	input_unregister_device(akm->input_dev);
	free_irq(client->irq, akm);
	kfree(akm);
	AKMDBG("successfully removed.");
	return 0;
}

static const struct i2c_device_id akm8973_id[] = {
	{AKM8973_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver akm8973_driver = {
	.probe		= akm8973_probe,
	.remove 	= akm8973_remove,
	.id_table	= akm8973_id,
	.driver = {
		.name = AKM8973_I2C_NAME,
	},
};

static int __init akm8973_init(void)
{
	printk(KERN_INFO "AKM8973 compass driver: initialize\n");
	return i2c_add_driver(&akm8973_driver);
}

static void __exit akm8973_exit(void)
{
	printk(KERN_INFO "AKM8973 compass driver: release\n");
	i2c_del_driver(&akm8973_driver);
}

module_init(akm8973_init);
module_exit(akm8973_exit);

MODULE_AUTHOR("viral wang <viral_wang@htc.com>");
MODULE_DESCRIPTION("AKM8973 compass driver");
MODULE_LICENSE("GPL");

