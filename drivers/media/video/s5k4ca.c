/*
 * Driver for S5K4CA (QXGA camera) from Samsung Electronics
 *
 * 1/4" 3.2Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videodev2_samsung.h>
#include <media/s5k4ca_platform.h>

#include "s5k4ca.h"

#define S5K4CA_DRIVER_NAME	"S5K4CA"

#define S5K4CA_USE_BURSTMODE

#define VIEW_FUNCTION_CALL

#ifdef VIEW_FUNCTION_CALL
#define TRACE_CALL	\
	printk("[S5k4CA] function %s line %d executed\n", __func__, __LINE__);
#else
#define TRACE_CALL
#endif

/*
 * Specification
 * Parallel : ITU-R. 656/601 YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Serial : MIPI CSI2 (single lane) YUV422, RGB565, RGB888 (Up to VGA), RAW10
 * Resolution : 1024 (H) x 768 (V)
 * Image control : Brightness, Contrast, Saturation, Sharpness, Glamour
 * Effect : Mono, Negative, Sepia, Aqua, Sketch
 * FPS : 15fps @full resolution, 30fps @VGA, 24fps @720p
 */

/* Camera functional setting values configured by user concept */
struct s5k4ca_userset {
	signed int exposure_bias;	/* V4L2_CID_EXPOSURE */
	unsigned int ae_lock;
	unsigned int awb_lock;
	unsigned int auto_wb;		/* V4L2_CID_AUTO_WHITE_BALANCE */
	unsigned int wb_temp;		/* V4L2_CID_WHITE_BALANCE_TEMPERATURE */
	unsigned int effect;		/* Color FX (AKA Color tone) */
	unsigned int contrast;		/* V4L2_CID_CONTRAST */
	unsigned int saturation;	/* V4L2_CID_SATURATION */
	unsigned int sharpness;		/* V4L2_CID_SHARPNESS */
	unsigned int glamour;
};

struct s5k4ca_state {
	struct s5k4ca_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct s5k4ca_userset userset;

	int freq;
	int isize;
	int ver;
	int fps;

	int preview_in_init;
	int preview_in_init_af;
	int previous_scene_mode;
	int previous_WB_mode;
	unsigned short lux_value;
	int macroType;

	int powered;
};

static inline struct s5k4ca_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4ca_state, sd);
}

static inline int s5k4ca_sensor_read(struct i2c_client *client,
				unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { client->addr, 0, 2, buf };

	TRACE_CALL;
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (unlikely(ret == -EIO))
		goto error;

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (unlikely(ret == -EIO))
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static inline int s5k4ca_sensor_write(struct i2c_client *client,
				unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { client->addr, 0, 4, buf };

	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

	return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int s5k4ba_write_regs(struct v4l2_subdev *sd,
					s5k4ca_short_t regs[], int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, err;

	TRACE_CALL;

	for (i = 0; i < size; i++) {
		err = s5k4ca_sensor_write(client, regs[i].subaddr, regs[i].value);
		if (unlikely(err < 0)) {
			v4l_info(client, "%s: register set failed\n", __func__);
			return err;
		}
	}

	return 0;
}

#ifdef S5K4CA_USE_BURSTMODE
static u8 s5k4ca_buf_for_burstmode[2500];

static inline int s5k4ca_sensor_burst_write(struct i2c_client *client,
					s5k4ca_short_t table[], int size)
{
	int i = 0;
	int err = 0;
	u8 *ptr;

	struct i2c_msg msg = { client->addr, 0, 0, s5k4ca_buf_for_burstmode };

	s5k4ca_buf_for_burstmode[0] = 0x0f;
	s5k4ca_buf_for_burstmode[1] = 0x12;

	ptr = &s5k4ca_buf_for_burstmode[2];

	for (i = 0; i < size && !err; ++i) {
		if (table[i].subaddr != 0x0f12
		    && ptr != &s5k4ca_buf_for_burstmode[2]) {
			/* write in burstmode */
			msg.len = ptr - s5k4ca_buf_for_burstmode;
			err = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
			ptr = &s5k4ca_buf_for_burstmode[2];
		}

		switch (table[i].subaddr) {
		case 0xFCFC:
		case 0x0028:
		case 0x002A:
			/* Set Address */
			ptr = &s5k4ca_buf_for_burstmode[2];
			err = s5k4ca_sensor_write(client,
					table[i].subaddr, table[i].value);
			break;
		case 0x0F12:
			/* make and fill buffer for burst mode write */
			*ptr++ = table[i].value >> 8;
			*ptr++ = table[i].value & 0xff;
			break;
		case 0xFFFF:
			break;
		}
	}

	if (ptr != &s5k4ca_buf_for_burstmode[2]) {
		/* write in burstmode */
		msg.len = ptr - s5k4ca_buf_for_burstmode;
		err = i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
		ptr = &s5k4ca_buf_for_burstmode[2];
	}

	if (unlikely(err < 0)) {
		v4l_info(client, "%s: register set failed\n", __func__);
		return err;
	}

	return 0;
}
#endif /*S5K4CA_USE_BURSTMODE*/

/*
 * Clock configuration
 * Configure expected MCLK from host and return EINVAL if not supported clock
 * frequency is expected
 * 	freq : in Hz
 * 	flag : not supported for now
 */
static int s5k4ca_s_crystal_freq(struct v4l2_subdev *sd, u32 freq, u32 flags)
{
	int err = -EINVAL;

	TRACE_CALL;

	return err;
}

static int s5k4ca_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	int err = 0;

	TRACE_CALL;

	return err;
}

static int s5k4ca_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	int err = 0;

	TRACE_CALL;

	return err;
}

static int s5k4ca_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	TRACE_CALL;

	dev_dbg(&client->dev, "%s\n", __func__);

	return err;
}

static int s5k4ca_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;

	TRACE_CALL;

	dev_dbg(&client->dev, "%s: numerator %d, denominator: %d\n",
		__func__, param->parm.capture.timeperframe.numerator,
		param->parm.capture.timeperframe.denominator);

	return err;
}

static int s5k4ca_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	struct s5k4ca_userset userset = state->userset;
	int err = -EINVAL;

	TRACE_CALL;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ctrl->value = userset.exposure_bias;
		err = 0;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = userset.auto_wb;
		err = 0;
		break;

	case V4L2_CID_COLORFX:
		ctrl->value = userset.effect;
		err = 0;
		break;

	case V4L2_CID_CONTRAST:
		ctrl->value = userset.contrast;
		err = 0;
		break;

	case V4L2_CID_SATURATION:
		ctrl->value = userset.saturation;
		err = 0;
		break;

	case V4L2_CID_SHARPNESS:
		ctrl->value = userset.saturation;
		err = 0;
		break;

	default:
		dev_err(&client->dev, "%s: no such ctrl\n", __func__);
		break;
	}

	return err;
}

static int s5k4ca_sensor_change_wb(struct v4l2_subdev *sd, int type)
{
	int ret = -1;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	switch (type) {
	case 0:
	default:
		state->previous_WB_mode = 0;
		printk("-> WB auto mode\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_wb_auto,
					ARRAY_SIZE(s5k4ca_wb_auto));
		break;
	case 1:
		state->previous_WB_mode = 1;
		printk("-> WB Sunny mode\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_wb_sunny,
					ARRAY_SIZE(s5k4ca_wb_sunny));
		break;
	case 2:
		state->previous_WB_mode = 2;
		printk("-> WB Cloudy mode\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_wb_cloudy,
					ARRAY_SIZE(s5k4ca_wb_cloudy));
		break;
	case 3:
		state->previous_WB_mode = 3;
		printk("-> WB Tungsten mode\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_wb_tungsten,
					ARRAY_SIZE(s5k4ca_wb_tungsten));
		break;
	case 4:
		state->previous_WB_mode = 4;
		printk("-> WB Flourescent mode\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_wb_fluorescent,
					ARRAY_SIZE(s5k4ca_wb_fluorescent));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_effect(struct v4l2_subdev *sd, int type)
{
	int ret = -1;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Effects Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> Mode None\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_off,
					ARRAY_SIZE(s5k4ca_effect_off));
		break;
	case 1:
		printk("-> Mode Gray\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_gray,
					ARRAY_SIZE(s5k4ca_effect_gray));
		break;
	case 2:
		printk("-> Mode Negative\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_negative,
					ARRAY_SIZE(s5k4ca_effect_negative));
		break;
	case 3:
		printk("-> Mode Sepia\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_sepia,
					ARRAY_SIZE(s5k4ca_effect_sepia));
		break;
	case 4:
		printk("-> Mode Aqua\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_aqua,
					ARRAY_SIZE(s5k4ca_effect_aqua));
		break;
	case 5:
		printk("-> Mode Sketch\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_effect_sketch,
					ARRAY_SIZE(s5k4ca_effect_sketch));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_scene_mode(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("\n[S5k4ca] scene mode type is %d\n", type);

	s5k4ba_write_regs(sd, s5k4ca_scene_auto, ARRAY_SIZE(s5k4ca_scene_auto));

	switch (type) {
	case 0:
		printk("\n\n Scene mode auto\n\n");
		state->previous_scene_mode = 0;
		return 0;
	case 1:
		printk("\n\n Scene mode portrait\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_portrait,
					ARRAY_SIZE(s5k4ca_scene_portrait));
		break;
	case 2:
		printk("\n\n Scene mode landscape\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_landscape,
					ARRAY_SIZE(s5k4ca_scene_landscape));
		break;
	case 3:
		printk("\n\n Scene mode sport\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_sport,
					ARRAY_SIZE(s5k4ca_scene_sport));
		break;
	case 4:
		printk("\n\n Scene mode sunset\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_sunset,
					ARRAY_SIZE(s5k4ca_scene_sunset));
		break;
	case 5:
		printk("\n\n Scene mode candlelight\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_candlelight,
					ARRAY_SIZE(s5k4ca_scene_candlelight));
		break;
	case 6:
		printk("\n\n Scene mode fireworks\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_fireworks,
					ARRAY_SIZE(s5k4ca_scene_fireworks));
		break;
	case 7:
		printk("\n\n Scene mode text\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_text,
					ARRAY_SIZE(s5k4ca_scene_text));
		break;
	case 8:
		printk("\n\n Scene mode night\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_night,
					ARRAY_SIZE(s5k4ca_scene_night));
		break;
	case 9:
		printk("\n\n Scene mode beach and snow\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_beach,
					ARRAY_SIZE(s5k4ca_scene_beach));
		break;
	case 10:
		printk("\n\n Scene mode party\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_party,
					ARRAY_SIZE(s5k4ca_scene_party));
		break;
	case 11:
		printk("\n\n Scene mode backlight\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_backlight,
					ARRAY_SIZE(s5k4ca_scene_backlight));
		break;
	case 12://[CDH] this number can changed by Application team. it's temporary number for duskdawn
		printk("\n\n Scene mode dusk and dawn\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_duskdawn,
					ARRAY_SIZE(s5k4ca_scene_duskdawn));
		break;
	case 13:
		printk("\n\n Scene mode fall-color\n\n");
		state->previous_scene_mode = type;
		ret = s5k4ba_write_regs(sd, s5k4ca_scene_fallcolor,
					ARRAY_SIZE(s5k4ca_scene_fallcolor));
		break;
	default:
		printk("\n\n Scene mode default and error\n\n");
		ret = 0;
		break;
	}

	return 0;
}

static int s5k4ca_sensor_change_br(struct v4l2_subdev *sd, int type)
{
	int ret = 0;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Brightness Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> Brightness Minus 4\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_minus4,
					ARRAY_SIZE(s5k4ca_br_minus4));
		break;
	case 1:
		printk("-> Brightness Minus 3\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_minus3,
					ARRAY_SIZE(s5k4ca_br_minus3));
		break;
	case 2:
		printk("-> Brightness Minus 2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_minus2,
					ARRAY_SIZE(s5k4ca_br_minus2));
		break;
	case 3:
		printk("-> Brightness Minus 1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_minus1,
					ARRAY_SIZE(s5k4ca_br_minus1));
		break;
	case 4:
		printk("-> Brightness Zero\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_zero,
					ARRAY_SIZE(s5k4ca_br_zero));
		break;
	case 5:
		printk("-> Brightness Plus 1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_plus1,
					ARRAY_SIZE(s5k4ca_br_plus1));
		break;
	case 6:
		printk("-> Brightness Plus 2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_plus2,
					ARRAY_SIZE(s5k4ca_br_plus2));
		break;
	case 7:
		printk("-> Brightness Plus 3\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_plus3,
					ARRAY_SIZE(s5k4ca_br_plus3));
		break;
	case 8:
		printk("-> Brightness Plus 4\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_br_plus4,
					ARRAY_SIZE(s5k4ca_br_plus4));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_contrast(struct v4l2_subdev *sd, int type)
{
	int ret = 0;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Contras Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Contrast -2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_contrast_m2,
					ARRAY_SIZE(s5k4ca_contrast_m2));
		break;
	case 1:
		printk("-> Contrast -1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_contrast_m1,
					ARRAY_SIZE(s5k4ca_contrast_m1));
		break;
	default:
	case 2:
		printk("-> Contrast 0\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_contrast_0,
					ARRAY_SIZE(s5k4ca_contrast_0));
		break;
	case 3:
		printk("-> Contrast +1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_contrast_p1,
					ARRAY_SIZE(s5k4ca_contrast_p1));
		break;
	case 4:
		printk("-> Contrast +2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_contrast_p2,
					ARRAY_SIZE(s5k4ca_contrast_p2));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_saturation(struct v4l2_subdev *sd, int type)
{
	int ret;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Saturation Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Saturation -2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Saturation_m2,
					ARRAY_SIZE(s5k4ca_Saturation_m2));
		break;
	case 1:
		printk("-> Saturation -1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Saturation_m1,
					ARRAY_SIZE(s5k4ca_Saturation_m1));
		break;
	case 2:
	default:
		printk("-> Saturation 0\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Saturation_0,
					ARRAY_SIZE(s5k4ca_Saturation_0));
		break;
	case 3:
		printk("-> Saturation +1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Saturation_p1,
					ARRAY_SIZE(s5k4ca_Saturation_p1));
		break;
	case 4:
		printk("-> Saturation +2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Saturation_p2,
					ARRAY_SIZE(s5k4ca_Saturation_p2));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_sharpness(struct v4l2_subdev *sd, int type)
{
	int ret;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sharpness Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Sharpness -2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Sharpness_m2,
					ARRAY_SIZE(s5k4ca_Sharpness_m2));
		break;
	case 1:
		printk("-> Sharpness -1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Sharpness_m1,
					ARRAY_SIZE(s5k4ca_Sharpness_m1));
		break;
	case 2:
	default:
		printk("-> Sharpness 0\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Sharpness_0,
					ARRAY_SIZE(s5k4ca_Sharpness_0));
		break;
	case 3:
		printk("-> Sharpness +1\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Sharpness_p1,
					ARRAY_SIZE(s5k4ca_Sharpness_p1));
		break;
	case 4:
		printk("-> Sharpness +2\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_Sharpness_p2,
					ARRAY_SIZE(s5k4ca_Sharpness_p2));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_iso(struct v4l2_subdev *sd, int type)
{
	int ret;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Iso Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> ISO AUTO\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_iso_auto,
					ARRAY_SIZE(s5k4ca_iso_auto));
		break;
	case 1:
		printk("-> ISO 50\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_iso50,
					ARRAY_SIZE(s5k4ca_iso50));
		break;
	case 2:
		printk("-> ISO 100\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_iso100,
					ARRAY_SIZE(s5k4ca_iso100));
		break;
	case 3:
		printk("-> ISO 200\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_iso200,
					ARRAY_SIZE(s5k4ca_iso200));
		break;
	case 4:
		printk("-> ISO 400\n");
		ret = s5k4ba_write_regs(sd, s5k4ca_iso400,
					ARRAY_SIZE(s5k4ca_iso400));
		break;
	}

	return ret;
}

static int s5k4ca_sensor_change_photometry(struct v4l2_subdev *sd, int type)
{
	int ret;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Photometry Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Photometry SPOT\n");
		ret = s5k4ba_write_regs(sd,
				s5k4ca_measure_brightness_spot,
				ARRAY_SIZE(s5k4ca_measure_brightness_spot));
		break;
	case 1:
	default:
		printk("-> Photometry Default\n");
		ret = s5k4ba_write_regs(sd,
				s5k4ca_measure_brightness_default,
				ARRAY_SIZE(s5k4ca_measure_brightness_default));
		break;
	case 2:
		printk("-> Photometry CENTER\n");
		ret = s5k4ba_write_regs(sd,
				s5k4ca_measure_brightness_center,
				ARRAY_SIZE(s5k4ca_measure_brightness_center));
		break;
	}

	return ret;
}

static int s5k4ca_preview_set(struct v4l2_subdev *sd, int preview)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	int ret = 0;
	int delay;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sensor Mode ");

	if (preview) {
		printk("-> Preview ");
		if (!state->preview_in_init)
			ret = s5k4ba_write_regs(sd, s5k4ca_preview,
						ARRAY_SIZE(s5k4ca_preview));
		else
			state->preview_in_init = 0;
		delay = 0;
	} else {
		printk("-> Capture ");

		//AE/AWB UNLOCK
		printk("AF_AWB_UNLOCK ON PREVIEW\n");
		if(state->previous_WB_mode==0)
			s5k4ba_write_regs(sd, s5k4ca_ae_awb_unlock,
					ARRAY_SIZE(s5k4ca_ae_awb_unlock));
		else
			s5k4ba_write_regs(sd, s5k4ca_ae_mwb_unlock,
					ARRAY_SIZE(s5k4ca_ae_mwb_unlock));

		s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(client, 0x002C, 0x7000);
		s5k4ca_sensor_write(client, 0x002E, 0x12FE);

		s5k4ca_sensor_read(client, 0x0F12, &state->lux_value);

		if (state->lux_value <= 0x40) { /* Low light */
			if (state->previous_scene_mode == 8) { //scene night
				printk("Night Low Light light=0x%04x\n", state->lux_value);
				delay = 1600;
				ret = s5k4ba_write_regs(sd,
					s5k4ca_snapshot_nightmode,
					ARRAY_SIZE(s5k4ca_snapshot_nightmode));
			} else {
				printk("Normal Low Light light=0x%04x\n", state->lux_value);
				delay = 800;
				ret = s5k4ba_write_regs(sd,
					s5k4ca_snapshot_low,
					ARRAY_SIZE(s5k4ca_snapshot_low));
			}
		} else {
			printk("Normal Normal Light light=0x%04x\n", state->lux_value);
			delay = 200;
			ret = s5k4ba_write_regs(sd, s5k4ca_snapshot_normal,
					ARRAY_SIZE(s5k4ca_snapshot_normal));
		}
	}

	msleep(delay);

	printk("[CAM-SENSOR] =delay time(%d msec)\n", delay);

	return ret;
}

static int s5k4ca_framerate_set(struct v4l2_subdev *sd, int rate)
{
	int ret = 0;
	int delay = 300;

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sensor Mode ");

	switch (rate) {
	case FRAME_RATE_AUTO:
	default:
		ret = s5k4ba_write_regs(sd, s5k4ca_fps_nonfix,
						ARRAY_SIZE(s5k4ca_fps_nonfix));
		break;
	case FRAME_RATE_7:
		ret = s5k4ba_write_regs(sd, s5k4ca_fps_7fix,
						ARRAY_SIZE(s5k4ca_fps_7fix));
		break;
	case FRAME_RATE_15:
		ret = s5k4ba_write_regs(sd, s5k4ca_fps_15fix,
						ARRAY_SIZE(s5k4ca_fps_15fix));
		break;
	case FRAME_RATE_30:
		ret = s5k4ba_write_regs(sd, s5k4ca_fps_30fix,
						ARRAY_SIZE(s5k4ca_fps_30fix));
		break;
	}

	msleep(delay);

	printk("[CAM-SENSOR] =delay time(%d msec)\n", delay);

	return ret;
}

static int s5k4ca_sensor_af_control(struct v4l2_subdev *sd, int type)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	int count = 50;
	u16 tmpVal = 0;
	int ret = -1;
	int size = 0;
	int i = 0;
	unsigned short light = 0;
#ifdef VIEW_FUNCTION_CALL
	printk("[S5k4CA] %s function %d line launched!\n", __func__, __LINE__);
	printk("[S5k4CA] %s function type is %d\n", __func__, type);
#endif
	switch (type) {
	case 0: // CASE 0 for AF Release

		//AE/AWB UNLOCK
		printk("AF_AWB_UNLOCK on AF RELEASE~!!!\n");
		if (state->previous_WB_mode == 0
		    && state->previous_scene_mode != 4)
			s5k4ba_write_regs(sd, s5k4ca_ae_awb_unlock,
					ARRAY_SIZE(s5k4ca_ae_awb_unlock));
		else
			s5k4ba_write_regs(sd, s5k4ca_ae_mwb_unlock,
					ARRAY_SIZE(s5k4ca_ae_mwb_unlock));

		if (!state->macroType) { //normal AF
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x00F0);
			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000);    //set manual AF
			msleep(133); // 1frame delay, 7.5fps = 133ms

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x00FF);    //00FF: infinity
		} else { //macro AF
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x005F);
			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000);    //set manual AF
			msleep(133); // 1frame delay, 7.5fps = 133ms

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x0050);    //0050: macro
		}
		msleep(100);
		break;

	case 1:
		printk("Focus Mode -> Single\n");


		s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(client, 0x002C, 0x7000);
		s5k4ca_sensor_write(client, 0x002E, 0x12FE);

		s5k4ca_sensor_read(client, 0x0F12, &light);
		if (light < 0x80) { /* Low light AF*/

			size = (ARRAY_SIZE(s5k4ca_af_low_lux_val));
			for (i = 0; i < size; i++)	{
				s5k4ca_sensor_write(client, s5k4ca_af_low_lux_val[i].subaddr,
						    s5k4ca_af_low_lux_val[i].value);
			}
			printk("[CAM-SENSOR] =Low Light AF Single light=0x%04x\n",light);
		} else {
			size = (ARRAY_SIZE(s5k4ca_af_normal_lux_val));
			for (i = 0; i < size; i++)	{
				s5k4ca_sensor_write(client, s5k4ca_af_normal_lux_val[i].subaddr,
						    s5k4ca_af_normal_lux_val[i].value);
			}
			printk("[CAM-SENSOR] =Normal Light AF Single light=0x%04x\n",light);
		}
		s5k4ba_write_regs(sd, s5k4ca_ae_awb_lock,
					ARRAY_SIZE(s5k4ca_ae_awb_lock));

		if (!state->macroType) { //normal AF
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x00DF);  //030E = 00FF

			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual

			msleep(130);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x00E0);

			msleep(50);
		} else {
			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x0028, 0x7000);

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x005F);    //?? ??? ??

			s5k4ca_sensor_write(client, 0x002A, 0x030C);
			s5k4ca_sensor_write(client, 0x0F12, 0x0000);    //set manual AF

			msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

			s5k4ca_sensor_write(client, 0x002A, 0x030E);
			s5k4ca_sensor_write(client, 0x0F12, 0x0050);    //0050: macro

			msleep(50);
		}

		s5k4ca_sensor_write(client, 0x002A, 0x030C);
		s5k4ca_sensor_write(client, 0x0F12, 0x0002); //AF Single
		msleep(50);

		do {
			if( count == 0)
				break;

			s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(client, 0x002C, 0x7000);
			s5k4ca_sensor_write(client, 0x002E, 0x130E);
			if (light < 0x80)
				msleep(250);
			else
				msleep(100);
			s5k4ca_sensor_read(client, 0x0F12, &tmpVal);

			count--;

			printk("CAM 3M AF Status Value = %x \n", tmpVal);
		} while( (tmpVal & 0x3) != 0x3 && (tmpVal & 0x3) != 0x2 );

		if (!count) {
			if (!state->macroType) { //normal AF
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x00DF);  //030E = 00FF

				s5k4ca_sensor_write(client, 0x002A, 0x030C);
				s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual

				msleep(130);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x00E0);

				msleep(50);
			} else {
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x005F);    //?? ??? ??

				s5k4ca_sensor_write(client, 0x002A, 0x030C);
				s5k4ca_sensor_write(client, 0x0F12, 0x0000);    //set manual AF

				msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x0050);    //0050: macro

				msleep(50);
			}

			ret = -1;
			printk("CAM 3M AF_Single Mode Fail.==> TIMEOUT \n");
		}

		if ((tmpVal & 0x3) == 0x02) {
			if (!state->macroType) { //normal AF
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x00DF);  //030E = 00FF

				s5k4ca_sensor_write(client, 0x002A, 0x030C);
				s5k4ca_sensor_write(client, 0x0F12, 0x0000); // AF Manual

				msleep(130);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x00E0);

				msleep(50);
			} else {
				s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(client, 0x0028, 0x7000);

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x005F);    //?? ??? ??

				s5k4ca_sensor_write(client, 0x002A, 0x030C);
				s5k4ca_sensor_write(client, 0x0F12, 0x0000);    //set manual AF

				msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

				s5k4ca_sensor_write(client, 0x002A, 0x030E);
				s5k4ca_sensor_write(client, 0x0F12, 0x0050);    //0050: macro

				msleep(50);
			}

			ret = -1;

			printk("CAM 3M AF_Single Mode Fail.==> FAIL \n");
		}

		if((tmpVal & 0x3) == 0x3) {
			printk("CAM 3M AF_Single Mode SUCCESS. \r\n");
			ret = 0;
		}

		printk("CAM:3M AF_SINGLE SET \r\n");

		break;

	case 2: //Normal AF
		if (state->preview_in_init_af || state->macroType) {
			state->macroType = 0;
			printk("[S5k4CA] Macro Mode off\n");
			size = (ARRAY_SIZE(s5k4ca_af_macro_off));
			for (i = 0; i < size; i++)
				s5k4ca_sensor_write(client, s5k4ca_af_macro_off[i].subaddr,
						    s5k4ca_af_macro_off[i].value);
			state->preview_in_init_af = 0;
		}
		break;
	case 4: //Macro AF
		if (state->preview_in_init_af || !state->macroType) {
			state->macroType = 1;
			printk("[S5k4CA] Macro Mode on\n");
			size = (ARRAY_SIZE(s5k4ca_af_macro_on));
			for (i = 0; i < size; i++)
				s5k4ca_sensor_write(client, s5k4ca_af_macro_on[i].subaddr,
						    s5k4ca_af_macro_on[i].value);
			state->preview_in_init_af = 0;
		}
		break;
	default:
		break;
	}

	return ret;
}

#if 0
/*
 * Arun c
 * Read the exposure time; and use the lux_value from the last
 * picture captured. Fill these values to the pointer supplied
 * by the platform
 */
static int s5k4ca_sensor_exif_read(struct v4l2_subdev *sd, int type)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	unsigned short exif_data[2];

	/* Read the exposure value */
	s5k4ca_sensor_write(client, 0xFCFC, 0xD000);
	s5k4ca_sensor_write(client, 0x002C, 0x7000);
	s5k4ca_sensor_write(client, 0x002E, 0x1C3C);
	s5k4ca_sensor_read(client, 0x0F12, &exif_data[0]);

	exif_data[0] = exif_data[0] / 100;
	exif_data[1] = state->lux_value;

	copy_to_user((void __user*)type, exif_data, sizeof(exif_data));

	printk("[CAM-SENSOR] =%s extime=%d, lux=%d,\n", __func__, exif_data[0], state->lux_value);

	return 0;
}
#endif

static int s5k4ca_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	int err = -EINVAL;

	TRACE_CALL;

	if (!state->powered)
		return -EINVAL;

	printk("[S5k4CA] %s function ctrl->id : %d \n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_CAM_PREVIEW_ONOFF:
		err = s5k4ca_preview_set(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_FRAME_RATE:
		err = s5k4ca_framerate_set(sd, ctrl->value);
		break;	
	case V4L2_CID_FOCUS_AUTO:
		err = s5k4ca_sensor_af_control(sd, ctrl->value);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		err = s5k4ca_sensor_change_wb(sd, ctrl->value);
		break;
	case V4L2_CID_COLORFX:
		err = s5k4ca_sensor_change_effect(sd, ctrl->value);
		break;
	case V4L2_CID_SCENEMODE:
		err = s5k4ca_sensor_change_scene_mode(sd, ctrl->value);
		break;
	case V4L2_CID_BRIGHTNESS:
		err = s5k4ca_sensor_change_br(sd, ctrl->value);
		break;
	case V4L2_CID_CONTRAST:
		err = s5k4ca_sensor_change_contrast(sd, ctrl->value);
		break;
	case V4L2_CID_SATURATION:
		err = s5k4ca_sensor_change_saturation(sd, ctrl->value);
		break;
	case V4L2_CID_SHARPNESS:
		err = s5k4ca_sensor_change_sharpness(sd, ctrl->value);
		break;
	case V4L2_CID_CAM_ISO:
		err = s5k4ca_sensor_change_iso(sd, ctrl->value);
		break;
	case V4L2_CID_PHOTOMETRY:
		err = s5k4ca_sensor_change_photometry(sd, ctrl->value);
		break;
#if 0
	case V4L2_CID_EXIF_DATA:
		err = s5k4ca_sensor_exif_read(sd, ctrl->value);
		break;
#endif
	default:
		dev_err(&client->dev, "%s: no such control\n", __func__);
		err = 0;
		break;
	}

	if (err < 0)
		dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);

	return err;
}

#ifdef S5K4CA_USE_BURSTMODE
static inline void s5k4ca_write_init_table(struct i2c_client *client,
					s5k4ca_short_t table[], int size)
{
	s5k4ca_sensor_burst_write(client, table, size);
}
#else
static inline void s5k4ca_write_init_table(struct i2c_client *client,
					s5k4ca_short_t table[], int size)
{
	int i;
	for (i = 0; i < size; ++i) {
		err = s5k4ca_sensor_write(client,
					table[i].subaddr, table[i].value);
		if (unlikely(err < 0)) {
			v4l_info(client, "%s: register set failed\n",
				 __func__);
			return err;
		}
	}
}
#endif

static int s5k4ca_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k4ca_state *state = to_state(sd);
	int ret = 0;
	int i;

	TRACE_CALL;

	if (!!on == state->powered)
		return 0;

	if (!on) {
		state->powered = 0;
		if (state->pdata->set_power)
			state->pdata->set_power(0);
		return 0;
	};

	v4l_info(client, "%s: camera initialization start\n", __func__);

	if (state->pdata->set_power)
		state->pdata->set_power(1);

	for (i = 0; i < ARRAY_SIZE(s5k4ca_init0); i++) {
		ret = s5k4ca_sensor_write(client, s5k4ca_init0[i].subaddr,
							s5k4ca_init0[i].value);
		if (unlikely(ret < 0))
			goto err;
	}

	msleep(100);

	TRACE_CALL;

	s5k4ca_write_init_table(client, s5k4ca_init1, ARRAY_SIZE(s5k4ca_init1));

	state->preview_in_init = 1;
	state->preview_in_init_af = 1;

	for (i = 0; i < ARRAY_SIZE(s5k4ca_preview); i++) {
		ret = s5k4ca_sensor_write(client,
			s5k4ca_preview[i].subaddr, s5k4ca_preview[i].value);
		if (unlikely(ret < 0))
			goto err;
	}

	TRACE_CALL;

	state->powered = 1;
	return 0;

err:
	v4l_err(client, "Sensor initialization failed.\n");
	return ret;
}

static const struct v4l2_subdev_core_ops s5k4ca_core_ops = {
	.s_power = s5k4ca_s_power,
	.g_ctrl = s5k4ca_g_ctrl,
	.s_ctrl = s5k4ca_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5k4ca_video_ops = {
	.s_crystal_freq = s5k4ca_s_crystal_freq,
	.enum_framesizes = s5k4ca_enum_framesizes,
	.enum_frameintervals = s5k4ca_enum_frameintervals,
	.g_parm = s5k4ca_g_parm,
	.s_parm = s5k4ca_s_parm,

};

static const struct v4l2_subdev_ops s5k4ca_ops = {
	.core = &s5k4ca_core_ops,
	.video = &s5k4ca_video_ops,
};

/*
 * s5k4ca_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int s5k4ca_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct s5k4ca_platform_data *pdata;
	struct s5k4ca_state *state;
	struct v4l2_subdev *sd;

	TRACE_CALL;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}

	state = kzalloc(sizeof(struct s5k4ca_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	state->previous_scene_mode = -1;
	sd = &state->sd;
	strcpy(sd->name, S5K4CA_DRIVER_NAME);

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (pdata->default_width && pdata->default_height) {
		state->pix.width = pdata->default_width;
		state->pix.height = pdata->default_height;
	}

	if (!pdata->pixelformat)
		state->pix.pixelformat = V4L2_PIX_FMT_VYUY;
	else
		state->pix.pixelformat = pdata->pixelformat;

	if (!pdata->freq)
		state->freq = 24000000;	/* 24MHz default */
	else
		state->freq = pdata->freq;

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5k4ca_ops);

	dev_info(&client->dev, "s5k4ca has been probed\n");

	return 0;
}

static int s5k4ca_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	TRACE_CALL;

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id s5k4ca_id[] = {
	{ S5K4CA_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5k4ca_id);

static struct i2c_driver s5k4ca_i2c_driver = {
	.driver = {
		.name = S5K4CA_DRIVER_NAME,
	},
	.probe		= s5k4ca_probe,
	.remove		= s5k4ca_remove,
	.id_table	= s5k4ca_id,
};

static int __init s5k4ca_mod_init(void)
{
	return i2c_add_driver(&s5k4ca_i2c_driver);
}

static void __exit s5k4ca_mod_exit(void)
{
	i2c_del_driver(&s5k4ca_i2c_driver);
}

module_init(s5k4ca_mod_init);
module_exit(s5k4ca_mod_exit);

MODULE_DESCRIPTION("Samsung Electronics S5K4CA UXGA camera driver");
MODULE_AUTHOR("Jinsung Yang <jsgood.yang@samsung.com>");
MODULE_LICENSE("GPL");
