/*
 * Driver for S5K4CA (QXGA camera) from Samsung Electronics
 *
 * 1/4" 3.2Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Original driver for Samsung Galaxy GT-i5800:
 * Copyright (C) 2009, Jinsung Yang <jsgood.yang@samsung.com>
 *
 * Complete rewrite:
 * Copyright 2012, Tomasz Figa <tomasz.figa at gmail.com>
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

#define S5K4CA_DRIVER_NAME	"s5k4ca"

#define VIEW_FUNCTION_CALL

#ifdef VIEW_FUNCTION_CALL
#define TRACE_CALL	\
	printk("[S5k4CA] function %s line %d executed\n", __func__, __LINE__);
#else
#define TRACE_CALL
#endif

struct s5k4ca_state {
	struct i2c_client *client;
	struct s5k4ca_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;

	int frame_rate;
	int focus_mode;
	int auto_focus_result;
	int color_effect;
	int scene_mode;
	int brightness;
	int contrast;
	int saturation;
	int sharpness;
	int iso;
	int photometry;
	int white_balance;
	int capture;
	int ae_awb_lock;

	int freq;
	int isize;
	int ver;
	int fps;

	int powered;

	u8 burst_buffer[2500];
};

static inline struct s5k4ca_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4ca_state, sd);
}

static int s5k4ca_sensor_write(struct s5k4ca_state *state,
				unsigned short subaddr, unsigned short val)
{
	struct i2c_client *client = state->client;
	unsigned char buf[] = {
		subaddr >> 8, subaddr & 0xff, val >> 8, val & 0xff
	};
	return i2c_master_send(client, buf, sizeof(buf));
}

static int s5k4ca_write_regs(struct s5k4ca_state *state,
					struct s5k4ca_request table[], int size)
{
	struct i2c_client *client = state->client;
	u8 *buffer = state->burst_buffer;
	u8 *ptr = &buffer[2];
	int err = 0;
	int i = 0;

	buffer[0] = S5K4CA_DATA_MAGIC >> 8;
	buffer[1] = S5K4CA_DATA_MAGIC & 0xff;

	for (i = 0; i < size && err >= 0; ++i) {
		switch (table[i].subaddr) {
		case S5K4CA_BANK_MAGIC:
		case S5K4CA_PAGE_MAGIC:
		case S5K4CA_REG_MAGIC:
			if (ptr != &buffer[2]) {
				/* write in burst mode */
				err = i2c_master_send(client,
							buffer, ptr - buffer);
				ptr = &buffer[2];
				if (err < 0)
					break;
			}
			/* Set Address */
			err = s5k4ca_sensor_write(state,
					table[i].subaddr, table[i].value);
			break;
		case S5K4CA_DATA_MAGIC:
			/* make and fill buffer for burst mode write */
			*ptr++ = table[i].value >> 8;
			*ptr++ = table[i].value & 0xff;
			break;
		case S5K4CA_MSLEEP_MAGIC:
			msleep(table[i].value);
			break;
		}
	}

	if (ptr != &buffer[2])
		/* write in burst mode */
		err = i2c_master_send(client, buffer, ptr - buffer);

	if (unlikely(err < 0)) {
		v4l_err(client, "%s: register set failed\n", __func__);
		return err;
	}

	return 0;
}

static int s5k4ca_sensor_read(struct s5k4ca_state *state,
				unsigned short subaddr, unsigned short *data)
{
	struct i2c_client *client = state->client;
	unsigned char buf[] = { 0x0F, 0x12 };
	int ret;

	TRACE_CALL;

	s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
	s5k4ca_sensor_write(state, 0x002C, 0x7000);
	s5k4ca_sensor_write(state, 0x002E, subaddr);

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		goto error;

	ret = i2c_master_recv(client, buf, sizeof(buf));
	if (ret < 0)
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

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
	int err = 0;

	TRACE_CALL;

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_FRAME_RATE:
		ctrl->value = state->frame_rate;
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		ctrl->value = state->focus_mode;
		break;
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		ctrl->value = state->auto_focus_result;
		break;
	case V4L2_CID_CAMERA_EFFECT:
		ctrl->value = state->color_effect;
		break;
	case V4L2_CID_CAMERA_SCENE_MODE:
		ctrl->value = state->scene_mode;
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		ctrl->value = state->brightness;
		break;
	case V4L2_CID_CAMERA_CONTRAST:
		ctrl->value = state->contrast;
		break;
	case V4L2_CID_CAMERA_SATURATION:
		ctrl->value = state->saturation;
		break;
	case V4L2_CID_CAMERA_SHARPNESS:
		ctrl->value = state->sharpness;
		break;
	case V4L2_CID_CAMERA_ISO:
		ctrl->value = state->iso;
		break;
	case V4L2_CID_CAMERA_METERING:
		ctrl->value = state->photometry;
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		ctrl->value = state->white_balance;
		break;
	default:
		dev_err(&client->dev, "%s: no such ctrl\n", __func__);
		err = -EINVAL;
		break;
	}

	return err;
}

static int s5k4ca_set_wb(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	switch (type) {
	case WHITE_BALANCE_AUTO:
		state->white_balance = 0;
		printk("-> WB auto mode\n");
		ret = s5k4ca_write_regs(state, s5k4ca_wb_auto,
					ARRAY_SIZE(s5k4ca_wb_auto));
		break;
	case WHITE_BALANCE_SUNNY:
		state->white_balance = 1;
		printk("-> WB Sunny mode\n");
		ret = s5k4ca_write_regs(state, s5k4ca_wb_sunny,
					ARRAY_SIZE(s5k4ca_wb_sunny));
		break;
	case WHITE_BALANCE_CLOUDY:
		state->white_balance = 2;
		printk("-> WB Cloudy mode\n");
		ret = s5k4ca_write_regs(state, s5k4ca_wb_cloudy,
					ARRAY_SIZE(s5k4ca_wb_cloudy));
		break;
	case WHITE_BALANCE_TUNGSTEN:
		state->white_balance = 3;
		printk("-> WB Tungsten mode\n");
		ret = s5k4ca_write_regs(state, s5k4ca_wb_tungsten,
					ARRAY_SIZE(s5k4ca_wb_tungsten));
		break;
	case WHITE_BALANCE_FLUORESCENT:
		state->white_balance = 4;
		printk("-> WB Flourescent mode\n");
		ret = s5k4ca_write_regs(state, s5k4ca_wb_fluorescent,
					ARRAY_SIZE(s5k4ca_wb_fluorescent));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->white_balance = type;
	return 0;
}

static int s5k4ca_set_effect(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Effects Mode %d", type);

	switch (type) {
	case IMAGE_EFFECT_NONE:
		printk("-> Mode None\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_off,
					ARRAY_SIZE(s5k4ca_effect_off));
		break;
	case IMAGE_EFFECT_BNW:
		printk("-> Mode Gray\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_gray,
					ARRAY_SIZE(s5k4ca_effect_gray));
		break;
	case IMAGE_EFFECT_NEGATIVE:
		printk("-> Mode Negative\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_negative,
					ARRAY_SIZE(s5k4ca_effect_negative));
		break;
	case IMAGE_EFFECT_SEPIA:
		printk("-> Mode Sepia\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_sepia,
					ARRAY_SIZE(s5k4ca_effect_sepia));
		break;
	case IMAGE_EFFECT_AQUA:
		printk("-> Mode Aqua\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_aqua,
					ARRAY_SIZE(s5k4ca_effect_aqua));
		break;
	case IMAGE_EFFECT_ANTIQUE:
		printk("-> Mode Sketch\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_sketch,
					ARRAY_SIZE(s5k4ca_effect_sketch));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->color_effect = type;
	return 0;
}

static int s5k4ca_set_scene_mode(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("\n[S5k4ca] scene mode type is %d\n", type);

	ret = s5k4ca_write_regs(state, s5k4ca_scene_auto,
						ARRAY_SIZE(s5k4ca_scene_auto));
	if (ret < 0)
		return ret;

	switch (type) {
	case SCENE_MODE_NONE:
		break;
	case SCENE_MODE_PORTRAIT:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_portrait,
					ARRAY_SIZE(s5k4ca_scene_portrait));
		break;
	case SCENE_MODE_LANDSCAPE:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_landscape,
					ARRAY_SIZE(s5k4ca_scene_landscape));
		break;
	case SCENE_MODE_SPORTS:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_sport,
					ARRAY_SIZE(s5k4ca_scene_sport));
		break;
	case SCENE_MODE_SUNSET:
	case SCENE_MODE_CANDLE_LIGHT:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_sunset_candlelight,
				ARRAY_SIZE(s5k4ca_scene_sunset_candlelight));
		break;
	case SCENE_MODE_FIREWORKS:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_fireworks,
					ARRAY_SIZE(s5k4ca_scene_fireworks));
		break;
	case SCENE_MODE_TEXT:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_text,
					ARRAY_SIZE(s5k4ca_scene_text));
		break;
	case SCENE_MODE_NIGHTSHOT:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_night,
					ARRAY_SIZE(s5k4ca_scene_night));
		break;
	case SCENE_MODE_BEACH_SNOW:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_beach,
					ARRAY_SIZE(s5k4ca_scene_beach));
		break;
	case SCENE_MODE_PARTY_INDOOR:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_party,
					ARRAY_SIZE(s5k4ca_scene_party));
		break;
	case SCENE_MODE_BACK_LIGHT:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_backlight,
					ARRAY_SIZE(s5k4ca_scene_backlight));
		break;
	case SCENE_MODE_DUST_DAWN:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_duskdawn,
					ARRAY_SIZE(s5k4ca_scene_duskdawn));
		break;
	case SCENE_MODE_FALL_COLOR:
		ret = s5k4ca_write_regs(state, s5k4ca_scene_fallcolor,
					ARRAY_SIZE(s5k4ca_scene_fallcolor));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->scene_mode = type;
	return 0;
}

static int s5k4ca_set_br(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Brightness Mode %d", type);

	switch (type) {
	case EV_MINUS_4:
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus4,
						ARRAY_SIZE(s5k4ca_br_minus4));
		break;
	case EV_MINUS_3:
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus3,
						ARRAY_SIZE(s5k4ca_br_minus3));
		break;
	case EV_MINUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus2,
						ARRAY_SIZE(s5k4ca_br_minus2));
		break;
	case EV_MINUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus1,
						ARRAY_SIZE(s5k4ca_br_minus1));
		break;
	case EV_DEFAULT:
		ret = s5k4ca_write_regs(state, s5k4ca_br_zero,
						ARRAY_SIZE(s5k4ca_br_zero));
		break;
	case EV_PLUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus1,
						ARRAY_SIZE(s5k4ca_br_plus1));
		break;
	case EV_PLUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus2,
						ARRAY_SIZE(s5k4ca_br_plus2));
		break;
	case EV_PLUS_3:
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus3,
						ARRAY_SIZE(s5k4ca_br_plus3));
		break;
	case EV_PLUS_4:
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus4,
						ARRAY_SIZE(s5k4ca_br_plus4));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->brightness = type;
	return 0;
}

static int s5k4ca_set_contrast(struct v4l2_subdev *sd, int type)
{
	int ret = 0;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Contras Mode %d",type);

	switch (type) {
	case CONTRAST_MINUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_m2,
					ARRAY_SIZE(s5k4ca_contrast_m2));
		break;
	case CONTRAST_MINUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_m1,
					ARRAY_SIZE(s5k4ca_contrast_m1));
		break;
	case CONTRAST_DEFAULT:
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_0,
					ARRAY_SIZE(s5k4ca_contrast_0));
		break;
	case CONTRAST_PLUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_p1,
					ARRAY_SIZE(s5k4ca_contrast_p1));
		break;
	case CONTRAST_PLUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_p2,
					ARRAY_SIZE(s5k4ca_contrast_p2));
		break;
	}

	if (ret < 0)
		return ret;

	state->contrast = type;
	return 0;
}

static int s5k4ca_set_saturation(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Saturation Mode %d",type);

	switch (type) {
	case SATURATION_MINUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_m2,
					ARRAY_SIZE(s5k4ca_Saturation_m2));
		break;
	case SATURATION_MINUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_m1,
					ARRAY_SIZE(s5k4ca_Saturation_m1));
		break;
	case SATURATION_DEFAULT:
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_0,
					ARRAY_SIZE(s5k4ca_Saturation_0));
		break;
	case SATURATION_PLUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_p1,
					ARRAY_SIZE(s5k4ca_Saturation_p1));
		break;
	case SATURATION_PLUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_p2,
					ARRAY_SIZE(s5k4ca_Saturation_p2));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->saturation = type;
	return 0;
}

static int s5k4ca_set_sharpness(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sharpness Mode %d",type);

	switch (type) {
	case SHARPNESS_MINUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_m2,
					ARRAY_SIZE(s5k4ca_Sharpness_m2));
		break;
	case SHARPNESS_MINUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_m1,
					ARRAY_SIZE(s5k4ca_Sharpness_m1));
		break;
	case SHARPNESS_DEFAULT:
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_0,
					ARRAY_SIZE(s5k4ca_Sharpness_0));
		break;
	case SHARPNESS_PLUS_1:
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_p1,
					ARRAY_SIZE(s5k4ca_Sharpness_p1));
		break;
	case SHARPNESS_PLUS_2:
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_p2,
					ARRAY_SIZE(s5k4ca_Sharpness_p2));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->sharpness = type;
	return 0;
}

static int s5k4ca_set_iso(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Iso Mode %d",type);

	switch (type) {
	case ISO_AUTO:
		printk("-> ISO AUTO\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso_auto,
					ARRAY_SIZE(s5k4ca_iso_auto));
		break;
	case ISO_50:
		printk("-> ISO 50\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso50,
					ARRAY_SIZE(s5k4ca_iso50));
		break;
	case ISO_100:
		printk("-> ISO 100\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso100,
					ARRAY_SIZE(s5k4ca_iso100));
		break;
	case ISO_200:
		printk("-> ISO 200\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso200,
					ARRAY_SIZE(s5k4ca_iso200));
		break;
	case ISO_400:
		printk("-> ISO 400\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso400,
					ARRAY_SIZE(s5k4ca_iso400));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->iso = type;
	return 0;
}

static int s5k4ca_set_photometry(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Photometry Mode %d", type);

	switch (type) {
	case METERING_SPOT:
		ret = s5k4ca_write_regs(state, s5k4ca_photometry_spot,
					ARRAY_SIZE(s5k4ca_photometry_spot));
		break;
	case METERING_MATRIX:
		ret = s5k4ca_write_regs(state, s5k4ca_photometry_matrix,
					ARRAY_SIZE(s5k4ca_photometry_matrix));
		break;
	case METERING_CENTER:
		ret = s5k4ca_write_regs(state, s5k4ca_photometry_center,
					ARRAY_SIZE(s5k4ca_photometry_center));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->photometry = type;
	return 0;
}

static int s5k4ca_set_ae_awb_lock(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =AE AWB Lock Mode %d", type);

	switch (type) {
	case AE_UNLOCK_AWB_UNLOCK:
		ret = s5k4ca_write_regs(state, s5k4ca_awb_ae_unlock,
					ARRAY_SIZE(s5k4ca_awb_ae_unlock));
		break;
	case AE_LOCK_AWB_UNLOCK:
		ret = s5k4ca_write_regs(state, s5k4ca_awb_ae_lock,
					ARRAY_SIZE(s5k4ca_awb_ae_lock));
		break;
	case AE_UNLOCK_AWB_LOCK:
		ret = s5k4ca_write_regs(state, s5k4ca_mwb_ae_unlock,
					ARRAY_SIZE(s5k4ca_mwb_ae_unlock));
		break;
	case AE_LOCK_AWB_LOCK:
		ret = s5k4ca_write_regs(state, s5k4ca_mwb_ae_lock,
					ARRAY_SIZE(s5k4ca_mwb_ae_lock));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->ae_awb_lock = type;
	return 0;
}

struct s5k4ca_format {
	unsigned int width;
	unsigned int height;
	struct s5k4ca_request *table;
	unsigned int table_length;
};

#define S5K4CA_FORMAT(w, h, table) \
	{ (w), (h), (table), ARRAY_SIZE((table)) }

static struct s5k4ca_format s5k4ca_formats[] = {
	S5K4CA_FORMAT(640, 480, s5k4ca_res_vga),
	S5K4CA_FORMAT(1024, 768, s5k4ca_res_xga),
	S5K4CA_FORMAT(1280, 960, s5k4ca_res_sxga),
	S5K4CA_FORMAT(1600, 1200, s5k4ca_res_uxga),
	S5K4CA_FORMAT(2048, 1536, s5k4ca_res_qxga),
};

static int s5k4ca_s_mbus_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *fmt)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret;
	int i;

	TRACE_CALL;

	if (!state->powered)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(s5k4ca_formats); ++i)
		if (fmt->width < s5k4ca_formats[i].width
		    && fmt->height < s5k4ca_formats[i].height)
			break;

	if (i == ARRAY_SIZE(s5k4ca_formats))
		return -EINVAL;

	ret = s5k4ca_write_regs(state, s5k4ca_formats[i].table,
						s5k4ca_formats[i].table_length);
	if (ret < 0)
		return ret;

	fmt->width = s5k4ca_formats[i].width;
	fmt->height = s5k4ca_formats[i].height;
	fmt->code = V4L2_MBUS_FMT_VYUY8_2X8;
	msleep(300);

	return 0;
}

static int s5k4ca_framerate_set(struct v4l2_subdev *sd, int rate)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =frame rate = %d\n", rate);

	switch (rate) {
	case FRAME_RATE_AUTO:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_auto,
						ARRAY_SIZE(s5k4ca_fps_auto));
		break;
	case FRAME_RATE_7:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_7,
						ARRAY_SIZE(s5k4ca_fps_7));
		break;
	case FRAME_RATE_15:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_15,
						ARRAY_SIZE(s5k4ca_fps_15));
		break;
	case FRAME_RATE_30:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_30,
						ARRAY_SIZE(s5k4ca_fps_30));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	msleep(300);
	state->frame_rate = rate;
	return 0;
}

static int s5k4ca_set_focus_mode(struct v4l2_subdev *sd, int mode)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret;

	switch(mode) {
	case FOCUS_MODE_AUTO:
		ret = s5k4ca_write_regs(state, s5k4ca_focus_mode_normal,
					ARRAY_SIZE(s5k4ca_focus_mode_normal));
		break;
	case FOCUS_MODE_MACRO:
		ret = s5k4ca_write_regs(state, s5k4ca_focus_mode_macro,
					ARRAY_SIZE(s5k4ca_focus_mode_macro));
		break;
	case FOCUS_MODE_INFINITY:
		ret = s5k4ca_write_regs(state, s5k4ca_focus_mode_infinity,
					ARRAY_SIZE(s5k4ca_focus_mode_infinity));
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	state->focus_mode = mode;
	return ret;
}

static int s5k4ca_set_capture(struct v4l2_subdev *sd, int mode)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret;

	if (mode)
		ret = s5k4ca_write_regs(state, s5k4ca_snapshot,
						ARRAY_SIZE(s5k4ca_snapshot));
	else
		ret = s5k4ca_write_regs(state, s5k4ca_preview,
						ARRAY_SIZE(s5k4ca_preview));

	if (ret < 0)
		return ret;

	state->capture = mode;
	return ret;
}

static int s5k4ca_set_auto_focus(struct v4l2_subdev *sd, int val)
{
	struct s5k4ca_state *state = to_state(sd);
	int count = 50;
	u16 stat = 0;
	u16 lux_value = 0;
	int ret;

	if (state->focus_mode == FOCUS_MODE_INFINITY)
		return 0;

	if (val == AUTO_FOCUS_OFF)
		return s5k4ca_set_focus_mode(sd, state->focus_mode);

	/* Get lux_value. */
	ret = s5k4ca_sensor_read(state, 0x12FE, &lux_value);
	if (ret < 0)
		return ret;

	if (lux_value < 128)  /* Low light AF */
		ret = s5k4ca_write_regs(state, s5k4ca_af_low_lux_val,
					ARRAY_SIZE(s5k4ca_af_low_lux_val));
	else
		ret = s5k4ca_write_regs(state, s5k4ca_af_normal_lux_val,
					ARRAY_SIZE(s5k4ca_af_normal_lux_val));

	if (ret < 0)
		return ret;

	if (state->focus_mode == FOCUS_MODE_MACRO)
		ret = s5k4ca_write_regs(state, s5k4ca_af_start_macro,
					ARRAY_SIZE(s5k4ca_af_start_macro));
	else
		ret = s5k4ca_write_regs(state, s5k4ca_af_start_normal,
					ARRAY_SIZE(s5k4ca_af_start_normal));

	if (ret < 0)
		return ret;

	do {
		if (lux_value < 128)
			msleep(250);
		else
			msleep(100);

		ret = s5k4ca_sensor_read(state, 0x130E, &stat);
		if (ret < 0)
			return ret;
	} while (--count && (stat & 3) < 2);

	if (!count || (stat & 3) == 2) {
		if (state->focus_mode == FOCUS_MODE_MACRO)
			ret = s5k4ca_write_regs(state, s5k4ca_af_stop_macro,
					ARRAY_SIZE(s5k4ca_af_stop_macro));
		else
			ret = s5k4ca_write_regs(state, s5k4ca_af_stop_normal,
					ARRAY_SIZE(s5k4ca_af_stop_normal));

		if (ret < 0)
			return ret;

		printk("[CAM-SENSOR] =Auto focus failed\n");

		state->auto_focus_result = 0;
		return 0;
	}

	printk("[CAM-SENSOR] =Auto focus successful\n");
	state->auto_focus_result = 1;
	return 0;
}

static int s5k4ca_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k4ca_state *state = to_state(sd);
	int err;

	TRACE_CALL;

	if (!state->powered)
		return -EINVAL;

	printk("[S5k4CA] %s function ctrl->id : %d \n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_FRAME_RATE:
		err = s5k4ca_framerate_set(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		err = s5k4ca_set_focus_mode(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		err = s5k4ca_set_auto_focus(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		err = s5k4ca_set_wb(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_EFFECT:
		err = s5k4ca_set_effect(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_SCENE_MODE:
		err = s5k4ca_set_scene_mode(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_BRIGHTNESS:
		err = s5k4ca_set_br(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_CONTRAST:
		err = s5k4ca_set_contrast(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_SATURATION:
		err = s5k4ca_set_saturation(sd, ctrl->value);
		break;
	case V4L2_CID_SHARPNESS:
		err = s5k4ca_set_sharpness(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_SHARPNESS:
		err = s5k4ca_set_iso(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_METERING:
		err = s5k4ca_set_photometry(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_AEAWB_LOCK_UNLOCK:
		err = s5k4ca_set_ae_awb_lock(sd, ctrl->value);
		break;
	case V4L2_CID_CAMERA_CAPTURE:
		err = s5k4ca_set_capture(sd, ctrl->value);
		break;
	default:
		return -EINVAL;
	}

	return err;
}

static int s5k4ca_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret = 0;

	TRACE_CALL;

	if (!!on == state->powered)
		return 0;

	if (!on) {
		state->powered = 0;
		if (state->pdata->set_power)
			state->pdata->set_power(0);
		return 0;
	};

	v4l_info(state->client, "%s: camera initialization start\n", __func__);

	if (state->pdata->set_power)
		state->pdata->set_power(1);

	TRACE_CALL;

	ret = s5k4ca_write_regs(state, s5k4ca_init, ARRAY_SIZE(s5k4ca_init));
	if (ret < 0)
		goto err;

	ret = s5k4ca_write_regs(state, s5k4ca_preview,
						ARRAY_SIZE(s5k4ca_preview));
	if (ret < 0)
		goto err;

	TRACE_CALL;

	state->powered = 1;
	return 0;

err:
	v4l_err(state->client, "Sensor initialization failed.\n");
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
	.s_mbus_fmt = s5k4ca_s_mbus_fmt,
};

static const struct v4l2_subdev_ops s5k4ca_ops = {
	.core = &s5k4ca_core_ops,
	.video = &s5k4ca_video_ops,
};

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

	state->client = client;
	state->pdata = pdata;
	state->scene_mode = -1;
	sd = &state->sd;
	strcpy(sd->name, S5K4CA_DRIVER_NAME);

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

MODULE_DESCRIPTION("Samsung Electronics S5K4CA QXGA camera driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPL");
