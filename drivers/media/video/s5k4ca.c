/*
 * Driver for S5K4CA (QXGA camera) from Samsung Electronics
 *
 * 1/4" 3.2Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (C) 	2009, Jinsung Yang <jsgood.yang@samsung.com>
 *			2012, Tomasz Figa <tomasz.figa at gmail.com>
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

	int freq;
	int isize;
	int ver;
	int fps;

	int preview_in_init;
	int preview_in_init_af;
	unsigned short lux_value;

	int powered;

	u8 burst_buffer[2500];
};

static inline struct s5k4ca_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4ca_state, sd);
}

static inline int s5k4ca_sensor_read(struct s5k4ca_state *state,
				unsigned short subaddr, unsigned short *data)
{
	struct i2c_client *client = state->client;
	unsigned char buf[] = { subaddr >> 8, subaddr & 0xff };
	int ret;

	TRACE_CALL;

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

static inline int s5k4ca_sensor_write(struct s5k4ca_state *state,
				unsigned short subaddr, unsigned short val)
{
	struct i2c_client *client = state->client;
	unsigned char buf[] = {
		subaddr >> 8, subaddr & 0xff, val >> 8, val & 0xff
	};
	return i2c_master_send(client, buf, sizeof(buf));
}

static inline int s5k4ca_write_regs(struct s5k4ca_state *state,
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
	case V4L2_CID_COLORFX:
		ctrl->value = state->color_effect;
		break;
	case V4L2_CID_SCENEMODE:
		ctrl->value = state->scene_mode;
		break;
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = state->brightness;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = state->contrast;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = state->saturation;
		break;
	case V4L2_CID_SHARPNESS:
		ctrl->value = state->sharpness;
		break;
	case V4L2_CID_CAM_ISO:
		ctrl->value = state->iso;
		break;
	case V4L2_CID_PHOTOMETRY:
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
	int ret = -1;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	switch (type) {
	case WHITE_BALANCE_AUTO:
	default:
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
	}

	return ret;
}

static int s5k4ca_set_effect(struct v4l2_subdev *sd, int type)
{
	int ret = -1;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Effects Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> Mode None\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_off,
					ARRAY_SIZE(s5k4ca_effect_off));
		break;
	case 1:
		printk("-> Mode Gray\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_gray,
					ARRAY_SIZE(s5k4ca_effect_gray));
		break;
	case 2:
		printk("-> Mode Negative\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_negative,
					ARRAY_SIZE(s5k4ca_effect_negative));
		break;
	case 3:
		printk("-> Mode Sepia\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_sepia,
					ARRAY_SIZE(s5k4ca_effect_sepia));
		break;
	case 4:
		printk("-> Mode Aqua\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_aqua,
					ARRAY_SIZE(s5k4ca_effect_aqua));
		break;
	case 5:
		printk("-> Mode Sketch\n");
		ret = s5k4ca_write_regs(state, s5k4ca_effect_sketch,
					ARRAY_SIZE(s5k4ca_effect_sketch));
		break;
	}

	return ret;
}

static int s5k4ca_set_scene_mode(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("\n[S5k4ca] scene mode type is %d\n", type);

	s5k4ca_write_regs(state, s5k4ca_scene_auto, ARRAY_SIZE(s5k4ca_scene_auto));

	switch (type) {
	case 0:
		printk("\n\n Scene mode auto\n\n");
		state->scene_mode = 0;
		return 0;
	case 1:
		printk("\n\n Scene mode portrait\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_portrait,
					ARRAY_SIZE(s5k4ca_scene_portrait));
		break;
	case 2:
		printk("\n\n Scene mode landscape\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_landscape,
					ARRAY_SIZE(s5k4ca_scene_landscape));
		break;
	case 3:
		printk("\n\n Scene mode sport\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_sport,
					ARRAY_SIZE(s5k4ca_scene_sport));
		break;
	case 4:
		printk("\n\n Scene mode sunset\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_sunset,
					ARRAY_SIZE(s5k4ca_scene_sunset));
		break;
	case 5:
		printk("\n\n Scene mode candlelight\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_candlelight,
					ARRAY_SIZE(s5k4ca_scene_candlelight));
		break;
	case 6:
		printk("\n\n Scene mode fireworks\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_fireworks,
					ARRAY_SIZE(s5k4ca_scene_fireworks));
		break;
	case 7:
		printk("\n\n Scene mode text\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_text,
					ARRAY_SIZE(s5k4ca_scene_text));
		break;
	case 8:
		printk("\n\n Scene mode night\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_night,
					ARRAY_SIZE(s5k4ca_scene_night));
		break;
	case 9:
		printk("\n\n Scene mode beach and snow\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_beach,
					ARRAY_SIZE(s5k4ca_scene_beach));
		break;
	case 10:
		printk("\n\n Scene mode party\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_party,
					ARRAY_SIZE(s5k4ca_scene_party));
		break;
	case 11:
		printk("\n\n Scene mode backlight\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_backlight,
					ARRAY_SIZE(s5k4ca_scene_backlight));
		break;
	case 12://[CDH] this number can changed by Application team. it's temporary number for duskdawn
		printk("\n\n Scene mode dusk and dawn\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_duskdawn,
					ARRAY_SIZE(s5k4ca_scene_duskdawn));
		break;
	case 13:
		printk("\n\n Scene mode fall-color\n\n");
		state->scene_mode = type;
		ret = s5k4ca_write_regs(state, s5k4ca_scene_fallcolor,
					ARRAY_SIZE(s5k4ca_scene_fallcolor));
		break;
	default:
		printk("\n\n Scene mode default and error\n\n");
		ret = 0;
		break;
	}

	return 0;
}

static int s5k4ca_set_br(struct v4l2_subdev *sd, int type)
{
	int ret = 0;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Brightness Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> Brightness Minus 4\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus4,
					ARRAY_SIZE(s5k4ca_br_minus4));
		break;
	case 1:
		printk("-> Brightness Minus 3\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus3,
					ARRAY_SIZE(s5k4ca_br_minus3));
		break;
	case 2:
		printk("-> Brightness Minus 2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus2,
					ARRAY_SIZE(s5k4ca_br_minus2));
		break;
	case 3:
		printk("-> Brightness Minus 1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_minus1,
					ARRAY_SIZE(s5k4ca_br_minus1));
		break;
	case 4:
		printk("-> Brightness Zero\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_zero,
					ARRAY_SIZE(s5k4ca_br_zero));
		break;
	case 5:
		printk("-> Brightness Plus 1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus1,
					ARRAY_SIZE(s5k4ca_br_plus1));
		break;
	case 6:
		printk("-> Brightness Plus 2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus2,
					ARRAY_SIZE(s5k4ca_br_plus2));
		break;
	case 7:
		printk("-> Brightness Plus 3\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus3,
					ARRAY_SIZE(s5k4ca_br_plus3));
		break;
	case 8:
		printk("-> Brightness Plus 4\n");
		ret = s5k4ca_write_regs(state, s5k4ca_br_plus4,
					ARRAY_SIZE(s5k4ca_br_plus4));
		break;
	}

	return ret;
}

static int s5k4ca_set_contrast(struct v4l2_subdev *sd, int type)
{
	int ret = 0;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Contras Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Contrast -2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_m2,
					ARRAY_SIZE(s5k4ca_contrast_m2));
		break;
	case 1:
		printk("-> Contrast -1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_m1,
					ARRAY_SIZE(s5k4ca_contrast_m1));
		break;
	default:
	case 2:
		printk("-> Contrast 0\n");
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_0,
					ARRAY_SIZE(s5k4ca_contrast_0));
		break;
	case 3:
		printk("-> Contrast +1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_p1,
					ARRAY_SIZE(s5k4ca_contrast_p1));
		break;
	case 4:
		printk("-> Contrast +2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_contrast_p2,
					ARRAY_SIZE(s5k4ca_contrast_p2));
		break;
	}

	return ret;
}

static int s5k4ca_set_saturation(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Saturation Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Saturation -2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_m2,
					ARRAY_SIZE(s5k4ca_Saturation_m2));
		break;
	case 1:
		printk("-> Saturation -1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_m1,
					ARRAY_SIZE(s5k4ca_Saturation_m1));
		break;
	case 2:
	default:
		printk("-> Saturation 0\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_0,
					ARRAY_SIZE(s5k4ca_Saturation_0));
		break;
	case 3:
		printk("-> Saturation +1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_p1,
					ARRAY_SIZE(s5k4ca_Saturation_p1));
		break;
	case 4:
		printk("-> Saturation +2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Saturation_p2,
					ARRAY_SIZE(s5k4ca_Saturation_p2));
		break;
	}

	return ret;
}

static int s5k4ca_set_sharpness(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sharpness Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Sharpness -2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_m2,
					ARRAY_SIZE(s5k4ca_Sharpness_m2));
		break;
	case 1:
		printk("-> Sharpness -1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_m1,
					ARRAY_SIZE(s5k4ca_Sharpness_m1));
		break;
	case 2:
	default:
		printk("-> Sharpness 0\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_0,
					ARRAY_SIZE(s5k4ca_Sharpness_0));
		break;
	case 3:
		printk("-> Sharpness +1\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_p1,
					ARRAY_SIZE(s5k4ca_Sharpness_p1));
		break;
	case 4:
		printk("-> Sharpness +2\n");
		ret = s5k4ca_write_regs(state, s5k4ca_Sharpness_p2,
					ARRAY_SIZE(s5k4ca_Sharpness_p2));
		break;
	}

	return ret;
}

static int s5k4ca_set_iso(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Iso Mode %d",type);

	switch (type) {
	case 0:
	default:
		printk("-> ISO AUTO\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso_auto,
					ARRAY_SIZE(s5k4ca_iso_auto));
		break;
	case 1:
		printk("-> ISO 50\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso50,
					ARRAY_SIZE(s5k4ca_iso50));
		break;
	case 2:
		printk("-> ISO 100\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso100,
					ARRAY_SIZE(s5k4ca_iso100));
		break;
	case 3:
		printk("-> ISO 200\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso200,
					ARRAY_SIZE(s5k4ca_iso200));
		break;
	case 4:
		printk("-> ISO 400\n");
		ret = s5k4ca_write_regs(state, s5k4ca_iso400,
					ARRAY_SIZE(s5k4ca_iso400));
		break;
	}

	return ret;
}

static int s5k4ca_set_photometry(struct v4l2_subdev *sd, int type)
{
	int ret;
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Photometry Mode %d",type);

	switch (type) {
	case 0:
		printk("-> Photometry SPOT\n");
		ret = s5k4ca_write_regs(state,
				s5k4ca_measure_brightness_spot,
				ARRAY_SIZE(s5k4ca_measure_brightness_spot));
		break;
	case 1:
	default:
		printk("-> Photometry Default\n");
		ret = s5k4ca_write_regs(state,
				s5k4ca_measure_brightness_default,
				ARRAY_SIZE(s5k4ca_measure_brightness_default));
		break;
	case 2:
		printk("-> Photometry CENTER\n");
		ret = s5k4ca_write_regs(state,
				s5k4ca_measure_brightness_center,
				ARRAY_SIZE(s5k4ca_measure_brightness_center));
		break;
	}

	return ret;
}

static int s5k4ca_s_mbus_fmt(struct v4l2_subdev *sd,
						struct v4l2_mbus_framefmt *fmt)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret = 0;
	int delay;
	int preview;

	TRACE_CALL;

	if (!state->powered)
		return -EINVAL;

	if (fmt->width > 2048 || fmt->height > 1536)
		return -EINVAL;

	if (fmt->width <= 1024 && fmt->height <= 768) {
		preview = 1;
		fmt->width = 1024;
		fmt->height = 768;
	} else {
		preview = 0;
		fmt->width = 2048;
		fmt->height = 1536;
	}

	fmt->code = V4L2_MBUS_FMT_VYUY8_2X8;

	printk("[CAM-SENSOR] =Sensor Mode ");

	if (preview) {
		printk("-> Preview ");
		if (!state->preview_in_init)
			ret = s5k4ca_write_regs(state, s5k4ca_preview,
						ARRAY_SIZE(s5k4ca_preview));
		else
			state->preview_in_init = 0;
		delay = 0;
	} else {
		printk("-> Capture ");

		//AE/AWB UNLOCK
		printk("AF_AWB_UNLOCK ON PREVIEW\n");
		if(state->white_balance==0)
			s5k4ca_write_regs(state, s5k4ca_ae_awb_unlock,
					ARRAY_SIZE(s5k4ca_ae_awb_unlock));
		else
			s5k4ca_write_regs(state, s5k4ca_ae_mwb_unlock,
					ARRAY_SIZE(s5k4ca_ae_mwb_unlock));

		s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(state, 0x002C, 0x7000);
		s5k4ca_sensor_write(state, 0x002E, 0x12FE);

		s5k4ca_sensor_read(state, 0x0F12, &state->lux_value);

		if (state->lux_value <= 0x40) { /* Low light */
			if (state->scene_mode == 8) { //scene night
				printk("Night Low Light light=0x%04x\n", state->lux_value);
				delay = 1600;
				ret = s5k4ca_write_regs(state,
					s5k4ca_snapshot_nightmode,
					ARRAY_SIZE(s5k4ca_snapshot_nightmode));
			} else {
				printk("Normal Low Light light=0x%04x\n", state->lux_value);
				delay = 800;
				ret = s5k4ca_write_regs(state,
					s5k4ca_snapshot_low,
					ARRAY_SIZE(s5k4ca_snapshot_low));
			}
		} else {
			printk("Normal Normal Light light=0x%04x\n", state->lux_value);
			delay = 200;
			ret = s5k4ca_write_regs(state, s5k4ca_snapshot_normal,
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
	struct s5k4ca_state *state = to_state(sd);

	TRACE_CALL;

	printk("[CAM-SENSOR] =Sensor Mode ");

	switch (rate) {
	case FRAME_RATE_AUTO:
	default:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_nonfix,
						ARRAY_SIZE(s5k4ca_fps_nonfix));
		break;
	case FRAME_RATE_7:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_7fix,
						ARRAY_SIZE(s5k4ca_fps_7fix));
		break;
	case FRAME_RATE_15:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_15fix,
						ARRAY_SIZE(s5k4ca_fps_15fix));
		break;
	case FRAME_RATE_30:
		ret = s5k4ca_write_regs(state, s5k4ca_fps_30fix,
						ARRAY_SIZE(s5k4ca_fps_30fix));
		break;
	}

	msleep(delay);

	printk("[CAM-SENSOR] =delay time(%d msec)\n", delay);

	return ret;
}

static int s5k4ca_set_focus_mode(struct v4l2_subdev *sd, int mode)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret;

	switch(mode) {
	case FOCUS_MODE_AUTO:
		ret = s5k4ca_write_regs(state,
					s5k4ca_focus_mode_normal,
					ARRAY_SIZE(s5k4ca_focus_mode_normal));
		break;
	case FOCUS_MODE_MACRO:
		ret = s5k4ca_write_regs(state,
					s5k4ca_focus_mode_macro,
					ARRAY_SIZE(s5k4ca_focus_mode_macro));
		break;
	case FOCUS_MODE_INFINITY:
		ret = s5k4ca_write_regs(state,
					s5k4ca_focus_mode_infinity,
					ARRAY_SIZE(s5k4ca_focus_mode_infinity));
		break;
	default:
		return -EINVAL;
	}

	if (!ret)
		state->focus_mode = mode;

	return ret;
}

static int s5k4ca_set_auto_focus(struct v4l2_subdev *sd, int val)
{
	struct s5k4ca_state *state = to_state(sd);
	int ret = 0, count = 50;
	u16 tmpVal = 0;

	// Get lux_value.
	s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
	s5k4ca_sensor_write(state, 0x002C, 0x7000);
	s5k4ca_sensor_write(state, 0x002E, 0x12FE);
	s5k4ca_sensor_read(state, 0x0F12, &state->lux_value);
	if (state->lux_value < 0x80)  //Low light AF
		s5k4ca_write_regs(state, s5k4ca_af_low_lux_val, ARRAY_SIZE(s5k4ca_af_low_lux_val));
	else
		s5k4ca_write_regs(state, s5k4ca_af_normal_lux_val, ARRAY_SIZE(s5k4ca_af_normal_lux_val));

	if (state->focus_mode == FOCUS_MODE_MACRO) {
		s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(state, 0x0028, 0x7000);
		s5k4ca_sensor_write(state, 0x002A, 0x030E);
		s5k4ca_sensor_write(state, 0x0F12, 0x0030);
		s5k4ca_sensor_write(state, 0x002A, 0x030C);
		s5k4ca_sensor_write(state, 0x0F12, 0x0000); //AF manual
		msleep(140);
		s5k4ca_sensor_write(state, 0x002A, 0x030E);
		s5k4ca_sensor_write(state, 0x0F12, 0x0040);
		msleep(100);
	} else {
		s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(state, 0x0028, 0x7000);
		s5k4ca_sensor_write(state, 0x002A, 0x030E);
		s5k4ca_sensor_write(state, 0x0F12, 0x00FF);
		s5k4ca_sensor_write(state, 0x002A, 0x030C);
		s5k4ca_sensor_write(state, 0x0F12, 0x0000); // AF Manual
		msleep(140);
		s5k4ca_sensor_write(state, 0x002A, 0x030E);
		s5k4ca_sensor_write(state, 0x0F12, 0x00F1);
		msleep(50);
		s5k4ca_sensor_write(state, 0x002A, 0x030C);
		s5k4ca_sensor_write(state, 0x0F12, 0x0003); // AF Freeze
		msleep(50);
	}

	s5k4ca_sensor_write(state, 0x002A, 0x030C);
	s5k4ca_sensor_write(state, 0x0F12, 0x0002);

	do
	{
		if (count == 0)
			break;
		s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(state, 0x002C, 0x7000);
		s5k4ca_sensor_write(state, 0x002E, 0x130E);
		if (state->lux_value < 0x80)
			msleep(250);
		else
			msleep(100);
		s5k4ca_sensor_read(state, 0x0F12, &tmpVal);
		count--;
	} while((tmpVal & 0x3) != 0x3 && (tmpVal & 0x3) != 0x2);

	if (!count) {
		ret = 0;
		printk("[CAM-SENSOR] =CAM 3M AF_Single Mode Fail.==> TIMEOUT \n");
	}

	if ((tmpVal & 0x3) == 0x02) {
		if (state->focus_mode != FOCUS_MODE_MACRO) { //normal AF
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);
			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00FF);
			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000);
			msleep(140);
			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00F1);
			msleep(50);
			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0003);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);
			s5k4ca_sensor_write(state, 0x002A, 0x161C);
			s5k4ca_sensor_write(state, 0x0F12, 0x82A8);
		} else {
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);
			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x0030);
			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000);
			msleep(140);
			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x0040);
		}
		ret = 0;
		printk("[CAM-SENSOR] =CAM 3M AF_Single Mode Fail.==> FAIL \n");
	}

	if ((tmpVal & 0x3) == 0x03) {
		ret = 1;
		printk("[CAM-SENSOR] =CAM 3M AF_Single Mode SUCCESS. \r\n");
	}
	state->auto_focus_result = ret;
	return ret;
}

#if 0
static int s5k4ca_sensor_af_control(struct v4l2_subdev *sd, int type)
{
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
		if (state->white_balance == 0
		    && state->scene_mode != 4)
			s5k4ca_write_regs(state, s5k4ca_ae_awb_unlock,
					ARRAY_SIZE(s5k4ca_ae_awb_unlock));
		else
			s5k4ca_write_regs(state, s5k4ca_ae_mwb_unlock,
					ARRAY_SIZE(s5k4ca_ae_mwb_unlock));

		if (state->focus_mode != FOCUS_MODE_MACRO) { //normal AF
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00F0);
			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000);    //set manual AF
			msleep(133); // 1frame delay, 7.5fps = 133ms

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00FF);    //00FF: infinity
		} else { //macro AF
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x005F);
			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000);    //set manual AF
			msleep(133); // 1frame delay, 7.5fps = 133ms

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x0050);    //0050: macro
		}
		msleep(100);
		break;

	case 1:
		printk("Focus Mode -> Single\n");


		s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
		s5k4ca_sensor_write(state, 0x002C, 0x7000);
		s5k4ca_sensor_write(state, 0x002E, 0x12FE);

		s5k4ca_sensor_read(state, 0x0F12, &light);
		if (light < 0x80) { /* Low light AF*/

			size = (ARRAY_SIZE(s5k4ca_af_low_lux_val));
			for (i = 0; i < size; i++)	{
				s5k4ca_sensor_write(state, s5k4ca_af_low_lux_val[i].subaddr,
						    s5k4ca_af_low_lux_val[i].value);
			}
			printk("[CAM-SENSOR] =Low Light AF Single light=0x%04x\n",light);
		} else {
			size = (ARRAY_SIZE(s5k4ca_af_normal_lux_val));
			for (i = 0; i < size; i++)	{
				s5k4ca_sensor_write(state, s5k4ca_af_normal_lux_val[i].subaddr,
						    s5k4ca_af_normal_lux_val[i].value);
			}
			printk("[CAM-SENSOR] =Normal Light AF Single light=0x%04x\n",light);
		}
		s5k4ca_write_regs(state, s5k4ca_ae_awb_lock,
					ARRAY_SIZE(s5k4ca_ae_awb_lock));

		if (state->focus_mode != FOCUS_MODE_MACRO) { //normal AF
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00DF);  //030E = 00FF

			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000); // AF Manual

			msleep(130);

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x00E0);

			msleep(50);
		} else {
			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x0028, 0x7000);

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x005F);    //?? ??? ??

			s5k4ca_sensor_write(state, 0x002A, 0x030C);
			s5k4ca_sensor_write(state, 0x0F12, 0x0000);    //set manual AF

			msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

			s5k4ca_sensor_write(state, 0x002A, 0x030E);
			s5k4ca_sensor_write(state, 0x0F12, 0x0050);    //0050: macro

			msleep(50);
		}

		s5k4ca_sensor_write(state, 0x002A, 0x030C);
		s5k4ca_sensor_write(state, 0x0F12, 0x0002); //AF Single
		msleep(50);

		do {
			if( count == 0)
				break;

			s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
			s5k4ca_sensor_write(state, 0x002C, 0x7000);
			s5k4ca_sensor_write(state, 0x002E, 0x130E);
			if (light < 0x80)
				msleep(250);
			else
				msleep(100);
			s5k4ca_sensor_read(state, 0x0F12, &tmpVal);

			count--;

			printk("CAM 3M AF Status Value = %x \n", tmpVal);
		} while( (tmpVal & 0x3) != 0x3 && (tmpVal & 0x3) != 0x2 );

		if (!count) {
			if (state->focus_mode != FOCUS_MODE_MACRO) { //normal AF
				s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(state, 0x0028, 0x7000);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x00DF);  //030E = 00FF

				s5k4ca_sensor_write(state, 0x002A, 0x030C);
				s5k4ca_sensor_write(state, 0x0F12, 0x0000); // AF Manual

				msleep(130);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x00E0);

				msleep(50);
			} else {
				s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(state, 0x0028, 0x7000);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x005F);    //?? ??? ??

				s5k4ca_sensor_write(state, 0x002A, 0x030C);
				s5k4ca_sensor_write(state, 0x0F12, 0x0000);    //set manual AF

				msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x0050);    //0050: macro

				msleep(50);
			}

			ret = -1;
			printk("CAM 3M AF_Single Mode Fail.==> TIMEOUT \n");
		}

		if ((tmpVal & 0x3) == 0x02) {
			if (state->focus_mode != FOCUS_MODE_MACRO) { //normal AF
				s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(state, 0x0028, 0x7000);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x00DF);  //030E = 00FF

				s5k4ca_sensor_write(state, 0x002A, 0x030C);
				s5k4ca_sensor_write(state, 0x0F12, 0x0000); // AF Manual

				msleep(130);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x00E0);

				msleep(50);
			} else {
				s5k4ca_sensor_write(state, 0xFCFC, 0xD000);
				s5k4ca_sensor_write(state, 0x0028, 0x7000);

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x005F);    //?? ??? ??

				s5k4ca_sensor_write(state, 0x002A, 0x030C);
				s5k4ca_sensor_write(state, 0x0F12, 0x0000);    //set manual AF

				msleep(133); // 1frame delay, 7.5fps = 133ms    //????? ?? ????? ?? ???, ???????? ???? ????.

				s5k4ca_sensor_write(state, 0x002A, 0x030E);
				s5k4ca_sensor_write(state, 0x0F12, 0x0050);    //0050: macro

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
	default:
		break;
	}

	return ret;
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
	case V4L2_CID_COLORFX:
		err = s5k4ca_set_effect(sd, ctrl->value);
		break;
	case V4L2_CID_SCENEMODE:
		err = s5k4ca_set_scene_mode(sd, ctrl->value);
		break;
	case V4L2_CID_BRIGHTNESS:
		err = s5k4ca_set_br(sd, ctrl->value);
		break;
	case V4L2_CID_CONTRAST:
		err = s5k4ca_set_contrast(sd, ctrl->value);
		break;
	case V4L2_CID_SATURATION:
		err = s5k4ca_set_saturation(sd, ctrl->value);
		break;
	case V4L2_CID_SHARPNESS:
		err = s5k4ca_set_sharpness(sd, ctrl->value);
		break;
	case V4L2_CID_CAM_ISO:
		err = s5k4ca_set_iso(sd, ctrl->value);
		break;
	case V4L2_CID_PHOTOMETRY:
		err = s5k4ca_set_photometry(sd, ctrl->value);
		break;
	default:
		dev_err(&client->dev, "%s: no such control\n", __func__);
		err = 0;
		break;
	}

	if (err < 0)
		dev_dbg(&client->dev, "%s: vidioc_s_ctrl failed\n", __func__);

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

	state->preview_in_init = 1;
	state->preview_in_init_af = 1;

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

	state->client = client;
	state->pdata = pdata;
	state->scene_mode = -1;
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

MODULE_DESCRIPTION("Samsung Electronics S5K4CA QXGA camera driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPL");
