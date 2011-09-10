/*
 * max9877.c
 * MAX9877 dailess amplifier driver
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * Original max9877 driver:
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/soc-dapm.h>
#include <linux/mutex.h>

#include "max9877.h"

/* codec private data */
struct max9877_priv {
	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned int mix_power;
	unsigned int mix_mode;
	struct mutex mix_mutex;
};

/* default register values */
static u8 max9877_regs[MAX9877_CACHEREGNUM] = {
	0x40, /* MAX9877_INPUT_MODE	(0x00) */
	0x00, /* MAX9877_SPK_VOLUME	(0x01) */
	0x00, /* MAX9877_HPL_VOLUME	(0x02) */
	0x00, /* MAX9877_HPR_VOLUME	(0x03) */
	0x09, /* MAX9877_OUTPUT_MODE	(0x04) */
};

static const unsigned int max9877_pgain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 1, TLV_DB_SCALE_ITEM(0, 900, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0),
};

static const unsigned int max9877_output_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0, 7, TLV_DB_SCALE_ITEM(-7900, 400, 1),
	8, 15, TLV_DB_SCALE_ITEM(-4700, 300, 0),
	16, 23, TLV_DB_SCALE_ITEM(-2300, 200, 0),
	24, 31, TLV_DB_SCALE_ITEM(-700, 100, 0),
};

static const char *max9877_osc_mode[] = {
	"1176KHz",
	"1100KHz",
	"700KHz",
};
SOC_ENUM_SINGLE_DECL(max9877_osc_mode_enum, MAX9877_OUTPUT_MODE,
					MAX9877_OSC_SHIFT, max9877_osc_mode);

static const struct snd_kcontrol_new max9877_controls[] = {
	SOC_SINGLE_TLV("MAX9877 PGAINA Playback Volume",
			MAX9877_INPUT_MODE, 0, 2, 0, max9877_pgain_tlv),
	SOC_SINGLE_TLV("MAX9877 PGAINB Playback Volume",
			MAX9877_INPUT_MODE, 2, 2, 0, max9877_pgain_tlv),
	SOC_SINGLE_TLV("MAX9877 Amp Speaker Playback Volume",
			MAX9877_SPK_VOLUME, 0, 31, 0, max9877_output_tlv),
	SOC_DOUBLE_R_TLV("MAX9877 Amp HP Playback Volume",
			MAX9877_HPL_VOLUME, MAX9877_HPR_VOLUME, 0, 31, 0,
			max9877_output_tlv),
	SOC_SINGLE("MAX9877 INB Stereo Switch",
			MAX9877_INPUT_MODE, MAX9877_INB, 1, 1),
	SOC_SINGLE("MAX9877 INA Stereo Switch",
			MAX9877_INPUT_MODE, MAX9877_INA, 1, 1),
	SOC_SINGLE("MAX9877 Zero-crossing detection Switch",
			MAX9877_INPUT_MODE, MAX9877_ZCD, 1, 0),
	SOC_ENUM("MAX9877 Oscillator Mode", max9877_osc_mode_enum),
	SOC_DAPM_PIN_SWITCH("INA PGA"),
	SOC_DAPM_PIN_SWITCH("INB PGA"),
};

/* Output MIX */
#define MIX_OUT_POWER	(1 << 0)
#define MIX_HP_POWER	(1 << 1)
#define MIX_INA		(1 << 0)
#define MIX_INB		(1 << 1)

/* Called with mux_mutex held. */
static void max9877_update_mux(struct snd_soc_codec *codec)
{
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);
	unsigned int mode = max9877->mix_mode;
	unsigned int power = max9877->mix_power;
	u8 reg;

	printk("%s: mode = %u, power = %u\n", __func__,
					max9877->mix_mode, max9877->mix_power);

	if (!power || !mode) {
		reg = snd_soc_read(codec, MAX9877_OUTPUT_MODE);
		reg &= ~MAX9877_SHDN;
		snd_soc_write(codec, MAX9877_OUTPUT_MODE, reg);
		return;
	}

	reg = snd_soc_read(codec, MAX9877_OUTPUT_MODE);
	reg &= ~MAX9877_OUTMODE_MASK;
	reg |= (3*(mode - 1) + power) | MAX9877_SHDN;
	snd_soc_write(codec, MAX9877_OUTPUT_MODE, reg);
}

static int max9877_ina_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&max9877->mix_mutex);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mix_mode &= ~MIX_INA;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mix_mode |= MIX_INA;
		max9877_update_mux(codec);
		break;
	}

	mutex_unlock(&max9877->mix_mutex);

	return 0;
}

static int max9877_inb_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&max9877->mix_mutex);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mix_mode &= ~MIX_INB;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mix_mode |= MIX_INB;
		max9877_update_mux(codec);
		break;
	}

	mutex_unlock(&max9877->mix_mutex);

	return 0;
}

static int max9877_hp_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&max9877->mix_mutex);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mix_power &= ~MIX_HP_POWER;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mix_power |= MIX_HP_POWER;
		max9877_update_mux(codec);
		break;
	}

	mutex_unlock(&max9877->mix_mutex);

	return 0;
}

static int max9877_out_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&max9877->mix_mutex);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mix_power &= ~MIX_OUT_POWER;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mix_power |= MIX_OUT_POWER;
		max9877_update_mux(codec);
		break;
	}

	mutex_unlock(&max9877->mix_mutex);

	return 0;
}

/* OUT bypass */

static const char *max9877_out_bypass_texts[] = {"OUT Amp", "RXIN"};
SOC_ENUM_SINGLE_DECL(max9877_out_bypass_enum, MAX9877_OUTPUT_MODE,
				MAX9877_BYPASS_SHIFT, max9877_out_bypass_texts);
static const struct snd_kcontrol_new max9877_out_bypass_control =
				SOC_DAPM_ENUM("Route", max9877_out_bypass_enum);

/* DAPM widgets */

static const struct snd_soc_dapm_widget max9877_dapm_widgets[] = {
	/* INPUTS */
	SND_SOC_DAPM_INPUT("INA1"),
	SND_SOC_DAPM_INPUT("INA2"),
	SND_SOC_DAPM_INPUT("INB1"),
	SND_SOC_DAPM_INPUT("INB2"),
	SND_SOC_DAPM_INPUT("RXIN"),

	/* OUTPUTS */
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
	SND_SOC_DAPM_OUTPUT("OUT"),

	/* MUXES */
	SND_SOC_DAPM_MUX("OUT Bypass", SND_SOC_NOPM, 0, 0,
				&max9877_out_bypass_control),

	/* MIXERS */
	SND_SOC_DAPM_MIXER("IN Mix", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* PGAs */
	SND_SOC_DAPM_PGA_E("INA PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
				max9877_ina_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA_E("INB PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
				max9877_inb_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA_E("HP Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
				max9877_hp_amp_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA_E("OUT Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
				max9877_out_amp_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route max9877_dapm_routes[] = {
	{"INA PGA", NULL, "INA1"},
	{"INA PGA", NULL, "INA2"},
	{"INB PGA", NULL, "INB1"},
	{"INB PGA", NULL, "INB2"},

	{"IN Mix", NULL, "INA PGA"},
	{"IN Mix", NULL, "INB PGA"},

	{"OUT Amp", NULL, "IN Mix"},
	{"HP Amp", NULL, "IN Mix"},

	{"OUT Bypass", "OUT Amp", "OUT Amp"},
	{"OUT Bypass", "RXIN", "RXIN"},

	{"HPL", NULL, "HP Amp"},
	{"HPR", NULL, "HP Amp"},
	{"OUT", NULL, "OUT Bypass"},
};

static int max9877_probe(struct snd_soc_codec *codec)
{
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);
	int ret;
	int i;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, max9877->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(max9877_regs); ++i)
		snd_soc_write(codec, i, max9877_regs[i]);

	return ret;
}

static int max9877_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_max9877 = {
	.probe = max9877_probe,
	.remove = max9877_remove,
	.reg_cache_size = MAX9877_CACHEREGNUM,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = max9877_regs,
	.controls = max9877_controls,
	.num_controls = ARRAY_SIZE(max9877_controls),
	.dapm_widgets = max9877_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(max9877_dapm_widgets),
	.dapm_routes = max9877_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(max9877_dapm_routes),
};

static int __devinit max9877_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct max9877_priv *max9877;
	int ret;

	max9877 = kzalloc(sizeof(struct max9877_priv), GFP_KERNEL);
	if (max9877 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, max9877);
	max9877->control_data = client;
	max9877->control_type = SND_SOC_I2C;
	max9877->mix_power = 0;
	max9877->mix_mode = 0;
	mutex_init(&max9877->mix_mutex);

	ret = snd_soc_register_codec(&client->dev,
					&soc_codec_dev_max9877, NULL, 0);
	if (ret < 0)
		kfree(max9877);
	return ret;
}

static __devexit int max9877_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id max9877_i2c_id[] = {
	{ "max9877", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9877_i2c_id);

static struct i2c_driver max9877_i2c_driver = {
	.driver = {
		.name = "max9877",
		.owner = THIS_MODULE,
	},
	.probe = max9877_i2c_probe,
	.remove = __devexit_p(max9877_i2c_remove),
	.id_table = max9877_i2c_id,
};

static int __init max9877_init(void)
{
	return i2c_add_driver(&max9877_i2c_driver);
}
module_init(max9877_init);

static void __exit max9877_exit(void)
{
	i2c_del_driver(&max9877_i2c_driver);
}
module_exit(max9877_exit);

MODULE_DESCRIPTION("ASoC MAX9877 amp driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPL");
