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

#include "max9877.h"

/* codec private data */
struct max9877_priv {
	enum snd_soc_control_type control_type;
	void *control_data;
	unsigned int mux_power;
	unsigned int mux_mode;
};

/* default register values */
static u8 max9877_regs[MAX9877_CACHEREGNUM] = {
	0x40, /* MAX9877_INPUT_MODE	(0x00) */
	0x00, /* MAX9877_SPK_VOLUME	(0x01) */
	0x00, /* MAX9877_HPL_VOLUME	(0x02) */
	0x00, /* MAX9877_HPR_VOLUME	(0x03) */
	0x49, /* MAX9877_OUTPUT_MODE	(0x04) */
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
};

/* Output MUX */

static const char *max9877_mux_texts[] = {"INA", "INB", "IN Mix"};
static const struct soc_enum max9877_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max9877_mux_texts),
			max9877_mux_texts);
static struct snd_kcontrol_new max9877_mux_control =
	SOC_DAPM_ENUM_VIRT("Route", max9877_mux_enum);

#define MUX_OUT_POWER	1
#define MUX_HP_POWER	2

/* TODO: This should be called with some lock held */
static void max9877_update_mux(struct snd_soc_codec *codec)
{
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);
	unsigned int mode = max9877->mux_mode;
	unsigned int power = max9877->mux_power;
	u8 reg;

	printk("%s: mode = %u, power = %u\n", __func__,
					max9877->mux_mode, max9877->mux_power);

	if (!power) {
		reg = snd_soc_read(codec, MAX9877_OUTPUT_MODE);
		reg &= ~MAX9877_SHDN;
		snd_soc_write(codec, MAX9877_OUTPUT_MODE, reg);
		return;
	}

	reg = snd_soc_read(codec, MAX9877_OUTPUT_MODE);
	reg &= ~MAX9877_OUTMODE_MASK;
	reg |= 3*mode | power | MAX9877_SHDN;
	snd_soc_write(codec, MAX9877_OUTPUT_MODE, reg);
}

static int max9877_mux_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.enumerated.item[0];
	struct snd_soc_dapm_widget *widget = snd_kcontrol_chip(kcontrol);
	struct snd_soc_codec *codec = widget->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	/* TODO: This needs synchronization. */

	max9877->mux_mode = value;
	max9877_update_mux(codec);

	return snd_soc_dapm_put_enum_virt(kcontrol, ucontrol);
}

static int max9877_hp_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	/* TODO: This needs synchronization. */

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mux_power &= ~MUX_HP_POWER;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mux_power |= MUX_HP_POWER;
		max9877_update_mux(codec);
		break;
	}

	return 0;
}

static int max9877_out_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);

	/* TODO: This needs synchronization. */

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		max9877->mux_power &= ~MUX_OUT_POWER;
		max9877_update_mux(codec);
		break;
	case SND_SOC_DAPM_POST_PMU:
		max9877->mux_power |= MUX_OUT_POWER;
		max9877_update_mux(codec);
		break;
	}

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
	SND_SOC_DAPM_INPUT("INA"),
	SND_SOC_DAPM_INPUT("INB"),
	SND_SOC_DAPM_INPUT("RXIN"),

	/* OUTPUTS */
	SND_SOC_DAPM_OUTPUT("HP"),
	SND_SOC_DAPM_OUTPUT("OUT"),

	/* MUXES */
	SND_SOC_DAPM_MUX("Internal Mux", SND_SOC_NOPM, 0, 0,
				&max9877_mux_control),
	SND_SOC_DAPM_MUX("OUT Bypass", SND_SOC_NOPM, 0, 0,
				&max9877_out_bypass_control),

	/* MIXERS */
	SND_SOC_DAPM_MIXER("IN Mix", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* PGAs */
	SND_SOC_DAPM_PGA_E("HP Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			max9877_hp_amp_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("OUT Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			max9877_out_amp_event,
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"IN Mix", NULL, "INA"},
	{"IN Mix", NULL, "INB"},

	{"Internal Mux", "INA", "INA"},
	{"Internal Mux", "INB", "INB"},
	{"Internal Mux", "IN Mix", "IN Mix"},

	{"OUT Amp", NULL, "Internal Mux"},
	{"HP Amp", NULL, "Internal Mux"},

	{"OUT Bypass", "OUT Amp", "OUT Amp"},
	{"OUT Bypass", "RXIN", "RXIN"},

	{"HP", NULL, "HP Amp"},
	{"OUT", NULL, "OUT Bypass"},
};

static int max9877_probe(struct snd_soc_codec *codec)
{
	struct max9877_priv *max9877 = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->hw_write = (hw_write_t)i2c_master_send;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, max9877->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

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
	max9877->mux_power = 3;
	max9877->mux_mode = 9;

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
	/* FIXME: This should be done in a more appropriate way... */
	max9877_mux_control.put = max9877_mux_put;
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
