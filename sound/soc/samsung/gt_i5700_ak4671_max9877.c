/*
 * gt_i5700_ak4671_max9877.c  --  SoC audio for Samsung GT-i5700 (Spica)
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa @ gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/soc-dai.h>
#include <sound/gt_i5700.h>

#include <asm/mach-types.h>

#include "dma.h"
#include "i2s.h"

#include "../codecs/ak4671.h"
#include "../codecs/max9877.h"

static struct snd_soc_card gt_i5700;
static struct gt_i5700_audio_pdata *gt_i5700_pdata;

static int gt_i5700_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
					     SND_SOC_DAIFMT_NB_NF |
					     SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					   SND_SOC_DAIFMT_NB_NF |
					   SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* Use PCLK for I2S signal generation */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_RCLKSRC_1,
							0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* Gate the RCLK output on PAD */
	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
							0, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 19200000, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * GT-i5700 AK4671 HiFi DAI opserations.
 */
static struct snd_soc_ops gt_i5700_hifi_ops = {
	.hw_params = gt_i5700_hifi_hw_params,
};

static int gt_i5700_mic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k,
				int event)
{
	static int mic_ref_cnt = 0;

	if (SND_SOC_DAPM_EVENT_OFF(event)) {
		--mic_ref_cnt;

		if (mic_ref_cnt == 0)
			gt_i5700_pdata->set_micbias(0);

		if (mic_ref_cnt < 0)
			mic_ref_cnt = 0;

		return 0;
	}

	if (mic_ref_cnt == 0)
		gt_i5700_pdata->set_micbias(1);

	++mic_ref_cnt;

	return 0;
}

static const struct snd_kcontrol_new gt_i5700_direct_controls[] = {
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("GSM Send"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Sub Mic"),
	SOC_DAPM_PIN_SWITCH("Jack Mic"),
	SOC_DAPM_PIN_SWITCH("GSM Receive"),
};

static const struct snd_kcontrol_new gt_i5700_amp_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static const struct snd_soc_dapm_widget gt_i5700_dapm_direct_widgets[] = {
	SND_SOC_DAPM_LINE("Earpiece", NULL),
	SND_SOC_DAPM_LINE("GSM Send", NULL),
	SND_SOC_DAPM_MIC("Main Mic", gt_i5700_mic_event),
	SND_SOC_DAPM_MIC("Sub Mic", gt_i5700_mic_event),
	SND_SOC_DAPM_MIC("Jack Mic", gt_i5700_mic_event),
	SND_SOC_DAPM_MIC("GSM Receive", NULL),
};

static const struct snd_soc_dapm_widget gt_i5700_dapm_amp_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route dapm_direct_routes[] = {
	/* Codec outputs */
	{"Earpiece", NULL, "LOUT1"},
	{"Earpiece", NULL, "ROUT1"},
	{"GSM Send", NULL, "LOUT3"},
	{"GSM Send", NULL, "ROUT3"},	

	/* Codec inputs */
	{"LIN1", NULL, "Main Mic"},
	{"RIN1", NULL, "Main Mic"},
	{"LIN2", NULL, "Sub Mic"},
	{"RIN2", NULL, "Sub Mic"},
	{"LIN3", NULL, "Jack Mic"},
	{"RIN3", NULL, "Jack Mic"},
	{"LIN4", NULL, "GSM Receive"},
	{"RIN4", NULL, "GSM Receive"},
};

static const struct snd_soc_dapm_route dapm_amp_routes[] = {
	/* Amplifier inputs */
	{"INA1", NULL, "LOUT2"},
	{"INA2", NULL, "ROUT2"},
	{"INB1", NULL, "LOUT2"},
	{"INB2", NULL, "ROUT2"},

	/* Amplifier outputs */
	{"Headphones", NULL, "HPL"},
	{"Headphones", NULL, "HPR"},
	{"Speaker", NULL, "OUT"},
};

/*
 * This is an example machine initialisation for a ak4671 connected to a
 * gt_i5700. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int gt_i5700_ak4671_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int err;

	pr_debug("Entered %s\n", __func__);

	/* Add gt_i5700 specific widgets */
	err = snd_soc_dapm_new_controls(dapm, gt_i5700_dapm_direct_widgets,
				ARRAY_SIZE(gt_i5700_dapm_direct_widgets));
	if (err < 0)
		return err;

	err = snd_soc_add_controls(codec, gt_i5700_direct_controls,
					ARRAY_SIZE(gt_i5700_direct_controls));
	if (err < 0)
		return err;

	/* set up gt_i5700 specific audio routes */
	err = snd_soc_dapm_add_routes(dapm, dapm_direct_routes,
						ARRAY_SIZE(dapm_direct_routes));
	if (err < 0)
		return err;

	snd_soc_dapm_disable_pin(dapm, "Earpiece");
	snd_soc_dapm_disable_pin(dapm, "GSM Send");
	snd_soc_dapm_disable_pin(dapm, "Main Mic");
	snd_soc_dapm_disable_pin(dapm, "Sub Mic");
	snd_soc_dapm_disable_pin(dapm, "Jack Mic");
	snd_soc_dapm_disable_pin(dapm, "GSM Receive");

	snd_soc_dapm_sync(dapm);
	return 0;
}

static int gt_i5700_max9877_init(struct snd_soc_dapm_context *dapm)
{
	int err;

	/* Add gt_i5700 specific widgets */
	err = snd_soc_dapm_new_controls(dapm, gt_i5700_dapm_amp_widgets,
					ARRAY_SIZE(gt_i5700_dapm_amp_widgets));
	if (err < 0)
		return err;

	err = snd_soc_add_controls(dapm->codec, gt_i5700_amp_controls,
					ARRAY_SIZE(gt_i5700_amp_controls));
	if (err < 0)
		return err;

	/* set up gt_i5700 specific audio routes */
	err = snd_soc_dapm_add_routes(dapm, dapm_amp_routes,
						ARRAY_SIZE(dapm_amp_routes));
	if (err < 0)
		return err;

	snd_soc_dapm_nc_pin(dapm, "RXIN");
	snd_soc_dapm_disable_pin(dapm, "Headphones");
	snd_soc_dapm_disable_pin(dapm, "Speaker");

	snd_soc_dapm_sync(dapm);
	return 0;
}

static struct snd_soc_dai_link gt_i5700_dai[] = {
	{ /* Hifi Playback - for similatious use with voice below */
		.name = "ak4671",
		.stream_name = "ak4671 HiFi",
		.platform_name = "samsung-audio",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "ak4671-hifi",
		.codec_name = "ak4671-codec.3-0012",
		.init = gt_i5700_ak4671_init,
		.ops = &gt_i5700_hifi_ops,
	},
};

static struct snd_soc_aux_dev gt_i5700_aux[] = {
	{ /* Headphone/Speaker amplifier */
		.name = "max9877",
		.codec_name = "max9877.3-004d",
		.init = gt_i5700_max9877_init,
	},
};

static struct snd_soc_card gt_i5700 = {
	.name		= "GT-i5700",
	.dai_link	= gt_i5700_dai,
	.num_links	= ARRAY_SIZE(gt_i5700_dai),
	.aux_dev	= gt_i5700_aux,
	.num_aux_devs	= ARRAY_SIZE(gt_i5700_aux),
};

static int __init gt_i5700_probe(struct platform_device *pdev)
{
	struct gt_i5700_audio_pdata *pdata = pdev->dev.platform_data;
	struct platform_device *audio_pdev;
	int ret = 0;

	if (!machine_is_gt_i5700()) {
		dev_err(&pdev->dev, "Only GT-i5700 is supported by this ASoC driver\n");
		return -ENODEV;
	}

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "Only a single instance is allowed\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data specified\n");
		return -EINVAL;
	}

	if (!pdata->set_micbias) {
		dev_err(&pdev->dev, "No micbias control function provided\n");
		return -EINVAL;
	}

	audio_pdev = platform_device_alloc("soc-audio", -1);
	if (!audio_pdev)
		return -ENOMEM;

	platform_set_drvdata(audio_pdev, &gt_i5700);

	gt_i5700_pdata = pdata;

	ret = platform_device_add(audio_pdev);
	if (ret)
		goto err_free_pdev;

	return 0;

err_free_pdev:
	platform_device_put(audio_pdev);

	return ret;
}

static void gt_i5700_shutdown(struct platform_device *pdev)
{
	struct gt_i5700_audio_pdata *pdata = pdev->dev.platform_data;

	pdata->set_micbias(0);
}

static struct platform_driver gt_i5700_driver = {
	.shutdown = gt_i5700_shutdown,
	.driver = {
		.name = "gt_i5700_audio",
	},
};

static int __init gt_i5700_init(void)
{
	return platform_driver_probe(&gt_i5700_driver, gt_i5700_probe);
}

module_init(gt_i5700_init);

/* Module information */
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("ALSA SoC AK4671 MAX9877 GT-i5700");
MODULE_LICENSE("GPL");
