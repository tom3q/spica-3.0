/*
 * Driver for the DFBM-CS320 bluetooth module
 * Copyright 2011 Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static struct snd_soc_dai_driver dfbmcs320_dai = {
	.name = "dfbmcs320-pcm",
	.playback = {
		.stream_name = "PCM Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "PCM Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static const struct snd_soc_dapm_widget dfbmcs320_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("Playback", "PCM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("Capture", "PCM Capture", 0, SND_SOC_NOPM, 0, 0),
};

static struct snd_soc_codec_driver soc_codec_dev_dfbmcs320 = {
	.dapm_widgets = dfbmcs320_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(dfbmcs320_dapm_widgets),
};

static int __devinit dfbmcs320_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_dfbmcs320,
			&dfbmcs320_dai, 1);
}

static int __devexit dfbmcs320_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver dfmcs320_driver = {
	.driver = {
		.name = "dfbmcs320",
		.owner = THIS_MODULE,
	},
	.probe = dfbmcs320_probe,
	.remove = __devexit_p(dfbmcs320_remove),
};

static int __init dfbmcs320_init(void)
{
	return platform_driver_register(&dfmcs320_driver);
}
module_init(dfbmcs320_init);

static void __exit dfbmcs320_exit(void)
{
	platform_driver_unregister(&dfmcs320_driver);
}
module_exit(dfbmcs320_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("ASoC DFBM-CS320 bluethooth module driver");
MODULE_LICENSE("GPL");
