#ifndef _AUDIO_GT_I5700_H_
#define _AUDIO_GT_I5700_H_ 1

struct gt_i5700_audio_pdata {
	int gpio_audio_en;
	void (*set_micbias)(bool);
};

#endif
