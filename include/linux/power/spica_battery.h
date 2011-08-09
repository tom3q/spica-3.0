#ifndef _SPICA_BATTERY_H
#define _SPICA_BATTERY_H

#include <linux/power_supply.h>

enum spica_battery_supply {
	SPICA_BATTERY_NONE	= -1,
	SPICA_BATTERY_USB	= 0,
	SPICA_BATTERY_AC	= 1,
	SPICA_BATTERY_NUM	= 2
};

struct spica_battery_threshold {
	/* Start of ADC range */
	int adc;
	/* Start of value range */
	int val;
};

typedef void spica_battery_notify_func_t(struct platform_device *,
						enum spica_battery_supply);

struct spica_battery_pdata {
	/* GPIO used to monitor external power supply presence */
	int gpio_pok;
	int gpio_pok_inverted;

	/* GPIO used to monitor charging status */
	int gpio_chg;
	int gpio_chg_inverted;

	/* GPIO used to control charging */
	int gpio_en;
	int gpio_en_inverted;

	/* Percentage lookup table in 0.001% */
	const struct spica_battery_threshold *percent_lut;
	unsigned int percent_lut_cnt;

	/* Voltage lookup table in microvolts */
	const struct spica_battery_threshold *volt_lut;
	unsigned int volt_lut_cnt;

	/* Temperature lookup table in 0.001*C */
	const struct spica_battery_threshold *temp_lut;
	unsigned int temp_lut_cnt;

	/* Overheat temperature in 0.001*C */
	int low_temp_enter;
	int low_temp_exit;
	int high_temp_enter;
	int high_temp_exit;

	/* Battery technology */
	int technology;

	/* ADC channels */
	unsigned int volt_channel;
	unsigned int temp_channel;

	/* Get type of connected external power supply */
	void (*supply_detect_init)(spica_battery_notify_func_t *);
	void (*supply_detect_cleanup)(void);
};

#endif /* _SPICA_BATTERY_H */
