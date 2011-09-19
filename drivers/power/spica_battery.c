/*
 * ADC-based battery driver for boards with Samsung SoCs.
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/android_alarm.h>

#include <linux/power/spica_battery.h>

#include <plat/adc.h>

#include <mach/map.h>
#include <mach/regs-sys.h>
#include <mach/regs-syscon-power.h>
#include <mach/irqs.h>

/* Time between samples (in milliseconds) */
#define BAT_POLL_INTERVAL	10000

/* Time to wake up phone and recheck battery after (in seconds) */
#define SUSPEND_INTERVAL	3600

/* Number of samples for averaging (a power of two!) */
#define NUM_SAMPLES		4

/*
 * Linear interpolation
 */

/* Interpolation table entry */
struct lookup_entry {
	int start;
	int end;
	int base;
	int delta;
};

/* Interpolation table descriptor */
struct lookup_data {
	struct lookup_entry	*table;
	unsigned int		size;
};

/* Calculates interpolated value based on given input value */
static int lookup_value(struct lookup_data *data, int value)
{
	struct lookup_entry *table = data->table;
	unsigned int min = 0, max = data->size - 1;

	/* Do a binary search in the lookup table */
	while (min != max) {
		if (value < table[(min + max) / 2].start) {
			max = -1 + (min + max) / 2;
			continue;
		}
		if (value >= table[(min + max) / 2].end) {
			min = 1 + (min + max) / 2;
			continue;
		}
		min = (min + max) / 2;
		break;
	}

	/* We should have the correct table entry pointer here */
	return table[min].base + (value - table[min].start)*table[min].delta;
}

/* Creates interpolation table based on platform data */
static int create_lookup_table(
			const struct spica_battery_threshold *thresholds,
			int threshold_count, struct lookup_data *data)
{
	struct lookup_entry *entry;

	if (!thresholds || threshold_count < 2)
		return -EINVAL;

	data->size = threshold_count + 1;

	entry = kmalloc(data->size * sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	data->table = entry;

	/* Don't account the last entry */
	--threshold_count;

	/* Add the bottom sentinel */
	entry->start = 0;
	entry->end = thresholds->adc;
	entry->base = thresholds->val;
	entry->delta = 0;
	++entry;

	do {
		int adc_range, val_range;
		adc_range = (thresholds+1)->adc - thresholds->adc;
		val_range = (thresholds+1)->val - thresholds->val;
		entry->start = thresholds->adc;
		entry->end = (thresholds+1)->adc;
		entry->delta = val_range / adc_range;
		entry->base = thresholds->val;
		++thresholds;
		++entry;
	} while (--threshold_count);

	/* Add the top sentinel */
	entry->start = thresholds->adc;
	entry->end = 0xffffffff;
	entry->base = thresholds->val;
	entry->delta = 0;

	return 0;
}

/* 
 * Avaraging algorithm
 */

/* Algorithm data */
struct average_data {
	int				samples[NUM_SAMPLES];
	unsigned int			cur_sample;
	int				sum;
};

/* Here the assumption that NUM_SAMPLES is a power of two shows its importance,
 * as both the modulo and division operations will be translated to simple
 * bitwise AND and shift operations. */
static int put_sample_get_avg(struct average_data *avg, int sample)
{
	avg->sum -= avg->samples[avg->cur_sample];
	avg->samples[avg->cur_sample] = sample;
	avg->sum += sample;
	avg->cur_sample = (avg->cur_sample + 1) % NUM_SAMPLES;

	return avg->sum / NUM_SAMPLES;
}

/*
 * Battery driver
 */

/* Driver data */
struct spica_battery {
	struct power_supply		bat;
	struct power_supply		psy[SPICA_BATTERY_NUM];

	struct s3c_adc_client		*client;
	struct spica_battery_pdata	*pdata;
	struct work_struct		work;
	struct delayed_work		poll_work;
	struct mutex			mutex;
	struct device			*dev;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock	wakelock;
	struct wake_lock	chg_wakelock;
	struct wake_lock	fault_wakelock;
#endif
#ifdef CONFIG_RTC_INTF_ALARM
	struct alarm		alarm;
#endif
	unsigned int irq_pok;
	unsigned int irq_chg;

	int percent_value;
	int volt_value;
	int temp_value;
	int status;
	int health;
	int online[SPICA_BATTERY_NUM];
	int fault;
	int chg_enable;
	enum spica_battery_supply supply;

	unsigned int		interval;
	struct average_data	volt_avg;
	struct average_data	temp_avg;
	struct lookup_data	percent_lookup;
	struct lookup_data	volt_lookup;
	struct lookup_data	temp_lookup;
};

#ifdef CONFIG_RTC_INTF_ALARM
/* Alarm handler */
static void spica_battery_alarm(struct alarm *alarm)
{
	/* Nothing to do here */
}
#endif

/* Battery fault handling */
static void spica_battery_set_fault_enable(bool enable)
{
	unsigned long flags;
	u32 reg, val;

	if (enable)
		val = S3C64XX_PWRCFG_CFG_BATFLT_IRQ;
	else
		val = S3C64XX_PWRCFG_CFG_BATFLT_IGNORE;

	local_irq_save(flags);

	reg = readl(S3C64XX_PWR_CFG);
	reg &= ~S3C64XX_PWRCFG_CFG_BATFLT_MASK;
	reg |= val;
	writel(reg, S3C64XX_PWR_CFG);

	local_irq_restore(flags);
}

static irqreturn_t spica_battery_fault_irq(int irq, void *dev_id)
{
	struct spica_battery *bat = dev_id;
	unsigned long flags;
	u32 reg;
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&bat->fault_wakelock);
#endif
	local_irq_save(flags);

	spica_battery_set_fault_enable(0);

	reg = readl(S3C64XX_OTHERS);
	reg |= S3C64XX_OTHERS_CLEAR_BATF_INT;
	writel(reg, S3C64XX_OTHERS);

	bat->fault = 1;

	local_irq_restore(flags);

	schedule_work(&bat->work);

	return IRQ_HANDLED;
}

/* Polling function */
static void spica_battery_poll(struct work_struct *work)
{
	struct delayed_work *dwrk = to_delayed_work(work);
	struct spica_battery *bat =
			container_of(dwrk, struct spica_battery, poll_work);
	struct spica_battery_pdata *pdata = bat->pdata;
	int volt_sample, volt_value, temp_sample, temp_value, percent_value;
	int health, update = 0;

	/* Get a voltage sample from the ADC */
	volt_sample = s3c_adc_read(bat->client, bat->pdata->volt_channel);
	if (volt_sample < 0) {
		dev_warn(bat->dev, "Failed to get ADC sample.\n");
		goto error;
	}
	volt_sample = put_sample_get_avg(&bat->volt_avg, volt_sample);
	volt_value = lookup_value(&bat->volt_lookup, volt_sample);
	percent_value = lookup_value(&bat->percent_lookup, volt_sample);

	/* Get a temperature sample from the ADC */
	temp_sample = s3c_adc_read(bat->client, bat->pdata->temp_channel);
	if (temp_sample < 0) {
		dev_warn(bat->dev, "Failed to get ADC sample.\n");
		goto error;
	}
	temp_sample = put_sample_get_avg(&bat->temp_avg, temp_sample);
	temp_value = lookup_value(&bat->temp_lookup, temp_sample);

	/* Update driver data (locked) */
	mutex_lock(&bat->mutex);

	bat->volt_value = volt_value;
	bat->percent_value = percent_value;
	bat->temp_value = temp_value;

	health = bat->health;

	if (temp_value <= pdata->low_temp_enter)
		health = POWER_SUPPLY_HEALTH_COLD;

	if (temp_value >= pdata->high_temp_enter)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;

	if (temp_value >= pdata->low_temp_exit
	    && temp_value <= pdata->high_temp_exit)
		health = POWER_SUPPLY_HEALTH_GOOD;

	if (bat->health != health) {
		bat->health = health;
		update = 1;
	}

	mutex_unlock(&bat->mutex);

error:
	/* Schedule next poll */
	if (update) {
		schedule_work(&bat->work);
	} else {
		schedule_delayed_work(&bat->poll_work,
					msecs_to_jiffies(bat->interval));
		power_supply_changed(&bat->bat);
	}
}

static void spica_battery_work(struct work_struct *work)
{
	struct spica_battery *bat =
			container_of(work, struct spica_battery, work);
	struct spica_battery_pdata *pdata = bat->pdata;
	int is_plugged, is_healthy, chg_enable;
	enum spica_battery_supply type;
	int i;

	/* Cancel any pending works */
	cancel_delayed_work_sync(&bat->poll_work);

	/* Check for external power supply connection */
	is_plugged = gpio_get_value(pdata->gpio_pok) ^ pdata->gpio_pok_inverted;

	/* We're going to access shared data */
	mutex_lock(&bat->mutex);

	type = bat->supply;
	if (type == SPICA_BATTERY_NONE)
		is_plugged = 0;

	for (i = 0; i < SPICA_BATTERY_NUM; ++i)
		bat->online[i] = (type == i);

	/* Default polling interval if not overheated nor charging */
	is_healthy = (bat->health == POWER_SUPPLY_HEALTH_GOOD);
	if (!is_healthy) {
		if (bat->health == POWER_SUPPLY_HEALTH_OVERHEAT)
			dev_warn(bat->dev,
				 "Battery overheated, disabling charger.\n");

		if (bat->health == POWER_SUPPLY_HEALTH_COLD);
			dev_warn(bat->dev,
				 "Battery temperature too low, disabling charger.\n");
	}

	/* Update charging status and polling interval */
	chg_enable = is_plugged && is_healthy;

	if (bat->chg_enable == chg_enable)
		goto no_change;

	bat->status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (chg_enable) {
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock(&bat->chg_wakelock);
#endif
		bat->status = POWER_SUPPLY_STATUS_FULL;

		/* Enable the charger */
		gpio_set_value(pdata->gpio_en, !pdata->gpio_en_inverted);

		/* Get charging state */
		if (gpio_get_value(pdata->gpio_chg) ^ pdata->gpio_chg_inverted)
			bat->status = POWER_SUPPLY_STATUS_CHARGING;

		bat->fault = 0;
#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(&bat->fault_wakelock);
#endif
	} else {
		/* Disable the charger */
		gpio_set_value(pdata->gpio_en, pdata->gpio_en_inverted);

		/* Enable battery fault interrupt */
		spica_battery_set_fault_enable(1);
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_timeout(&bat->chg_wakelock, HZ / 2);
#endif
	}

	bat->chg_enable = chg_enable;

no_change:
	/* We're no longer accessing shared data */
	mutex_unlock(&bat->mutex);

	/* Update the values and spin the polling loop */
	spica_battery_poll(&bat->poll_work.work);

	/* Notify anyone interested */
	power_supply_changed(&bat->bat);
	for (i = 0; i < SPICA_BATTERY_NUM; ++i)
		power_supply_changed(&bat->psy[i]);
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&bat->wakelock);
#endif
}

static void spica_battery_supply_notify(struct platform_device *pdev,
						enum spica_battery_supply type)
{
	struct spica_battery *bat = platform_get_drvdata(pdev);

	mutex_lock(&bat->mutex);

	bat->supply = type;

	mutex_unlock(&bat->mutex);

	schedule_work(&bat->work);
}

/*
 * Battery specific code
 */

/* Gets a property */
static int spica_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct spica_battery *bat = dev_get_drvdata(psy->dev->parent);
	int ret = 0;

	mutex_lock(&bat->mutex);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 100000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (bat->fault)
			val->intval = 0;
		else
			val->intval = bat->percent_value;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bat->volt_value;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bat->temp_value / 100;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bat->fault)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else
			val->intval = bat->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		/* Battery is assumed to be present. */
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bat->pdata->technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bat->fault)
			val->intval = 0;
		else
			val->intval = bat->percent_value / 1000;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&bat->mutex);

	return ret;
}

/* Properties which we support */
static enum power_supply_property spica_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY
};

static const struct power_supply spica_bat_template = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= spica_battery_props,
	.num_properties		= ARRAY_SIZE(spica_battery_props),
	.get_property		= spica_battery_get_property,
};

/*
 * Charger specific code
 */

/* State change irq */
static irqreturn_t spica_charger_irq(int irq, void *dev_id)
{
	struct spica_battery *bat = dev_id;
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&bat->wakelock);
#endif
	schedule_work(&bat->work);

	return IRQ_HANDLED;
}

/* Gets a property */
static int spica_charger_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct spica_battery *bat = dev_get_drvdata(psy->dev->parent);
	int id = psy - bat->psy;
	int ret = 0;

	mutex_lock(&bat->mutex);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* Battery is assumed to be present. */
		val->intval = bat->online[id];
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&bat->mutex);

	return ret;
}

static enum power_supply_property spica_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply spica_chg_templates[SPICA_BATTERY_NUM] = {
	[SPICA_BATTERY_USB] = {
		.properties		= spica_charger_props,
		.num_properties		= ARRAY_SIZE(spica_charger_props),
		.get_property		= spica_charger_get_property,
		.name			= "usb",
		.type			= POWER_SUPPLY_TYPE_USB,
	},
	[SPICA_BATTERY_AC] = {
		.properties		= spica_charger_props,
		.num_properties		= ARRAY_SIZE(spica_charger_props),
		.get_property		= spica_charger_get_property,
		.name			= "ac",
		.type			= POWER_SUPPLY_TYPE_MAINS,
	}
};

/*
 * Platform driver
 */

static int spica_battery_probe(struct platform_device *pdev)
{
	struct s3c_adc_client	*client;
	struct spica_battery_pdata *pdata = pdev->dev.platform_data;
	struct spica_battery *bat;
	int ret, i, irq;

	/* Platform data is required */
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		return -ENODEV;
	}

	/* Check GPIOs */
	if (!gpio_is_valid(pdata->gpio_pok)) {
		dev_err(&pdev->dev, "Invalid gpio pin for POK line\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_chg)) {
		dev_err(&pdev->dev, "Invalid gpio pin for CHG line\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_en)) {
		dev_err(&pdev->dev, "Invalid gpio pin for EN line\n");
		return -EINVAL;
	}

	if (!pdata->supply_detect_init) {
		dev_err(&pdev->dev, "Supply detection is required\n");
		return -EINVAL;
	}

	/* Register ADC client */
	client = s3c_adc_register(pdev, NULL, NULL, 0);
	if (IS_ERR(client)) {
		dev_err(&pdev->dev, "could not register adc\n");
		return PTR_ERR(client);
	}

	/* Allocate driver data */
	bat = kzalloc(sizeof(struct spica_battery), GFP_KERNEL);
	if (!bat) {
		dev_err(&pdev->dev, "could not allocate driver data\n");
		ret = -ENOMEM;
		goto err_free_adc;
	}

	/* Claim and setup GPIOs */
	ret = gpio_request(pdata->gpio_pok, dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Failed to request POK pin: %d\n", ret);
		goto err_free;
	}

	ret = gpio_direction_input(pdata->gpio_pok);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set POK to input: %d\n", ret);
		goto err_gpio_pok_free;
	}

	ret = gpio_request(pdata->gpio_chg, dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Failed to request CHG pin: %d\n", ret);
		goto err_gpio_pok_free;
	}

	ret = gpio_direction_input(pdata->gpio_chg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set CHG to input: %d\n", ret);
		goto err_gpio_chg_free;
	}

	ret = gpio_request(pdata->gpio_en, dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Failed to request EN pin: %d\n", ret);
		goto err_gpio_chg_free;
	}

	ret = gpio_direction_output(pdata->gpio_en, pdata->gpio_en_inverted);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set EN to output: %d\n", ret);
		goto err_gpio_en_free;
	}

	platform_set_drvdata(pdev, bat);

	bat->dev = &pdev->dev;
	bat->client = client;
	bat->pdata = pdata;
	bat->status = POWER_SUPPLY_STATUS_DISCHARGING;
	bat->health = POWER_SUPPLY_HEALTH_GOOD;
	bat->supply = SPICA_BATTERY_NONE;
	bat->interval = BAT_POLL_INTERVAL;

	ret = create_lookup_table(pdata->percent_lut,
				pdata->percent_lut_cnt, &bat->percent_lookup);
	if (ret) {
		dev_err(&pdev->dev, "could not get create percentage lookup table");
		goto err_gpio_en_free;
	}

	ret = create_lookup_table(pdata->volt_lut,
				pdata->volt_lut_cnt, &bat->volt_lookup);
	if (ret) {
		dev_err(&pdev->dev, "could not get create voltage lookup table");
		goto err_percent_free;
	}

	ret = create_lookup_table(pdata->temp_lut,
				pdata->temp_lut_cnt, &bat->temp_lookup);
	if (ret) {
		dev_err(&pdev->dev, "could not get create temperature lookup table");
		goto err_volt_free;
	}

	INIT_WORK(&bat->work, spica_battery_work);
	INIT_DELAYED_WORK(&bat->poll_work, spica_battery_poll);
	mutex_init(&bat->mutex);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&bat->wakelock, WAKE_LOCK_SUSPEND, "battery");
	wake_lock_init(&bat->chg_wakelock, WAKE_LOCK_SUSPEND, "charger");
	wake_lock_init(&bat->fault_wakelock,
					WAKE_LOCK_SUSPEND, "battery fault");
#endif
#ifdef CONFIG_RTC_INTF_ALARM
	alarm_init(&bat->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
							spica_battery_alarm);
#endif

	/* Get some initial data for averaging */
	for (i = 0; i < NUM_SAMPLES; ++i) {
		int sample;
		/* Get a voltage sample from the ADC */
		sample = s3c_adc_read(bat->client, bat->pdata->volt_channel);
		if (sample < 0) {
			dev_warn(&pdev->dev, "Failed to get ADC sample.\n");
			continue;
		}
		/* Put the sample and get the new average */
		bat->volt_value = put_sample_get_avg(&bat->volt_avg, sample);
		/* Get a temperature sample from the ADC */
		sample = s3c_adc_read(bat->client, bat->pdata->temp_channel);
		if (sample < 0) {
			dev_warn(&pdev->dev, "Failed to get ADC sample.\n");
			continue;
		}
		/* Put the sample and get the new average */
		bat->temp_value = put_sample_get_avg(&bat->temp_avg, sample);
	}

	bat->percent_value = lookup_value(&bat->percent_lookup, bat->volt_value);
	bat->volt_value = lookup_value(&bat->volt_lookup, bat->volt_value);
	bat->temp_value = lookup_value(&bat->temp_lookup, bat->temp_value);

	/* Register the power supplies */
	for (i = 0; i < SPICA_BATTERY_NUM; ++i) {
		bat->psy[i] = spica_chg_templates[i];
		ret = power_supply_register(&pdev->dev, &bat->psy[i]);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to register power supply %s (%d)\n",
							bat->psy[i].name, ret);
			break;
		}
	}

	/* Undo the loop on error */
	if (i-- != SPICA_BATTERY_NUM) {
		for (; i >= 0; --i)
			power_supply_unregister(&bat->psy[i]);
		goto err_temp_free;
	}

	/* Register the battery */
	bat->bat = spica_bat_template;
	ret = power_supply_register(&pdev->dev, &bat->bat);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register battery power supply: %d\n", ret);
		goto err_psy_unreg;
	}

	/* Claim IRQs */
	irq = gpio_to_irq(pdata->gpio_pok);
	if (irq <= 0) {
		dev_err(&pdev->dev, "POK irq invalid.\n");
		goto err_bat_unreg;
	}
	bat->irq_pok = irq;

	ret = request_any_context_irq(irq, spica_charger_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&pdev->dev), bat);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request POK irq (%d)\n", ret);
		goto err_bat_unreg;
	}

	irq = gpio_to_irq(pdata->gpio_chg);
	if (irq <= 0) {
		dev_err(&pdev->dev, "CHG irq invalid.\n");
		goto err_pok_irq_free;
	}
	bat->irq_chg = irq;

	ret = request_any_context_irq(irq, spica_charger_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&pdev->dev), bat);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request CHG irq (%d)\n", ret);
		goto err_pok_irq_free;
	}

	ret = request_irq(IRQ_BATF, spica_battery_fault_irq,
						0, dev_name(&pdev->dev), bat);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to request battery fault irq (%d)\n", ret);
		goto err_chg_irq_free;
	}

	enable_irq_wake(bat->irq_pok);
	enable_irq_wake(bat->irq_chg);

	spica_battery_set_fault_enable(1);

	/* Finish */
	dev_info(&pdev->dev, "successfully loaded\n");
	device_init_wakeup(&pdev->dev, 1);

	pdata->supply_detect_init(spica_battery_supply_notify);

	/* Schedule work to check current status */
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&bat->wakelock);
#endif
	schedule_work(&bat->work);

	return 0;

err_chg_irq_free:
	free_irq(bat->irq_chg, bat);
err_pok_irq_free:
	free_irq(bat->irq_pok, bat);
err_bat_unreg:
	power_supply_unregister(&bat->bat);
err_psy_unreg:
	for (i = 0; i < SPICA_BATTERY_NUM; ++i)
		power_supply_unregister(&bat->psy[i]);
err_temp_free:
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&bat->wakelock);
	wake_lock_destroy(&bat->chg_wakelock);
	wake_lock_destroy(&bat->fault_wakelock);
#endif
	kfree(bat->temp_lookup.table);
err_volt_free:
	kfree(bat->volt_lookup.table);
err_percent_free:
	kfree(bat->percent_lookup.table);
err_gpio_en_free:
	gpio_free(pdata->gpio_en);
err_gpio_chg_free:
	gpio_free(pdata->gpio_chg);
err_gpio_pok_free:
	gpio_free(pdata->gpio_pok);
err_free:
	kfree(bat);
err_free_adc:
	s3c_adc_release(client);
	return ret;
}

static int spica_battery_remove(struct platform_device *pdev)
{
	struct spica_battery *bat = platform_get_drvdata(pdev);
	struct spica_battery_pdata *pdata = bat->pdata;
	int i;

	disable_irq_wake(bat->irq_pok);
	disable_irq_wake(bat->irq_chg);

	free_irq(IRQ_BATF, bat);
	free_irq(bat->irq_chg, bat);
	free_irq(bat->irq_pok, bat);

	if (pdata->supply_detect_cleanup)
		pdata->supply_detect_cleanup();

	power_supply_unregister(&bat->bat);
	for (i = 0; i < SPICA_BATTERY_NUM; ++i)
		power_supply_unregister(&bat->psy[i]);

	cancel_work_sync(&bat->work);
	cancel_delayed_work_sync(&bat->poll_work);

	s3c_adc_release(bat->client);

	kfree(bat->temp_lookup.table);
	kfree(bat->volt_lookup.table);
	kfree(bat->percent_lookup.table);

	gpio_set_value(pdata->gpio_en, pdata->gpio_en_inverted);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&bat->fault_wakelock);
	wake_lock_destroy(&bat->chg_wakelock);
	wake_lock_destroy(&bat->wakelock);
#endif
	kfree(bat);

	return 0;
}

static int spica_battery_suspend(struct platform_device *pdev,
							pm_message_t state)
{
	struct spica_battery *bat = platform_get_drvdata(pdev);
	ktime_t now, start, end;

	cancel_work_sync(&bat->work);
	cancel_delayed_work_sync(&bat->poll_work);
#ifdef CONFIG_RTC_INTF_ALARM
	now = alarm_get_elapsed_realtime();
	start = ktime_set(SUSPEND_INTERVAL - 10, 0);
	start = ktime_add(now, start);
	end = ktime_set(SUSPEND_INTERVAL + 20, 0);
	end = ktime_add(now, end);
	alarm_start_range(&bat->alarm, start, end);
#endif
	return 0;
}

static int spica_battery_resume(struct platform_device *pdev)
{
	struct spica_battery *bat = platform_get_drvdata(pdev);
	int volt_value = -1, temp_value = -1;
	int i;
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&bat->wakelock);
#endif
#ifdef CONFIG_RTC_INTF_ALARM
	alarm_cancel(&bat->alarm);
#endif
	/* Get some initial data for averaging */
	for (i = 0; i < NUM_SAMPLES; ++i) {
		int sample;
		/* Get a voltage sample from the ADC */
		sample = s3c_adc_read(bat->client, bat->pdata->volt_channel);
		if (sample < 0) {
			dev_warn(&pdev->dev, "Failed to get ADC sample.\n");
			continue;
		}
		/* Put the sample and get the new average */
		volt_value = put_sample_get_avg(&bat->volt_avg, sample);
		/* Get a temperature sample from the ADC */
		sample = s3c_adc_read(bat->client, bat->pdata->temp_channel);
		if (sample < 0) {
			dev_warn(&pdev->dev, "Failed to get ADC sample.\n");
			continue;
		}
		/* Put the sample and get the new average */
		temp_value = put_sample_get_avg(&bat->temp_avg, sample);
	}

	if (volt_value > 0) {
		bat->percent_value = lookup_value(&bat->percent_lookup, volt_value);
		bat->volt_value = lookup_value(&bat->volt_lookup, volt_value);
	}

	if (temp_value > 0)
		bat->temp_value = lookup_value(&bat->temp_lookup, temp_value);

	/* Schedule timer to check current status */
	schedule_work(&bat->work);

	return 0;
}

static struct platform_driver spica_battery_driver = {
	.driver		= {
		.name	= "spica-battery",
	},
	.probe		= spica_battery_probe,
	.remove		= spica_battery_remove,
	.suspend	= spica_battery_suspend,
	.resume		= spica_battery_resume,
};

/*
 * Kernel module
 */

static int __init spica_battery_init(void)
{
	return platform_driver_register(&spica_battery_driver);
}
module_init(spica_battery_init);

static void __exit spica_battery_exit(void)
{
	platform_driver_unregister(&spica_battery_driver);
}
module_exit(spica_battery_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_DESCRIPTION("ADC-based battery driver for Samsung Spica");
MODULE_LICENSE("GPL");
