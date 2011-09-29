/*
 * max8698.c - Maxim MAX8698 voltage regulator driver
 *
 * Copyright (C) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/max8698.h>

/*
 * Driver data
 */

struct max8698_data {
	struct device		*dev;
	struct i2c_client	*i2c_client;
	int			num_regulators;
	struct regulator_dev	**rdev;
	struct mutex		lock;
};

/*
 * I2C Interface
 */

static int max8698_i2c_device_read(struct max8698_data *max8698, u8 reg, u8 *dest)
{
	struct i2c_client *client = max8698->i2c_client;
	int ret;

	mutex_lock(&max8698->lock);

	ret = i2c_smbus_read_byte_data(client, reg);

	mutex_unlock(&max8698->lock);

	if (ret < 0)
		return ret;

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int max8698_i2c_device_update(struct max8698_data *max8698, u8 reg,
				     u8 val, u8 mask)
{
	struct i2c_client *client = max8698->i2c_client;
	int ret;

	mutex_lock(&max8698->lock);

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(client, reg, new_val);
		if (ret >= 0)
			ret = 0;
	}

	mutex_unlock(&max8698->lock);

	return ret;
}

/*
 * Voltage regulator
 */

struct voltage_map_desc {
	int min;
	int max;
	int step;
};

/* Voltage maps */
static const struct voltage_map_desc ldo23_voltage_map_desc = {
	.min = 800,	.step = 50,	.max = 1300,
};
static const struct voltage_map_desc ldo45679_voltage_map_desc = {
	.min = 1600,	.step = 100,	.max = 3600,
};
static const struct voltage_map_desc ldo8_voltage_map_desc = {
	.min = 3000,	.step = 100,	.max = 3600,
};
static const struct voltage_map_desc buck12_voltage_map_desc = {
	.min = 750,	.step = 50,	.max = 1500,
};
static const struct voltage_map_desc buck3_voltage_map_desc = {
	.min = 1600,	.step = 100,	.max = 3600,
};

static const struct voltage_map_desc *ldo_voltage_map[] = {
	&ldo23_voltage_map_desc,	/* LDO2 */
	&ldo23_voltage_map_desc,	/* LDO3 */
	&ldo45679_voltage_map_desc,	/* LDO4 */
	&ldo45679_voltage_map_desc,	/* LDO5 */
	&ldo45679_voltage_map_desc,	/* LDO6 */
	&ldo45679_voltage_map_desc,	/* LDO7 */
	&ldo8_voltage_map_desc,		/* LDO8 */
	&ldo45679_voltage_map_desc,	/* LDO9 */
	&buck12_voltage_map_desc,	/* BUCK1 */
	&buck12_voltage_map_desc,	/* BUCK2 */
	&buck3_voltage_map_desc,	/* BUCK3 */
};

static inline int max8698_get_ldo(struct regulator_dev *rdev)
{
	return rdev_get_id(rdev);
}

static int max8698_list_voltage(struct regulator_dev *rdev,
				unsigned int selector)
{
	const struct voltage_map_desc *desc;
	int ldo = max8698_get_ldo(rdev);
	int val;

	if (ldo >= ARRAY_SIZE(ldo_voltage_map))
		return -EINVAL;

	desc = ldo_voltage_map[ldo];
	if (desc == NULL)
		return -EINVAL;

	val = desc->min + desc->step * selector;
	if (val > desc->max)
		return -EINVAL;

	return val * 1000;
}

enum {
	MAX8698_REG_ONOFF1,
	MAX8698_REG_ONOFF2,
	MAX8698_REG_ADISCHG_EN1,
	MAX8698_REG_ADISCHG_EN2,
	MAX8698_REG_DVSARM12,
	MAX8698_REG_DVSARM34,
	MAX8698_REG_DVSINT12,
	MAX8698_REG_BUCK3,
	MAX8698_REG_LDO23,
	MAX8698_REG_LDO4,
	MAX8698_REG_LDO5,
	MAX8698_REG_LDO6,
	MAX8698_REG_LDO7,
	MAX8698_REG_LDO8_BKCHR,
	MAX8698_REG_LDO9,
	MAX8698_REG_LBCNFG
};

static int max8698_get_enable_register(struct regulator_dev *rdev,
					int *reg, int *shift)
{
	int ldo = max8698_get_ldo(rdev);

	switch (ldo) {
	case MAX8698_LDO2 ... MAX8698_LDO5:
		*reg = MAX8698_REG_ONOFF1;
		*shift = 4 - (ldo - MAX8698_LDO2);
		break;
	case MAX8698_LDO6 ... MAX8698_LDO9:
		*reg = MAX8698_REG_ONOFF2;
		*shift = 7 - (ldo - MAX8698_LDO6);
		break;
	case MAX8698_BUCK1 ... MAX8698_BUCK3:
		*reg = MAX8698_REG_ONOFF1;
		*shift = 7 - (ldo - MAX8698_BUCK1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max8698_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	int ret, reg, shift = 8;
	u8 val;

	ret = max8698_get_enable_register(rdev, &reg, &shift);
	if (ret)
		return ret;

	ret = max8698_i2c_device_read(max8698, reg, &val);
	if (ret)
		return ret;

	return val & (1 << shift);
}

static int max8698_ldo_enable(struct regulator_dev *rdev)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	int reg, shift = 8, ret;

	ret = max8698_get_enable_register(rdev, &reg, &shift);
	if (ret)
		return ret;

	return max8698_i2c_device_update(max8698, reg, 1<<shift, 1<<shift);
}

static int max8698_ldo_disable(struct regulator_dev *rdev)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	int reg, shift = 8, ret;

	ret = max8698_get_enable_register(rdev, &reg, &shift);
	if (ret)
		return ret;

	return max8698_i2c_device_update(max8698, reg, 0, 1<<shift);
}

static int max8698_get_voltage_register(struct regulator_dev *rdev,
				int *_reg, int *_shift, int *_mask)
{
	int ldo = max8698_get_ldo(rdev);
	int reg, shift = 0, mask = 0xff;

	switch (ldo) {
	case MAX8698_LDO2 ... MAX8698_LDO3:
		reg = MAX8698_REG_LDO23;
		mask = 0xf;
		if (ldo == MAX8698_LDO3)
			shift = 4;
		break;
	case MAX8698_LDO4 ... MAX8698_LDO7:
		reg = MAX8698_REG_LDO4 + (ldo - MAX8698_LDO4);
		break;
	case MAX8698_LDO8:
		reg = MAX8698_REG_LDO8_BKCHR;
		mask = 0xf;
		shift = 4;
		break;
	case MAX8698_LDO9:
		reg = MAX8698_REG_LDO9;
		break;
	case MAX8698_BUCK1:
		reg = MAX8698_REG_DVSARM12;
		mask = 0xf;
		break;
	case MAX8698_BUCK2:
		reg = MAX8698_REG_DVSINT12;
		mask = 0xf;
		break;
	case MAX8698_BUCK3:
		reg = MAX8698_REG_BUCK3;
		break;
	default:
		return -EINVAL;
	}

	*_reg = reg;
	*_shift = shift;
	*_mask = mask;

	return 0;
}

static int max8698_get_voltage(struct regulator_dev *rdev)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	int reg, shift = 0, mask, ret;
	u8 val;

	ret = max8698_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	ret = max8698_i2c_device_read(max8698, reg, &val);
	if (ret)
		return ret;

	val >>= shift;
	val &= mask;

	return max8698_list_voltage(rdev, val);
}

static int max8698_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	const struct voltage_map_desc *desc;
	int min_vol = min_uV / 1000, max_vol = max_uV / 1000;
	int ldo = max8698_get_ldo(rdev);
	int reg = 0, shift = 0, mask = 0, ret;
	int i = 0;
	int sel_mV;

	if (ldo >= ARRAY_SIZE(ldo_voltage_map))
		return -EINVAL;

	desc = ldo_voltage_map[ldo];
	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	sel_mV = desc->min;
	while (sel_mV < min_vol && sel_mV < desc->max) {
		sel_mV += desc->step;
		++i;
	}

	if (sel_mV > max_vol)
		return -EINVAL;

	*selector = i;

	ret = max8698_get_voltage_register(rdev, &reg, &shift, &mask);
	if (ret)
		return ret;

	return max8698_i2c_device_update(max8698, reg, i<<shift, mask<<shift);
}

static struct regulator_ops max8698_regulator_ops = {
	.list_voltage		= max8698_list_voltage,
	.is_enabled		= max8698_ldo_is_enabled,
	.enable			= max8698_ldo_enable,
	.disable		= max8698_ldo_disable,
	.get_voltage		= max8698_get_voltage,
	.set_voltage		= max8698_set_voltage,
	.set_suspend_enable	= max8698_ldo_enable,
	.set_suspend_disable	= max8698_ldo_disable,
};

static int max8698_set_buck12_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	struct max8698_data *max8698 = rdev_get_drvdata(rdev);
	const struct voltage_map_desc *desc;
	int min_vol = min_uV / 1000, max_vol = max_uV / 1000;
	int previous_vol = 0;
	int sel_mV;
	int ldo = max8698_get_ldo(rdev), i = 0, ret;
	int difference, rate;
	u8 val = 0;

	if (ldo >= ARRAY_SIZE(ldo_voltage_map))
		return -EINVAL;

	desc = ldo_voltage_map[ldo];
	if (desc == NULL)
		return -EINVAL;

	if (max_vol < desc->min || min_vol > desc->max)
		return -EINVAL;

	sel_mV = desc->min;
	while (sel_mV < min_vol && sel_mV < desc->max) {
		sel_mV += desc->step;
		++i;
	}

	if (sel_mV > max_vol)
		return -EINVAL;

	*selector = i;

	/* Read ramp rate */
	ret = max8698_i2c_device_read(max8698, MAX8698_REG_ADISCHG_EN2, &val);
	if (ret)
		goto err;
	rate = (val & 0xf) + 1;

	previous_vol = max8698_get_voltage(rdev);

	switch (ldo) {
	case MAX8698_BUCK1:
		ret = max8698_i2c_device_update(max8698,
				MAX8698_REG_DVSARM12, (i << 4) | i, 0xff);
		if (ret)
			goto err;
		ret = max8698_i2c_device_update(max8698,
				MAX8698_REG_DVSARM34, (i << 4) | i, 0xff);
		if (ret)
			goto err;
		break;
	case MAX8698_BUCK2:
		ret = max8698_i2c_device_update(max8698,
				MAX8698_REG_DVSINT12, (i << 4) | i, 0xff);
		if (ret)
			goto err;
		break;
	}

	difference = desc->min + desc->step*i - previous_vol/1000;
	if (difference < 0)
		difference = -difference;

	/* wait for ramp delay */
	udelay(difference / rate);

err:
	return ret;
}

static struct regulator_ops max8698_regulator_buck12_ops = {
	.list_voltage		= max8698_list_voltage,
	.is_enabled		= max8698_ldo_is_enabled,
	.enable			= max8698_ldo_enable,
	.disable		= max8698_ldo_disable,
	.get_voltage		= max8698_get_voltage,
	.set_voltage		= max8698_set_buck12_voltage,
	.set_suspend_enable	= max8698_ldo_enable,
	.set_suspend_disable	= max8698_ldo_disable,
};

static struct regulator_desc regulators[] = {
	{
		.name		= "LDO2",
		.id		= MAX8698_LDO2,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO3",
		.id		= MAX8698_LDO3,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO4",
		.id		= MAX8698_LDO4,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO5",
		.id		= MAX8698_LDO5,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO6",
		.id		= MAX8698_LDO6,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO7",
		.id		= MAX8698_LDO7,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO8",
		.id		= MAX8698_LDO8,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "LDO9",
		.id		= MAX8698_LDO9,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "BUCK1",
		.id		= MAX8698_BUCK1,
		.ops		= &max8698_regulator_buck12_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "BUCK2",
		.id		= MAX8698_BUCK2,
		.ops		= &max8698_regulator_buck12_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	}, {
		.name		= "BUCK3",
		.id		= MAX8698_BUCK3,
		.ops		= &max8698_regulator_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
};

/*
 * I2C driver
 */

static int max8698_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct max8698_platform_data *pdata = dev_get_platdata(&i2c->dev);
	struct regulator_dev **rdev;
	struct max8698_data *max8698;
	int i, ret, size;

	if (!pdata) {
		dev_err(&i2c->dev, "No platform init data supplied\n");
		return -ENODEV;
	}

	max8698 = kzalloc(sizeof(struct max8698_data), GFP_KERNEL);
	if (!max8698)
		return -ENOMEM;

	size = sizeof(struct regulator_dev *) * pdata->num_regulators;
	max8698->rdev = kzalloc(size, GFP_KERNEL);
	if (!max8698->rdev) {
		kfree(max8698);
		return -ENOMEM;
	}

	mutex_init(&max8698->lock);
	rdev = max8698->rdev;
	max8698->dev = &i2c->dev;
	max8698->i2c_client = i2c;
	max8698->num_regulators = pdata->num_regulators;
	i2c_set_clientdata(i2c, max8698);

	for (i = 0; i < pdata->num_regulators; i++) {
		const struct voltage_map_desc *desc;
		int id = pdata->regulators[i].id;

		if (id >= ARRAY_SIZE(regulators))
			continue;

		desc = ldo_voltage_map[id];
		if (desc) {
			int count = (desc->max - desc->min) / desc->step + 1;
			regulators[id].n_voltages = count;
		}
		rdev[i] = regulator_register(&regulators[id], max8698->dev,
				pdata->regulators[i].initdata, max8698);
		if (IS_ERR(rdev[i])) {
			ret = PTR_ERR(rdev[i]);
			dev_err(max8698->dev, "regulator init failed\n");
			rdev[i] = NULL;
			goto err;
		}
	}

	max8698_i2c_device_update(max8698, MAX8698_REG_LBCNFG,
		(pdata->lbhyst & 3) << 4 | (pdata->lbth & 7) << 1, 0x3e);

	return 0;
err:
	for (i = 0; i < max8698->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	kfree(max8698->rdev);
	kfree(max8698);

	return ret;
}

static int __devexit max8698_remove(struct i2c_client *i2c)
{
	struct max8698_data *max8698 = i2c_get_clientdata(i2c);
	struct regulator_dev **rdev = max8698->rdev;
	int i;

	for (i = 0; i < max8698->num_regulators; i++)
		if (rdev[i])
			regulator_unregister(rdev[i]);

	kfree(max8698->rdev);
	kfree(max8698);

	return 0;
}

static const struct i2c_device_id max8698_i2c_id[] = {
       { "max8698", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, max8698_i2c_id);

static struct i2c_driver max8698_i2c_driver = {
	.driver = {
		   .name = "max8698",
		   .owner = THIS_MODULE,
	},
	.probe = max8698_probe,
	.remove = max8698_remove,
	.id_table = max8698_i2c_id,
};

static int __init max8698_init(void)
{
	return i2c_add_driver(&max8698_i2c_driver);
}
subsys_initcall(max8698_init);

static void __exit max8698_exit(void)
{
	i2c_del_driver(&max8698_i2c_driver);
}
module_exit(max8698_exit);

MODULE_DESCRIPTION("Maxim 8698 voltage regulator driver");
MODULE_AUTHOR("Tomasz Figa <tomasz.figa at gmail.com>");
MODULE_LICENSE("GPLv2");
