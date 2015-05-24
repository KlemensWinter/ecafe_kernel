/*
 * SBS11 battery driver
 *
 * Copyright (C) 2010 Joe Chen <joechen@soltech.com.cn>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/slab.h>



#define DRIVER_VERSION			"1.5.0"

#define SBS11_REG_TEMP		0x08	/* Temperature */
#define SBS11_REG_VOLT		0x09	/* Voltage */
#define SBS11_REG_CURR		0x0A	/* Current */
#define SBS11_REG_AI		0x0B	/* Average Current */
#define SBS11_REG_RSOC		0x0D	/* Relative State-of-Charge */

#define SBS11_REG_CHARGING_V	0x15	/* Charging voltage */
#define SBS11_REG_STATUS	0x16	/* Battery Status */
#define SBS11_REG_CYCLE_COUNT	0x17	/* Cycle count */
#define SBS11_REG_DESIGN_CAP	0x18	/* Design Capacity */
#define SBS11_REG_DESIGN_VOLT	0x19	/* Design Voltage */
#define SBS11_REG_MAN_NAME	0x20	/* Manufacturer Name */
#define SBS11_REG_DEV_NAME	0x21	/* Device Name */


#define SBS11_MIN_CAP	10		// Power ON Red LED at 10% 
// #define SBS11_FULL_CAP	2400000	// Battery Capacity 
#define SBS11_FULL_CAP		6800000		// Battery Capacity
#define SBS11_MAX_VOLTAGE	8400000
#define SBS11_MIN_VOLTAGE	6500000

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static void (*na04_power_led) (int);

struct sbs11_device_info;
struct sbs11_access_methods {
	int (*read)(u8 reg, int *rt_value, struct sbs11_device_info *di);
};

struct sbs11_device_info {
	struct device 		*dev;
	int			id;
	struct sbs11_access_methods	*bus;
	struct power_supply	bat;
	struct power_supply	charger;
	int 			charge_status;
	struct workqueue_struct	*monitor_wqueue;
	struct delayed_work	monitor_work;

	int bat_max;
	int bat_min;
	int bat_cap;



	struct i2c_client	*client;
};

static enum power_supply_property sbs11_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property ac_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

/*
 * Common code for SBS11 devices
 */

static int sbs11_read(u8 reg, int *rt_value, struct sbs11_device_info *di)
{
	struct i2c_client *client = di->client;

	if (!client->adapter)
		return -ENODEV;

	*rt_value = i2c_smbus_read_word_data(client, reg);
	*rt_value = (short) *rt_value; 
	return 0;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int sbs11_battery_temperature(struct sbs11_device_info *di)
{
	int ret;
	int temp = 0;

	ret = di->bus->read(SBS11_REG_TEMP, &temp, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	temp = temp - 2730;
	return temp;
}

/*
 * Return the battery Voltage in uV
 * Or < 0 if something fails.
 */
static int sbs11_battery_voltage(struct sbs11_device_info *di)
{
	int ret;
	int volt = 0;

	ret = di->bus->read(SBS11_REG_VOLT, &volt, di);
	if (ret) {
		ret = di->bus->read(SBS11_REG_VOLT, &volt, di);
		if (ret) {
			dev_err(di->dev, "error reading voltage\n");
			return ret;
		}	
	}
	return volt * 1000;
}

/*
 * Return the battery average current in uA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int sbs11_battery_current(struct sbs11_device_info *di)
{
	int ret;
	//s16 curr = 0;
	int curr = 0;


	ret = di->bus->read(SBS11_REG_CURR, &curr, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	if(curr < 0 )
		curr = curr * -1;
	return curr * 1000;
}

/*
 * Return the battery average current in uA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int sbs11_battery_average_current(struct sbs11_device_info *di)
{
	int ret;
	//s16 curr = 0;
	int curr = 0;


	ret = di->bus->read(SBS11_REG_AI, &curr, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	if(curr < 0 )
		curr = curr * -1;
	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int sbs11_battery_rsoc(struct sbs11_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = di->bus->read(SBS11_REG_RSOC, &rsoc, di);
	if (ret) {
		ret = di->bus->read(SBS11_REG_RSOC, &rsoc, di);
		if (ret) {
			dev_err(di->dev, "error reading relative State-of-Charge\n");
			return ret;
		}
	}

	if ((rsoc < SBS11_MIN_CAP)  && (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING)) {
		if(na04_power_led)
			na04_power_led(1);
	}
	else {
		if(na04_power_led)
			na04_power_led(0);
	}
	return rsoc;
}

static int sbs11_battery_capacity(struct sbs11_device_info *di)
{
	int ret;
	int status = 0;

	ret = di->bus->read(SBS11_REG_DESIGN_CAP, &status, di);
	if (ret) {
		dev_err(di->dev, "error reading battery status\n");
		return ret;
	}
	return status;
}

/*
 * Return the battery status
 * Or < 0 if something fails.
 */
static int sbs11_battery_status(struct sbs11_device_info *di)
{
	int ret;
	int status = 0;
//	s16 curr = 0;
	int curr = 0;

	ret = di->bus->read(SBS11_REG_CURR, &curr, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	
//	printk("na04 battery current %d\n",curr);

	if (curr < 0)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (curr > 0)
		{
		ret = di->bus->read(SBS11_REG_STATUS, &status, di);
		if (ret) {
			dev_err(di->dev, "error reading battery status\n");
			return ret;
			}
//		printk("na04 battery status %d\n",status);

		if (status & 0x0020)
			return POWER_SUPPLY_STATUS_FULL;
		else return POWER_SUPPLY_STATUS_CHARGING;
		}

	return POWER_SUPPLY_STATUS_NOT_CHARGING;
}




static void sbs11_battery_update_status(struct sbs11_device_info *di)
{
	int old_charge_status = di->charge_status;

	di->charge_status = sbs11_battery_status(di);

	if (di->charge_status != old_charge_status)
	{
		power_supply_changed(&di->bat);
	}
}

static void sbs11_battery_work(struct work_struct *work)
{
	struct sbs11_device_info *di = container_of(work,
						     struct sbs11_device_info,
						     monitor_work.work);
	int interval = HZ / 10; // * 60;

	interval = msecs_to_jiffies(2000);

	sbs11_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static int sbs11_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct sbs11_device_info *di =
		container_of((psy), struct sbs11_device_info, bat);
	int remain_capacity;

//	printk("na04 battery get property %d\n",psp);


	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sbs11_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sbs11_battery_average_current(di)*10;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = sbs11_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = di->bat_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = di->bat_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = di->bat_min; 
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		remain_capacity = sbs11_battery_rsoc(di);
		val->intval = (sbs11_battery_voltage(di)/1000) * ((di->bat_cap * remain_capacity)/100000 ); 
		if(val->intval <0 )
			val->intval = val->intval * -1;
		break;

	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = (sbs11_battery_voltage(di)/1000) * (di->bat_cap /1000 ); 
		if(val->intval <0 )
			val->intval = val->intval * -1;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		val->intval = (di->bat_min/1000) * ((di->bat_cap * SBS11_MIN_CAP) /100000);
		if(val->intval <0 )
			val->intval = val->intval * -1;
		break;

	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = (di->bat_max/1000) * (di->bat_cap /1000 ); 
		if(val->intval <0 )
			val->intval = val->intval * -1;
		break;


	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = (sbs11_battery_voltage(di)/1000) * (sbs11_battery_current(di)/1000);
		if(val->intval <0 )
			val->intval = val->intval * -1;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = sbs11_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = sbs11_battery_status(di);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	default:
		return -EINVAL;
	}
//	printk("na04 battery get property return %d\n",val->intval);

	return 0;
}

static int ac_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sbs11_device_info *di =
		container_of((psy), struct sbs11_device_info, charger);

//	sbs11_battery_update_status();

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if( di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING)
			val->intval = 0;
		else
			val->intval = 1;
//		val->intval = di->charge_status;
		return 0;
//		return gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CLE));
	default:
		break;
	}
	return -EINVAL;
}

static int sbs11_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct sbs11_device_info *di;
	struct sbs11_access_methods *bus;
	int num;
	int retval = 0;

#if 0	//joechen ++
/*!
 * Power Key initialization.
 */
	int irq, ret;
	irq = IOMUX_TO_IRQ(MX51_PIN_NANDF_CLE);
	set_irq_type(irq, IRQF_TRIGGER_RISING);
	ret = request_irq(irq, power_key_int, 0, "power_key", 0);
	if (ret)
		pr_info("register on-off key interrupt failed\n");
	else
		enable_irq_wake(irq);
	return ret;
}

#endif

	struct mxc_sbs11_platform_data *plat = client->dev.platform_data;
	if (plat->power_led)
			na04_power_led = plat->power_led;
		else
			na04_power_led = NULL;


	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "BAT%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &sbs11_read;
	di->bus = bus;
	di->client = client;
	di->bat.name = name;
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = sbs11_battery_props;
	di->bat.num_properties = ARRAY_SIZE(sbs11_battery_props);
	di->bat.get_property = sbs11_battery_get_property;
	di->bat.use_for_apm = 1;
	di->bat.external_power_changed = NULL;
	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

	di->charger.name	= "ac_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = ac_charger_props;
	di->charger.num_properties = ARRAY_SIZE(ac_charger_props);
	di->charger.get_property = ac_charger_get_property;
	retval = power_supply_register(&client->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto batt_failed_5;
	}

	di->bat_max = SBS11_MAX_VOLTAGE;
	di->bat_min = SBS11_MIN_VOLTAGE;
	di->bat_cap = sbs11_battery_capacity(di) * 1000;


	INIT_DELAYED_WORK(&di->monitor_work, sbs11_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto batt_failed_6;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, msecs_to_jiffies(2000));//;HZ / 2);


	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed_6:
	power_supply_unregister(&di->charger);
batt_failed_5:
	power_supply_unregister(&di->bat);
batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int sbs11_battery_remove(struct i2c_client *client)
{
	struct sbs11_device_info *di = i2c_get_clientdata(client);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id sbs11_id[] = {
	{ "sbs11-battery", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sbs11_id);

static struct i2c_driver sbs11_battery_driver = {
	.driver = {
		.name = "sbs11-battery",
//		.owner = THIS_MODULE,
	},
	.probe = sbs11_battery_probe,
	.remove = sbs11_battery_remove,
	.id_table = sbs11_id,
};

static int __init sbs11_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&sbs11_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register SBS11 driver\n");

	return ret;
}
module_init(sbs11_battery_init);

static void __exit sbs11_battery_exit(void)
{
	i2c_del_driver(&sbs11_battery_driver);
}
module_exit(sbs11_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("SBS11 battery monitor driver");
MODULE_LICENSE("GPL");
