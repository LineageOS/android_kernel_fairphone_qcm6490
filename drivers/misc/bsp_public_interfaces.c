/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <soc/qcom/socinfo.h>
#include <linux/bsp_public_interfaces.h>
#ifdef CONFIG_ESIM_REGULATOR_ON
#include <linux/regulator/consumer.h>
#endif

struct board_platform_data *pdata;

int get_option (char **str, int *pint);
static int android_recoveryreason = 0;
static int android_mode_charger = 0;
static bool hotswap_status = false;

static int __init android_boot_recoveryreason(char *str)
{
	get_option(&str, &android_recoveryreason);
	return 1;
}
__setup("androidboot.recoveryreason=", android_boot_recoveryreason);

static int __init android_boot_mode(char *str)
{
	if (!strcmp(str, "charger"))
		android_mode_charger = 1;
	return 1;
}
__setup("androidboot.mode=", android_boot_mode);

bool is_hot_swap(void)
{
	return hotswap_status;
}
EXPORT_SYMBOL(is_hot_swap);

void set_hot_swap(bool status)
{
	hotswap_status = status;
}
EXPORT_SYMBOL(set_hot_swap);

int isChargerMode(void)
{
	return android_mode_charger;
}
EXPORT_SYMBOL(isChargerMode);

int isRecoveryMode(void)
{
	return android_recoveryreason;
}
EXPORT_SYMBOL(isRecoveryMode);

int get_hw_version(void)
{
	return pdata->hw_version;
}
EXPORT_SYMBOL(get_hw_version);



static ssize_t sysfs_hw_version_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", pdata->board_version);
	//return sprintf(buf, "%d\n", pdata->hw_version);
}

static struct kobj_attribute sysfs_attrs[] = {
	__ATTR(hw_version, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
			 sysfs_hw_version_show, NULL),
};

/*
static struct attribute *devices_attributes[] = {
	&dev_attr_hw_version.attr,
	NULL,
};

static struct attribute_group board_config_attr = {
	.attrs = devices_attributes,
};
*/
static int get_board_gpio_value(unsigned gpio, const char *label)
{
	int err;
	int value;

	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, label);
		if (err) {
			pr_err("request board_id GPIO#%d failed, err=%d\n", gpio, err);
			gpio_free(gpio);
			return -1;
		}
	}

	err = gpio_direction_input(gpio);
	if (err) {
		pr_err("Set GPIO#%d input fail, err=%d\n", gpio, err);
	}

	value = gpio_get_value_cansleep(gpio);

	return value;
}

static int board_check_hwid(struct board_platform_data *pdata)
{
	
	pdata->id0_val = get_board_gpio_value(pdata->board_id0, "board_id0");
	pdata->id1_val = get_board_gpio_value(pdata->board_id1, "board_id1");
	pdata->id2_val = get_board_gpio_value(pdata->board_id2, "board_id2");
	pdata->id3_val = get_board_gpio_value(pdata->board_id3, "board_id3");


	pdata->hw_version = (pdata->id3_val << 3)|
						(pdata->id2_val << 2)|
                        (pdata->id1_val << 1)|
                        (pdata->id0_val << 0);

	if (pdata->hw_version == 0) {
		strcpy(pdata->board_version,"EVT");
	} else if (pdata->hw_version == 1) {
		strcpy(pdata->board_version,"DVT1");
	} else if (pdata->hw_version == 2) {
		strcpy(pdata->board_version,"DVT2");
	} else if (pdata->hw_version == 3) {
		strcpy(pdata->board_version,"MVT");
	} else {
		strcpy(pdata->board_version,"UNKNOWN");
	}
	printk(KERN_INFO"HW Version:(%d).\n", pdata->hw_version);

	return 0;
}

static int board_parse_dt(struct device *dev,
			struct board_platform_data *pdata)
{
	const char *name;
    int retval;
	struct device_node *np = dev->of_node;

	if (of_find_property(np, "board_id0",NULL)) {
		pdata->board_id0= of_get_named_gpio_flags(np,
				"board_id0", 0, NULL);
	} else {
		pdata->board_id0= -1;
	}


	if (of_find_property(np, "board_id1",NULL)) {
		pdata->board_id1= of_get_named_gpio_flags(np,
				"board_id1", 0, NULL);
	} else {
		pdata->board_id1= -1;
	}

	if (of_find_property(np, "board_id2",NULL)) {
		pdata->board_id2 = of_get_named_gpio_flags(np,
				"board_id2", 0, NULL);
	} else {
		pdata->board_id2= -1;
	}

	if (of_find_property(np, "board_id3",NULL)) {
		pdata->board_id3 = of_get_named_gpio_flags(np,
				"board_id3", 0, NULL);
	} else {
		pdata->board_id3= -1;
	}

	retval = of_property_read_string(np, "board,name", &name);
	if (retval == -EINVAL)
		pdata->name= NULL;
	else if (retval < 0)
		return retval;
	else
		pdata->name = name;

	return 0;
}

static int bsp_public_interfaces_probe(struct platform_device * pdev)
{
	int ret;
	unsigned char attr_count;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct board_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&pdev->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = board_parse_dt(&pdev->dev, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "pdata is invalid\n");
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("board_detect: Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	pin_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL(pin_default)) {
		pr_err("board_detect: Failed to look up default state\n");
		return PTR_ERR(pinctrl);
	}

	ret = pinctrl_select_state(pinctrl, pin_default);
	if (ret) {
		pr_err("board_detect: Can't select pinctrl state\n");
		return ret;
	}

	platform_set_drvdata(pdev, pdata);

	pdata->sysinfo_kobj = kobject_create_and_add("info", NULL) ;
	if (pdata->sysinfo_kobj == NULL) {
		dev_err(&pdev->dev,"kobject_create_and_add() failed!");
		return -ENOMEM;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(sysfs_attrs); attr_count++) {
		ret = sysfs_create_file(pdata->sysinfo_kobj, &sysfs_attrs[attr_count].attr);
		if (ret) {
			dev_err(&pdev->dev, "%s: sysfs_create_group_file failed\n", __func__);
			goto destroy_sysfs;
		}
	}
    board_check_hwid(pdata);
#ifdef CONFIG_ESIM_REGULATOR_ON
	pdata->esim_power = regulator_get(&pdev->dev, "esim_regulator");
		if (IS_ERR(pdata->esim_power)) {
			printk("regulator_get regulator fail\n");
			//return -EINVAL;
		}
		else {
	    	ret = regulator_set_voltage(pdata->esim_power, 1810000, 1810000); 
	    		if (ret)
	    		{
	        		printk("Could not set to 1.81v.\n");
	    		}
		}

	ret = regulator_enable(pdata->esim_power); 
	    	if (ret)
	    	{
			dev_err(&pdev->dev, "%s: Could not enable esim regulator.\n", __func__);
	    	}
#endif
	printk("####BSP public interface Probe Success####\n");
	return 0;

destroy_sysfs:
	for (attr_count = 0; attr_count < ARRAY_SIZE(sysfs_attrs); attr_count++) {
		sysfs_remove_file(&pdev->dev.kobj, &sysfs_attrs[attr_count].attr);
	}
	return ret;
}

static int bsp_public_interfaces_remove(struct platform_device *pdev)
{
	unsigned char attr_count;
	struct board_platform_data *pdata = platform_get_drvdata(pdev);

	gpio_free(pdata->board_id0);
	gpio_free(pdata->board_id1);
	gpio_free(pdata->board_id2);
	gpio_free(pdata->board_id3);
#ifdef CONFIG_ESIM_REGULATOR_ON
	regulator_disable(pdata->esim_power); 
#endif
	for (attr_count = 0; attr_count < ARRAY_SIZE(sysfs_attrs); attr_count++) {
		sysfs_remove_file(&pdev->dev.kobj, &sysfs_attrs[attr_count].attr);
	}
	return 0;
}

static struct of_device_id board_match_table[] = {
	{ .compatible = "t2,bsp-public-interfaces",},
	{ },
};

static struct platform_driver bsp_public_interfaces_driver = {
	.probe = bsp_public_interfaces_probe,
	.remove = bsp_public_interfaces_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "board_id_detect",
		.of_match_table = board_match_table,
	},
};

static int __init bsp_public_interfaces_driver_init(void)
{
	platform_driver_register(&bsp_public_interfaces_driver);
	return 0;
}

static void __exit bsp_public_interfaces_driver_exit(void)
{
	platform_driver_unregister(&bsp_public_interfaces_driver);
}

subsys_initcall(bsp_public_interfaces_driver_init);
module_exit(bsp_public_interfaces_driver_exit);
MODULE_LICENSE("GPL v2");
