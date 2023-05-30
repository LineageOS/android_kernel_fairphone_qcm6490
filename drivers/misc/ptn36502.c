/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define PTN_DRIVER_NAME                     "ptn36502"
#define PTN_REGISTER_DEBUG

struct i2c_client *ptn_client;
static int redrive_ldo1v8_en;
static int redriver_en;
bool ptn_dp_enabled = false; //add by yushixian for FP5-733 20230511

static int ptn_write_reg(u8 reg, u8 val)
{
	int ret=0;

	ret = i2c_smbus_write_byte_data(ptn_client, reg, val);

	if (ret >= 0)
		return 0;

	pr_info("%s: reg 0x%x, val 0x%x, err %d\n", __func__, reg, val, ret);

	return ret;
}


static int ptn_read_reg(u8 reg, u8 *data)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(ptn_client, reg);
	if (ret < 0) {
		pr_err("%s: can't read from reg 0x%02X\n", __func__, reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;
}



static int g_orientation = 0;
	//modified by yushixian for FP5-733 20230511 start
void ptn_orientation_switch(int orientation, bool enable)
{
	pr_err("%s: orientation=%d enable = %d\n", __func__, orientation,enable);
	if (orientation == 0) {
		ptn_write_reg(0x0b, 0x00); //enter deep power-saving mode when DP connector plug out
	} else if (orientation == 1) {
		//ptn_write_reg(0x0b, 0x01);
		//gpio_direction_output(aux_switch_gpio, 0); //zxz notice it 
	} else if (orientation == 2) {
		//ptn_write_reg(0x0b, 0x21);
		//gpio_direction_output(aux_switch_gpio, 1); //zxz notice it 
	}
	ptn_dp_enabled = enable;
	//modified by yushixian for FP5-733 20230511 end
	g_orientation = orientation;
}
EXPORT_SYMBOL(ptn_orientation_switch);

void ptn_usb_orientation_switch(int orientation)
{
	pr_err("%s: orientation=%d\n", __func__, orientation);

	if (orientation == 1)
		ptn_write_reg(0x0b, 0x01);
	else if (orientation == 2)
		ptn_write_reg(0x0b, 0x21);
	else if (orientation == 0)
		ptn_write_reg(0x0b, 0x00); //enter deep power-saving mode when USB plug out
}
EXPORT_SYMBOL(ptn_usb_orientation_switch);

void ptn_lane_switch(int lane, int bwcode)
{
	pr_err("%s: lane=%d, orientation=%d bwcode=%d\n", __func__, lane, g_orientation, bwcode);
	if (lane == 2 && g_orientation == 1) {
		ptn_write_reg(0x0d, 0x80);
		ptn_write_reg(0x0b, 0x0a);
		ptn_write_reg(0x06, 0x0a);
		// if (bwcode == 10) {
		// 	ptn_write_reg(0x06, 0x09);
		// } else {
		// 	ptn_write_reg(0x06, 0x0a);
		// }
		ptn_write_reg(0x07, 0x29);
		ptn_write_reg(0x08, 0x29);
		ptn_write_reg(0x09, 0x29);
		ptn_write_reg(0x0A, 0x29);
	} else if (lane == 2 && g_orientation == 2) {
		ptn_write_reg(0x0d, 0x80);
		ptn_write_reg(0x0b, 0x2a);
		ptn_write_reg(0x06, 0x0a);
		// if (bwcode == 10) {
		// 	ptn_write_reg(0x06, 0x09);
		// } else {
		// 	ptn_write_reg(0x06, 0x0a);
		// }
		ptn_write_reg(0x07, 0x29);
		ptn_write_reg(0x08, 0x29);
		ptn_write_reg(0x09, 0x29);
		ptn_write_reg(0x0A, 0x29);
	} else if (lane == 4 && g_orientation == 1) {
		ptn_write_reg(0x0d, 0x80);
		ptn_write_reg(0x0b, 0x0b);
		ptn_write_reg(0x06, 0x0e);
		ptn_write_reg(0x07, 0x29);
		ptn_write_reg(0x08, 0x29);
		ptn_write_reg(0x09, 0x29);
		ptn_write_reg(0x0A, 0x29);
	} else if (lane == 4 && g_orientation == 2) {
		ptn_write_reg(0x0d, 0x80);
		ptn_write_reg(0x0b, 0x2b);
		ptn_write_reg(0x06, 0x0e);
		ptn_write_reg(0x07, 0x29);
		ptn_write_reg(0x08, 0x29);
		ptn_write_reg(0x09, 0x29);
		ptn_write_reg(0x0A, 0x29);
	}
}
EXPORT_SYMBOL(ptn_lane_switch);
#if 0
static struct pinctrl		*pinctrl;
static struct pinctrl_state	*pin_default;

static int ptn_pinctrl_init(struct i2c_client *client)
{
	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}
	pin_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL(pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(pin_default);
	}
	return 0;
}
#endif

#ifdef PTN_REGISTER_DEBUG
static ssize_t ptn_store_registers(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && (reg < 0x0E) && (reg >0x03)) {
		pr_err("ptn_store_registers, reg=%x , val=%x \n",reg,val);
		ptn_write_reg((u8)reg, (u8)val);
	}else{
		pr_err("ptn_store_registers, write register fail, the reg range should be 0x03<reg< 0x0E ,format like : '0x04 0x2' \n");
	}

	return count;
}

static ssize_t ptn_show_registers(struct device *dev,
struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "PTN Reg");
	for (addr = 0x0; addr <= 0x0D; addr++) {
		ret = ptn_read_reg(addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static DEVICE_ATTR(ptn_registers, S_IRUGO | S_IWUSR, ptn_show_registers, ptn_store_registers);


static struct attribute *ptn_attributes[] = {
&dev_attr_ptn_registers.attr,
NULL,
};

static const struct attribute_group ptn_attr_group = {
	.attrs = ptn_attributes,
};
#endif


static int ptn36502_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = 0;
	struct device_node *np = NULL;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;

    pr_err("ptn36502 driver prboe..0x21.\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("I2C not supported");
        return -ENODEV;
    }

    ptn_client = client;
	np = client->dev.of_node;
	if (np){

		redrive_ldo1v8_en = of_get_named_gpio(np, "redrive_ldo1v8_en", 0);

		if ((!gpio_is_valid(redrive_ldo1v8_en))) {
			pr_err("invalid redrive_ldo1v8_en %d from dt \n", redrive_ldo1v8_en);
			return -EINVAL;
		}else{
			rc = gpio_request(redrive_ldo1v8_en, "redrive_ldo1v8_en");
        	if (rc) {
            	pr_err("%s: redrive_ldo1v8_en request failed", __func__);
				return -EINVAL;
        	}

		}

		redriver_en = of_get_named_gpio(np, "redriver_en", 0);

		if ((!gpio_is_valid(redriver_en))) {
			pr_err("invalid redriver_en %d from dt \n", redriver_en);
			return -EINVAL;
		}else{
			rc = gpio_request(redriver_en, "redriver_en");
        	if (rc) {
            	pr_err("%s: redriver_en request failed", __func__);
				return -EINVAL;
        	}

		}
	}else
		pr_err("%s: of_node is null");

	pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("ptn36502_probe: Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	pin_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL(pin_default)) {
		pr_err("ptn36502_probe: Failed to look up default state\n");
		return PTR_ERR(pinctrl);
	}

	rc = pinctrl_select_state(pinctrl, pin_default);
	if (rc) {
		pr_err("ptn36502_probe: Can't select pinctrl state\n");
		return rc;
	}

	gpio_direction_output(redrive_ldo1v8_en,1);
	msleep(20);

	//zxz add , the en pin need hi-z status .  V01 had change to high no need sw gpio control .
	//gpio_direction_output(redriver_en,1); 


	ptn_write_reg(0x0b, 0x01);
	// ptn_write_reg(0x07, 0x29);
	// ptn_write_reg(0x08, 0x29);
	// ptn_write_reg(0x09, 0x29);
	// ptn_write_reg(0x0A, 0x29);
	//ptn_pinctrl_init(client);
	//ptn_state_switch(2, 1);

	// if (!ptn_pinctrl_init(client)) {
	// 	rc = pinctrl_select_state(pinctrl, pin_default);
	// 	if (rc) {
	// 		pr_err("%s: Can't select pinctrl state\n", __func__);
	// 	}
	// }

	#ifdef PTN_REGISTER_DEBUG
		rc = sysfs_create_group(&client->dev.kobj, &ptn_attr_group);
		if (rc) {
			dev_err(&client->dev, "failed to register sysfs. err: %d\n", rc);
		}
	#endif

    return 0;
}

static int ptn36502_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id ptn36502_id[] = {
    {PTN_DRIVER_NAME, 0},
    {},
};
static const struct of_device_id ptn36502_dt_match[] = {
    {.compatible = "ptn36502,ptn", },
    {},
};
MODULE_DEVICE_TABLE(of, ptn36502_dt_match);

static struct i2c_driver ptn36502_driver = {
    .probe = ptn36502_probe,
    .remove = ptn36502_remove,
    .driver = {
        .name = PTN_DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ptn36502_dt_match),
    },
    .id_table = ptn36502_id,
};

static int __init ptn36502_init(void)
{
    int ret = 0;
    pr_err("ptn36502_init...\n");
    ret = i2c_add_driver(&ptn36502_driver);
    if ( ret != 0 ) {
        pr_err("ptn36502 driver init failed!");
    }
    return ret;
}

static void __exit ptn36502_exit(void)
{
    i2c_del_driver(&ptn36502_driver);
}
module_init(ptn36502_init);
module_exit(ptn36502_exit);

MODULE_AUTHOR("Haojun.chen@tcl.com");
MODULE_DESCRIPTION("PTN36502 Driver");
MODULE_LICENSE("GPL v2");
