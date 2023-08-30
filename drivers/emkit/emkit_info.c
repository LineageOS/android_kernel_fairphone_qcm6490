//////////////////////////////////////////////////////////////////
// Please only code related to module detection is put here.    //
// Other than that, don't add it here, put it in emkit_ctrl.o . //
//////////////////////////////////////////////////////////////////

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
#include <linux/uaccess.h>
#include <soc/qcom/socinfo.h>
//#include <linux/sysfs.h>


#include <emkit/emkit_info.h>

struct kobject *g_emkit_kobj;
EXPORT_SYMBOL_GPL(g_emkit_kobj);



struct emkit_info_data g_emkit_info = {
    .kobj = NULL,

    .gpio_phone_id = -1,
	.gpio_key_id = -1,

    .board_revision = HW_REV_DEF,
    .board_model_name = {0,},
    .board_title_name = {0,},
    .board_module_name = {{0,},},
};



static int __init get_android_boot_hardware_revision(char *str)
{
	get_option(&str, &g_emkit_info.board_revision);
	EMLOG("DETECT: REVISION = %d", g_emkit_info.board_revision);
	return 1;
}
__setup("androidboot.hardware.revision=", get_android_boot_hardware_revision);

void SetRevision(int hw_rev)
{
    g_emkit_info.board_revision = hw_rev;
	EMLOG("DETECT: REVISION = %d", g_emkit_info.board_revision);
}
EXPORT_SYMBOL(SetRevision);

int GetRevision(void)
{
	pr_info("board_revision:%d\n", g_emkit_info.board_revision);
	return g_emkit_info.board_revision;
}
EXPORT_SYMBOL(GetRevision);

void SetModuleName(int kind, const char *name, const char *caller)
{
    sprintf(g_emkit_info.board_module_name[kind], "%s", name);
    EMLOG("DETECT: %s (%s) = %s", GetModuleTypeName(kind), caller, g_emkit_info.board_module_name[kind]);
}
EXPORT_SYMBOL(SetModuleName);

int GetPhoneId(void)
{
	return GetGpioValue(g_emkit_info.gpio_phone_id, "gpio_phone_id");
}
EXPORT_SYMBOL(GetPhoneId);



static void board_check_cpu(void)
{
	
    if (!strcmp(g_emkit_info.board_module_name[MODULE_CPU], "")) {
        switch (socinfo_get_id()) {
            case 206:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "MSM8916");
                break;
            case 245:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "MSM8909");
                break;
            case 247:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "APQ8016");
                break;
            case 265:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "APQ8009");
                break;
            case 317:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "SDM660");
                break;
            case 324:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "SDA660");
                break;
            case 417:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "SM4250");
                break;
            case 444:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "SM6115");
                break;
            case 469:
                sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "QCM4290");
                break;
		    // start [PM560] sijoo(2022.07.15) : Add PM560 module type
		    case 470:
				sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "QCS4290");
	            break;
			case 497:
				sprintf(g_emkit_info.board_module_name[MODULE_CPU], "%s", "QCM6490");
	            break;
        }
    }
	
    EMLOG("DETECT: CPU ID = %d", socinfo_get_id());
    EMLOG("DETECT: CPU = %s", g_emkit_info.board_module_name[MODULE_CPU]);
}


/*
static ssize_t features_read(struct file *file, char __user *ubuf, size_t cnt, loff_t *ppos)
{
    char features[MAX_LEN] = {0,};
    int i = 0, pos = 0;
    for (; i < MODULE_COUNT; i++) {
        EMLOG("%s = [ %s ]", GetModuleTypeName(i), g_emkit_info.board_module_name[i]);
        if (strlen(g_emkit_info.board_module_name[i]) > 0) {
            if (features[0] != 0) {
                strcat(features, ",");
            }
            strcat(features, GetModuleTypeName(i));
        }
    }
    strcat(features, ";");
    pos = strlen(features);
    EMLOG("###ZEUS### features_read : %s (%d)", features, pos);
	return simple_read_from_buffer(ubuf, cnt, ppos, features, pos);
}

static const struct file_operations features_fops = {
	.open = simple_open,
	.read = features_read,
	.llseek = generic_file_llseek,
};
*/






#ifndef INFO_FUNC
#define INFO_FUNC(variable, name) \
static ssize_t name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
{ \
	return sprintf(buf, "%s \n", variable); \
} \
static ssize_t name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) __attribute__((unused)); \
static ssize_t name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) \
{ \
    if (1 > count || count > MAX_LEN) return -EFAULT; \
    memset(variable, 0, MAX_LEN); \
    return sprintf(variable, "%s", buf); \
}
#endif


INFO_FUNC(g_emkit_info.board_module_name[MODULE_CPU], emkit_cpu);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_MEMORY], emkit_memory);

INFO_FUNC(g_emkit_info.board_module_name[MODULE_ACCELERATION], acceleration);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_GYROSCOPE], gyroscope);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_LIGHT], light);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_MAGNETIC], magnetic);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_PROXIMITY], proximity);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_PRESSURE], pressure);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_MEMORY_VENDOR], emkit_memory_vendor);

/*Add by T2M-mingwu.zhang for FP5-538 remarks: TP/LCD Device Information Development.[Begin]*/	
INFO_FUNC(g_emkit_info.board_module_name[MODULE_DISPLAY], display);
INFO_FUNC(g_emkit_info.board_module_name[MODULE_TOUCH], touch);
/*Add by T2M-mingwu.zhang [End]*/

/*Add by T2M-xianzhu.zhang for FP5-569 : get battery id value. [Begin]*/
INFO_FUNC(g_emkit_info.board_module_name[MODULE_BATTERY_ID], battery_id);
/*Add by T2M-xianzhu.zhang [End]*/

static struct kobj_attribute emkit_attrs[] = {
    __ATTR(emkit_cpu, S_IRUGO|S_IWUSR|S_IWGRP, emkit_cpu_show, emkit_cpu_store),
    __ATTR(emkit_memory, S_IRUGO|S_IWUSR|S_IWGRP, emkit_memory_show, emkit_memory_store),
	__ATTR(emkit_memory_vendor, S_IRUGO|S_IWUSR|S_IWGRP, emkit_memory_vendor_show, emkit_memory_vendor_store),
	
    __ATTR(acceleration, S_IRUGO|S_IWUSR|S_IWGRP, acceleration_show, acceleration_store),
    __ATTR(gyroscope, S_IRUGO|S_IWUSR|S_IWGRP, gyroscope_show, gyroscope_store),
    __ATTR(light, S_IRUGO|S_IWUSR|S_IWGRP, light_show, light_store),
    __ATTR(magnetic, S_IRUGO|S_IWUSR|S_IWGRP, magnetic_show, magnetic_store),
    __ATTR(proximity, S_IRUGO|S_IWUSR|S_IWGRP, proximity_show, proximity_store), 
    __ATTR(pressure, S_IRUGO|S_IWUSR|S_IWGRP, pressure_show, pressure_store), 

/*Add by T2M-mingwu.zhang for FP5-538 remarks: TP/LCD Device Information Development.[Begin]*/
    __ATTR(display, S_IRUGO|S_IWUSR|S_IWGRP, display_show, display_store),
    __ATTR(touch, S_IRUGO|S_IWUSR|S_IWGRP, touch_show, touch_store),
/*Add by T2M-mingwu.zhang [End]*/

/*Add by T2M-xianzhu.zhang for FP5-569 : get battery id value. [Begin]*/
    __ATTR(battery_id, S_IRUGO|S_IWUSR|S_IWGRP, battery_id_show, battery_id_store), 
/*Add by T2M-xianzhu.zhang [End]*/
};

#define EMKIT_ATTRS_NUM (sizeof(emkit_attrs) / sizeof(emkit_attrs[0]))



static int emkit_info_probe(struct platform_device * pdev)
{
	int ret=0, i =0;
	EMLOG("BEGIN");
	g_emkit_kobj = kobject_create_and_add("emkit", NULL);
	if (g_emkit_kobj == NULL) {
		EMLOG("kobject_create_and_add() failed!");
		return -ENOMEM;
	}


	g_emkit_info.kobj = kobject_create_and_add("info", g_emkit_kobj);
	if (g_emkit_info.kobj == NULL) {
		EMLOG("kobject_create_and_add() failed!");
		return -ENOMEM;
	}

	 for (i = 0; i < EMKIT_ATTRS_NUM; i++) {
        ret = sysfs_create_file(g_emkit_info.kobj, &emkit_attrs[i].attr);
        if (ret){
			
            EMLOG("sysfs_create_group() failed! (ret=%d)", ret);
			goto err_create_sysfs;
        }
    }

	
	board_check_cpu();
   

    EMLOG("DONE");
	return 0;

err_create_sysfs:
	kobject_put(g_emkit_info.kobj);
	return ret;
}

static int emkit_info_remove(struct platform_device *pdev)
{
	 int i;

    for (i = 0; i < EMKIT_ATTRS_NUM; i++)
        sysfs_remove_file(g_emkit_info.kobj, &emkit_attrs[i].attr);
	
	kobject_put(g_emkit_info.kobj);

	
	return 0;
}

static const struct dev_pm_ops emkit_info_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};


static struct of_device_id emkit_info_match_table[] = {
	{ .compatible = "emkit-info,sys-emkit-info",},
	{ },
};

static struct platform_driver emkit_info_driver = {
	.probe = emkit_info_probe,
	.remove = emkit_info_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "emkit-info",
		.of_match_table = emkit_info_match_table,
		.pm = &emkit_info_pm_ops,
	},
};

static int __init emkit_info_init(void)
{
	int err = platform_driver_register(&emkit_info_driver);
	if (err) {
		EMLOG("platform_driver_register() failed! (err=%d)", err);
		return err;
	}
	return 0;
}

static void __exit emkit_info_exit(void)
{
	platform_driver_unregister(&emkit_info_driver);
}
device_initcall(emkit_info_init);

module_exit(emkit_info_exit);
MODULE_LICENSE("GPL v2");
