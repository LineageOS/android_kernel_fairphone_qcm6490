#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>

#define show_function(name) \
    static ssize_t show_##name(struct device *dev,\
            struct device_attribute *att,\
            char *buf)\
{\
    return snprintf(buf, PAGE_SIZE, "%s\n", name);\
}\

static struct class *deviceinfo_class = NULL;
static struct device *deviceinfo_dev = NULL;

unsigned char CamNameB[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamNameB);
show_function(CamNameB);
DEVICE_ATTR(CamNameB, 0444, show_CamNameB, NULL);

unsigned char CamNameF[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamNameF);
show_function(CamNameF);
DEVICE_ATTR(CamNameF, 0444, show_CamNameF, NULL);

unsigned char CamNameB2[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamNameB2);
show_function(CamNameB2);
DEVICE_ATTR(CamNameB2, 0444, show_CamNameB2, NULL);

unsigned char CamOTPB[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamOTPB);
show_function(CamOTPB);

unsigned char CamOTPF[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamOTPF);
show_function(CamOTPF);

unsigned char CamOTPB2[128] = "NA:NA:NA";
EXPORT_SYMBOL(CamOTPB2);
show_function(CamOTPB2);

/*[bugfix]-add-begin,by jinghuang@tcl.com,7217863 on 2018.12.11*/
/*add function:check otp format*/
static ssize_t CamOTPB_store(struct device *dev,struct  device_attribute *attr,const char *buf,size_t count)
{
    int camerab_otp_flag;
    sscanf(buf,"%d",&camerab_otp_flag);
    if(camerab_otp_flag==1){
        printk("otp flag=1\n");
        sprintf(CamOTPB,"%s","1");
    }else{
        printk("otp flag = 0\n");
        sprintf(CamOTPB,"%s%","0");
    }

    return count;
}

static ssize_t CamOTPF_store(struct device *dev,struct  device_attribute *attr,const char *buf,size_t count)
{
    int camerab_otp_flag;
    sscanf(buf,"%d",&camerab_otp_flag);
    if(camerab_otp_flag==1){
        printk("otp flag=1\n");
        sprintf(CamOTPF,"%s","1");
    }else{
        printk("otp flag = 0\n");
        sprintf(CamOTPF,"%s","0");
    }

    return count;
}

static ssize_t CamOTPB2_store(struct device *dev,struct  device_attribute *attr,const char *buf,size_t count)
{
        int camerab_otp_flag;
        sscanf(buf,"%d",&camerab_otp_flag);

        if(camerab_otp_flag==1){
                printk("otp flag=1\n");
                sprintf(CamOTPB2,"%s","1");
        }else{
            printk("otp flag = 0\n");
            sprintf(CamOTPB2,"%s","0");
        }

        return count;
}

DEVICE_ATTR(CamOTPB, 0664, show_CamOTPB, CamOTPB_store);
DEVICE_ATTR(CamOTPF, 0664, show_CamOTPF, CamOTPF_store);
DEVICE_ATTR(CamOTPB2, 0664, show_CamOTPB2, CamOTPB2_store);
/*[bugfix]-add-end,by jinghuang@tcl.com,7217863 on 2018.12.11*/


static struct device_attribute *attr[] = {
    &dev_attr_CamNameB,
    &dev_attr_CamNameF,
    &dev_attr_CamNameB2,
    &dev_attr_CamOTPB,
    &dev_attr_CamOTPF,
    &dev_attr_CamOTPB2,
    NULL,
};

static int create_deviceinfo_nodes(void)
{
    int ret, i = 0;

    while (attr[i] != NULL) {
        ret = device_create_file(deviceinfo_dev,  attr[i]);
        if (ret < 0) {
            pr_err("Failed to create file for node:%s\n", attr[i]->attr.name);
            i = i - 1;
            while (i >= 0) {
                device_remove_file(deviceinfo_dev, attr[i]);
                i--;
            }
            return -EFAULT;
        }
        i++;
    }

    return 0;
}

static void remove_deviceinfo_nodes(void)
{
    int i = 0;

    while (attr[i] != NULL) {
        device_remove_file(deviceinfo_dev, attr[i]);
        i++;
    }
}

static int __init deviceinfo_init(void)
{
    int ret = -1;

    deviceinfo_class = class_create(THIS_MODULE, "deviceinfo");
    if (IS_ERR(deviceinfo_class)) {
        pr_err("Failed to create device info class!\n");
        return -ENOMEM;
    }

    deviceinfo_dev = device_create(deviceinfo_class, NULL, 0, NULL,
                                   "device_info");
    if (IS_ERR(deviceinfo_dev)) {
        pr_err("Failed to create device info device!\n");
        return -ENOMEM;
    }

    ret = create_deviceinfo_nodes();
    if (ret) {
        pr_err("Failed to create all device info nodes!\n");
        return -EFAULT;
    }

    return ret;
}

static void __exit deviceinfo_exit(void)
{
    remove_deviceinfo_nodes();
    device_destroy(deviceinfo_class, 0);
    class_destroy(deviceinfo_class);

    return;
}

module_init(deviceinfo_init);
module_exit(deviceinfo_exit);


MODULE_AUTHOR("falin.luo@tcl.com");
MODULE_DESCRIPTION("Create device information node!");
MODULE_LICENSE("GPL v2");

