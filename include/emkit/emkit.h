#ifndef __EMKIT_
#define __EMKIT_

#ifndef EMLOG_T
#define EMLOG_T(tag, fmt, args...) pr_err("%s: %s():%d: " fmt "\n", tag, __func__, __LINE__, ##args)
#endif

#ifndef MAX_LEN
#define MAX_LEN 256
#endif

#ifndef DEF_UNKNOWN
#define DEF_UNKNOWN "Unknown"
#endif

#include <linux/kobject.h>
extern struct kobject *g_emkit_kobj;

#include <linux/of_gpio.h>
inline static int GetGpioValue(unsigned gpio, const char *label)
{
	int err;
	int value;

	if (gpio < 0) {
		EMLOG_T("EMKIT", "%s is not defined, err", label);
		return 0;
	}

	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, label);
		if (err) {
			EMLOG_T("EMKIT", "request GPIO#%d failed, err=%d", gpio, err);
		}
	} else {
		EMLOG_T("EMKIT", "request GPIO#%d is not valid, err", gpio);
		return 0;
	}

	err = gpio_direction_input(gpio);
	if (err) {
		EMLOG_T("EMKIT", "Set GPIO#%d input fail, err=%d", gpio, err);
        return 0;
	}

	value = gpio_get_value_cansleep(gpio);
	EMLOG_T("EMKIT", "Get GPIO#%d value=%d", gpio, value);
	return value;
}

#endif
