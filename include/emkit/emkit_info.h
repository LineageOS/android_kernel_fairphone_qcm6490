//////////////////////////////////////////////////////////////////
// Please only code related to module detection is put here.    //
// Other than that, don't add it here, put it in emkit_ctrl.h . //
//////////////////////////////////////////////////////////////////

#ifndef __EMKIT_INFO_
#define __EMKIT_INFO_

#include <emkit/emkit.h>

#ifndef EMLOG
#define EMLOG(fmt, args...) EMLOG_T("EMKIT-INFO", fmt, ##args)
#endif

///* Constants *///

enum {
	HW_REV_0 = 0,
	HW_REV_1,
	HW_REV_2,
	HW_REV_3,
	HW_REV_4,
	HW_REV_5,
	HW_REV_6,
	HW_REV_7,
	HW_REV_8,
	HW_REV_9,
	HW_REV_10,
	HW_REV_11,

	HW_REV_EVT = HW_REV_0,
	HW_REV_DVT = HW_REV_1,
	HW_REV_MVT = HW_REV_2,
	HW_REV_MAX = HW_REV_11,
};
#define HW_REV_PRE_EVT HW_REV_EVT
#define HW_REV_DEF     HW_REV_EVT
inline static char const *GetRevisionName(int index)
{
	switch (index) {
		case HW_REV_EVT: return "EVT";
		case HW_REV_DVT: return "DVT";
		case HW_REV_MVT: return "MVT";
		default: return DEF_UNKNOWN;
	}
}

enum {
    MODULE_CPU = 0,
    MODULE_KEYBOARD,
    MODULE_TOUCH,
    MODULE_DISPLAY,
    MODULE_BLUETOOTH,
    MODULE_WLAN,
    MODULE_PHONE,
    MODULE_GPS,
    MODULE_MEMORY,
	MODULE_MEMORY_VENDOR,
    MODULE_CAMERA_B,
    MODULE_SCANNER,
    MODULE_NFC,
    MODULE_ACCELERATION,
    MODULE_LIGHT,
    MODULE_TEMPERATURE,
    MODULE_GYROSCOPE,
    MODULE_MAGNETIC,
    MODULE_PRESSURE,
    MODULE_PROXIMITY,
    MODULE_SAM,
    MODULE_UHF,
    MODULE_CAMERA_F,
    MODULE_MSR,
    MODULE_PRINTER,
    MODULE_FISCAL,
    MODULE_FINGERPRINT,
    MODULE_ICCR,
    MODULE_BEACON,
    MODULE_BATTERY_ID,
    MODULE_MAX
};
#define MODULE_COUNT MODULE_MAX
inline static char const *GetModuleTypeName(int kind)
{
    switch (kind) {
        case MODULE_CPU:            return "cpu";
        case MODULE_KEYBOARD:       return "keyboard";
        case MODULE_TOUCH:          return "touch";
        case MODULE_DISPLAY:        return "display";
        case MODULE_BLUETOOTH:      return "bluetooth";
        case MODULE_WLAN:           return "wlan";
        case MODULE_PHONE:          return "phone";
        case MODULE_GPS:            return "gps";
        //case MODULE_MEMORY:         return "memory";
        case MODULE_CAMERA_B:       return "camera_b";
        case MODULE_SCANNER:        return "scanner";
        case MODULE_NFC:            return "nfc";
        case MODULE_ACCELERATION:   return "acceleration";
        case MODULE_LIGHT:          return "light";
        case MODULE_TEMPERATURE:    return "temperature";
        case MODULE_GYROSCOPE:      return "gyroscope";
        case MODULE_MAGNETIC:       return "magnetic";
        case MODULE_PRESSURE:       return "pressure";
        case MODULE_PROXIMITY:      return "proximity";
        case MODULE_SAM:            return "sam";
        case MODULE_UHF:            return "uhf";
        case MODULE_CAMERA_F:       return "camera_f";
        case MODULE_MSR:            return "msr";
        case MODULE_PRINTER:        return "printer";
        case MODULE_FISCAL:         return "fiscal";
        case MODULE_FINGERPRINT:    return "fingerprint";
        case MODULE_ICCR:           return "iccr";
        case MODULE_BEACON:            return "beacon";
        default: return DEF_UNKNOWN;
    }
}

struct emkit_info_data {

    // sys/emkit/info
    struct kobject *kobj;

    // GPIO data
	unsigned gpio_board_id0;
	unsigned gpio_board_id1;
	unsigned gpio_board_id2;
	unsigned gpio_board_id3;
	unsigned gpio_lcd_id0;
	unsigned gpio_lcd_id1;
	unsigned gpio_phone_id;
	unsigned gpio_key_id;

    //GUN
	int gpio_pogo_5v_en1;
	int gpio_pogo_5v_en2;

    // Detect data
    unsigned board_revision; // For the stage of HW such as such as EVT0, EVT1... DVT...MVT, etc on.
    unsigned lcd_id;
    char board_model_name[MAX_LEN];
    char board_title_name[MAX_LEN];
    char board_module_name[MODULE_COUNT][MAX_LEN];
};

///* Functions *///

void SetRevision(int);
int GetRevision(void);
void SetModuleName(int, const char *, const char *);
int GetPhoneId(void);

#endif
