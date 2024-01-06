#ifndef __MANDATA_H_INCLUDED__
#define __MANDATA_H_INCLUDED__

// #include "device_types_legacy.h"
//#include <emkit/device_types.h>

#define MANDATA_RAM_SIZE          0x1000

//----------------------------------------
// Honeywell Manufacturing Data Structure
//----------------------------------------

// Manufacturing data special version numbers
#define MVER_VERSION 0x424C5430     // "BLT0" for board-level test
#define TRIM_VERSION 0x5452494D     // "TRIM" for real-time clock trim
#define MANU_VERSION 0x4D414E55     // "MANU" version >= this value
#define PANL_VERSION 0x50414E4C     // "PANL" for Touch Panel Defaults
#define CONT_VERSION 0x434F4E54     // "CONT" for Display Contrast Defaults
#define LCD0_VERSION 0x4C434430     // "LCD0" for Manufacturer Specific LCD Parameters
#define RMSK_VERSION 0x524D534B     // "RMSK" for Radio mask bits
#define BTH_VERSION  0x42544430     // "BTD0" for bluetooth configuration data
#define BDRV_VERSION 'BDRV'         // "BDRV" for board revision
#define LMBT_VERSION 'LMBT'         // "LMBT" for low batt voltage threshold setting
#define GSM_VERSION  'GSM0'         // "GSM0" for GSM configuration data
#define AIC0_VERSION 'AIC0'         // "AIC0" for Aimer configuration data
#define MACV_VERSION 'MACV'         // "MACV" for MAC Address
#define PSM1_VERSION "PSM1"         // "IPSM" for valid IPSM format parameters
#define USER_VERSION 'USER'         // "USER" for persistent user store size
#define MANDATA_VERSION_MASK 0x00FFFFFF

//----------------------------------------------------------------------------
//+ Copied this definitions from NX8 Android

// This structure contains the xldr/eboot/ipl/os image informations.
//
//
///
typedef struct {
	unsigned short Major;
	unsigned short Minor;
	unsigned short Distri;
	unsigned short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	unsigned short Second;
	unsigned short QfeYear;		// for WinCE
	unsigned short QfeMonth;	// for WinCE
	unsigned short AkuMajor;	// for WEH
	unsigned short AkuMinor;	// for WEH
	unsigned short AkuQfeLevel;	// for WEH
	unsigned short ImageType;	// 1=XLDR, 2=EBOOT, 3=IPL, 4=NK
	unsigned short Locale;		// 0x0409=WWE
} IMAGEINFO_T, *P_IMAGEINFO_T;
//----------------------------------------------------------------------------

// CSR module setup parameters
struct CSR_NVRAM // 128 bytes
{
    union
    {
        struct
        {
/* 0xAF8:   4 : BLUETOOTH_VALID_SIGNATURE */ u32 valid; // "BTD0", when bt struct is initialized
/* 0xAFC:   8 : BLUETOOTH_ADDRESS         */ u8  btaddress[8]; // Element 0x01
/* 0xB04:   1 : BLUETOOTH_COUNTRY_CODE    */ u8  btcountrycode; // Element 0x02
/* 0xB05: 243 : RESERVED                  */ u32 dwBTOn; // bluetooth states, used in drvglob not for mfgdata
                                             u32 UartD4; // UART states, used in drvglob not for mfgdata
        }; // anonymous struct

        char    pad[128];
    }; // anonymous union
};

// WWAN setup parameters
// 128 bytes
struct WWAN_DATA
{
	u8	Imei[8];		// IMEI
    char    meid[15];       // MEID
	u8	WwanRes[105];	// reserved
};

// TI 1273 WiFi module setup parameters
struct WLAN_DATA // 512 bytes
{
    union
    {
        struct
        {
            /// The EEPROM data can contain the actual MacAddress, but we'll store
            /// the MAC address separately to make is simple for manufacturing.
            /// The WLAN driver will combine the MAC address and EEPROM
            /// data below and download it to the module during power up.
            /// Could use checksum to safeguard the data??
/* 0xBF8:   6 : WLAN_ADDRESS         */ u8   MacAddress[6];
/* 0xBFE:   2 : WLAN_EEPROM_DATA_LEN */ u16 E2PLen;
/* 0xC00: 502 : WLAN_EEPROM_DATA     */ u8   E2PData[502];
/* 0xDF6:   2 : WLAN_MODULE_VER      */ u16 ModuleVer;
        }; // anonymous struct
    }; // anonymous union
};

// Manufacturing data is stored in the last 4096 bytes of the
// bootloader in flash.  The first 3.5 Kbytes of the structure are
// reserved for future use.  The last 512 bytes contain data
// that is initialized in final assembly


// Note: ManData_Area2 grows toward low memory.  In order to maintain
// backwared compatibility, new members must be added to the BEGINNING of
// this structure.  3.5 KBytes are reserved for ManData_Area2
struct ManData_Area2 // 784 bytes
{
// **** Add new members immediately below this comment line ****

/* 0xAF0:   8 : ODM_TRACKING_NUMBER         */ char ODMTrackingNo[8]; //Original Device Manufacturer tracking number

/* 0xAF8: 128 : BT                          */ struct CSR_NVRAM BTManufData;
/* 0xB78: 128 : WWAN	                    */ struct WWAN_DATA WWANManufData;
/* 0xBF8: 512 : WLAN                        */ struct WLAN_DATA WLANManufData;

/* 0xDF8 :   4 : BOARD_LEVEL_TEST_SIGNATURE */ u32 mver;  // Set to "BLT0" when this structure is inited.
/* 0xDFC :   4 : MODEL_NUMBER               */ u32 model; // Set to decimal 7850 or 9600
    // Do NOT add new members here
};

struct lcdDisplayParameters // 16 bytes
{
/* 0xE50 :   4 : LCD_SIGNATURE      */ u32 version; // "LCD0"
/* 0xE54 :   1 : LCD_MODEL          */ u8  displayModel; // LCD_DISPLAY types (defined in hhpioctl.h)
/* 0xE55 :   1 : EOL_PIXEL_CLK_WAIT */ u8  eolPixelClkWait; // end of line pixel clock wait: 0x00-0xFF
/* 0xE56 :   1 : PIXEL_CLK_DIVISOR  */ u8  pixelClkDiv; // Pixel Clock Divisor: 0x00-0xFF
/* 0xE57 :   1 : AC_BIAS_FREQUENCY  */ u8  acBiasFreq; // A/C Bias Pin Frequency
/* 0xE58 :   8 : RESERVED           */ u8  extra[8]; // Extra entries
};

#define MANU_DATA_SIZE              MANDATA_RAM_SIZE
#define MANDATA_FA_DATE_LENGTH      8
#define MANDATA_FA_SPECPART_LENGTH  16
struct HoneywellManData // 4096 bytes
{
    struct // anonymous struct containing board level test data.
    {
        char   pad_blt[(MANU_DATA_SIZE-512)-sizeof(struct ManData_Area2)]; // 2800 bytes (= 4096 - 512 - 784)
        struct ManData_Area2 blt; // This struct was formerly the "board level test" structure, but
                                  // is no longer filled in at board level test.  The "blt" name is
                                  // being maintained to minimize code porting requirements.
    };

    // Note all character strings are left-justified and NUL-terminated unless otherwise noted.
    union
    {
        // Note: This anonymous structure grows toward high memory.  In order to maintain
        // backwared compatibility, new members must be added to the END of
        // this structure.  Do not change the order or size of existing members.  Do
        // not use conditional compiler statements in the structure definition.
        struct
        {
            // Do NOT add new members here

            // final assembly initialized data.
            struct
            {
/* 0xE00 :   8 : FINAL_ASSEMBLY_DATE                   */ char  date[MANDATA_FA_DATE_LENGTH]; // "yyyymmdd" (no NUL termination character)
/* 0xE08 :  12 : UNUSED-LEGACY_PRODUCT_SERIAL          */ char  _NOT_USED_serialNo[12]; // Device serial number
/* 0xE14 :  16 : UNUSED-LEGACY_PART_NUMBER             */ char  _NOT_USED_partNo[16]; // Hand Held Products part number.

/* 0xE24 :   4 : UNUSED-LEGACY_MANUFACTURING_SIGNATURE */ u32 ver; // When set to "MANU", when specPartNo is initialized
/* 0xE28 :  16 : UNUSED-LEGACY_CUSTOM_PART_NUMBER      */ char  specPartNo[MANDATA_FA_SPECPART_LENGTH]; // Customer specified part number.
/* 0xE38 :   8 : UNUSED                                */ u8  filler[8];
            } fa;

/* 0xE40 :   4 : LICENSE_KEY                           */ u32 dwLicenseKey; // Code that indicates what software is authorized on this device
/* 0xE44 :   4 : RESERVED                              */ u32 dwReserved1;

            // final calibration
            struct
            {
/* 0xE48 :   4 : CONTRAST_SIGNATURE                    */ u32 validLcdContrast; // "CONT" - Magic value indicated valid data
/* 0xE4C :   4 : CONTRAST_CALIBRATION                  */ u32 contrast; // LCD Contrast (mono panel), or VCOM (color panel)
            } fc;

            // LCD info required by Sys Info app
/* 0xE50 :  16 : LCD                                   */ struct lcdDisplayParameters lcdDisplay;

/* 0xE60 :   4 : UNUSED on Dolphin Black-ValidIPSM     */ u32 validIPSM; // "PSM1" - Magic value indicating valid
/* 0xE64 :   4 : UNUSED on Dolphin Black-dwIPSMOffset  */ u32 dwIPSMOffset; // IPSM offset from beginning of flash chip(s)

            struct
            {
/* 0xE68 :   4 : EX_MANUFACTURING_SIGNATURE            */ u32 valid; // "MANU", when fa_ex struct is initialized
/* 0xE6C :  10 : EX_SERIAL_NUMBER                      */ char  serialNoEx[10];
/* 0xE76 :   6 : RESERVED                              */ char  filler1[6];
/* 0xE7C :  18 : EX_PART_NUMBER                        */ char  partNoEx[18];
/* 0xE8E :   6 : RESERVED                              */ char  filler2[6];
/* 0xE94 :   4 : EX_CONFIGURATION_SIGNATURE            */ u32 valid_custSpecSN; // "MANU", when custSpecificSN contains valid data
/* 0xE98 :  20 : EX_CONFIGURATION_NUMBER               */ char  custSpecificSN[20];
/* 0xEAC :   4 : RESERVED                              */ char  filler3[4];
            } fa_ex;                        // Extended final assembly initialized data.

            // Real-time clock calibration values
            struct
            {
/* 0xEB0 :   4 : REAL_TIME_CLOCK_TRIM_SIGNATURE        */ u32 validRTTR; // "TRIM" - Magic value indicates valid trim data
#define  RTC_RTTR_VALID   0x5452494D
#if 0
//               unsigned short clock_divider;
//               unsigned short trim_delete;
//               unsigned short atr_dtr_reg;
//               unsigned int rttr;    // real-time clock trim register value.
//               u8 filler[2];
#else
/* 0xEB4 :  12 : RESERVED                              */ u8 filler[12];
#endif
            } rtc;

            // Custom Serial number
            struct
            {
/* 0xEC0 :   4 : ASSET_SIGNATURE                       */ u32 valid; // "MANU", when CustSerialNo contains valid data
/* 0xEC4 :  10 : ASSET_NUMBER                          */ char  CustSerialNo[10];
/* 0xECE :   2 : RESERVED                              */ char  filler[2];
            } Cust;

/* 0xED0 : 300 : RESERVED                              */
/* 0xFFC :   4 : MFG_CRC32                             */

// **** Add new members immediately above this comment line ****
        }; // anonymous struct

        char pad[512];
    }; // anonymous union
};

//------------------------------------------
// PointMobile Manufacturing Data Structure
//------------------------------------------

struct PointMobileManData1 {
/* offset: size: field_name                            */
/* 0x000 :   4 : MANUFACTURING_DATA_IDENTIFIER         */ u32 manufacturing_data_identifier; // 0x00000000
/* 0x004 :  32 : XLDR_IMAGE_INFO                       */ IMAGEINFO_T xldr_image_info;
/* 0x024 :  32 : EBOOT_IMAGE_INFO                      */ IMAGEINFO_T eboot_image_info; // uboot.bin for android
/* 0x044 :  32 : IPL_IMAGE_INFO                        */ IMAGEINFO_T ipl_image_info;   // recovery.img for android
/* 0x064 :  32 : OS_IMAGE_INFO                         */ IMAGEINFO_T os_image_info;    // boot.img for android
/* 0x084 :  16 : DEVICE_ID                             */ u8 device_id[16];
/* 0x094 :  32 : SCANNER_SN                            */ u8 scanner_sn[32];
/* 0x0B4 :  32 : SCANNER_FOCUS                         */ u8 scanner_focus_reserved[32];	// scanner_focus[32] deprecated
/* 0x0D4 :  32 : IMAGER_INFO                           */ u8 imager_info_reserved[32];		// imager_info[32] deprecated
/* 0x0F4 :  32 : MAGIC_NUMBER                          */ u8 magic_number[32];
/* 0x114 :   4 : EMMC_BULK_DISK__START_SECTOR          */ u32 BulkDiskStart;
/* 0x118 :   4 : WEH_COA_SIGNATURE                     */ u32 weh_coa_signature;
/* 0x11C :  32 : ANDROID_IMAGE_INFO                    */ IMAGEINFO_T android_image_info;
/* 0x13C */
} __attribute__ ((packed));

struct PointMobileManData2 {
/* offset: size: field_name                            */
/* 0xAF0 :   8 : ODM_TRACKING_NUMBER                   */ char odm_tracking_number[8];

/* 0xAF8 :   4 : BLUETOOTH_VALID_SIGNATURE             */ u32 bluetooth_valid_signature; // "BTD0"
/* 0xAFC :   8 : BLUETOOTH_ADDRESS                     */ u8  bluetooth_address[8];
/* 0xB04 :   1 : BLUETOOTH_COUNTRY_CODE                */ u8  bluetooth_country_code;
/* 0xB05 : 243 : RESERVED                              */ u8  _reserved_0[243];

/* 0xBF8 :   6 : WLAN_ADDRESS                          */ u8   wlan_address[6];
/* 0xBFE :   2 : WLAN_EEPROM_DATA_LEN                  */ u16 wlan_eeprom_data_len;
/* 0xC00 : 502 : WLAN_EEPROM_DATA                      */ u8   wlan_eeprom_data[502];
/* 0xDF6 :   2 : WLAN_MODULE_VER                       */ u16 wlan_module_ver;

/* 0xDF8 :   4 : BOARD_LEVEL_TEST_SIGNATURE            */ u32 board_level_test_signature; // "BLT0"
/* 0xDFC :   4 : MODEL_NUMBER                          */ u32 model_number;

/* 0xE00 :   8 : FINAL_ASSEMBLY_DATE                   */ char  final_assembly_date[MANDATA_FA_DATE_LENGTH]; // "yyyymmdd" (no NUL termination character)
/* 0xE08 :  12 : UNUSED-LEGACY_PRODUCT_SERIAL          */ char  legacy_product_serial[12];
/* 0xE14 :  16 : UNUSED-LEGACY_PART_NUMBER             */ char  legacy_part_number[16];

/* 0xE24 :   4 : UNUSED-LEGACY_MANUFACTURING_SIGNATURE */ u32 legacy_manufacturing_signature; // "MANU"
/* 0xE28 :  16 : UNUSED-LEGACY_CUSTOM_PART_NUMBER      */ char  legacy_custom_part_number[MANDATA_FA_SPECPART_LENGTH];
/* 0xE38 :   8 : RESERVED                              */ u8  _reserved_1[8];

/* 0xE40 :   4 : LICENSE_KEY                           */ u32 license_key;
/* 0xE44 :   4 : RESERVED                              */ char  _reserved_2[4];

/* 0xE48 :   4 : CONTRAST_SIGNATURE                    */ u32 contrast_signature; // "CONT"
/* 0xE4C :   4 : CONTRAST_CALIBRATION                  */ u32 contrast_calibration;

/* 0xE50 :   4 : LCD_SIGNATURE                         */ u32 lcd_signature; // "LCD0"
/* 0xE54 :   1 : LCD_MODEL                             */ u8  lcd_model;
/* 0xE55 :   1 : EOL_PIXEL_CLK_WAIT                    */ u8  eol_pixel_clk_wait;
/* 0xE56 :   1 : PIXEL_CLK_DIVISOR                     */ u8  pixel_clk_divisor;
/* 0xE57 :   1 : AC_BIAS_FREQUENCY                     */ u8  ac_bias_frequency;
/* 0xE58 :   8 : RESERVED                              */ u8  _reserved_3[8];

/* 0xE60 :   4 : IPSM_SIGNATURE                        */ u32 ipsm_signature; // "PSM1"
/* 0xE64 :   4 : IPSM_OFFSET                           */ u32 ipsm_offset;

/* 0xE68 :   4 : EX_MANUFACTURING_SIGNATURE            */ u32 ex_manufacturing_signature; // "MANU"
/* 0xE6C :  10 : EX_SERIAL_NUMBER                      */ char  ex_serial_number[10];
/* 0xE76 :   6 : RESERVED                              */ char  _reserved_4[6];
/* 0xE7C :  18 : EX_PART_NUMBER                        */ char  ex_part_number[18];
/* 0xE8E :   6 : RESERVED                              */ char  _reserved_5[6];
/* 0xE94 :   4 : EX_CONFIGURATION_SIGNATURE            */ u32 ex_configuration_signature; // "MANU"
/* 0xE98 :  20 : EX_CONFIGURATION_NUMBER               */ char  ex_configuration_number[20];
/* 0xEAC :   4 : RESERVED                              */ char  _reserved_6[4];

#define  RTC_RTTR_VALID   0x5452494D
/* 0xEB0 :   4 : REAL_TIME_CLOCK_TRIM_SIGNATURE        */ u32 real_time_clock_trim_signature; // "TRIM"
/* 0xEB4 :  12 : RESERVED - deprecated                 */ //u8  _reserved_7[12];
/* 0xEB4 :   6 : RESERVED                              */ u8 filler0[6];
/* 0xEBA :   4 : REAL_TIME_CLOCK_TRIM_REGISTER_VALUE   */ u8 real_time_clock_trim_register[4];    // real-time clock trim register value.
/* 0xEBE :   2 : RESERVED                              */ u8 filler1[2];

/* 0xEC0 :   4 : ASSET_SIGNATURE                       */ u32 asset_signature; // "MANU"
/* 0xEC4 :  10 : ASSET_NUMBER                          */ char  asset_number[10];
/* 0xECE :   2 : RESERVED                              */ char  _reserved_8[2];

/* 0xED0 :  64 : CUSTOMER_MFG_DATA                     */ char customer_mfg_data[64];
/* 0xF10 : 236 : RESERVED                              */ char  _reserved_9[236];
/* 0xFFC :   4 : MFG_CRC32                             */ u32 mfg_crc32;
} __attribute__ ((packed));

//------------------------------------------
// Unified Manufacturing Data Structure
//------------------------------------------

typedef struct ManDataStruct MANDATA_T, *P_MANDATA_T;
struct ManDataStruct { // 4KB
	union {
		// Honeywell Manufacturing Data
		struct HoneywellManData hmd;

		// PointMobile Manufacturing Data
		struct {
			struct PointMobileManData1 pmd1;
			char pad [MANDATA_RAM_SIZE - sizeof(struct PointMobileManData1) - sizeof(struct PointMobileManData2)];
			struct PointMobileManData2 pmd2;
		}; // anonymous struct

	}; // anonymous union
};

#endif // __MANDATA_H_INCLUDED__
