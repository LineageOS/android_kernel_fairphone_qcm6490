/**
  ******************************************************************************
  * File Name          : DW9784_SET_API.H
  * Description        : Main program c file
  * DongWoon Anatech   :
  * Version            : 1.0
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 DWANATECH
  * DW9784 Setup Program for SET
  * Revision History
  * 2022.06.14 by SJ cho
  ******************************************************************************
**/
//add by jinghuang
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <asm/arch_timer.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_ois_dev.h"
#include "cam_cci_dev.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

int debug_printf(char *buf, ...);
void os_mdelay(int count);
void GenerateFirmwareContexts(void);
int dw9784_download_open_camera(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_whoami_chk(struct cam_ois_ctrl_t *o_ctrl);
unsigned int dw9784_chip_id_chk(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_checksum_fw_chk(struct cam_ois_ctrl_t *o_ctrl);
unsigned int dw9784_fw_ver_chk(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_fw_type(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_fw_info(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_erase_mtp_rewritefw(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_download_fw(struct cam_ois_ctrl_t *o_ctrl,int module_state);
void dw9784_flash_acess(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_code_pt_off(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_pid_erase(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_fw_eflash_erase(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_shutdown_mode(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_ois_reset(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_ois_on(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_servo_on(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_servo_off(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_checksum_all_chk(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_checksum_fw_chk(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_set_gyro_select(struct cam_ois_ctrl_t *o_ctrl,unsigned int new_gyro_id);
int dw9784_gyro_direction_setting(struct cam_ois_ctrl_t *o_ctrl,int gyro_arrangement, int gyro_degree);
int dw9784_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_set_cal_store(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_wait_check_register(struct cam_ois_ctrl_t *o_ctrl,unsigned int reg, unsigned int ref);
void dw9784_fw_read(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_flash_if_ram_read(struct cam_ois_ctrl_t *o_ctrl);
void dw9784_flash_ldt_register_read(struct cam_ois_ctrl_t *o_ctrl);
int dw9784_module_cal_store(struct cam_ois_ctrl_t *o_ctrl);

typedef struct
{
	unsigned int driverIc;
	unsigned int *fwContentPtr;
	unsigned int version;
}FirmwareContex;

#define LOOP_A							200
#define	LOOP_B							LOOP_A-1
#define WAIT_TIME						100

/* fw_update_item */
#define UPDATE_ITEM_LENGTH 3

/* fw downlaod */
#define DW9784_CHIP_ID_ADDRESS			0x7000
#define DW9784_CHIP_ID					0x9784
#define DW9784_FW_VER_ADDR				0x7001
#define DW9784_FW_DATE_ADDR				0x7002
#define DW9784_FW_PRJ_ADDR				0x7003
#define DW9784_FW_TYPE_ADDR				0x7006

#define MODULE_FW						0
#define SET_FW							1
#define EIS_VSYNC						0
#define EIS_QTIME						1

#define FUNC_PASS						0
#define FUNC_FAIL						-1

#define EOK 							0
#define ERROR_SECOND_ID 				1
#define ERROR_WHOAMI					2
#define ERROR_FW_VALID					3
#define ERROR_FW_VERIFY 				4
#define ERROR_FW_CHECKSUM				5
#define ERROR_FW_DOWN_FMC				6
#define ERROR_FW_DWON_FAIL				7
#define ERROR_ALL_CHECKUM				8
#define ERROR_WHO_AMI					9

#define MAX_FIRMWARE_NUMBER 			5
#define DATPKT_SIZE 					256

#define DW9784_CHIP_ID					0x9784
#define AUTORD_RV_OK					0x0000

#define MCS_START_ADDRESS				0x8000
#define IF_START_ADDRESS				0x8000
#define MCS_SIZE_W						10240	//20KB
#define PID_SIZE_W						256		//0.5KB

/* gyro offset calibration */
#define GYRO_OFS_CAL_DONE_FAIL			0xFF
#define X_AXIS_GYRO_OFS_PASS			0x1
#define X_AXIS_GYRO_OFS_FAIL			0x1
#define Y_AXIS_GYRO_OFS_PASS			0x2
#define Y_AXIS_GYRO_OFS_FAIL			0x2
#define X_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x10
#define Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT	0x20
#define XY_AXIS_CHECK_GYRO_RAW_DATA		0x800

#define GYRO_FRONT_LAYOUT				0
#define GYRO_BACK_LAYOUT				1

#define GYRO_DEGREE_0					0
#define GYRO_DEGREE_90					90
#define GYRO_DEGREE_180					180
#define	GYRO_DEGREE_270					270

/* gyro type */
#define ST_LSM6DSOQ				0x04
#define INVEN_ICM42631			0x05
#define BOSCH_BMI260_GYRO		0x06

/* project info */
#define PJT_SOLI				0x01
#define PJT_ATHENA				0x02
#define PJT_BOSU				0X03

/* ref_stroke[um/deg] */
#define REF_STROKE_SOLI			151		/* efl = 8.67mm */
#define REF_STROKE_ATHENA		132		/* efl = 7.61mm */

/* ref_gyro_result [code/deg] */
#define REF_GYRO_RESULT			1000
#define OIS_ERROR			(-1)
#define OIS_SUCCESS			(0)
enum regrw_flag {
    REGRW_WRITE = 0,
    REGRW_READ = 1,
};

