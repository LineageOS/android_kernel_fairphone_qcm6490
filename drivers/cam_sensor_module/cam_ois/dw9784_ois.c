/**
  ******************************************************************************
  * File Name          : DW9784_SET_API.C
  * Description        : Main program c file
  * DongWoon Anatech   :
  * Version            : 1.0
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 DWANATECH
  * DW9784 Setup Program for SET
  * Revision History
  * 2022.06.14 by SJ cho
  * 2022.11.16 by SJ Cho 
  *            - Attempt to download firmware twice  
  ******************************************************************************
**/
#define DEBUG_API
#ifdef DEBUG_API
#include "dw9784_ois.h"
#include <linux/delay.h>
#include "DW9784_T2M_FW_V0201_D0116.h"

//#include "func.h"
#else
//#include "main.h"
#include <stdarg.h>
//#include <stdio.h>
//#include <string.h>
//#include "DW9784_OSOM_FW_V0101_D0602.h"
#include "DW9784_T2M_FW_V0201_D0116.h"

#include "dw9784_ois.h"

extern unsigned char DW9784_ID;
int debug_printf(char *buf, ...);
typedef int(*ptr_func)(const char*, ...);
ptr_func logi = debug_printf;
//void HAL_Delay(unsigned int Delay);
int debug_printf(char *buf, ...)
{
	int len = 0;
	char pBuf[512];
	va_list ap;
	va_start(ap, buf);
	len = vsprintf(pBuf, buf, ap);
	pBuf[len] = '\r';
	pBuf[len + 1] = '\n';
	pBuf[len + 2] = '\0';
	printf(pBuf, ap);
	va_end(ap);
	return 0;
}
#endif

struct cam_ois_ctrl_t *g_o_ctrl;
static struct class *ois_debug_class;

/*Global buffer for flash download*/
FirmwareContex g_firmwareContext;
unsigned int g_downloadByForce;
unsigned int g_updateFw;

void os_mdelay(int count)
{
	//HAL_Delay(count);
	mdelay(count);
}

static int write_reg_16bit_value_16bit(struct cam_ois_ctrl_t *o_ctrl,uint32_t addr, uint32_t data){
	int ret = OIS_ERROR;
//	int cnt;
	enum i2c_freq_mode temp_freq;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	int num_byte =1;


	if (o_ctrl == NULL ) {
		CAM_ERR(CAM_OIS,"Invalid Args o_ctrl: %pK", o_ctrl);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_ERR(CAM_OIS,"Not in right state to start soc writes: %d",
							o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
    /*Modify i2c freq to 1M*/
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;//I2C_FAST_PLUS_MODE;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
			kzalloc(sizeof(struct cam_sensor_i2c_reg_array) *
							num_byte, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL) {
		CAM_ERR(CAM_OIS,"kzalloc failed");
		return OIS_ERROR;
	}
	i2c_reg_setting.reg_setting[0].reg_addr = addr;
	i2c_reg_setting.reg_setting[0].reg_data = data;
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;

	ret = camera_io_dev_write(&(o_ctrl->io_master_info),	&i2c_reg_setting);
	if (ret < 0)
		 CAM_ERR(CAM_OIS,"err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;
	kfree(i2c_reg_setting.reg_setting);

	return ret;
}

static int read_reg_16bit_value_16bit(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t *data)
{
	enum i2c_freq_mode temp_freq;
	int ret = OIS_ERROR;

	if (o_ctrl == NULL || data == NULL) {
		CAM_ERR(CAM_OIS,"Invalid Args o_ctrl: %pK, data: %pK", o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_ERR(CAM_OIS,"Not in right state to start soc reads: %d",
							o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	/* Modify i2c freq to 1M */
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	ret = camera_io_dev_read(&(o_ctrl->io_master_info), addr, data, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (ret < 0)
		CAM_ERR(CAM_OIS,"err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;

	return ret;
}
static int  i2c_block_write_reg( struct cam_ois_ctrl_t *o_ctrl, uint32_t addr, uint32_t *data, uint32_t count ){
    uint32_t total_num =0,cnt=0;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	enum i2c_freq_mode temp_freq;
	int ret = OIS_ERROR;
	if (o_ctrl == NULL ) {
		CAM_ERR(CAM_OIS,"Invalid Args o_ctrl: %pK", o_ctrl);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_ERR(CAM_OIS,"Not in right state to start soc writes: %d",
							o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;
	/* Modify i2c freq to 1M */
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	
	total_num = count;
    i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
    i2c_reg_setting.size = total_num;
    i2c_reg_setting.delay = 0;
    i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
                            kzalloc(sizeof(struct cam_sensor_i2c_reg_array) *
                            total_num, GFP_KERNEL);
    if (i2c_reg_setting.reg_setting == NULL) {
        CAM_ERR(CAM_OIS,"kzalloc failed");
        return OIS_ERROR;
    }

    for (cnt = 0; cnt < total_num;cnt++,data++) {
    	i2c_reg_setting.reg_setting[cnt].reg_addr =addr;
    	i2c_reg_setting.reg_setting[cnt].reg_data = *data;
    	i2c_reg_setting.reg_setting[cnt].delay = 0;
    	i2c_reg_setting.reg_setting[cnt].data_mask = 0;
    }

   /*0:use SEQ mode for i2c continuous write 1:use BURST mode for i2c continuous write*/
    ret = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),&i2c_reg_setting, 0);

	if (ret < 0)
		 CAM_ERR(CAM_OIS,"err! ret:%d", ret);
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;
	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

void GenerateFirmwareContexts(void)
{
	g_firmwareContext.version = DW9784_FW_VERSION;
	g_firmwareContext.driverIc = 0x9784;
	g_firmwareContext.fwContentPtr = DW9784_FW;
	g_downloadByForce = 0;
	g_updateFw = 0;
}

int dw9784_download_open_camera(struct cam_ois_ctrl_t *o_ctrl)
{
	int ret=0;
	int err_whoami = 0;
	//unsigned int fwchecksum = 0;
	//unsigned int first_chip_id = 0;
	//unsigned int fw_version_current = 0;
	//unsigned int fw_version_latest = 0;
  //unsigned int fw_type = 0;
	//unsigned char eis_type = 0;
	//unsigned char fw_kind = 0;
    unsigned int pre_module_state = 0; /* 0x0000: normal, 0xFFFF: abnormal */
    //int store_flag = 0;
    printk("[dw9784_download_open_camera] enter");
    GenerateFirmwareContexts();

	err_whoami = dw9784_whoami_chk(o_ctrl);
	if (err_whoami == ERROR_SECOND_ID)
	{
		printk("[dw9784_download_open_camera] failed to check the second_id(0x0020) inside the ic");
		printk("[dw9784_download_open_camera] stop the dw9784 ic boot operation");
		return ERROR_WHO_AMI;
	}else if (err_whoami == ERROR_FW_VALID)
	{
		/* it works when the function is enabled in dw9784_whoami_chk() */
		g_downloadByForce = 1; 
		printk("[dw9784_download_open_camera] force to recovery firmware");
	}
	
	if (dw9784_chip_id_chk(o_ctrl) == DW9784_CHIP_ID)
	{
		printk("[dw9784_download_open_camera] dw9784 chip_id check pass");
	} else
	{
		g_downloadByForce = 1; 
		printk("[dw9784_download_open_camera] dw9784 chip_id check failed");
		printk("[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_checksum_fw_chk(o_ctrl) == ERROR_FW_CHECKSUM)
	{
		g_downloadByForce = 1; 
		printk("[dw9784_download_open_camera] The firmware checksum check of the flash memory failed.");
		printk("[dw9784_download_open_camera] force to recovery firmware");
	}
	
	if (dw9784_fw_ver_chk(o_ctrl) != DW9784_FW_VERSION )
	{
		g_updateFw = 1; 
		printk("[dw9784_download_open_camera] update fw to the latest version");
	}else
	{
		printk("[dw9784_download_open_camera] the firmware version is up to date, ver: 0x%04X", DW9784_FW_VERSION);
	}
	
#if 0
	/* if SET's firmware is used separately, the code below applies */
	if (dw9784_fw_type(o_ctrl) != SET_FW )
	{
		g_updateFw = 1; 
		printk("[dw9784_download_open_camera] SET's firmware update is required(current module firmware)");
	}else
	{
		printk("[dw9784_download_open_camera] SET's firmware has already been applied, so no update is required");
	}
#endif
	if (g_downloadByForce == 1)
	{
		pre_module_state = 0xFFFF; /* abnormal state */
	}else if( g_updateFw == 1)
	{
		pre_module_state = 0x0000; /* normal state */
	}
	
	if (g_downloadByForce || g_updateFw)
	{
		printk("[dw9784_download_open_camera] start downloading the latest version firmware, ver: 0x%04X", DW9784_FW_VERSION);
		if(dw9784_download_fw(o_ctrl,pre_module_state) == EOK)
		{
			/* fw download success */
			printk("[dw9784_download_open_camera] complete fw download");
			return EOK;
		}else
		{
			pre_module_state = 0x0001; /* Firmware download failed once */
			
			write_reg_16bit_value_16bit(o_ctrl,0xD002, 0x0001); /* logic reset */
			os_mdelay(4);
			
			if(dw9784_download_fw(o_ctrl,pre_module_state) == EOK)
			{
				/* fw download success */
				printk("[dw9784_download_open_camera] complete fw download");
				return EOK;
			}else{
				/* fw download failed */
				dw9784_shutdown_mode(o_ctrl);
				printk("[dw9784_download_open_camera] enter ic shutdown(sleep) mode");
				return ERROR_FW_DWON_FAIL;
			}
		}	
	}
/*use ST_LSM6DSOQ for fp5*/
/*jinghuang:no need config for fp5,alread config in fw*/
#if 0
	store_flag += dw9784_set_gyro_select(o_ctrl,4); /* ST_LSM6DSOQ */
	store_flag += dw9784_gyro_direction_setting(o_ctrl,GYRO_BACK_LAYOUT, GYRO_DEGREE_0);
	
	if (store_flag)
	{
		ret = dw9784_set_cal_store(o_ctrl);
	}
#endif
/*no need config for fp5*/

	return ret;
}

//unsigned int buf_temp[10240];
int dw9784_download_fw(struct cam_ois_ctrl_t *o_ctrl,int module_state)
{
	//unsigned char ret = ERROR_FW_VERIFY;
	unsigned int i;
	unsigned int addr;
	unsigned int FMC;
	//unsigned int buf[g_firmwareContext.size];
	//memset(buf_temp, 0, MCS_SIZE_W * sizeof(unsigned int));
	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	write_reg_16bit_value_16bit(o_ctrl,0xd001, 0x0000);
	
	/* step 2: MTP setup */
	dw9784_flash_acess(o_ctrl);
	os_mdelay(1);
	
	/* step 3. FMC register check */
	write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x0000); // FMC block FW select
	os_mdelay(1);
	read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
	if (FMC != 0)
	{
		printk("[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x0000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value
		
		read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
		if (FMC != 0)
		{
			printk("[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			printk("[dw9784_download_fw] stop f/w download");
			return ERROR_FW_DOWN_FMC;
		}
	}
	
	/* step 4. code protection off */
	dw9784_code_pt_off(o_ctrl);
	
	/* step 5. erase flash fw data */
	dw9784_fw_eflash_erase(o_ctrl);
	
	printk("[dw9784_download_fw] start firmware download");
	/* step 6. firmware sequential write to flash */
	/* updates the module status before firmware download */
	*(g_firmwareContext.fwContentPtr + MCS_SIZE_W -1) = module_state;
	/*write 1block(256*2bytes)/once*/
	for (i = 0; i < MCS_SIZE_W; i += DATPKT_SIZE)
	{
		addr = MCS_START_ADDRESS + i;
		i2c_block_write_reg(o_ctrl, addr, g_firmwareContext.fwContentPtr + i, DATPKT_SIZE );
	}
	/*write 2Bytes/once*/
	/*for (i = 0; i < MCS_SIZE_W; i++)
	{
		addr = MCS_START_ADDRESS + i;
		write_reg_16bit_value_16bit(o_ctrl,addr, *(g_firmwareContext.fwContentPtr + i));
	}*/
		
#if 0
	/* Check by reading fw flash directly to i2c */
	/* It is disabled by default */
	/* firmware sequential read from flash */
	for (i = 0; i <  MCS_SIZE_W; i += DATPKT_SIZE)
	{
		addr = MCS_START_ADDRESS + i;
		i2c_block_read_reg(addr, buf_temp + i, DATPKT_SIZE);
	}
	printk("[dw9784_download_fw] read firmware from flash");
	/* firmware verify */
	for (i = 0; i < MCS_SIZE_W; i++)
	{
		if (g_firmwareContext.fwContentPtr[i] != buf_temp[i])
		{
			printk("[dw9784_download_fw] firmware verify NG!!! ADDR:%04X -- firmware:%04x -- READ:%04x ", MCS_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i], buf_temp[i]);
			return ERROR_FW_VERIFY;
		}
	}
	printk("[dw9784_download_fw] firmware verification pass");
#endif

	/* step 6. Writes 512Byte FW(PID) data to IF flash.	(FMC register check) */
	write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x1000);
	os_mdelay(1);
	read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
	os_mdelay(1);
	if (FMC != 0x1000)
	{
		printk("[dw9784_download_fw] IF FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x1000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value
		
		read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
		if (FMC != 0x1000)
		{
			printk("[dw9784_download_fw] 2nd IF FMC register value 2nd fail : %04x", FMC);
			printk("[dw9784_download_fw] stop firmware download");
			return ERROR_FW_DOWN_FMC;
		}
	}

	/* step 7. erease IF(FW/PID) eFLASH  */
    dw9784_pid_erase(o_ctrl);
	
	printk("[dw9784_download_fw] start firmware/pid download");
	/* step 8. firmware sequential write to flash */
	/*write 1block(256*2bytes)/once*/
	for (i = 0; i < PID_SIZE_W; i += DATPKT_SIZE)
	{
		addr = IF_START_ADDRESS + i;
		i2c_block_write_reg(o_ctrl, addr, g_firmwareContext.fwContentPtr + MCS_SIZE_W + i, DATPKT_SIZE );
	}

	/*write 2Bytes/once*/
/*	for (i = 0; i < PID_SIZE_W; i++)
	{
		addr = IF_START_ADDRESS + i;
		write_reg_16bit_value_16bit(o_ctrl,addr, *(g_firmwareContext.fwContentPtr + MCS_SIZE_W + i));
	}*/
	printk("[dw9784_download_fw] write firmware/pid to flash");
	
#if 0
	/* Check by reading fw flash directly to i2c */
	/* It is disabled by default */
	memset(buf_temp, 0, MCS_SIZE_W * sizeof(unsigned int));
	
	/* step 9. firmware sequential read from flash */
	for (i = 0; i <  PID_SIZE_W; i += DATPKT_SIZE)
	{
		addr = IF_START_ADDRESS + i;
		i2c_block_read_reg(addr, buf_temp + i, DATPKT_SIZE);
	}
	printk("[dw9784_download_fw] read firmware/pid from flash");
	/* step 10. firmware verify */
	for (i = 0; i < PID_SIZE_W; i++)
	{
		if (g_firmwareContext.fwContentPtr[i + MCS_SIZE_W] != buf_temp[i])
		{
			printk("[dw9784_download_fw] firmware/pid verify fail! ADDR:%04X -- firmware:%04x -- READ:%04x ", MCS_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i + MCS_SIZE_W], buf_temp[i]);
			return ERROR_FW_VERIFY;
		}
	}
	printk("[dw9784_download_fw] firmware/pid verification pass");
#endif
	/* step 14. ic reboot */
	dw9784_ois_reset(o_ctrl);
	
	/* step 15. check fw_checksum */
	if(dw9784_checksum_fw_chk(o_ctrl) == 0)
	{
		printk("[dw9784_download_fw] fw download success.");
		printk("[dw9784_download_fw] finish");
		return EOK;
	} else
	{
		printk("[dw9784_download_fw] fw download cheksum fail.");
		printk("[dw9784_download_fw] finish");
		return ERROR_FW_CHECKSUM;
	}
}

int dw9784_whoami_chk(struct cam_ois_ctrl_t *o_ctrl)
{
	//unsigned int fw_flag1, fw_flag2,fw_flag3;
	unsigned int sec_chip_id;
	write_reg_16bit_value_16bit(o_ctrl,0xD000, 0x0001); /* chip enable */
	os_mdelay(4);
	write_reg_16bit_value_16bit(o_ctrl,0xD001, 0x0000); /* dsp off mode */
	os_mdelay(1);
	
	dw9784_flash_acess(o_ctrl); /* All protection */
	
	read_reg_16bit_value_16bit(o_ctrl,0xD060, &sec_chip_id); /* 2nd chip id */
	printk("[dw9784_ois_ready_check] sec_chip_id : 0x%04x", sec_chip_id);
	if(sec_chip_id != 0x0020)
	{
		printk("[dw9784] second_chip_id check fail : 0x%04X", sec_chip_id);
		printk("[dw9784] second_enter shutdown mode");
		printk("[dw9784] dw9784 cannot work");
		write_reg_16bit_value_16bit(o_ctrl,0xD000, 0x0000); /* ic */
		return ERROR_SECOND_ID;
	}

#if 0
	/* check the start and end data of the firmware */
	/* don't use it first, and guide the customer later when they need it */
	read_reg_16bit_value_16bit(o_ctrl,0x8000, &fw_flag1); /* check firmware validation flag1 */
	read_reg_16bit_value_16bit(o_ctrl,0x8001, &fw_flag2); /* check firmware validation flag2 */
	read_reg_16bit_value_16bit(o_ctrl,0xA7FC, &fw_flag3); /* check firmware validation flag3 */
	printk("[dw9784_ois_ready_check] firmware validation flag1 : 0x%04x", fw_flag1);
	printk("[dw9784_ois_ready_check] firmware validation flag2 : 0x%04x", fw_flag2);
	printk("[dw9784_ois_ready_check] firmware validation flag3 : 0x%04x", fw_flag3);
	
	if((fw_flag1 == 0xF073) && (fw_flag2 == 0x2045) && (fw_flag3 == g_firmwareContext.driverIc))
	{
		printk("[dw9784_ois_ready_check] the firmware data on the flash is ok");
	} else {
		write_reg_16bit_value_16bit(o_ctrl,0xD002, 0x0001); /* dw9784 reset */
		os_mdelay(4);
		printk("[dw9784_ois_ready_check] the firmware on the flash is partially corrupted.");
		return ERROR_FW_VALID;
	}
#endif
	dw9784_ois_reset(o_ctrl); /* ois reset */
	return EOK;
}

unsigned int dw9784_chip_id_chk(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned int chip_id;
	read_reg_16bit_value_16bit(o_ctrl,DW9784_CHIP_ID_ADDRESS, &chip_id);
	
	printk("[dw9784_chip_id] chip_id : 0x%04X", chip_id);
	return chip_id;
}

unsigned int dw9784_fw_ver_chk(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned int fw_ver;
	unsigned int fw_date;
	//unsigned int fw_ver_tmp;
	read_reg_16bit_value_16bit(o_ctrl,0x7001, &fw_ver);
	read_reg_16bit_value_16bit(o_ctrl,0x7002, &fw_date);
	
	printk("[dw9784_chip_id] fw version : 0x%04X", fw_ver);
	printk("[dw9784_chip_id] fw date : 0x%04X", fw_date);

	//write_reg_16bit_value_16bit(o_ctrl,0x7001, 0x0201);
	//mdelay(2);
	//read_reg_16bit_value_16bit(o_ctrl,0x7001, &fw_ver_tmp);
	//printk("[dw9784_chip_id] test write and read fw version : 0x%04X", fw_ver_tmp);
	
	
	return fw_ver;
}

int dw9784_fw_type(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned int r_data;
	unsigned int eis; /* vsync(0) or qtime(1) */
	unsigned int fw_type; /* module(0) or set(1) */
	
	read_reg_16bit_value_16bit(o_ctrl,0x7006, &r_data);
	
	printk("[dw9784_chip_id] fw type : 0x%04X", r_data);
	
	eis = (r_data >> 8) & 0x1; 
	fw_type = r_data & 0x1;

	if ( eis == EIS_VSYNC )
	{
		printk("[dw9784_chip_id] eis mode : vsync");
	}else if ( eis == EIS_QTIME )
	{
		printk("[dw9784_chip_id] eis mode : qtime");
	}
	
	if ( fw_type == MODULE_FW )
	{
		printk("[dw9784_chip_id] fw type : module fw");
	}else if ( fw_type == SET_FW )
	{
		printk("[dw9784_chip_id] fw type : set fw");
	}
	
	return fw_type;
}

void dw9784_fw_info(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned int r_data;
	printk("[dw9784_fw_info] start");
	
	read_reg_16bit_value_16bit(o_ctrl,0x7000, &r_data);
	printk("[dw9784_fw_info] chip_id : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x7001, &r_data);
	printk("[dw9784_fw_info] fw version : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x7002, &r_data);
	printk("[dw9784_fw_info] fw_date : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x7003, &r_data);
	printk("[dw9784_fw_info] set & project : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x7006, &r_data);
	printk("[dw9784_fw_info] vsync/qtime & set_module_fw info : 0x%04X", r_data);
	
	read_reg_16bit_value_16bit(o_ctrl,0x71FC, &r_data);
	printk("[dw9784_fw_info] param_date: 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x71FD, &r_data);
	printk("[dw9784_fw_info] module_id : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x71FE, &r_data);
	printk("[dw9784_fw_info] act_id : 0x%04X", r_data);
		
	read_reg_16bit_value_16bit(o_ctrl,0x700A, &r_data);
	printk("[dw9784_fw_info] reg_module_cal_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x700B, &r_data);
	printk("[dw9784_fw_info] reg_set_cal_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x700C, &r_data);
	printk("[dw9784_fw_info] reg_fw_checksum : 0x%04X", r_data);
	read_reg_16bit_value_16bit(o_ctrl,0x700D, &r_data);
	printk("[dw9784_fw_info] reg_checksum_status : 0x%04X", r_data);
}

int dw9784_erase_mtp_rewritefw(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned int FMC;
	printk("dw9784 erase for rewritefw starting..");
	/* dsp off */
	write_reg_16bit_value_16bit(o_ctrl,0xd001, 0x0000);
	os_mdelay(1);
	/* all protection off */
	dw9784_flash_acess(o_ctrl);
	
	/* FMC register check */
	write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x0000); // FMC block FW select
	os_mdelay(1);
	read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
	if (FMC != 0)
	{
		printk("[dw9784_erase_mtp_rewritefw] FMC register value 1st warning : %04x", FMC);
		write_reg_16bit_value_16bit(o_ctrl,0xDE01, 0x0000);
		os_mdelay(1);
		FMC = 0; // initialize FMC value
		
		read_reg_16bit_value_16bit(o_ctrl,0xDE01, &FMC);
		if (FMC != 0)
		{
			printk("[dw9784_erase_mtp_rewritefw] 2nd FMC register value 2nd warning : %04x", FMC);
			printk("[dw9784_erase_mtp_rewritefw] stop f/w download");
			return ERROR_FW_DOWN_FMC;
		}
	}
	 /* code protection off */
	dw9784_code_pt_off(o_ctrl);
	
	/* 512 byte page */
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0027);
	os_mdelay(1);
	/* page erase */
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0008);
	os_mdelay(10);

	write_reg_16bit_value_16bit(o_ctrl,0xD000, 0x0000); /* chip shutdown */
	printk("dw9784 enter shutdown mode");
	
	return EOK;
}

void dw9784_flash_acess(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_flash_acess] start");
	/* release all protection */
	write_reg_16bit_value_16bit(o_ctrl,0xFAFA, 0x98AC);
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xF053, 0x70BD);
	os_mdelay(1);
	printk("[dw9784_flash_acess] finish");
}

void dw9784_code_pt_off(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_code_pt_off] start");
	/* release all protection */
	write_reg_16bit_value_16bit(o_ctrl,0xFD00, 0x5252);
	os_mdelay(1);
	printk("[dw9784_code_pt_off] finish");
}

void dw9784_pid_erase(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_pid_erase] start pid flash(IF) erase");
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0000);			// page 0
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0008);			// page erase
	os_mdelay(10);											// need to delay after erase
	printk("[dw9784_pid_erase] finish");	
}

void dw9784_fw_eflash_erase(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_fw_eflash_erase] start fw flash erase");
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0000);			// 4k Sector_0
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase		
	
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0008);			// 4k Sector_1
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase		
	
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0010);			// 4k Sector_2
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase		
	
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0018);			// 4k Sector_3
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase		
	
	write_reg_16bit_value_16bit(o_ctrl,0xde03, 0x0020);			// 4k Sector_4
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0xde04, 0x0002);			// 4k Sector Erase
	os_mdelay(10);						// need to delay after erase
	printk("[dw9784_fw_eflash_erase] finish");
}

void dw9784_shutdown_mode(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[all_prot_off] enter ic shutdown mode");
	write_reg_16bit_value_16bit(o_ctrl,0xD000, 0x0000);
	os_mdelay(1);
}

void dw9784_ois_reset(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_ois_reset] start ois reset");
	write_reg_16bit_value_16bit(o_ctrl,0xD002, 0x0001); /* logic reset */
	os_mdelay(4);
	write_reg_16bit_value_16bit(o_ctrl,0xD001, 0x0001); /* Active mode (DSP ON) */
	os_mdelay(25);
	write_reg_16bit_value_16bit(o_ctrl,0xEBF1, 0x56FA); /* User protection release */
	printk("[dw9784_ois_reset] finish");
}

int dw9784_ois_on(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_ois_on] ois on mode change");
	write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x0001); /* set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0000); /* ois on */
	os_mdelay(1);
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0x1000) == FUNC_PASS) { /* busy check */
		printk("[dw9784_ois_on] ois on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}
int dw9784_ois_off(struct cam_ois_ctrl_t *o_ctrl)
{
    printk("[dw9784_ois_off] servo on mode change");
    write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x0001); /* set control mode */
    os_mdelay(1);
    write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0001); /* ois off*/
    os_mdelay(1);
    if(dw9784_wait_check_register(o_ctrl,0x7010, 0x1001) == FUNC_PASS) { /* busy check */
        printk("[dw9784_ois_off] ois off success");
        return FUNC_PASS;
    }
    return FUNC_FAIL;
}


int dw9784_servo_on(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_servo_on] servo on mode change");
	write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x0001); /* set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0001); /* servo on */
	os_mdelay(1);
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0x1001) == FUNC_PASS) { /* busy check */
		printk("[dw9784_servo_on] servo on success");
		return FUNC_PASS;
	}

	return FUNC_FAIL;
}

int dw9784_servo_off(struct cam_ois_ctrl_t *o_ctrl)
{
	printk("[dw9784_servo_off] servo off mode change");
	write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x0001); /* Set control mode */
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0002); /* servo off */
	os_mdelay(1);
	
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0x1002) == FUNC_PASS) { /* busy check */
		printk("[dw9784_servo_off] servo off success");
		return FUNC_PASS;
	}
	
	return FUNC_FAIL;
}

int dw9784_checksum_all_chk(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Bit [0]: FW checksum error 
	Bit [1]: Module cal. checksum error
	Bit [2]: Set cal. checksum error
	*/
	unsigned int reg_fw_checksum;			// 0x700C
	unsigned int reg_module_cal_checksum;	// 0x700A
	unsigned int reg_set_cal_checksum;		// 0x700B
	unsigned int reg_checksum_status;		// 0x700D

	read_reg_16bit_value_16bit(o_ctrl,0x700C, &reg_fw_checksum);
	read_reg_16bit_value_16bit(o_ctrl,0x700A, &reg_module_cal_checksum);
	read_reg_16bit_value_16bit(o_ctrl,0x700B, &reg_set_cal_checksum);

	read_reg_16bit_value_16bit(o_ctrl,0x700D, &reg_checksum_status);
	
	printk("[dw9784_checksum_all_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	printk("[dw9784_checksum_all_chk] ref_fw_checksum : 0x%04X, reg_fw_checksum : 0x%04X", DW9784_FW_CHECKSUM, reg_fw_checksum);

	if( (reg_checksum_status & 0x0111) == 0)
	{
		printk("[dw9784_checksum_all_chk] fw checksum pass");
		return EOK;
	}else
	{
		if (reg_checksum_status & 0x0001)
			printk("[dw9784_checksum_all_chk] fw checksum error");
		
		if (reg_checksum_status & 0x0010)
			printk("[dw9784_checksum_all_chk] module cal. checksum error");
		
		if (reg_checksum_status & 0x0100)
			printk("[dw9784_checksum_all_chk] set cal. checksum error");
		
		printk("[dw9784_checksum_all_chk] module_cal_checksum : 0x%04X", reg_module_cal_checksum);
		printk("[dw9784_checksum_all_chk] set_cal_checksum : 0x%04X", reg_set_cal_checksum);		
		
		return ERROR_ALL_CHECKUM;
	}
}

int dw9784_checksum_fw_chk(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Bit [0]: FW checksum error 
	Bit [1]: Module cal. checksum error
	Bit [2]: Set cal. checksum error
	*/
	unsigned int reg_fw_checksum;			// 0x700C
	unsigned int reg_checksum_status;		// 0x700D

	read_reg_16bit_value_16bit(o_ctrl,0x700C, &reg_fw_checksum);
	read_reg_16bit_value_16bit(o_ctrl,0x700D, &reg_checksum_status);
	
	printk("[dw9784_checksum_fw_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	printk("[dw9784_checksum_fw_chk] ref_fw_checksum : 0x%04X, reg_fw_checksum : 0x%04X", DW9784_FW_CHECKSUM, reg_fw_checksum);

	if((reg_checksum_status&0x0001)  == 0)
	{
		printk("[dw9784_checksum_fw_chk] fw checksum pass");
		return EOK;
	}else
	{
		printk("[dw9784_checksum_fw_chk] fw checksum error");
		return ERROR_FW_CHECKSUM;
	}
}

int dw9784_set_gyro_select(struct cam_ois_ctrl_t *o_ctrl,unsigned int new_gyro_id)
{
	unsigned int old_gyro_id;
	int upd_flag = 0;

	read_reg_16bit_value_16bit(o_ctrl,0x7194, &old_gyro_id); /* gyro sensor set in register */
	
	if( old_gyro_id != new_gyro_id )
	{
		write_reg_16bit_value_16bit(o_ctrl,0x7194, new_gyro_id);
		upd_flag = 1;
		printk("[dw9784_set_gyro_select] gyro sensor information has been updated");
		printk("[dw9784_set_gyro_select] before set_gyro: %d, after set_gyro: %d",old_gyro_id, new_gyro_id);
	}else
	{
		printk("[dw9784_set_gyro_select] the gyro sensor does not change, 0x%04X", new_gyro_id);
	}
	
	if(new_gyro_id == 0)
		printk("[dw9784_set_gyro_select] ICM_20690");
	else if(new_gyro_id == 2)
		printk("[dw9784_set_gyro_select] ST_LSM6DSM");
	else if(new_gyro_id == 4)
		printk("[dw9784_set_gyro_select] ST_LSM6DSOQ");
	else if(new_gyro_id == 5)
		printk("[dw9784_set_gyro_select] ICM_42631");
	else if(new_gyro_id == 6)
		printk("[dw9784_set_gyro_select] BMI260");
	else if(new_gyro_id == 7)
		printk("[dw9784_set_gyro_select] ICM_42602");
	else{
		printk("[dw9784_set_gyro_select] gyro sensor selection failed");
		return FUNC_FAIL;
	}
	
	return upd_flag;
}

int dw9784_gyro_direction_setting(struct cam_ois_ctrl_t *o_ctrl,int gyro_arrangement, int gyro_degree)
{
	int upd_flag = 0;
	unsigned int gyro_id;
	/* set register new value */
	unsigned int x_gyro_gain_pol=0, y_gyro_gain_pol;
	unsigned int x_gyro_mat_cos, x_gyro_mat_sin;
	unsigned int y_gyro_mat_cos, y_gyro_mat_sin;
	/* read register value */
	unsigned int x_reg_gyro_gain_pol, y_reg_gyro_gain_pol;
	unsigned int x_reg_mat_cos, x_reg_mat_sin; 
	unsigned int y_reg_mat_cos, y_reg_mat_sin;

	read_reg_16bit_value_16bit(o_ctrl,0x7194, &gyro_id); /* gyro sensor set in register */
	
	read_reg_16bit_value_16bit(o_ctrl,0x7184, &x_reg_gyro_gain_pol); /* x_gyro_gain_pol reg */
	read_reg_16bit_value_16bit(o_ctrl,0x7185, &y_reg_gyro_gain_pol); /* y_gyro_gain_pol reg */
	
	read_reg_16bit_value_16bit(o_ctrl,0x7186, &x_reg_mat_cos); /* x_gyro_mat_cos */
	read_reg_16bit_value_16bit(o_ctrl,0x7188, &y_reg_mat_cos); /* y_gyro_mat_cos */
	read_reg_16bit_value_16bit(o_ctrl,0x7187, &x_reg_mat_sin); /* x_gyromat_sin */
	read_reg_16bit_value_16bit(o_ctrl,0x7189, &y_reg_mat_sin); /* y_gyromat_sin */
	
	if (gyro_id == 6) { /* Bosch Gryo */
		printk("[dw9784_gyro_direction_setting] dw9784 bosch gyro direction is setting ");
		switch (gyro_degree) 
		{
		case GYRO_DEGREE_0:
			gyro_degree = GYRO_DEGREE_90;
			break;
		case GYRO_DEGREE_90:
			gyro_degree = GYRO_DEGREE_180;
			break;
		case GYRO_DEGREE_180:
			gyro_degree = GYRO_DEGREE_270;
			break;
		case GYRO_DEGREE_270:
			gyro_degree = GYRO_DEGREE_0;
			break;
		}
	}
	
	if (gyro_arrangement == GYRO_FRONT_LAYOUT) {
		/* Gyro Polarity - Front Side Layout */
		x_gyro_gain_pol = 0xFFFF; /* x_gyro_gain_pol reg: 0x7184 */
		y_gyro_gain_pol = 0xFFFF; /* y_gyro_gain_pol reg: 0x7185 */
	} else if (gyro_arrangement == GYRO_BACK_LAYOUT) {
		/* Gyro Polarity - Back Side Layout */
		x_gyro_gain_pol = 0x0001; /* x_gyro_gain_pol reg: 0x7184 */
		y_gyro_gain_pol = 0xFFFF; /* y_gyro_gain_pol reg: 0x7185 */
	} else
	{
		printk("[dw9784_gyro_direction_setting] the location information of the gyro sensor is incorrect");
	}
	
	switch (gyro_degree) 
	{
	case GYRO_DEGREE_0:
		x_gyro_mat_cos = 0x7FFF; /* x_gyro_mat_cos reg: 0x7186 */
		y_gyro_mat_cos = 0x7FFF; /* y_gyro_mat_cos reg: 0x7188 */
		x_gyro_mat_sin = 0x0000; /* x_gyro_mat_sin reg: 0x7187 */
		y_gyro_mat_sin = 0x0000; /* y_gyro_mat_sin reg: 0x7189 */
		break;
	case GYRO_DEGREE_90:
		x_gyro_mat_cos = 0x0000;
		y_gyro_mat_cos = 0x0000;
		x_gyro_mat_sin = 0x8000;
		y_gyro_mat_sin = 0x7FFF;
		break;
	case GYRO_DEGREE_180:
		x_gyro_mat_cos = 0x8000;
		y_gyro_mat_cos = 0x8000;
		x_gyro_mat_sin = 0x0000;
		y_gyro_mat_sin = 0x0000;
		break;
	case GYRO_DEGREE_270:
		x_gyro_mat_cos = 0x0000;
		y_gyro_mat_cos = 0x0000;
		x_gyro_mat_sin = 0x7FFF;
		y_gyro_mat_sin = 0x8000;
		break;
	default:
		printk("[dw9784_gyro_direction_setting] the rotation information of the gyro sensor is incorrect");
		printk("[dw9784_gyro_direction_setting] it is set to 0 degree by default");
		x_gyro_mat_cos = 0x7FFF; /* x_gyro_mat_cos reg: 0x7186 */
		y_gyro_mat_cos = 0x7FFF; /* y_gyro_mat_cos reg: 0x7188 */
		x_gyro_mat_sin = 0x0000; /* x_gyro_mat_sin reg: 0x7187 */
		y_gyro_mat_sin = 0x0000; /* y_gyro_mat_sin reg: 0x7189 */
		break;
	}
	
	if (x_gyro_gain_pol != x_reg_gyro_gain_pol && y_gyro_gain_pol != y_reg_gyro_gain_pol)
	{
		printk("the gyro gain polarity value has been updated");
		printk("x gyro gain pol: 0x%04X, y gyro gain pol: 0x%04X", x_gyro_gain_pol, y_gyro_gain_pol);
		
		write_reg_16bit_value_16bit(o_ctrl,0x7184, x_gyro_gain_pol);
		write_reg_16bit_value_16bit(o_ctrl,0x7185, y_gyro_gain_pol);
		upd_flag = 1;
	}
	
	if ( (x_gyro_mat_cos != x_reg_mat_cos) && (y_gyro_mat_cos != y_reg_mat_cos)
		&& (x_gyro_mat_sin != x_reg_mat_sin) && (y_gyro_mat_sin != y_reg_mat_sin))
	{
		printk("[dw9784_gyro_direction_setting] the gyro matrix value has been updated");
		printk("[dw9784_gyro_direction_setting] x_gyro_mat_cos: 0x%04X", x_gyro_mat_cos);
		printk("[dw9784_gyro_direction_setting] y_gyro_mat_cos: 0x%04X", y_gyro_mat_cos);
		printk("[dw9784_gyro_direction_setting] x_gyro_mat_sin: 0x%04X", x_gyro_mat_sin);
		printk("[dw9784_gyro_direction_setting] y_gyro_mat_sin: 0x%04X", y_gyro_mat_sin);
		
		write_reg_16bit_value_16bit(o_ctrl,0x7186, x_gyro_mat_cos);
		write_reg_16bit_value_16bit(o_ctrl,0x7188, y_gyro_mat_cos);
		write_reg_16bit_value_16bit(o_ctrl,0x7187, x_gyro_mat_sin);
		write_reg_16bit_value_16bit(o_ctrl,0x7189, y_gyro_mat_sin);
		upd_flag = 1;
	}
	return upd_flag;
}

int dw9784_gyro_ofs_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	* dw9784 gyro offset calibration
	Error code definition	
		-1 : FUNC_FAIL
		0 : No Error
	*/
	int msg = 0;
	//unsigned int r_data, x_ofs, y_ofs, gyro_status;
	unsigned int  x_ofs, y_ofs, gyro_status;
	printk("[dw9784_gyro_ofs_calibration] start");
	
	write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x0006); // gyro offset calibration
	os_mdelay(1);

	if(dw9784_wait_check_register(o_ctrl,0x7010, 0x6000) == FUNC_PASS) {
		write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0001);		// gyro ofs calibration execute command
		os_mdelay(100);
	}
	else {
		printk("[dw9784_gyro_ofs_calibration] FUNC_FAIL");
		return FUNC_FAIL;
	}
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0x6001) == FUNC_PASS) { // when calibration is done, Status changes to 0x6001
		printk("[dw9784_gyro_ofs_calibration]calibration function finish");
	}
	else {
		printk("[dw9784_gyro_ofs_calibration]calibration function error");
		return FUNC_FAIL;
	}

	read_reg_16bit_value_16bit(o_ctrl,0x7180, &x_ofs); /* x gyro offset */
	read_reg_16bit_value_16bit(o_ctrl,0x7181, &y_ofs); /* y gyro offset */
	read_reg_16bit_value_16bit(o_ctrl,0x7195, &gyro_status); /* gyro offset status */
	printk("[dw9784_gyro_ofs_calibration]x gyro offset: 0x%04X(%d)", x_ofs, (int)x_ofs);
	printk("[dw9784_gyro_ofs_calibration]y gyro offset: 0x%04X(%d)", y_ofs, (int)y_ofs);
	printk("[dw9784_gyro_ofs_calibration]gyro_status: 0x%04X", gyro_status);

	if( (gyro_status & 0x8000)== 0x8000) {	/* Read Gyro offset cailbration result status */
		if ((gyro_status & 0x1) == X_AXIS_GYRO_OFS_PASS) { 
			msg = EOK; 
			printk("[dw9784_gyro_ofs_calibration] x gyro ofs cal pass");
		}else
		{
			msg += X_AXIS_GYRO_OFS_FAIL; 
			printk("[dw9784_gyro_ofs_calibration] x gyro ofs cal fail");
		}
		
		if ( (gyro_status & 0x10) == X_AXIS_GYRO_OFS_OVER_MAX_LIMIT) { 
			msg += X_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			printk("[dw9784_gyro_ofs_calibration] x gyro ofs over the max. limit");
		}
		
		if ((gyro_status & 0x2) == Y_AXIS_GYRO_OFS_PASS) { 
			msg += EOK; 
			printk("[dw9784_gyro_ofs_calibration] y gyro ofs cal pass");
		}else
		{
			msg += Y_AXIS_GYRO_OFS_FAIL; 
			printk("[dw9784_gyro_ofs_calibration] y gyro ofs cal fail");
		}

		if ( (gyro_status & 0x20) == Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT) { 
			msg += Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			printk("[dw9784_gyro_ofs_calibration] y gyro ofs over the max. limit");
		}
		
		if ( (gyro_status & 0x800) == XY_AXIS_CHECK_GYRO_RAW_DATA) { 
			msg += XY_AXIS_CHECK_GYRO_RAW_DATA;
			printk("[dw9784_gyro_ofs_calibration] check the x/y gyro raw data");
		}
		printk("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");
		
		if(msg == EOK)
		{
			msg = dw9784_set_cal_store(o_ctrl);
		}
		return msg;
	} 
	else {
		printk("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration done fail");
		printk("[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");
		return GYRO_OFS_CAL_DONE_FAIL;
	}
}

int dw9784_set_cal_store(struct cam_ois_ctrl_t *o_ctrl)
{
	/*
	Error code definition
	0 : No Error
	-1 : FUNC_FAIL
	*/
	printk("[dw9784_set_cal_store] start");
	write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x000A); //Set store mode
	
	//When store is done, status changes to 0xA000
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0xA000) == FUNC_PASS) {
		printk("[dw9784_set_cal_store] successful entry into store mode");
	}
	else {
		printk("[dw9784_set_cal_store] failed to enter store mode");
		return FUNC_FAIL;
	}
	
	dw9784_code_pt_off(o_ctrl); /* code protection off */
	write_reg_16bit_value_16bit(o_ctrl,0x700F, 0x5959); //Set protect code
	os_mdelay(1);
	write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0001); //Execute store
	os_mdelay(40);
	
	//When store is done, status changes to 0xA001
	if(dw9784_wait_check_register(o_ctrl,0x7010, 0xA001) == FUNC_PASS) {
		dw9784_ois_reset(o_ctrl);
		printk("[dw9784_set_cal_store] finish");
	}
	else {
		printk("[dw9784_set_cal_store] store function fail");
		return FUNC_FAIL;
	}
	return FUNC_PASS;
}

int dw9784_wait_check_register(struct cam_ois_ctrl_t *o_ctrl,unsigned int reg, unsigned int ref)
{
	/* 
	reg : read target register
	ref : compare reference data
	*/
//	int ret = 0;
	unsigned int r_data;
	int i=0;

	for(i = 0; i < LOOP_A; i++) {
		read_reg_16bit_value_16bit(o_ctrl,reg, &r_data); //Read status
		if(r_data == ref) {
			break;
		}
		else {
			if (i >= LOOP_B) {
				printk("[dw9784_wait_check_register]fail: 0x%04X", r_data);
				return FUNC_FAIL;
			}
		}
		os_mdelay(WAIT_TIME);
	}	
	return FUNC_PASS;
}

void dw9784_fw_read(struct cam_ois_ctrl_t *o_ctrl)
{
	/* Read the data of fw memory using register */
	unsigned int buf_R[10240];
	int i = 0;
	printk("dw9784_fw_read");
	write_reg_16bit_value_16bit(o_ctrl,0xD001, 0x0000); /* dsp mode */
	os_mdelay(1);
	dw9784_flash_acess(o_ctrl);
	/* FW Register Read */
	for (i = 0; i < 10240; i++)
	{
		read_reg_16bit_value_16bit(o_ctrl,0x2000+i, buf_R+i);
	}
	
	for (i = 0; i < 10240; i+= 0x10)
	{
		/* log for debug */
		printk("[dw9784_fw_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X", 
		0x2000 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
		buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
	dw9784_ois_reset(o_ctrl);
}

void dw9784_flash_if_ram_read(struct cam_ois_ctrl_t *o_ctrl)
{
	/* Read the data of IF memory using RAM register */
	unsigned int buf_R[640];
	int i = 0;
	memset(buf_R, 0, 640 * sizeof(unsigned int));

	for (i = 0; i < 640; i++)
	{
		read_reg_16bit_value_16bit(o_ctrl,0x7180+i, buf_R+i);
	}
	printk("[dw9784_flash_if_ram_read] IF_Memory Data Log");
	for (i = 0; i < 640; i+= 0x10)
	{
		/* log for debug */
		printk("[dw9784_flash_if_ram_read] %04X = %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X %04X", 
		0x7180 + i, buf_R[i + 0], buf_R[i + 1], buf_R[i + 2],buf_R[i + 3],buf_R[i + 4],buf_R[i + 5], buf_R[i + 6], buf_R[i + 7], 
		buf_R[i + 8], buf_R[i + 9], buf_R[i + 10], buf_R[i + 11], buf_R[i + 12], buf_R[i + 13], buf_R[i + 14], buf_R[i + 15] ); 
	}
}

void dw9784_flash_ldt_register_read(struct cam_ois_ctrl_t *o_ctrl)
{
	/* Read the data of LDT memory using register */
	unsigned int buf_R[20];
	int i = 0;
	memset(buf_R, 0, 20 * sizeof(unsigned int));
	write_reg_16bit_value_16bit(o_ctrl,0xD001, 0x0000); /* dsp mode */
	os_mdelay(1);
	dw9784_flash_acess(o_ctrl);
	/* LDT Register Read */
	for (i = 0; i < 20; i++)
	{
		read_reg_16bit_value_16bit(o_ctrl,0xD060+i, (unsigned int*)buf_R+i);
		printk("[dw9784_flash_ldt_register_read] LDT : buf_R[%04X] = %04X", 0xD060 + i, buf_R[i]); /* log for debug */
	}
	dw9784_ois_reset(o_ctrl);
}
/*****************************************************************
jinghuang add beging
af drift comp
******************************************************************/
#if 0
static int dw9784_ois_acc_gain(struct cam_ois_ctrl_t *o_ctrl,int targetpos);

static int dw9784_ois_af_drift(int targetpos){
    struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
    int ret=0;
    ret = write_reg_16bit_value_16bit(o_ctrl,0x7070,targetpos);  //af drift comp
    dw9784_ois_acc_gain(o_ctrl,targetpos);//acc gain for lens shift
    return ret;
}
#endif

static ssize_t afdrift_show(struct class *class, struct class_attribute *attr, char *buf){
    printk("ois afdrift_show enter");
    return 0;
}

static ssize_t afdrift_store(struct class *class, struct class_attribute *attr,
							const char *buf, size_t count)
{
	int32_t AFDrift = 0, tagetPos = 0, accGain = 0;
	struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;

	sscanf(buf, "%x", &AFDrift);

	tagetPos = AFDrift & 0xFFFF;
	accGain = (AFDrift >> 16) & 0xFFFF;

	CAM_DBG(CAM_OIS,"ois afdrift_store AFDrift = 0x%x, tagetPos = 0x%x, accGain = 0x%x",AFDrift, tagetPos, accGain);

	write_reg_16bit_value_16bit(o_ctrl, 0x7070, tagetPos);//af drift comp
	write_reg_16bit_value_16bit(o_ctrl, 0x7076, accGain);//x
	write_reg_16bit_value_16bit(o_ctrl, 0x7077, accGain);//y

	return count;
}

/*****************************************************************
jinghuang add
acc gain  for lens shift
*****************************************************************/
int macroDAC = 0xd44;//default value
int infinityDAC = 0x200;//default value

#if 0
static int dw9784_ois_acc_gain(struct cam_ois_ctrl_t *o_ctrl,int targetpos){
    float k,distance,shift_h,normalized_h;
    int acc_gain,ret;
    k =((float)(10000.0-100.0))/((float)(macroDAC-infinityDAC));
    if(targetpos < macroDAC)
        //distance: mm  targetpos:DAC
        distance=((float)macroDAC-(float)targetpos)*k+100.0;
    else
        distance=100.0-k*((float)targetpos-(float)macroDAC);//distance:mm targetpos:DAC
    if(distance < 0)
        distance = 0;
    shift_h = (5.56/distance)*3.5;//shift_h:mm
    normalized_h=shift_h*15.41624;
    acc_gain =(int)(6500*normalized_h);

    CAM_DBG(CAM_OIS,"dw9784_ois_acc_gain acc_gain =0x%x",acc_gain);
    ret = write_reg_16bit_value_16bit(o_ctrl,0x7076,acc_gain);//x
    ret = write_reg_16bit_value_16bit(o_ctrl,0x7077,acc_gain);//y
    return ret;
}
#endif

static ssize_t accgain_show(struct class *class, struct class_attribute *attr, char *buf){
    printk("ois accgain_show enter");
    return 0;
}

static ssize_t accgain_store(struct class *class, struct class_attribute *attr,
                            const char *buf, size_t count){
    sscanf(buf,"%x %x",&macroDAC,&infinityDAC);
    printk("ois gccgain_store macroDAC =0x%x,infinityDAC=0x%x",macroDAC,infinityDAC);

    return count;
}


void dw9784_ois_initial_setting(struct cam_ois_ctrl_t *o_ctrl)
{
	write_reg_16bit_value_16bit(o_ctrl,0x7019, 0x8000); /* enable tripod mode */
	write_reg_16bit_value_16bit(o_ctrl,0x701A, 0x8000); /* enable dd func. */
}
/*****************************************************************
jinghuang add beging for  mmi test funcion
******************************************************************/
/*store gyro gain for cal*/
int dw9784_module_cal_store(struct cam_ois_ctrl_t *o_ctrl)
{
    /*
    Error code definition
    0 : No Error
    -1 : FUNC_FAIL
    */
    printk("[dw9784_module_cal_store] start");
    write_reg_16bit_value_16bit(o_ctrl,0x7012, 0x000A); //Set store mode

    //When store is done, status changes to 0xA000
    if(dw9784_wait_check_register(o_ctrl,0x7010, 0xA000) == FUNC_PASS) {
        printk("[dw9784_module_cal_store] successful entry into store mode");
    }
    else {
        printk("[dw9784_module_cal_store] failed to enter store mode");
    return FUNC_FAIL;
    }
    dw9784_code_pt_off(o_ctrl); /* code protection off */
    write_reg_16bit_value_16bit(o_ctrl,0x700F, 0xA6A6); //Module protect code
    os_mdelay(1);
    write_reg_16bit_value_16bit(o_ctrl,0x7011, 0x0001); //Execute store
    os_mdelay(40);
    //When store is done, status changes to 0xA001
    if(dw9784_wait_check_register(o_ctrl,0x7010, 0xA001) == FUNC_PASS) {
        dw9784_ois_reset(o_ctrl);
        printk("[dw9784_module_cal_store] finish");
    }
    else {
        printk("[dw9784_module_cal_store] store function fail");
        return FUNC_FAIL;
    }
    return FUNC_PASS;
}
/****************************************
add by jinghuang
oisreg write:
echo [write:0x00] [reg_addr] [reg_data] > oisreg
example:echo 0x00 0x0009 0x0001 > oisreg

oisreg read:
echo [read 0x01] [reg_addr] > oisreg
cat oisreg
example: echo 0x01 0x0009 > oisreg
cat oisreg
****************************************/
char reg_data_buff[32];
static ssize_t oisreg_show(struct class * class,struct class_attribute * attr,char * buf){
    strcpy(buf,reg_data_buff);
    printk("oisreg_show %s",reg_data_buff);
    return sizeof(reg_data_buff);
}
static ssize_t oisreg_store(struct class *class, struct class_attribute *attr,
                            const char *buf, size_t count){
    struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
    int flag =0;
    int reg_addr = 0;
    int reg_data = 0;
    sscanf(buf,"%x ",&flag);
    if(flag == REGRW_WRITE){
        sscanf(buf,"%x %x %x",&flag,&reg_addr,&reg_data);
        write_reg_16bit_value_16bit(o_ctrl,reg_addr, reg_data);
    }else if(flag == REGRW_READ){
        sscanf(buf,"%x %x",&flag,&reg_addr);
        read_reg_16bit_value_16bit(o_ctrl,reg_addr, &reg_data);
    }
    printk("oisreg_store flag:0x%x,reg_addr:0x%x,reg_data:0x%x",flag,reg_addr,reg_data);
    sprintf(reg_data_buff,"%x",reg_data);
    return count;
}



/****************************************
add by jinghuang
show:
****************************************/
static ssize_t oisops_show(struct class *class, struct class_attribute *attr, char *buf){
    printk("ois oisops_show enter");
    return 0;
}
/****************************************
add by jinghuang
store cmd:
'0' --->dw9784_gyro_ofs_calibration
'1'---->dw9784_ois_reset
'2'---->dw9784_servo_on
'3'---->dw9784_ois_on
'4'---->dw9784_ois_off
'5'---->dw9784_module_cal_store
****************************************/
static ssize_t oisops_store(struct class *class, struct class_attribute *attr,
                            const char *buf, size_t count){
    struct cam_ois_ctrl_t *o_ctrl = g_o_ctrl;
    char cmd_buff[2];
    memset(cmd_buff,0,2);
    if(!o_ctrl){
        CAM_ERR(CAM_OIS, "o_ctrl is null");
        return count;
    }
    strncpy(cmd_buff,buf,2);
    printk("oisops_store:cmd_buff=%s",cmd_buff);

    if(cmd_buff[0]=='0')
        dw9784_gyro_ofs_calibration(o_ctrl);
    else if(cmd_buff[0]=='1')
        dw9784_ois_reset(o_ctrl);
    else if(cmd_buff[0]=='2')
        dw9784_servo_on(o_ctrl);
    else if(cmd_buff[0]=='3')
        dw9784_ois_on(o_ctrl);
    else if(cmd_buff[0]=='4')
        dw9784_ois_off(o_ctrl);
    else if(cmd_buff[0]=='5')
        dw9784_module_cal_store(o_ctrl);
    else
       printk("oisops_store:cmd is not support");
    return count;

}
/*******************************************************************************
 * Debug node
 * Path: /sys/class/debug_ois
 * ****************************************************************************/
static CLASS_ATTR_RW(oisops);
static CLASS_ATTR_RW(oisreg);
static CLASS_ATTR_RW(afdrift);
static CLASS_ATTR_RW(accgain);

int ois_creat_sysfs(struct cam_ois_ctrl_t *o_ctrl){
    int ret = -1;
    g_o_ctrl = o_ctrl;
    printk("ois ois_creat_sysfs start");
    if(!ois_debug_class){
        printk("ois ois_creat_sysfs!");
        ois_debug_class = class_create(THIS_MODULE,"debug_ois");
        ret = class_create_file(ois_debug_class,&class_attr_oisops);
        if(ret<0){
            printk("create oisops failed,ret %d",ret);
            return ret;
        }
        ret = class_create_file(ois_debug_class,&class_attr_oisreg);
        if(ret < 0){
            printk("create oisreg failed,ret %d",ret);
            return ret;
        }
        ret = class_create_file(ois_debug_class,&class_attr_afdrift);
        if(ret < 0){
            printk("create afdrift failed,ret %d",ret);
            return ret;
        }
        ret = class_create_file(ois_debug_class,&class_attr_accgain);
        if(ret < 0){
            printk("create accgain failed,ret %d",ret);
            return ret;
        }
    }
    return 0;
}
/*add end for  mmi test funcion*/

void hall_sensitivity_cal_example(struct cam_ois_ctrl_t *o_ctrl)
{
	unsigned char project_id = 0;
	unsigned int r_reg = 0;
	unsigned int gyro_gain_x = 0;
	unsigned int gyro_gain_y = 0;

	float hall_sensitivity_x = 0.0;
	float hall_sensitivity_y = 0.0;

	read_reg_16bit_value_16bit(o_ctrl,0x7003, &r_reg);		/* msb: set info, lsb: project info */
	project_id = (unsigned char) (r_reg & 0xFF);

	read_reg_16bit_value_16bit(o_ctrl,0x7182, &gyro_gain_x);	/* read gyro gain x */
	read_reg_16bit_value_16bit(o_ctrl,0x7183, &gyro_gain_y);	/* read gyro gain y */

	printk("[hall_sensitivity_cal] gyro gain x = %d\r\n", gyro_gain_x);
	printk("[hall_sensitivity_cal] gyro gain y = %d\r\n", gyro_gain_y);

	if( project_id == PJT_SOLI){
		printk("[hall_sensitivity_cal] Soli hall sensitivity results:");
		hall_sensitivity_x = ( REF_GYRO_RESULT * gyro_gain_x >> 13 ) / (float)REF_STROKE_SOLI;		/* hall code/um */
		hall_sensitivity_y = ( REF_GYRO_RESULT * gyro_gain_y >> 13 ) / (float)REF_STROKE_SOLI;	
	}
	else if( project_id == PJT_ATHENA ){
		printk("[hall_sensitivity_cal] Athena hall sensitivity results:");
		hall_sensitivity_x = ( REF_GYRO_RESULT * gyro_gain_x >> 13 ) / (float)REF_STROKE_ATHENA;	/* hall code/um */
		hall_sensitivity_y = ( REF_GYRO_RESULT * gyro_gain_y >> 13 ) / (float)REF_STROKE_ATHENA;	
	}
	else{
		
	}
		
	printk("[hall_sensitivity_cal] hall sensitivity x = %.1f\r\n", hall_sensitivity_x);
	printk("[hall_sensitivity_cal] hall sensitivity y = %.1f\r\n", hall_sensitivity_y);
}
