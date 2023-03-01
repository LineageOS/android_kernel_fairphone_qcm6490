
#ifndef __BSP_PUBLIC_INTERFACES_
#define __BSP_PUBLIC_INTERFACES_


struct board_platform_data {

	struct kobject * sysinfo_kobj;
#ifdef CONFIG_ESIM_REGULATOR_ON
	struct regulator *esim_power;
#endif
	const char *name;

	char board_version[8];

	unsigned board_id0;
	unsigned board_id1;
	unsigned board_id2;
	unsigned board_id3;

	int hw_version;

	int id0_val;
	int id1_val;
	int id2_val;
	int id3_val;
};


int isChargerMode(void);
int isRecoveryMode(void);
int get_hw_version(void);
bool is_hot_swap(void);
void set_hot_swap(bool status);


#endif



