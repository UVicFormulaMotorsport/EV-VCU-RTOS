/*
 * uvfr_settings.h
 *
 *  Created on: Oct 10, 2024
 *      Author: byo10
 */

#ifndef INC_UVFR_SETTINGS_H_
#define INC_UVFR_SETTINGS_H_


#include "motor_controller.h"
#include "driving_loop.h"
#include "uvfr_utils.h"
#include "main.h"
#include "daq.h"
#include "bms.h"



#define ENABLE_FLASH_SETTINGS 0



#define START_OF_USER_FLASH &_s_uvdata
#define	TOP_OF_USER_FLASH &_e_uvdata
#define TOP_OF_FLASH_SBLOCK (void*)0x080FFFFF
#define SBLOCK_CSR_OFFSET 0x0C00
#define SYS_DATA_OFFSET 0x00
#define SIZE_OF_MGROUP 0x0100
#define SBLOCK_CRC_REGION_OFFSET //(START_OF_USER_FLASH - 0xFF)

//size of user flash in bytes
#define SIZEOF_USER_FLASH 4096
#define SIZEOF_SBLOCK 4096


#define isPtrToFlash(p) ((p>=0x08000000)&&(p<TOP_OF_USER_FLASH))

//Positions of different settings in SBLOCK
#define GENERAL_VEH_INFO_MGROUP 0
#define GENERAL_VEH_INFO_OFFSET 32 //Start at 32 since first 32 bytes reserved for settings integrity checks

#define OS_SETTINGS_MGROUP 0
#define OS_SETTINGS_OFFSET 128
#define OS_SETTINGS_ADDR ((void*)(START_OF_USER_FLASH + OS_SETTINGS_MGROUP*256 + OS_SETTINGS_OFFSET))

#define MOTOR_MGROUP 1
#define MOTOR_OFFSET 0
#define MOTOR_ADDR (struct motor_controller_settings*)(START_OF_USER_FLASH + MOTOR_MGROUP*256 + MOTOR_OFFSET)

#define DRIVING_MGROUP 2
#define DRIVING_OFFSET 0
#define DRIVING_ADDR ((driving_loop_args*)(START_OF_USER_FLASH + DRIVING_MGROUP + DRIVING_OFFSET))

#define BMS_MGROUP 4
#define BMS_OFFSET 0
#define BMS_ADDR NULL

#define IMD_MGROUP 4
#define IMD_OFFSET 128
#define IMD_ADDR ((void*)(START_OF_USER_FLASH + IMD_MGROUP*256 + IMD_OFFSET))

#define PDU_MGROUP 5
#define PDU_OFFSET 0
#define PDU_ADDR ((void*)(START_OF_USER_FLASH + PDU_MGROUP*256 + PDU_OFFSET))

#define DAQ_HEAD_MGROUP 5
#define DAQ_HEAD_OFFSET 128
#define DAQ_HEAD_ADDR ((daq_loop_args*)(START_OF_USER_FLASH + DAQ_HEAD_MGROUP*256 + DAQ_HEAD_OFFSET))

#define DAQ_PARAMS1_MGROUP 6
#define DAQ_PARAMS1_OFFSET 0
#define DAQ_PARAMS1_ADDR ((void*)(START_OF_USER_FLASH + DAQ_PARAMS1_MGROUP*256 + DAQ_PARAMS1_OFFSET))

#define DAQ_PARAMS2_MGROUP 7
#define DAQ_PARAMS2_OFFSET 0
#define DAQ_PARAMS2_ADDR

#define DAQ_PARAMS3_MGROUP 8
#define DAQ_PARAMS3_OFFSET 0
#define DAQ_PARAMS3_ADDR

#define PERSISTENT_DATA_MGROUP 13
#define CRC_MGROUP1 14
#define CRC_MGROUP2 15

#define SETTING_BRANCH_SIZE (256*10)

#define CRC_POLY 0x04C11DB7
//CRC-32 (POSIX Checksum)



//AAA
typedef struct veh_gen_info{
	uint32_t wheel_size;
	uint32_t drive_ratio;
	uint16_t test1;
	uint16_t test2;
	uint16_t test3;
	uint8_t test4;
	uint8_t test5;

	uint32_t test6;

}veh_gen_info;

typedef enum uv_status_t uv_status;

typedef enum{
	HANDSHAKE = 0x01,
	ENTER_PROGRAMMING_MODE = 0x02,
	REQUEST_VCU_STATUS = 0x03,
	RESERVED_CMD1 = 0x04,
	REQUEST_VCU_FIRMWARE_VERSION = 0x05,
	GENERIC_ACK = 0x10,
	CANNOT_PERFORM_REQUEST = 0x11,
	SET_SPECIFIC_PARAM = 0x20,
	END_OF_SPECIFIC_PARAMS = 0x3F,
	REQUEST_ALL_SETTINGS = 0x40,
	REQUEST_ALL_JOURNAL_ENTRIES = 0x41,
	REQUEST_JOURNAL_ENTRIES_BY_TIME = 0x42,
	REQUEST_SPECIFIC_SETTING = 0x43,
	SAVE_AND_APPLY_NEW_SETTINGS = 0x80,
	DISCARD_NEW_SETTINGS = 0x82,
	DISCARD_NEW_SETTINGS_AND_EXIT = 0x83,
	FORCE_RESTORE_FACTORY_DEFAULT = 0x84

}laptop_CMD;

typedef struct uv_vehicle_settings{
	struct veh_gen_info* veh_info;

	struct uv_os_settings* os_settings;
	struct motor_controller_settings* mc_settings;

	driving_loop_args* driving_loop_settings;

	struct uv_imd_settings* imd_settings;
	bms_settings_t* bms_settings;

	daq_loop_args* daq_settings;
	daq_datapoint* daq_param_list;

	struct uv19_pdu_settings* pdu_settings;
	//struct motor_controller_settings motor_controller_settings;

	uint16_t flags; /**< Bitfield containing info on whether each settings instance is factory default. 0 default, 1 altered*/


}uv_vehicle_settings;



//typedef struct motor_controller_settings motor_controller_settings;

typedef struct motor_controller_settings{
    //can transmit 0x200
    uint32_t can_id_tx;
    //can receive 0x201
    uint32_t can_id_rx;
   //can timeout 2s?
    uint32_t mc_CAN_timeout;
    //double check length
    //kp - gain 0x2c - 10
    uint8_t proportional_gain;
    //ti - reset time 0x2d - 400
    uint32_t integral_time_constant;
    //Tim - max value integral 0x3b - 60%
    uint8_t integral_memory_max;
    //serial number

    //firmware version

}motor_controller_settings;


uv_status uvConfigSettingTask(void* args);

void nukeSettings(uv_vehicle_settings** settings_to_delete);
uv_status uvValidateSettingsFromFlash();

enum uv_status_t uvSettingsInit();
uv_status setupDefaultSettings();
uv_status uvSaveSettingsToFlash();
uv_status uvComputeMemRegionChecksum(void* sblock, uint32_t* csums, char mregion);
uv_status uvComputeSettingsCheckSums(void* sblock);
uv_status uvValidateChecksums(void* sblock);


#ifndef SRC_UVFR_SETTINGS_C_
//extern includes

extern uv_vehicle_settings* current_vehicle_settings;

#endif


#endif /* INC_UVFR_SETTINGS_H_ */



