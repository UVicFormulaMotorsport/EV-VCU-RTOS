/*
 * uvfr_settings.c
 *
 *  Created on: Oct 10, 2024
 *      Author: byo10
 */
#define SRC_UVFR_SETTINGS_C_

#include "uvfr_utils.h"
#include "main.h"
#include "stdlib.h"

#define VCU_TO_LAPTOP_ID 0x420
#define LAPTOP_TO_VCU_ID 0x520
//global variables for all to enjoy
uv_vehicle_settings* current_vehicle_settings = NULL;

extern struct uv_os_settings default_os_settings;
extern struct motor_controller_settings mc_default_settings;
extern struct driving_loop_args default_dl_settings;

//Status of laptop connection
static bool is_laptop_connected = false;
TickType_t last_contact_with_laptop = 0;

//Queue for messages coming to the setting setter program
QueueHandle_t settings_queue = NULL;

TaskHandle_t ptask_handle = NULL;

const uv_CAN_msg vcu_ack_msg = {
	.flags = 0x01,
	.dlc = 1,
	.data = {0x10,0,0,0,0,0,0,0},
	.msg_id = 0x420
};

const uv_CAN_msg vcu_ack_failed_msg = {
	.flags = 0x01,
	.dlc = 1,
	.data = {0x11,0,0,0,0,0,0,0},
	.msg_id = 0x420
};


uv_status uvValidateFlashSettings();
uv_status uvResetFlashToDefault();

static inline void settingCopy(uint8_t* from, uint8_t* to, uint16_t length){
	for(int i = 0; i<length; i++){
		*(to + i) = *(from + i);
	}
}

/** @brief Callback function for event where CAN message is received from laptop
 *
 */
void handleIncomingLaptopMsg(uv_CAN_msg* msg){
	uint8_t cmd_byte = msg->data[0];
	static uv_CAN_msg blank_msg = {
			.msg_id = 0x420,
			.flags = 0x01,
			.dlc = 8
	};

	last_contact_with_laptop = xTaskGetTickCount();

	switch(cmd_byte){
	case HANDSHAKE:
		is_laptop_connected = true;
		last_contact_with_laptop = xTaskGetTickCount();

		blank_msg.dlc = 8;
		blank_msg.data[0] = 0x01;
		serializeSmallE32(blank_msg.data,last_contact_with_laptop,1);
		//blank_msg.data[5] = FIRMWARE_MAJOR_RELEASE;
		//blank_msg.data[6] = FIRMWARE_MINOR_RELEASE;
		//blank_msg.data[7] = FIRMWARE_PATCH_NUM;

		uvSendCanMSG(&blank_msg);
		break;
	case ENTER_PROGRAMMING_MODE:

		if(vehicle_state == UV_READY){
			//Send relevant ACK MSG.
			changeVehicleState(PROGRAMMING);
		}else{
			//should not be programming if in either of these states
			uvSendCanMSG(&vcu_ack_failed_msg);
		}
		break;
	case REQUEST_VCU_STATUS:
		//TODO Respond with VCU status
		break;
	case GENERIC_ACK:
		//Could come at any time, however here we are
		break;
	case SET_SPECIFIC_PARAM ... END_OF_SPECIFIC_PARAMS:
	case REQUEST_ALL_SETTINGS:
	case REQUEST_ALL_JOURNAL_ENTRIES:
	case REQUEST_JOURNAL_ENTRIES_BY_TIME:
	case REQUEST_SPECIFIC_SETTING:
	case SAVE_AND_APPLY_NEW_SETTINGS:
	case DISCARD_NEW_SETTINGS:
	case DISCARD_NEW_SETTINGS_AND_EXIT:
	case FORCE_RESTORE_FACTORY_DEFAULT:
		//We need to be in programming mode for these.
		if(vehicle_state == PROGRAMMING){
			if(settings_queue != NULL){
				if(xQueueSend(settings_queue,msg,0) != pdTRUE){
					//error
					uvSendCanMSG(&vcu_ack_failed_msg);
					uvPanic("",0);
				}
			}
		}else{
			//send negative ACK
			uvSendCanMSG(&vcu_ack_failed_msg);
		}
		//otherwise we will ignore
	break;
	default:

		//cmd_byte we do not recognise. We assme the laptop is geeked, and move on

		break;
	}
}

/** @brief Function that allocates the neccessary space for all the vehicle settings, and
 * handles sets all of the settings structs to defaults
 *
 */
uv_status setupDefaultSettings(){
	if(current_vehicle_settings != NULL){
		uvFree(current_vehicle_settings);

	}
	//real trap shit
	current_vehicle_settings = uvMalloc(sizeof(uv_vehicle_settings));
	if(current_vehicle_settings == NULL){
		return UV_ERROR;
	}

	current_vehicle_settings->os_settings = &default_os_settings;
	current_vehicle_settings->mc_settings = &mc_default_settings;
	current_vehicle_settings->driving_loop_settings = &default_dl_settings;
	current_vehicle_settings->driving_loop_settings = NULL;
	current_vehicle_settings->imd_settings = NULL;
	current_vehicle_settings->bms_settings = NULL;
	current_vehicle_settings->daq_settings = NULL;
	current_vehicle_settings->pdu_settings = NULL;

	current_vehicle_settings->flags |= 0x0001; //This is default settings
	return UV_OK;
}

void nukeSettings(uv_vehicle_settings** settings_to_delete){


	uvFree(*settings_to_delete);
	*settings_to_delete = NULL;


}

/** @brief
 *
 * This loads us some new settings from the flash memory
 *
 */
uv_status uvLoadSettingsFromFlash(){
	if(current_vehicle_settings != NULL){
		uvFree(current_vehicle_settings);
	}

	//real trap shit
	current_vehicle_settings = uvMalloc(sizeof(uv_vehicle_settings));
	if(current_vehicle_settings == NULL){
		return UV_ERROR;
	}


	current_vehicle_settings->os_settings = (uv_os_settings*)(START_OF_USER_FLASH + OS_SETTINGS_MGROUP*256 + OS_SETTINGS_OFFSET);


	current_vehicle_settings->mc_settings = (motor_controller_settings*)(START_OF_USER_FLASH + MOTOR_MGROUP*256 + MOTOR_OFFSET);
	current_vehicle_settings->driving_loop_settings = (driving_loop_args*)(START_OF_USER_FLASH + DRIVING_MGROUP + DRIVING_OFFSET);
	current_vehicle_settings->imd_settings = IMD_ADDR;
	current_vehicle_settings->bms_settings = BMS_ADDR;
	current_vehicle_settings->daq_settings = DAQ_HEAD_ADDR;
	current_vehicle_settings->daq_param_list = DAQ_PARAMS1_ADDR;
	current_vehicle_settings->pdu_settings = (void*)(START_OF_USER_FLASH + PDU_MGROUP*256);
	return UV_OK;
}

/** @brief this function does one thing, and one thing only, it checks if we have custom settings, then it attempts to get them.
 * If it fails, then we revert to factory defaults.
 *
 */
uv_status uvSettingsInit(){
	insertCANMessageHandler(0x520,handleIncomingLaptopMsg); //Allows us to talk with laptop

	current_vehicle_settings = uvMalloc(sizeof(uv_vehicle_settings));

	if(current_vehicle_settings == NULL){
		//HMMM
		__uvInitPanic(); // deeply unfortunate
		return UV_ERROR;
	}

	//First we check the validity of the settings in flash.
	uv_status retval = uvValidateFlashSettings();

	if(retval == UV_OK){

		if(uvLoadSettingsFromFlash() == UV_OK){
			//Attempt to load flash settings. If that somehow fails, revert to factory defaults
			return UV_OK;
		}else{
			//Could not actually load from flash. BAD!
			return UV_ERROR;
		}

	}

	if(setupDefaultSettings() == UV_OK){

	}else if(setupDefaultSettings() != UV_OK){
		//FAILURE TO EVEN LOAD THE DEFAULTS, THIS CAR IS UNDRIVEABLE
		__uvInitPanic();
		return UV_ERROR;
	}


	return UV_OK;

}

uv_status uvUpdateTmpSettings(uint8_t* tmp_settings, uint8_t memgroup, uint8_t m_offset, uint8_t size, uint8_t* data){
	if(memgroup > 12){
		//out of range
		return UV_ERROR;
	}

	if(m_offset + size > 256){
		//cannot have setting extend between 2 memgroups
		return UV_ERROR;
	}

	uint32_t offset = memgroup*256 + m_offset;

	for(int i = 0; i<size; i++){
		*(tmp_settings + offset + i) = *(data + i); //ensure correct number comes through
	}



	return UV_OK;
}

uint32_t uvComputeChunkChecksum(uint32_t* chunk){
	uint32_t tmp;
	uint32_t buf[9];

	buf[8] = 0;
	for(int i = 0; i < 8; i++){
		buf[i] = chunk[i];
	}

	for(int i = 0; i< 256; i++){
		buf[i/32] ^= (CRC_POLY << i%32);
	}
	return UV_OK;
}

uv_status uvComputeMemRegionChecksum(void* sblock, uint32_t* csums ,char mregion){
	for(int i = 0; i<32 ; i++){
		*(csums + i*sizeof(uint32_t)) = uvComputeChunkChecksum(sblock + i*8*4);
	}

	return UV_OK;

}

uv_status uvComputeSettingsChecksums(void* sblock, uint32_t* csums){


	//Compute the checksums of the sblock
	//8 to 1 reducgtion

	uv_status retval = UV_OK;

	for(int i = 0; i < 12; i++){
		retval = uvComputeMemRegionChecksum(sblock, csums, i);

		if(retval != UV_OK){
			return retval;
		}

	}

	return UV_OK;

}



uv_status uvValidateChecksums(void* sblock){
	uint32_t* csums = uvMalloc(512);

	if(csums == NULL){
		//BIG UHH OHH
	}

	uvComputeSettingsChecksums(sblock, csums);


}





uv_status uvOverwriteCsr(void* sblock, uint32_t* csr){

	return UV_OK;
}

#define FLASH_OK 0
#define INVALID_SBLOCK 1
#define FLASH_NOT_UNLOCKED 2
#define DID_NOT_FINISH_PROGRAMMING 3
#define DATA_MISMATCH 4
#define PRE_CHECKSUM 5
#define POST_CHECKSUM 6

uv_status uvSaveSettingsToFlash(void* sblock, uint32_t* ecode){
	if(sblock == NULL){
		*ecode = INVALID_SBLOCK;
		return UV_ERROR;
	}

	void* tmp = sblock;
//	if(uvValidateChecksums(tmp) != UV_OK){
//		return UV_ABORTED;
//	}

	*((uint32_t*)(tmp + 0)) = 0x42069420;
	*((uint32_t*)(tmp + 4)) = 0x00000001; //Little reminder for future VCU that the settings were recently changed


	void* addr = START_OF_USER_FLASH;

	if(HAL_FLASH_Unlock() != HAL_OK){
		*ecode = FLASH_NOT_UNLOCKED;
		return UV_ERROR;
	}




	while(addr < TOP_OF_FLASH_SBLOCK){ //TODO Is this the right address?
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)addr,(uint32_t)sblock)!= HAL_OK){
			//ERROR ERROR AHHHH FUCK
			//uvPanic("FLASH_WRITE_ERROR");
			*ecode = DID_NOT_FINISH_PROGRAMMING;
			HAL_FLASH_Lock();
			return UV_ERROR;
		}
		addr += 4;
		tmp += 4;
	}

	HAL_FLASH_Lock();

	//Check to make sure the two match

	tmp = sblock;
	addr = START_OF_USER_FLASH;

	while(addr < TOP_OF_FLASH_SBLOCK){
		if(*((uint32_t*)tmp) != *((uint32_t*)addr)){
			*ecode = DATA_MISMATCH;
			return UV_ERROR; //Mismatch
		}

		tmp += 4;
		addr += 4;
	}

//	if(uvValidateChecksums(START_OF_USER_FLASH)!= UV_OK){
//		return UV_ERROR;
//	}

	return UV_OK;

}

/** @brief Function that creates a
 *
 */
void* uvCreateTmpSettingsCopy(){
	uint32_t* sblock = uvMalloc(SETTING_BRANCH_SIZE);
	if(sblock == NULL){
		return NULL;

	}

	uint32_t* tmp = (uint32_t*)(START_OF_USER_FLASH);

	for(int i = 0; i < SETTING_BRANCH_SIZE; i += 4){
		*sblock = *(tmp + 4);

	}

	return sblock;

}

/**
 *
 */
uv_status uvValidateFlashSettings(){
	if(*((uint32_t*)START_OF_USER_FLASH) != 0x42069420){
		return UV_ERROR;// Nice blank slate
	}

	return UV_OK;
}

#if 0
void uvSettingsProgrammerTask(void* args){
	uv_task_info* params = (uv_task_info*) args;
	void* new_tmp_settings = uvCreateTmpSettingsCopy();

	if(new_tmp_settings == NULL){
		//Did not create temp settings >:(
		uvSendCanMSG(&vcu_ack_failed_msg); //Let laptop know of our failures
	}

	ptask_handle = params->task_handle;

	uvSendCanMSG(&vcu_ack_msg);

	settings_queue = xQueueCreate(8, sizeof(uv_CAN_msg));

	if(settings_queue == NULL){
		uvPanic("Settings Queue Not Created",0);
	}

	uv_CAN_msg tmp_msg;

	uvSendCanMSG(&vcu_ack_msg);

	for(;;){
		if(xQueueReceive(settings_queue,&tmp_msg,100)!=pdTRUE){
			//laptop connection lost. Yowza. Discard whatever you're doin.
		}

		uint8_t cmd_byte = tmp_msg.data[0];

		switch(cmd_byte){
			case SET_SPECIFIC_PARAM ... END_OF_SPECIFIC_PARAMS:
				uint8_t mgroup = (cmd_byte &(0b00011111));
				uint8_t m_offset = tmp_msg.data[1];
				uint8_t d_type = tmp_msg.data[2];
				uint8_t d_size = data_size[d_type];

				if(uvUpdateTmpSettings(new_tmp_settings, mgroup, m_offset, d_size, (tmp_msg.data + 3)) == UV_OK){
					//Send back an ACK
					uvSendCanMSG(&vcu_ack_msg);
				}else{
					//reply that it failed
					uvSendCanMSG(&vcu_ack_failed_msg);
				}


				break;
			case REQUEST_ALL_SETTINGS:
				//Well I guess now we gotta transmit the whole shabang
				//Time for probably the most fucked task I've done in a hot minute
				break;
			case REQUEST_ALL_JOURNAL_ENTRIES:
				uvSendCanMSG(&vcu_ack_failed_msg);
				break;
			case REQUEST_JOURNAL_ENTRIES_BY_TIME:
				uvSendCanMSG(&vcu_ack_failed_msg);
				break;
			case REQUEST_SPECIFIC_SETTING:
				//We wish to know the value of a specific setting

				break;
			case SAVE_AND_APPLY_NEW_SETTINGS:
				//Save and apply. Hoo boy.
				uint32_t ecode = 0;
				if(uvSaveSettingsToFlash(new_tmp_settings, &ecode) == UV_OK){
					//Try once
				}else if(uvSaveSettingsToFlash(new_tmp_settings, &ecode) == UV_OK){
					//Try again
				}else{
					//This is a tricky one. High probability of data corruption.
				}


				break;
			case DISCARD_NEW_SETTINGS:
				if(uvFree(new_tmp_settings) == UV_OK){
					new_tmp_settings = uvCreateTmpSettingsCopy();


					if(new_tmp_settings != NULL){
						uvSendCanMSG(&vcu_ack_msg);
					}else{
						uvSendCanMSG(&vcu_ack_failed_msg);
						uvPanic("uhh oh",0);
					}
				}else{
					//Not OK
					uvSendCanMSG(&vcu_ack_failed_msg);
					uvPanic("uhh oh",0);
				}
				break;
			case DISCARD_NEW_SETTINGS_AND_EXIT:
				uvFree(new_tmp_settings);
				changeVehicleState(UV_READY);
				break;
			case FORCE_RESTORE_FACTORY_DEFAULT:
				break;
			default:
				//unrecognized command spotted
				break;
		}

		if(params->cmd_data == UV_KILL_CMD){
			//TODO add destructors here

			if(new_tmp_settings != NULL){
				uvFree(new_tmp_settings);
			}

			killSelf(params);
			while(1){

			}
		}
	}
}
#endif



uv_status uvResetFlashToDefault(){
	void* new_sblock = uvMalloc(SETTING_BRANCH_SIZE);

	if(new_sblock == NULL){
		return UV_ERROR; //definately not ideal
	}

	settingCopy((uint8_t*)(&default_os_settings),new_sblock + OS_SETTINGS_OFFSET,sizeof(uv_os_settings));
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);


	return UV_OK;

}

void sendAllSettingsWorker(void* args){

}

void sendJournalWorker(void* args){

}





