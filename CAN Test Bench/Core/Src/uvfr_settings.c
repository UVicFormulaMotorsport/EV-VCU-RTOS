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
#include "uvfr_vehicle_logger.h"

#define VCU_TO_LAPTOP_ID 0x420
#define LAPTOP_TO_VCU_ID 0x520

typedef struct output_channel_settings output_channel_settings;


extern PRIVILEGED_DATA uint8_t _s_uvdata; //Start and end of user flash symbols
extern PRIVILEGED_DATA uint8_t _e_uvdata;


//global variables for all to enjoy
uv_vehicle_settings* current_vehicle_settings = NULL;

extern struct uv_os_settings default_os_settings;
extern struct motor_controller_settings mc_default_settings;
extern struct driving_loop_args default_dl_settings;

extern struct daq_loop_args default_daq_settings;
extern struct uv_imd_settings default_imd_settings;
extern bms_settings_t default_bms_settings;
extern struct output_channel_settings default_output_channels;
extern daq_datapoint default_datapoints[];

/** These are arguments passed to the "Transmit All Settings Over CANbus"
 * Subroutine.
 *
 */
typedef struct tx_all_settings_args{
	uint8_t* sblock_origin;

}tx_all_settings_args;

/**
 *
 */
typedef struct tx_journal_args{
	uint32_t start_time;
	uint32_t end_time;
}tx_journal_args;

/**
 *
 */
typedef union helper_task_args{
	tx_all_settings_args setting_tx_args;
	tx_journal_args journal_tx_args;
}helper_task_args;

/**
 *
 */
typedef struct setting_helper_task{
	TaskHandle_t meta_task_handle;

	uint32_t status;


	helper_task_args args;

}setting_helper_task;

/**
 *
 */
veh_gen_info default_vehicle = {
	.wheel_size = 0xEEEEEEEE,
	.drive_ratio = 0xDDDDDDDD,
	.test1 = 0xCCCC,
	.test2 = 0xBBBB,
	.test3 = 0xAAAA,
	.test4 = 0x99,
	.test5 = 0x88,
	.test6 = 0x77777777
};

//Status of laptop connection
static bool is_laptop_connected = false;
TickType_t last_contact_with_laptop = 0;

//Queue for messages coming to the setting setter program
static QueueHandle_t settings_queue = NULL;

static TaskHandle_t ptask_handle = NULL;

static TaskHandle_t child_task_handle = NULL;
static bool child_task_active = false;

/** Pre-create message that is transmitted by the VCU
 * upon successful completion of a settings change or diagnostic
 * operation requested by a laptop
 *
 */
const uv_CAN_msg vcu_ack_msg = {
	.flags = 0x00,
	.dlc = 1,
	.data = {0x10,0,0,0,0,0,0,0},
	.msg_id = VCU_TO_LAPTOP_ID
};

/** Pre-created message that indicates the VCU has failed
 * to perform some requested action
 *
 */
const uv_CAN_msg vcu_ack_failed_msg = {
	.flags = 0x00,
	.dlc = 1,
	.data = {0x11,0,0,0,0,0,0,0},
	.msg_id = VCU_TO_LAPTOP_ID
};




uv_status uvValidateFlashSettings();
uv_status uvResetFlashToDefault();
void uvSettingsProgrammerTask(void* args);

/** @brief Internal function that is used to copy an arbitrary amount of data from point A to point B
 *
 */
static inline void settingCopy(uint8_t* from, uint8_t* to, uint16_t length) PRIVILEGED_FUNCTION{
	if(from == NULL || to == NULL || length == 0){
		return; //No null errors on my watch buddy boy
	}

	for(int i = 0; i<length; i++){
		*(to + i) = *(from + i);
	}
}

/** @brief Transmits the status of the vehicle, including errorcodes and similar
 *
 */
uv_status uvTransmitVehicleStatus(){
	return UV_OK;
}

uv_status uvAwaitLaptopAckMsg(){
	return UV_ABORTED;

}

/** @brief Callback function for event where CAN message is received from laptop
 *
 */

void handleIncomingLaptopMsg(uv_CAN_msg* msg) PRIVILEGED_FUNCTION{
	uint8_t cmd_byte = msg->data[0];
	static uv_CAN_msg blank_msg = {
			.msg_id = 0x420,
			.flags = 0x00,

			.dlc = 8
	};

	last_contact_with_laptop = xTaskGetTickCount();


	switch(cmd_byte){
	case LAPTOP_HANDSHAKE:
		is_laptop_connected = true;
		last_contact_with_laptop = xTaskGetTickCount();

		blank_msg.dlc = 8;
		blank_msg.data[0] = 0x01;
		serializeSmallE32(blank_msg.data,last_contact_with_laptop,1);
		blank_msg.data[5] = FIRMWARE_MAJOR_RELEASE;
		blank_msg.data[6] = FIRMWARE_MINOR_RELEASE;
		blank_msg.data[7] = FIRMWARE_PATCH_NUM;


		uvSendCanMSG(&blank_msg);
		break;
	case ENTER_PROGRAMMING_MODE:

		if(vehicle_state == UV_READY){

			//Relevant Ack. frame sent upon succesful startup of programmer task

			changeVehicleState(PROGRAMMING);
		}else{
			//should not be programming if in either of these states
			uvSendCanMSG(&vcu_ack_failed_msg);
		}
		break;
	case REQUEST_VCU_STATUS:
		//TODO Respond with VCU status

		if(uvTransmitVehicleStatus()!=UV_OK){
			//Handle this error.
		}

		break;
	case GENERIC_ACK:
		//Could come at any time, however here we are
		break;
	case REQUEST_ALL_JOURNAL_ENTRIES:
		flushLogsToCAN();
		break;
	case SET_SPECIFIC_PARAM ... END_OF_SPECIFIC_PARAMS:

	case CLEAR_FAULTS:
	case REQUEST_ALL_SETTINGS:
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
	//current_vehicle_settings->driving_loop_settings = NULL;
	current_vehicle_settings->imd_settings = NULL;
	current_vehicle_settings->bms_settings = &default_bms_settings;
	current_vehicle_settings->daq_settings = &default_daq_settings;
	current_vehicle_settings->daq_param_list = default_datapoints;
	current_vehicle_settings->pdu_settings = NULL;


	current_vehicle_settings->flags |= 0x0001; //This is default settings
	return UV_OK;
}

/** @brief
 *
 */
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



	current_vehicle_settings->veh_info = (veh_gen_info*)(START_OF_USER_FLASH + 256*GENERAL_VEH_INFO_MGROUP+GENERAL_VEH_INFO_OFFSET);


	current_vehicle_settings->os_settings = (uv_os_settings*)(START_OF_USER_FLASH + OS_SETTINGS_MGROUP*256 + OS_SETTINGS_OFFSET);


	current_vehicle_settings->mc_settings = (motor_controller_settings*)(START_OF_USER_FLASH + MOTOR_MGROUP*256 + MOTOR_OFFSET);
	current_vehicle_settings->driving_loop_settings = (driving_loop_args*)(START_OF_USER_FLASH + DRIVING_MGROUP + DRIVING_OFFSET);
	current_vehicle_settings->imd_settings = IMD_ADDR;
	current_vehicle_settings->bms_settings = BMS_ADDR;
	current_vehicle_settings->daq_settings = DAQ_HEAD_ADDR;
	current_vehicle_settings->daq_param_list = DAQ_PARAMS1_ADDR;

	current_vehicle_settings->pdu_settings = (void*)(START_OF_USER_FLASH + PDU_MGROUP*256 + PDU_OFFSET);
	return UV_OK;
}


/** @brief Function to setup the parameters of the setting setter task
 *
 */
uv_status uvConfigSettingTask(void* args){
	uv_task_info* setting_task = uvCreateTask();

	if(setting_task == NULL){
		return UV_ERROR;
	}

	setting_task->task_name = "Set_settings";

	setting_task->task_function = uvSettingsProgrammerTask;
	setting_task->task_priority = ABOVE_NORMAL;

	setting_task->stack_size = 512;

	setting_task->active_states = PROGRAMMING;
	setting_task->suspension_states = 0x00;
	setting_task->deletion_states = 0xFFFF & ~PROGRAMMING;

	setting_task->task_period = 20;
	setting_task->task_args = NULL;

	return UV_OK;

}


/** @brief this function does one thing, and one thing only, it checks if we have custom settings, then it attempts to get them.
 * If it fails, then we revert to factory defaults.
 *
 */

uv_status uvSettingsInit() PRIVILEGED_FUNCTION{

	insertCANMessageHandler(0x520,handleIncomingLaptopMsg); //Allows us to talk with laptop

	current_vehicle_settings = uvMalloc(sizeof(uv_vehicle_settings));


	if(current_vehicle_settings == NULL){
		//HMMM
		__uvInitPanic(); // deeply unfortunate
		return UV_ERROR;
	}


	//uvConfigSettingTask(current_vehicle_settings);

	//First we check the validity of the settings in flash.
	bool use_factory_default = true;
	bool force_flash_rewrite = false;

	//Check for existing valid flash settings
	uv_status retval = uvValidateFlashSettings();

	if(retval == UV_OK){ //If the thing responds with OK, then we attempt to load flash settings

		if(uvLoadSettingsFromFlash() == UV_OK){
			//Attempt to load flash settings. If that somehow fails, revert to factory defaults
			use_factory_default = false;
		}else if(uvLoadSettingsFromFlash()== UV_OK){
			//Could not actually load from flash. BAD!
			//In this case we would like to revert to factory defaults!
			use_factory_default = true;
		}

		//In this case, we need to check to see if we need to send out a msg for the VCU
		if((uint32_t)(*(START_OF_USER_FLASH + 4))){

			if(uvSendCanMSG(&vcu_ack_msg)!=UV_OK){
				return UV_ERROR;
			}
		}


	}else{
		use_factory_default = true;
		force_flash_rewrite = true;
	}

	if(use_factory_default == true){
		if(setupDefaultSettings() == UV_OK){
			//great success.
		}else if(setupDefaultSettings() != UV_OK){
			//FAILURE TO EVEN LOAD THE DEFAULTS, THIS CAR IS UNDRIVEABLE
			__uvInitPanic();
			return UV_ERROR;
		}
	}

	if(force_flash_rewrite == true){
		void* tmp_sblock = uvMalloc(SETTING_BRANCH_SIZE);
		if(uvResetFlashToDefault(tmp_sblock)==UV_OK){
			//Things to do upon success (this thing is somewhat self explanitory
		}else{

		}

		uvFree(tmp_sblock);
	}

	setupDefaultSettings();// Temporary and forces default settings
	return UV_OK;

}


/** @brief Updates a setting in a temporary SBlock by memory group and offset
 *
 */
uv_status uvUpdateTmpSettings(uint8_t* tmp_settings, uint8_t memgroup, uint8_t m_offset, uint8_t size, uint8_t* data){
	if(!tmp_settings){
		return UV_ERROR;
	}

	if(memgroup > 12){
		//out of range
		return UV_ERROR;
	}

	/** This indicates that you are attempting to set a setting that bridges the 128 byte "row"
	 * system the STM32F40xx uses to organise the flash memory.
	 * Attempting to write a parameter like this will result in a busfault.
	 */
	if(((m_offset + size)%128)<size){

		return UV_ERROR;
	}

	//Check for certain illegal combinations of things below:
	//This first case is to see if it is editing the reserved area of the sblock.
	if((memgroup == 0)&&(m_offset < 32)){
		return UV_ERROR;
	}

	uint32_t offset = memgroup*256 + m_offset;

	for(int i = 0; i<size; i++){
		*(tmp_settings + offset + i) = *(data + i); //ensure correct number comes through. //CONFIRMED THIS WORKS IF LAPTOP SENDS LITTLE ENDIAN
	}



	return UV_OK;
}

//NOTE: I Have not finished implementing the checksums here.
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
		return UV_ERROR;
	}

	uvComputeSettingsChecksums(sblock, csums);
	return UV_OK;

}

/** @brief Function to call when the Sblock Flash has become corrupted somehow.
 *
 */
void uvSBlockCorruptionHandler() PRIVILEGED_FUNCTION{
	//We need to forcibly revert to factory default. Unsure about ability to write to flash.
	void* addr = FLASH_SBLOCK_START;
	bool successful_write = false;

	taskENTER_CRITICAL();

	for(int i = 0; i<3; i++){
		if(HAL_FLASH_Unlock() == HAL_OK){
			break;
		}
	}

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

	//Set the start of the Sblock to 0, therefore causing uvValidate
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)addr,0x00000000)!= HAL_OK){
		//If this does not work, then more drastic measures may be needed.
		//Manually erase entire sector in hardware.
		//Note that this will also destroy any persistent data, but we have genuinely much bigger fish to fry at this point, like the literal vehicle settings are corrupted WTF do you want from me, man?
		while((FLASH->SR) & FLASH_SR_BSY){
				//Wait till no longer busy
		}

		FLASH->CR |= (FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_3); // Sector 11
		FLASH->CR |= FLASH_CR_SER; //Queue up a flash erase operation

		FLASH->CR |= FLASH_CR_STRT; //Tells the hardware to begin flash clear procces



		while((FLASH->SR) & FLASH_SR_BSY){
				//Wait till no longer busy
		}
	}

	HAL_FLASH_Lock();//Always remember to lock the flash.

	taskEXIT_CRITICAL();


};


uv_status uvOverwriteCsr(void* sblock, uint32_t* csr){

	return UV_OK;
}

/** These defines are for the return error code of the next function
 *
 */
#define FLASH_OK 0
#define INVALID_SBLOCK 1
#define FLASH_NOT_UNLOCKED 2
#define DID_NOT_FINISH_PROGRAMMING 3
#define DATA_MISMATCH 4
#define PRE_CHECKSUM 5
#define POST_CHECKSUM 6

/** @brief Function to save a finished settings block to flash memory
 *
 */
uv_status uvSaveSettingsToFlash(void* sblock, uint32_t* ecode) PRIVILEGED_FUNCTION{
	if(sblock == NULL){
		*ecode = INVALID_SBLOCK;
		return UV_ERROR;
	}

	void* tmp = sblock;
//	if(uvValidateChecksums(tmp) != UV_OK){
//		return UV_ABORTED;
//	}

	*((uint32_t*)(tmp + 0)) = MAGIC_NUMBER; //Identifies that this is in fact a valid S_Block
	*((uint32_t*)(tmp + 4)) = 0x00000001; //Little reminder for future VCU that the settings were recently changed
	*((uint16_t*)(tmp + 8)) = 0x1000;
	*((uint16_t*)(tmp + 10)) = FIRMWARE_MAJOR_RELEASE; //Identify the firmware version this is
	*((uint16_t*)(tmp + 12)) = FIRMWARE_MINOR_RELEASE; //Identify the firmware version
	*((uint16_t*)(tmp + 14)) = FIRMWARE_PATCH_NUM;

	//We save the sizes of the core setting structs, so that if someone adds or removes settings from the struct,
	//The program will notice and load factory defaults into flash
	*((uint8_t*)(tmp + 16)) = sizeof(veh_gen_info);
	*((uint8_t*)(tmp + 17)) = sizeof(uv_os_settings);
	*((uint8_t*)(tmp + 18)) = sizeof(motor_controller_settings);
	*((uint8_t*)(tmp + 19)) = (uint8_t)sizeof(driving_loop_args); //IDGAF if we got truncation error here, fuck you
	*((uint8_t*)(tmp + 20)) = sizeof(uv_imd_settings);
	*((uint8_t*)(tmp + 21)) = sizeof(bms_settings_t);
	*((uint8_t*)(tmp + 22)) = sizeof(daq_loop_args);
	*((uint8_t*)(tmp + 23)) = sizeof(daq_datapoint);
	*((uint8_t*)(tmp + 24)) = sizeof(output_channel_settings);



	void* addr = FLASH_SBLOCK_START;

	if(HAL_FLASH_Unlock() != HAL_OK){
		*ecode = FLASH_NOT_UNLOCKED;
		return UV_ERROR;
	}


	taskENTER_CRITICAL(); //I would very much prefer if nothing fucks with this


	while((FLASH->SR) & FLASH_SR_BSY){
		//Wait till no longer busy
	}

	FLASH->CR |= (FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_3); // Sector 11
	FLASH->CR |= FLASH_CR_SER; //Queue up a flash erase operation

	FLASH->CR |= FLASH_CR_STRT; //Tells the hardware to begin flash clear procces



	while((FLASH->SR) & FLASH_SR_BSY){
		//Wait till no longer busy
	}

	while(addr < TOP_OF_FLASH_SBLOCK){ //TODO Is this the right address?
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)addr,(uint32_t)(*(uint32_t*)tmp))!= HAL_OK){
			//ERROR ERROR AHHHH FUCK
			//uvPanic("FLASH_WRITE_ERROR");
			*ecode = DID_NOT_FINISH_PROGRAMMING;
			HAL_FLASH_Lock();
			return UV_ERROR;
		}
		addr += 4;
		tmp += 4;
	}

	//If we reach this point then the BSY bit of FLASH_SR register is not set

	//Clear these error flags
//	FLASH->SR |= (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR);
//
//	FLASH->CR |= (FLASH_CR_PSIZE_0 | FLASH_CR_PSIZE_1); //64 byte alignment
//
//
//	FLASH->CR |= FLASH_CR_PG; // Program enable



//	while(addr < TOP_OF_FLASH_SBLOCK){
//		while((FLASH->SR & FLASH_SR_BSY)==1){
//			//Dont be busy
//		}
//		*((uint64_t*)addr) = *((uint64_t*)tmp);
//
//		addr += 8;
//		tmp += 8;
//
//		if(FLASH->SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR)){
//			//An error bit has been set.
//			*ecode = DID_NOT_FINISH_PROGRAMMING; //UHH OH
//		}
//	}
//
//	while((FLASH->SR) & FLASH_SR_BSY){
//			//Wait till no longer busy
//	}

//	FLASH->CR &= (~FLASH_CR_PG); //Disable the programming
//
//	while((FLASH->SR) & FLASH_SR_BSY){
//			//Wait till no longer busy
//	}
//
	FLASH->CR |= FLASH_CR_LOCK; //LOCK FLASH


	taskEXIT_CRITICAL();

//	HAL_FLASH_Lock();

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

#define FLASH_OK 0
#define INVALID_SBLOCK 1
#define FLASH_NOT_UNLOCKED 2
#define DID_NOT_FINISH_PROGRAMMING 3
#define DATA_MISMATCH 4
#define PRE_CHECKSUM 5
#define POST_CHECKSUM 6



/** @brief Function that creates a
 *
 */



/** @brief This reminds the VCU to shoot off it's own message later when it reboots.
 *
 * @deperecated
 */
uv_status uvSetSettingResponseReminder(uint32_t reminder){
	if(HAL_FLASH_Unlock() != HAL_OK){
			return UV_ERROR;
		}

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(START_OF_USER_FLASH + 4),(uint32_t)&reminder)!=HAL_OK){
			HAL_FLASH_Lock();
			return UV_ERROR;
		}

		HAL_FLASH_Lock();
		return UV_OK;
}

/** @brief
 *
 * @deprecated
 *
 */
uv_status uvForceDefaultReversionUponDeviceReset(){
	uint32_t word = 0;

	if(HAL_FLASH_Unlock() != HAL_OK){
		return UV_ERROR;
	}

	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,(uint32_t)(START_OF_USER_FLASH),(uint32_t)&word)!=HAL_OK){
		HAL_FLASH_Lock();
		return UV_ERROR;
	}

	HAL_FLASH_Lock();
	return UV_OK;
}



/** @brief Function that creates a temporary copy of the existing settings
 *
 */
void* uvCreateTmpSettingsCopy(){

	uint8_t* sblock = uvMalloc(SETTING_BRANCH_SIZE);

	if(sblock == NULL){
		return NULL;

	}


	uint8_t* tmp = (uint8_t*)(START_OF_USER_FLASH);

	for(int i = 0; i < SETTING_BRANCH_SIZE; i++){

		if(uvIsPTRValid(sblock + i)!= UV_OK){
			void* a = sblock + i;
			while(1){

			}
		}

		if(uvIsPTRValid(tmp + i)!= UV_OK){
			void* a = tmp + i;
			while(1){

			}
		}


		*(sblock + i) = *(tmp + i);

	}

	return (void*)sblock;

}


/** @brief Function that locates a specific setting parameter and sends the value over canbus
 *
 */
uv_status uvSendSpecificParam(uint8_t* origin, uint8_t mgroup, uint8_t m_offset, uint8_t size){
	uv_CAN_msg blank_msg;
	blank_msg.flags = 0x00;
	blank_msg.msg_id = 0x420;
	blank_msg.data[0] = 0b00100000 | mgroup;
	blank_msg.data[1] = m_offset;
	blank_msg.dlc = 2 + size;

	uint8_t* ptr = START_OF_USER_FLASH + 256*mgroup + m_offset;

	for(int i = 0; i<4 ;i++){
		blank_msg.data[2+i] = *(ptr+i);
	}

	if(uvSendCanMSG(&blank_msg) == UV_OK){
		return UV_OK;
	}

	return UV_ERROR;
}

/** @brief Function that looks into the flash memory of the VCU, and determines if a valid set of settings exists
 *
 * This checks various parameters to ensure that the settings are useable.
 * The first thing it looks at is the very first word of the SBLOCK. If the first
 * word is 0x42069420, then this is a real sblock.
 *
 * The second thing it checks is the firmware version, which is stored immediately after the first word,
 * and 16 bytes of padding. These 3 16 bit ints need to be correct, in order for it to proceed. This prevents
 * settings made for incompatible firmware versions from being used.
 *
 */
uv_status uvValidateFlashSettings(){
	void* tmp = START_OF_USER_FLASH;
	if(*((uint32_t*)tmp) != MAGIC_NUMBER){
		return UV_ERROR;// Nice blank slate
	}

	//NEXT we check the firmware number, and that nobody screwed around with the struct contents


	//if any of these dont match, then the firmware does not match the settings block
	if(*((uint8_t*)(tmp + 16)) != sizeof(veh_gen_info)){
		return UV_ERROR;
	}

	if(	*((uint8_t*)(tmp + 17)) != sizeof(uv_os_settings)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 18)) != sizeof(motor_controller_settings)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 19)) != (uint8_t)(sizeof(driving_loop_args))){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 20)) != sizeof(uv_imd_settings)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 21)) != sizeof(bms_settings_t)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 22)) != sizeof(daq_loop_args)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 23)) != sizeof(daq_datapoint)){
		return UV_ERROR;
	}

	if(*((uint8_t*)(tmp + 24)) != sizeof(output_channel_settings)){
		return UV_ERROR;
	}

	return UV_OK;
}

/** @brief This is a task that executes the functionality necessary for vehicle diagnostics
 *
 * This task is only active once the car has been placed in programming mode.
 *
 */
void uvSettingsProgrammerTask(void* args) PRIVILEGED_FUNCTION{
	uv_task_info* params = (uv_task_info*) args;

	//Cache these vars for future usage
	uint8_t mgroup = 0;
	uint8_t m_offset = 0;
	uint8_t d_type = 0;
	uint8_t d_size = 0;

	uint8_t cmd_byte = 0;

	uint8_t* origin = NULL;

	uv_CAN_msg tmp_msg; //messages get written into here when

	//Allocates memory for the temporary settings that will be edited
	void* new_tmp_settings = uvCreateTmpSettingsCopy();

	if(new_tmp_settings == NULL){
		//Did not create temp settings >:(
		uvSendCanMSG(&vcu_ack_failed_msg); //Let laptop know of our failures
	}

	ptask_handle = params->task_handle;


	//uvSendCanMSG(&vcu_ack_msg);


	settings_queue = xQueueCreate(8, sizeof(uv_CAN_msg));

	if(settings_queue == NULL){
		uvPanic("Settings Queue Not Created",0);
	}




	//Cached temporary variables for incoming programming messages


	uvSendCanMSG(&vcu_ack_msg); //successfully entered the task


	for(;;){
		if(xQueueReceive(settings_queue,&tmp_msg,100)!=pdTRUE){
			//laptop connection lost. Yowza. Discard whatever you're doin.

//			if((xTaskGetTickCount() - last_contact_with_laptop) > 200){
//				//more than 200ms have passed since last communication.
//				changeVehicleState(UV_READY); // leave the task
//
//			}

			/* WARNING: This is a goto. I know goto is prety heinous.
			 * Fucking learn to deal with it u lil bitch boy
			 */
			goto __END_OF_SETTING_TASK_LOOP__;
		}

		cmd_byte = tmp_msg.data[0];

		switch(cmd_byte){
			case SET_SPECIFIC_PARAM ... END_OF_SPECIFIC_PARAMS:
				mgroup = (cmd_byte &(0b00011111));
				m_offset = tmp_msg.data[1];
				d_type = tmp_msg.data[2];
				d_size = data_size[d_type];


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

				//This is not yet implemented.
				uvSendCanMSG(&vcu_ack_failed_msg);
				break;
			case REQUEST_JOURNAL_ENTRIES_BY_TIME:
				//This is not yet implemented

				uvSendCanMSG(&vcu_ack_failed_msg);
				break;
			case REQUEST_SPECIFIC_SETTING:
				//We wish to know the value of a specific setting

				mgroup = tmp_msg.data[2];
				m_offset = tmp_msg.data[3];
				d_type = tmp_msg.data[4];
				d_size = data_size[d_type];

				origin = (tmp_msg.data[1])? FLASH_SBLOCK_START : new_tmp_settings;


				if(uvSendSpecificParam(origin,mgroup,m_offset,d_size) != UV_OK){
					//HANDLE THIS ERROR HERE
				}

				break;
			case SAVE_AND_APPLY_NEW_SETTINGS:
				//Save and apply. Hoo boy.
				uint32_t ecode = 0;
				if(uvSaveSettingsToFlash(new_tmp_settings, &ecode) == UV_OK){
					//Try once

					uvSendCanMSG(&vcu_ack_msg); //report success
				}else if(uvSaveSettingsToFlash(new_tmp_settings, &ecode) == UV_OK){
					//Try again
					uvSendCanMSG(&vcu_ack_msg);
				}else{
					//This is a tricky one. High probability of data corruption.

					if(ecode == INVALID_SBLOCK){
						//Womp womp, the SBLOCK you provided is not ok
					}else if(ecode == FLASH_NOT_UNLOCKED){
						//Failure to unlock flash. Why?
					}else if(ecode == DID_NOT_FINISH_PROGRAMMING){
						//Partially programmed. Indicates data has been corrupted.
						//FORCE RESET, CANNOT CONTINUE
					}else if(ecode == DATA_MISMATCH){
						//Data in flash does not match what has been programmed
						//FORCE RESET, CANNOT CONTINUE
					}else if(ecode == PRE_CHECKSUM){
						//Checksum failure of sblock prior to save attempt
					}else if(ecode == POST_CHECKSUM){
						//Checksum failure of flash, post save - CORRUPTION
					}else if(ecode != 0){
							/* This section of code is one of the worst places to be in the vehicle.
							 *
							 */
					}
				}


				break;
			case DISCARD_NEW_SETTINGS:
				if(uvFree(new_tmp_settings) == UV_OK){
					new_tmp_settings = uvCreateTmpSettingsCopy();


					if(new_tmp_settings != NULL){
						uvSendCanMSG(&vcu_ack_msg);
						//This means that we have succeeded, as we have deleted the old settings,
						//and new settings have taken their place

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

				//uvFree(new_tmp_settings); //No need to free, this happens in the destructor
				changeVehicleState(UV_READY);
				uvSendCanMSG(&vcu_ack_msg);
				break;
			case FORCE_RESTORE_FACTORY_DEFAULT:

				if(uvResetFlashToDefault(new_tmp_settings) == UV_OK){
					//Success
				}else{
					//failure

				}


				break;
			default:
				//unrecognized command spotted
				break;
		}



		__END_OF_SETTING_TASK_LOOP__:


		if(params->cmd_data == UV_KILL_CMD){
			//TODO add destructors here

			if(new_tmp_settings != NULL){
				uvFree(new_tmp_settings);
			}


			killSelf(params); //Me too programming and diagnostics task, me too
			while(1){ //hang

			}
		}

	}

//	settingCopy((uint8_t*)(&default_os_settings),new_sblock + OS_SETTINGS_OFFSET,sizeof(uv_os_settings));
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);


	return UV_OK;

}







uv_status uvResetFlashToDefault(void* new_sblock){
	//void* new_sblock = uvMalloc(SETTING_BRANCH_SIZE);

	if(new_sblock == NULL){
		return UV_ERROR; //definately not ideal
	}

	for(int i = 0; i<SETTING_BRANCH_SIZE; i += 4){
		*((uint32_t*)(new_sblock + i)) = 0xFFFFFFFF;
	} //Become Zeros

	*((uint32_t*)(new_sblock)) = MAGIC_NUMBER;

	//uint32_t pdiff = ((uint32_t)new_sblock) - ((uint32_t)START_OF_USER_FLASH);

	settingCopy((uint8_t*)(&default_vehicle),new_sblock + GENERAL_VEH_INFO_OFFSET,sizeof(veh_gen_info));

	//OS
	settingCopy((uint8_t*)(&default_os_settings),(new_sblock + OS_SETTINGS_OFFSET ),sizeof(uv_os_settings));

	//Motor controller
	settingCopy((uint8_t*)(&mc_default_settings),(new_sblock+256*MOTOR_MGROUP+MOTOR_OFFSET),sizeof(motor_controller_settings));

	//Driving Loop
	settingCopy((uint8_t*)(&default_dl_settings),(new_sblock+256*DRIVING_MGROUP+DRIVING_OFFSET),sizeof(driving_loop_args));

	//BMS
	settingCopy((uint8_t*)(&default_bms_settings),(new_sblock+256*BMS_MGROUP+BMS_OFFSET),sizeof(bms_settings_t));

	//IMD
	settingCopy((uint8_t*)(&default_imd_settings),(new_sblock+256*IMD_MGROUP+IMD_OFFSET),sizeof(uv_imd_settings));

	//PDU
	settingCopy((uint8_t*)(&default_output_channels),new_sblock + 256*PDU_MGROUP + PDU_OFFSET,sizeof(output_channel_settings));


	//DAQ Head + Meta settings
	settingCopy((uint8_t*)&default_daq_settings,(new_sblock+256*DAQ_HEAD_MGROUP+DAQ_HEAD_OFFSET),sizeof(daq_loop_args));

	//Settings Being Logged
	settingCopy((uint8_t*)(default_datapoints),(new_sblock + 256*DAQ_PARAMS1_MGROUP + DAQ_PARAMS1_OFFSET),sizeof(uv_imd_settings)*default_daq_settings.total_params_logged);
//	settingCopy(0,0,0);
//	settingCopy(0,0,0);


	uint32_t ecode = 0;
	if(uvSaveSettingsToFlash(new_sblock, &ecode) == UV_OK){
			//Try once
		return UV_OK;
		//uvSendCanMSG(&vcu_ack_msg); //report success
	}else if(uvSaveSettingsToFlash(new_sblock, &ecode) == UV_OK){
		//Try again
		///uvSendCanMSG(&vcu_ack_msg);
		return UV_OK;
	}

	//Once we reach this point we know that some error has in fact occurred.
	if(ecode == INVALID_SBLOCK){

	}else if(ecode == FLASH_NOT_UNLOCKED){

	}else if(ecode == DID_NOT_FINISH_PROGRAMMING){

	}else if(ecode == DATA_MISMATCH){

	}else if(ecode == PRE_CHECKSUM){

	}else if(ecode == POST_CHECKSUM){

	}else if(ecode != 0){
		/* This section of code is one of the worst places to be in the vehicle.
		 *
		 */
	}

	//If we reach this point, then both attempts at uvSaveSettingsToFlash have failed
	//This is not a good situation to be in, since data corruption may have occurred.


	return UV_ERROR;

}

/** @brief Helper function for sending all settings
 *
 */
uv_status uvSendSettingGroup(uint8_t* origin,uint8_t memgroup,uint8_t m_offset, uint16_t size){
	uint16_t newsize = size + size%4;

	//We always send the 32 bit word, because the VCU does not track what the individual types of struct members are.
	//Doing so would be stupid complicated, so not gonna bother.
	for(int i = 0; i<newsize; i+=4){
		//Send the actual msg
		if(uvSendSpecificParam(origin,memgroup,m_offset + i,4)!=UV_OK){
			return UV_ERROR;
		}

		//Find a way to await an ack message
	}

	return UV_OK;
}

/** @brief Sub-task that sends all settings from the VCU to the
 *
 */
void sendAllSettingsWorker(void* args){
	setting_helper_task* params = (setting_helper_task*) args;
	child_task_handle = params->meta_task_handle;


	uint8_t* origin = params->args.setting_tx_args.sblock_origin;
	if(!origin){
		origin = FLASH_SBLOCK_START;
	}

	params->status = UV_OK;

	child_task_active = true;



	//Get through all this stuff first
	if(uvSendSettingGroup(origin,GENERAL_VEH_INFO_MGROUP, GENERAL_VEH_INFO_OFFSET, sizeof(veh_gen_info))!= UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,OS_SETTINGS_MGROUP, OS_SETTINGS_OFFSET, sizeof(uv_os_settings))!= UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,MOTOR_MGROUP,MOTOR_OFFSET,sizeof(motor_controller_settings))!= UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,DRIVING_MGROUP,DRIVING_OFFSET,sizeof(driving_loop_args))!= UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,IMD_MGROUP,IMD_OFFSET,sizeof(uv_imd_settings))!=UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,BMS_MGROUP,BMS_OFFSET,sizeof(bms_settings_t))!= UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,DAQ_HEAD_MGROUP,DAQ_HEAD_OFFSET,sizeof(daq_loop_args))!=UV_OK){
		//Handle this error
	}else if(uvSendSettingGroup(origin,PDU_MGROUP,PDU_OFFSET,sizeof(output_channel_settings)) != UV_OK){
		//Handle this error
	}

	//Next is whatever in god's green earth the DAQ_datapoints are up to (Me when the edge case hits just right)
	uint16_t remaining_bytes = (((daq_loop_args*)(origin + 256*DAQ_HEAD_MGROUP + DAQ_HEAD_OFFSET))->total_params_logged)*sizeof(daq_datapoint);

	uint8_t t_mgroup = DAQ_PARAMS1_MGROUP;

	while(remaining_bytes > 256){
		if(uvSendSettingGroup(origin,t_mgroup,DAQ_PARAMS1_OFFSET,256)!=UV_OK){
			//Handle this error
		}

		remaining_bytes -= 256;
		t_mgroup++;
	}

	if(uvSendSettingGroup(origin,t_mgroup,DAQ_PARAMS1_OFFSET,remaining_bytes)!=UV_OK){
		//Handle this error
	}


	child_task_active = false;

	vTaskSuspend(NULL);
}

/** @brief Sub-task that reads out the journal
 *
 * @attention EVENT LOGGING IS NOT IMPLEMENTED YET, DO NOT TOUCH THIS
 */
void sendJournalWorker(void* args){

}


