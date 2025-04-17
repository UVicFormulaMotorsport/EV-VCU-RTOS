// This where the code to handle BMS errors and such will go

#include "main.h"
#include "bms.h"
#include "constants.h"
#include "pdu.h"
#include "can.h"
#include "tim.h"
#include "dash.h"
#include "uvfr_utils.h"

bms_settings_t default_bms_settings = {
	.BMS_CAN_timeout = 100,
	.max_temp = 60
};

void handleBMSVoltageMsg(uv_CAN_msg* msg){

	return;
}

void BMS_Init(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;

	osDelay(200);

	uv_init_task_response response = {UV_OK,BMS,0,NULL};

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
			//OOPS
	}




		//Kill yourself
	vTaskSuspend(params->meta_task_handle);
}





