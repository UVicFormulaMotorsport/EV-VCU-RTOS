// This where the code to handle BMS errors and such will go
//Quazi Heider

#include "main.h"
#include "bms.h"
#include "constants.h"
#include "pdu.h"
#include "can.h"
#include "tim.h"
#include "dash.h"
#include <string.h>
#include <stdint.h>



//two IDs for bms can message
// 0x6B0
// 0x6B1

volatile bms_state_t g_bms_state = {0};

#define curr_bms_settings (*(current_vehicle_settings->bms_settings))

bms_settings_t default_bms_settings = {
		.BMS_CAN_timeout = 100,

		.discharge_cold_fault_c = -20,
		.discharge_cold_warn_c  = -10,

		.charge_cold_fault_c    = 0,
		.charge_hot_fault_c     = 45,

		.discharge_hot_warn_c             = 50,
		.discharge_hot_derate_c           = 55,
		.discharge_hot_fault_c            = 60,

		.temp_plaus_min_c       = -40,
		.temp_plaus_max_c       = 100,

		.soc_low_warn_pct       = 20,
		.soc_regen_disable_pct  = 95,

		.derate_start_c         = 55,

		//Temps (need to be tuned to cell datasheet and comp-specific rules)


};
void BMS_msg1(uv_CAN_msg* msg){ // msg is raw CAN msg, gets processed in voltageCANdata
	// msg1 can handle current, voltage, charge, relay, checksum (corruption)
	g_bms_state.pack_current_dA = (msg->data[0]<<8 | msg->data[1]); // x 0.1A
	g_bms_state.pack_voltage_dV = (msg->data[2]<<8 | msg->data[3]); // x 0.1V

	g_bms_state.soc_pct = (msg->data[4])*2; // x2; this is an int

	g_bms_state.relayState = (msg->data[5]<<8 | msg ->data[6]);
	g_bms_state.msg1corrupt = (msg->data[7]);
	if (g_bms_state.soc_pct < curr_bms_settings.soc_low_warn_pct) {
		// flash low battery warning to screen
	}
	if (g_bms_state.soc_pct >= curr_bms_settings.soc_regen_disable_pct) {
		// disable regenerative breaking
	}
	// in all other cases battery should be functioning as normal



	// would make sense for voltageCANdata to turn into float
	//log voltage data to where, maybe setting global variable
}

void BMS_msg2(uv_CAN_msg* msg){
	// msg2 can handle DCL (Max current output), tempurature, checksum
	g_bms_state.dcl_dA = (msg->data[0]<<8 | msg->data[3]);
	g_bms_state.min_cell_temp = (msg->data[4]); // celsius
	g_bms_state.max_cell_temp = (msg->data[5]); // celsius
	g_bms_state.msg2corrupt = (msg->data[7]);

	 if (g_bms_state.min_cell_temp <= curr_bms_settings.discharge_cold_fault_c ) {
		// warning - temp is too low
		// stop operability?

		 }
	 if (g_bms_state.min_cell_temp > curr_bms_settings.discharge_cold_fault_c && g_bms_state.max_cell_temp < curr_bms_settings.discharge_hot_fault_c){
//		 normal operating tempurature range
//		 drive as normal
		 }
	 if (g_bms_state.max_cell_temp >= curr_bms_settings.discharge_hot_fault_c) {
//			 flash warning - approaching max temp
//			 maybe enable cooling fan/devices here
		 }
	}



void BMS_Init(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;

	osDelay(200);

	uv_init_task_response response = {UV_OK,BMS,0,NULL};

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
			//OOPS
	}

	insertCANMessageHandler(0x6B0, BMS_msg1);
	insertCANMessageHandler(0x6B1, BMS_msg2);

		//Kill yourself
	vTaskSuspend(params->meta_task_handle);
}







