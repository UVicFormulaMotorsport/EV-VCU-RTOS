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


// causes syntax error but not sure why
bms_settings_t g_bms_settings = {
		.can_timeout_ms = DEFAULT_BMS_CAN_TIMEOUT,

		//Temps (need to be tuned to cell datasheet and comp-specific rules)

		.discharge_cold_fault_c = -20;
};
void BMS_msg1(uv_CAN_msg* msg){ // msg is raw CAN msg, gets processed in voltageCANdata
	// msg1 can handle current, voltage, charge, relay, checksum (corruption)
	packCurrent = (msg->data[0]<<8 | msg->data[1]); // x 0.1A
	packVoltage = (msg->data[2]<<8 | msg->data[3]); // x 0.1V

	stateOfCharge = (msg->data[4])*2; // x2; this is an int

	relayState = (msg->data[5]<<8 | msg ->data[6]);
	msg1corrupt = (msg->data[7]);
	if (stateOfCharge < 20) {
		// flash low battery warning to screen
	}
	if (stateOfCharge >= 95) {
		// disable regenerative breaking
	}
	// in all other cases battery should be functioning as normal



	// would make sense for voltageCANdata to turn into float
	//log voltage data to where, maybe setting global variable
}

void BMS_msg2(uv_CAN_msg* msg){
	// msg2 can handle DCL (Max current output), tempurature, checksum
	packDCL = (msg->data[0]<<8 | msg->data[3]);
	lowTemp = (msg->data[4]); // celsius
	highTemp = (msg->data[5]); // celsius
	msg2corrupt = (msg->data[7]);

	 if (lowTemp <= -5 ) {
		// warning - temp is too low
		// stop operability?

	 }
	 else if (lowTemp > -5 && highTemp < 55){
//		 normal operating tempurature range
//		 drive as normal
		 if (highTemp >= 55.0) {
//			 flash warning - approaching max temp
//			 maybe enable cooling fan/devices here
		 }
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







