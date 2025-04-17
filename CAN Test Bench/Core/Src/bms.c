// This where the code to handle BMS errors and such will go

#include "main.h"
#include "bms.h"
#include "constants.h"
#include "pdu.h"
#include "can.h"
#include "tim.h"
#include "dash.h"

//two IDs for bms can message
// 0x6B0
// 0x6B1


void BMS_msg1(uv_CAN_msg* msg){ // msg is raw CAN msg, gets processed in voltageCANdata
	// msg1 can handle current, voltage, charge, relay, checksum (corruption)
	packCurrent = (msg->data[0] | msg->data[1]);
	packVoltage = (msg->data[2]);
	stateOfCharge = (msg->data[4]);
	relayState = (msg->data[6]);
	corrupt = (msg->data[7]);


	// would make sense for voltageCANdata to turn into float
	//log voltage data to where, maybe setting global variable
}

void BMS_msg2(uv_CAN_msg* msg){
	// msg2 can handle
	//obtain charge data from CAN message
	//does this needs to be constantly ran? to constantly monitor / update percentage on dash
	//example:
		//voltageCANdata = (msg->data[0] | (msg->data[1] <<8))
		// maybe save this to a global variable somewhere as a float
		// float charge = voltageCANData processed to a float
	// if (charge < 20.0) {
		// flash low battery warning to screen
	// if (charge > 95) {
		// regenBraking = false;
	// in all other cases battery should be functioning as normal

}

void BMS_tempurature(){
	// obtain tempurature data
	// tempCANData = (msg->data[0] | (msg->data[1] <<8)) - processing CAN msg NEED TO CHANGE
	// temp should also be a float to one decimal pt - so this is easier to manage
	// send to global as well?

	// if ( temp <= -5.0 ) {
		// warning - temp is too low
		// stop operability?

	// elif (temp > -5.0 && temp < 60.0) {
		// normal operating tempurature range
		// drive as normal
		// if (temp >= 55.0) {{
			// flash warning - approaching max temp
			// maybe enable cooling fan/devices here
		// }
	// }

}

void BMS_Init(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;

	osDelay(200);

	uv_init_task_response response = {UV_OK,BMS,0,NULL};

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
			//OOPS
	}

	insertCANMessageHandler(251, BMS_logVoltage);
	insertCANMessageHandler(252, BMS_stateOfCharge);
	insertCANMessageHandler(253, BMS_tempurature);

		//Kill yourself
	vTaskSuspend(params->meta_task_handle);
}





