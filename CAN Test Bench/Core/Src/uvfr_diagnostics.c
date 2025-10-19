/*
 * uvfr_diagnostics_system.c
 *
 *  Created on: Jun 14, 2025
 *      Author: byo10
 */

#include "uvfr_utils.h"

uv_status uvEnterDiagnosticMode(){
	return UV_OK;
}

uv_status uvExitDiagnosticMode(){
	return UV_OK;
}

uv_status uvInitDiagnostics(){
	return UV_OK;
}



void handleDiagnosticMsg(uv_CAN_msg* msg){

	uint8_t cmd_byte = msg->data[0];
	switch(cmd_byte){
	case ENTER_DIAGNOSTICS_MODE:

		break;
	case EXIT_DIAGNOSTICS_MODE:

		break;
	case REQUEST_STATE_CHANGE:
		if(changeVehicleState(msg->data[1]<<8 | msg->data[2]) != UV_OK){

		}
		break;
	case FORCE_STATE_CHANGE:

		break;
	case dCLEAR_FAULTS:

		break;
	default:

		break;
	}
}




void uvEventLogDaemon(void* args){

	for(;;){

	}
}

void uvAssertFailed(char* file, uint16_t line, TaskHandle_t task, char* condition){

}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ){
	//This is where we end up if one of the tasks has a stack overflow

	//STOP THE CAR

	//LOG WHAT HAPPENED

	//What task did it

	//How badly did it overflow?

	//Try to get a trace!

	//Hang:

	for(;;){

	}
}

void vApplicationMallocFailedHook(){
	//pvPortMalloc has failed

	//STOP THE CAR

	//LOG WHAT HAPPENED

	//What task did it

	//Hang
}


void vApplicationTickHook( void ){
	//This is not used but it makes the compiler STFU
}
