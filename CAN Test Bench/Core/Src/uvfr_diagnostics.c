/*
 * uvfr_diagnostics_system.c
 *
 *  Created on: Jun 14, 2025
 *      Author: byo10
 */
#define __UV_FILENAME__ "uvfr_diagnostics.c"
#include "uvfr_utils.h"

extern HeapStats_t xHeapStats;

void uvBackgroundDiagnosticsDaemon(void* args){
	uv_task_info* params = (uv_task_info*) args;

	for(;;){
		vTaskDelay(100);

		vPortGetHeapStats(&xHeapStats);
		printf("TEEHEE\n");

		if(params->cmd_data == UV_KILL_CMD){
			killSelf(params);
		} else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params);
		}

	}
}

uv_status uvEnterDiagnosticMode(){
	return UV_OK;
}

uv_status uvExitDiagnosticMode(){
	return UV_OK;
}

/** @brief Initialize the diagnostics of the vehicle
 *
 */
uv_status uvInitDiagnostics(){
	uint32_t var = 0;
	uv_task_info* diag_task = uvCreateServiceTask();
	diag_task->task_function = uvBackgroundDiagnosticsDaemon;
	diag_task->active_states = 0xFFFF;
	diag_task->task_name = "diagDaemon";


	uvStartTask(&var,diag_task);
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


/** overrides the weak function prototype of __io_putchar so that we can use the
 * ITM registers to send diagnostic data back to a debugger
 *
 */
int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
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
