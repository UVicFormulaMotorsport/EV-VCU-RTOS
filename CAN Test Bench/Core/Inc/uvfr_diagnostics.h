/*
 * uvfr_diagnostics_system.h
 *
 *  Created on: Jun 14, 2025
 *      Author: byo10
 */

#ifndef INC_UVFR_DIAGNOSTICS_H_
#define INC_UVFR_DIAGNOSTICS_H_

#define DIAGNOSTIC_RX_MSG_ID 0x421 //MSG from laptop or PCAN
#define DIAGNOSTIC_TX_MSG_ID 0x521 //MSG from VCU



#include<string.h>

#ifndef TEXTIFY
	#define TEXTIFY(A) #A
#endif



#define uvAssert( x ) if((x) == 0){\
	extern void uvAssertFailed(char* file, uint16_t line, TaskHandle_t task, char* condition);\
	TaskHandle_t ctask = xTaskGetCurrentTaskHandle();\
	uvAssertFailed(__UV_FILENAME__, __LINE__, ctask, TEXTIFY(x));\
}


typedef enum{
	ENTER_DIAGNOSTICS_MODE,
	EXIT_DIAGNOSTICS_MODE,
	REQUEST_STATE_CHANGE,
	FORCE_STATE_CHANGE,
	dCLEAR_FAULTS
}diagnostic_cmd;

typedef enum{
	LOG_FATAL_ERROR,
	LOG_ERROR,
	LOG_WARNING,
	LOG_STATE_CHANGE,
	LOG_GENERIC_EVENT
}logged_event_type;

uv_status uvInitDiagnostics();

uv_status logDiagnosticEvent(); //THIS NEEDS ARGS

#endif /* INC_UVFR_DIAGNOSTICS_H_ */
