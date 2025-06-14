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
