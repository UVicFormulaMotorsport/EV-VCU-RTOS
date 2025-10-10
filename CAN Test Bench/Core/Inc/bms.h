// Code to make human readable CAN messages for the device



#ifndef _BMS_H_
#define _BMS_H_

#include "main.h"
#include "uvfr_utils.h"
#include <stdint.h>
#include <stdbool.h>


#define DEFAULT_BMS_CAN_TIMEOUT ((uv_timespan_ms)200)


// Runtime state (telemetry)
typedef struct {

	uint16_t pack_current_dA; // 0.1 A units
	uint16_t pack_voltage_dV; // 0.1 V units
	uint16_t soc_pct; // 0-100%
	uint16_t relayState;
	bool msg1corrupt; //checksum

	uint16_t dcl_dA; // 0.1 A max discharge current limit
	uint16_t min_cell_temp; // celsius
	uint16_t max_cell_temp; // celsius
	bool msg2corrupt; // checksum

} bms_state_t;

extern volatile bms_state_t g_bms_state; // volatile as these values may update as the car runs
// config / bounds

typedef struct { //TODO Needs populating
	uint32_t BMS_CAN_timeout;
	uint32_t max_temp;

	// "_c" for temperature in C\
	// "_pct" for percentage value
	int16_t discharge_cold_fault_c; // -20
	int16_t discharge_cold_warn_c;

	int16_t charge_cold_fault_c;
	int16_t charge_hot_fault_c;
	int16_t hot_warn_c;
	int16_t hot_fault_c;

	int16_t temp_plaus_min_c;
	int16_t temp_plaus_max_c;

	uint8_t soc_low_warn_pct;
	uint8_t soc_regen_disable_pct;

	uint16_t derate_start_c;

	// extern uint16_t (var name) to call outside this file
} bms_settings_t;

extern bms_settings_t g_bms_settings;


void BMS_Init(void* args);
void BMS_OnMSg_0x6B0(uv_CAN_msg* msg);
void BMS_OnMSg_0x6B1(uv_CAN_msg* msg);

#endif



