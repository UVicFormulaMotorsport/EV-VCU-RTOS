/*
 * daq.h
 *
 *  Created on: Oct 15, 2024
 *      Author: byo10
 */

#ifndef INC_DAQ_H_
#define INC_DAQ_H_

#include "uvfr_utils.h"
#include "rb_tree.h"

#define _NUM_LOGGABLE_PARAMS



typedef enum{
	VCU_VEHICLE_STATE, /**< VCU Current Vehicle State */
	VCU_ERROR_BITFIELD1,
	VCU_ERROR_BITFIELD2,
	VCU_ERROR_BITFIELD3,
	VCU_ERROR_BITFIELD4,
	VCU_CURRENT_UPTIME,
	VCU_TOTAL_UPTIME,
	VEH_DISTANCE_RUN,
	VEH_DISTANCE_TOTAL,
	VEH_LAPNUM,
	VEH_SPEED,
	MOTOR_RPM, /**< RPM as reported by motor controller */
	MOTOR_TEMP, /**< Motor Temp as reported by motor controller */
	MOTOR_CURRENT, /**< Motor Phase currents as reported by motor controller */
	MC_VOLTAGE, /**< Pack voltage as measured by motor_controller*/
	MC_CURRENT, /**< Pack current as measured by motor_controller*/
	MC_TEMP, /**< Motor controller temperature*/
	MC_ERRORS, /**< Motor controller errors bitfield*/
	BMS_CURRENT, /**< Pack current measured by BMS*/
	BMS_VOLTAGE, /**< Pack voltage as measured by BMS*/
	BMS_ERRORS, /**< Error codes in BMS*/
	MAX_CELL_TEMP, /**< Max Temperature of a cell from BMS */
	MIN_CELL_TEMP, /**< Min Temperature of a cell*/
	AVG_CELL_TEMP,/**< Average Cell Temp*/
	ACCUM_SOC,/**< */
	ACCUM_SOH,/**< */
	ACCUM_POWER, /**< */
	ACCUM_POWER_LIMIT, /**< */
	APPS1_ADC_VAL, /**< */
	APPS2_ADC_VAL, /**< */
	BPS1_ADC_VAL, /**< */
	BPS2_ADC_VAL, /**< */
	ACCELERATOR_PEDAL_RATIO, /**< */
	BRAKE_PRESSURE_PA, /**< */
	POWER_DERATE_FACTOR, /**< */
	CURRENT_DRIVING_MODE, /**< */
	IMD_VOLTAGE, /**< Accumulator voltage as measured by IMD*/
	IMD_STATUS,
	IMD_ERRORS,
	SUS_DAMPER_FL,
	SUS_DUMPER_FR,
	SUS_DAMPER_RL,
	SUS_DAMPER_RR,
	WSS_FR,
	WSS_FL,
	WSS_RL,
	WSS_RR,
	WSS_F_AVG,
	WSS_R_AVG,
	WSS_SLIP,
	MAX_LOGGABLE_PARAMS /**< THIS MUST BE THE FINAL PARAM*/
}loggable_params;


typedef struct daq_datapoint{ //8 bytes, convenient, no?
	uint32_t can_id; /**< */
	uint16_t param;	/**< Which loggable param are we logging boys? */
	uint8_t period; /**< Time between transmissions in ms*/
	uint8_t type; /**< Datatype of the data */
}daq_datapoint; /**< */



/** @brief This struct holds info of what needs to be logged
 *
 */

typedef struct daq_loop_args{
	uint16_t total_params_logged;
	uint8_t throttle_daq_to_preserve_performance; /**< */
	uint8_t minimum_daq_period; /**< */
	uint8_t can_channel; /**< */
	uint8_t daq_child_priority;
}daq_loop_args;

typedef enum uv_status_t uv_status;


uv_status associateDaqParamWithVar(uint16_t paramID, void* var);
uv_status initDaqTask(void * args);
void daqMasterTask(void* args);




#endif /* INC_DAQ_H_ */
