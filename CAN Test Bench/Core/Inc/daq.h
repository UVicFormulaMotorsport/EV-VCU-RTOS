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
	MOTOR_RPM,
	MOTOR_TEMP,
	MOTOR_CURRENT,
	MC_VOLTAGE, /**< Pack voltage as measured by motor_controller*/
	MC_CURRENT, /**< Pack current as measured by motor_controller*/
	MC_TEMP, /**< Motor controller temperature*/
	MC_ERRORS, /**< Motor controller errors bitfield*/
	BMS_CURRENT, /**< Pack current measured by BMS*/
	BMS_VOLTAGE, /**< Pack voltage as measured by BMS*/
	BMS_ERRORS, /**< Error codes in BMS*/
	MAX_CELL_TEMP, /**< Max Temperature of a cell from BMS */
	MIN_CELL_TEMP, /**< */
	AVG_CELL_TEMP,/**< */
	ACC_POWER, /**< */
	ACC_POWER_LIMIT, /**< */
	APPS1_ADC_VAL, /**< */
	APPS2_ADC_VAL, /**< */
	BPS1_ADC_VAL, /**< */
	BPS2_ADC_VAL, /**< */
	ACCELERATOR_PEDAL_RATIO, /**< */
	BRAKE_PRESSURE_PA, /**< */
	POWER_DERATE_FACTOR, /**< */
	CURRENT_DRIVING_MODE, /**< */
	IMD_VOLTAGE, /**< Accumulator voltage as measured by IMD*/
	MAX_LOGGABLE_PARAMS /**< */
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
