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
	VCU_CURRENT_UPTIME,/*used, but from where?*/
	VCU_TOTAL_UPTIME,
	OS_AVAILABLE_HEAP,
	OS_LARGEST_FREE_BLOCK,
	OS_SMALLEST_FREE_BLOCK,
	OS_NUM_FREE_BLOCKS,
	OS_MIN_EVER_FREE_BYTES,
	OS_NUM_SUCCESSFUL_ALLOCS,
	OS_NUM_SUCCESSFUL_FREES,

	DIGI_IN_STAT,


	VEH_DISTANCE_RUN, //Trip Odometer?
	VEH_DISTANCE_TOTAL, // total odometer?
	VEH_LAPNUM,
	VEH_SPEED,
		VEH_DRIVE_MODE, // 1, 2, 3 represents drive mode
		MOTOR_RPM, /**< RPM as reported by motor controller */
		MOTOR_TEMP, /**< Motor Temp as reported by motor controller */
		MOTOR_TORQUE, /**< Torque requested from motor reported by motor controller */
	MOTOR_CURRENT, /**< Motor Phase currents as reported by motor controller */
	MC_VOLTAGE, /**< Pack voltage as measured by motor_controller*/
	MC_CURRENT, /**< Pack current as measured by motor_controller*/
	MC_TEMP, /**< Motor controller temperature*/
	MC_ERRORS, /**< Motor controller errors bitfield*/
	//
	DL_POWER_PERCENT,

	// BMS is on CAN1
	BMS_CURRENT, /**< Pack current measured by BMS*/
	BMS_VOLTAGE, /**< Pack voltage as measured by BMS*/
	BMS_ERRORS, /**< Error codes in BMS*/
	BMS_FAULT_HW, /**< BMS SDC Tripped*/
	//
	MAX_CELL_TEMP, /**< Max Temperature of a cell from BMS */
	MIN_CELL_TEMP, /**< Min Temperature of a cell*/
	AVG_CELL_TEMP,/**< Average Cell Temp*/
	ACCUM_SOC,/**< */
	ACCUM_SOH,/**< */
	ACCUM_POWER, /**< */
	ACCUM_POWER_LIMIT, /**< */
	APPS1_ADC_VAL, /**< USED IN DRIVING_LOOP.C*/
	APPS2_ADC_VAL, /**< USED IN DRIVING_LOOP.C*/
		APPS_PERCENT,
	BPS1_ADC_VAL, /**< USED IN DRIVING_LOOP.C*/
	BPS2_ADC_VAL,/**< USED IN DRIVING_LOOP.C*/
		BPS_PERCENT,
	BSPD_FAULT_HW, /**< BSPD SDC Tripped*/
	COOLANT_TEMP_ADC,/**< USED IN DAQ.C*/
	MOTOR_TEMP_ADC,/**< USED IN DAQ.C*/
	ACCELERATOR_PEDAL_RATIO, /**< */
	BRAKE_PRESSURE_PA_F, /**< */
	BRAKE_PRESSURE_PA_R,
	POWER_DERATE_FACTOR, /**< */
	CURRENT_DRIVING_MODE, /**< */
		IMD_VOLTAGE, /**< Accumulator voltage as measured by IMD*/
		IMD_STATUS, /*IMD STATUS*/
		IMD_ERRORS,
		IMD_SAFETOUCH,
		IMD_ISO_STATE,
		IMD_RP_RAW,
		IMD_RN_RAW,
		IMD_CP_NF,
		IMD_CN_NF,
		IMD_TEMP_RAW,
		IMD_FAULT_HW, /**< IMD SDC Tripped*/
	SUS_DAMPER_FL,
	SUS_DAMPER_FR,
	SUS_DAMPER_RL,
	SUS_DAMPER_RR,
	WSS_FR_RPM,
	WSS_FL_RPM,
	WSS_RL_RPM,
	WSS_RR_RPM,
	WSS_F_AVG,
	WSS_R_AVG,
	WSS_SLIP,
	// TMS (Thermal Management System) battery-pack temps, off CAN2. See tms.c
	TMS_PACK_TEMP_LOW,     /**< Lowest battery-pack temp from TMS (int8 C) */
	TMS_PACK_TEMP_HIGH,    /**< Highest battery-pack temp from TMS (int8 C) */
	TMS_PACK_TEMP_AVG,     /**< Average battery-pack temp from TMS (int8 C) */
	TMS_PACK_TEMP_HIGH_ID, /**< Pack index (0..5) reporting the high temp */
	TMS_PACK_TEMP_LOW_ID,  /**< Pack index (0..5) reporting the low temp */
	MAX_LOGGABLE_PARAMS /**< THIS MUST BE THE FINAL PARAM*/
}loggable_params;


typedef struct daq_msg{ //8 bytes, convenient, no?
	uint32_t can_id; /**< */
	uint16_t param[4];	/**< Which loggable param are we logging? */
	uint8_t type[4]; /**< Datatype of the data */
	uint8_t period; /**< Time between transmissions in ms*/

}daq_msg;



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
