// Code to make human readable CAN messages for the device

#include "main.h"


#ifndef _BMS_CAN_MSG_H_
#define _BMS_CAN_MSG_H_

#include<stdio.h>

void BMS_Parse_Message1(int DLC, uint8_t Data[]);
void BMS_Parse_Message2(int DLC, uint8_t Data[]);
void BMS_Parse_Message3(int DLC, uint8_t Data[]);
void reset_BMS_WDT();

//BMS error code masks, use these to map BMS errors to our own properietary system maybe?
#ifndef __BMS_Errors__
#define __BMS_Errors__

/* How to use to determine BMS errors:
 * By index. This is useful if accessing an array based on the error.
 *
 * By mask. Can manually figure shit out using it.
 *
 * By macro, it works kinda like a function
 */
//useful defines for various programs to use
#define MAX_ACCUMULATOR_TEMP 60
#define MIN_ACCUMULATOR_TEMP 0
#define MAX_BATTERY_VOLTAGE 4998 //the BMS multiplies voltage by 10 to send this, 499.8V in reality
#define MIN_BATTERY_VOLTAGE 3570


#define BMS_HEARTBEAT_ID 0x80
#define BMS_CAN_ID_1 (0x180 + 0x20)
#define BMS_CAN_ID_2 (0x280 + 0x20)
#define BMS_CAN_ID_3 (0x380 + 0x20)


#define BMS_ERRORS1_OVERCURRENT_index 0
#define BMS_ERRORS1_OVERCURRENT_mask (uint32_t)(0x01U << BMS_ERRORS1_OVERCURRENT_index)
#define BMS_ERRORS1_OVERCURRENT (BMS_errors_1 & BMS_ERRORS1_OVERCURRENT_mask)

#define BMS_ERRORS1_UNDERVOLTAGE_index 1
#define BMS_ERRORS1_UNDERVOLTAGE_mask (uint32_t)(0x01U << BMS_ERRORS1_UNDERVOLTAGE_index)
#define BMS_ERRORS1_UNDERVOLTAGE BMS_ERRORS1_UNDERVOLTAGE_mask

#define BMS_ERRORS1_OVERVOLTAGE_index 2
#define BMS_ERRORS1_OVERVOLTAGE_mask (uint32_t)(0x01U << BMS_ERRORS1_OVERVOLTAGE_index)
#define BMS_ERRORS1_OVERVOLTAGE BMS_ERRORS1_OVERVOLTAGE_mask

#define BMS_ERRORS1_LOW_OCH_TEMP_index 3
#define BMS_ERRORS1_LOW_OCH_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS1_LOW_OCH_TEMP_index)
#define BMS_ERRORS1_LOW_OCH_TEMP MS_ERRORS1_LOW_OCH_TEMP_mask

#define BMS_ERRORS1_HIGH_OCH_TEMP_index 4
#define BMS_ERRORS1_HIGH_OCH_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS1_HIGH_OCH_TEMP_index)
#define BMS_ERRORS1_HIGH_OCH_TEMP BMS_ERRORS1_HIGH_OCH_TEMP_mask

#define BMS_ERRORS1_BATT_COVER_index 5
#define BMS_ERRORS1_BATT_COVER_mask (uint32_t)(0x01U << BMS_ERRORS1_BATT_COVER_index)
#define BMS_ERRORS1_BATT_COVER BMS_ERRORS1_BATT_COVER_mask

#define BMS_ERRORS1_HIGH_HUMIDITY_index 6
#define BMS_ERRORS1_HIGH_HUMIDITY_mask (uint32_t)(0x01U << BMS_ERRORS1_HIGH_HUMIDITY_index)
#define BMS_ERRORS1_HIGH_HUMIDITY BMS_ERRORS1_HIGH_HUMIDITY_mask

#define BMS_ERRORS1_WATER_index 7
#define BMS_ERRORS1_WATER_mask (uint32_t)(0x01U << BMS_ERRORS1_WATER_index)
#define BMS_ERRORS1_WATER BMS_ERRORS1_WATER_mask

#define BMS_ERRORS1_HIGH_LOGICBRD_TEMP_index 8
#define BMS_ERRORS1_HIGH_LOGICBRD_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS1_HIGH_LOGICBRD_TEMP_index)
#define BMS_ERRORS1_HIGH_LOGICBRD_TEMP BMS_ERRORS1_HIGH_LOGICBRD_TEMP_mask

#define BMS_ERRORS1_LOGIC_OFFLINE_index 9
#define BMS_ERRORS1_LOGIC_OFFLINE_mask (uint32_t)(0x01U << BMS_ERRORS1_LOGIC_OFFLINE_index)
#define BMS_ERRORS1_LOGIC_OFFLINE BMS_ERRORS1_LOGIC_OFFLINE_mask

#define BMS_ERRORS1_CRITICAL_ERROR_index 10
#define BMS_ERRORS1_CRITICAL_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_CRITICAL_ERROR_index)
#define BMS_ERRORS1_CRITICAL_ERROR BMS_ERRORS1_CRITICAL_ERROR_mask

#define BMS_ERRORS1_CROWN_FORKLIFT_index 11
#define BMS_ERRORS1_CROWN_FORKLIFT_mask (uint32_t)(0x01U << BMS_ERRORS1_CROWN_FORKLIFT_index)
#define BMS_ERRORS1_CROWN_FORKLIFT BMS_ERRORS1_CROWN_FORKLIFT_mask

#define BMS_ERRORS1_CELL_COUNT_ERROR_index 12
#define BMS_ERRORS1_CELL_COUNT_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_CELL_COUNT_ERROR_index)
#define BMS_ERRORS1_CELL_COUNT_ERROR BMS_ERRORS1_CELL_COUNT_ERROR_mask

#define BMS_ERRORS1_HYG_FORKLIFT_index 13
#define BMS_ERRORS1_HYG_FORKLIFT_mask (uint32_t)(0x01U << BMS_ERRORS1_HYG_FORKLIFT_index)
#define BMS_ERRORS1_HYG_FORKLIFT BMS_ERRORS1_HYG_FORKLIFT_mask

#define BMS_ERRORS1_NEED_ACK_index 14
#define BMS_ERRORS1_NEED_ACK_mask (uint32_t)(0x01U << BMS_ERRORS1_NEED_ACK_index)
#define BMS_ERRORS1_NEED_ACK BMS_ERRORS1_NEED_ACK_mask

#define BMS_ERRORS1_COMBILIFT_FORKLIFT_index 15
#define BMS_ERRORS1_COMBILIFT_FORKLIFT_mask (uint32_t)(0x01U << BMS_ERRORS1_COMBILIFT_FORKLIFT_index)
#define BMS_ERRORS1_COMBILIFT_FORKLIFT

#define BMS_ERRORS1_SHORT_CIRCUIT_index 16
#define BMS_ERRORS1_SHORT_CIRCUIT_mask (uint32_t)(0x01U << BMS_ERRORS1_SHORT_CIRCUIT_index)
#define BMS_ERRORS1_SHORT_CIRCUIT

#define BMS_ERRORS1_HIGH_CONTACTOR_TEMP_index 17
#define BMS_ERRORS1_HIGH_CONTACTOR_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS1_CONTACTOR_TEMP_index)
#define BMS_ERRORS1_HIGH_CONTACTOR_TEMP

#define BMS_ERRORS1_LOGIC_COUNT_ERROR_index 18
#define BMS_ERRORS1_LOGIC_COUNT_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_OVERCURRENT_index)
#define BMS_ERRORS1_LOGIC_COUNT_ERROR

#define BMS_ERRORS1_ADC_ERROR_index 19
#define BMS_ERRORS1_ADC_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_ADC_ERROR_index)
#define BMS_ERRORS1_ADC_ERROR

#define BMS_ERRORS1_CURRENT_SENSOR_ERROR_index 20
#define BMS_ERRORS1_CURRENT_SENSOR_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_SENSOR_ERROR_index)
#define BMS_ERRORS1_CURRENT_SENSOR_ERROR

#define BMS_ERRORS1_CH_CONTACTOR_CYCLES_index 21
#define BMS_ERRORS1_CH_CONTACTOR_CYCLES_mask (uint32_t)(0x01U << BMS_ERRORS1_CH_CONTACTOR_CYCLES_index)
#define BMS_ERRORS1_CH_CONTACTOR_CYCLES

#define BMS_ERRORS1_DCH_CONTACTOR_CYCLES_index 22
#define BMS_ERRORS1_DCH_CONTACTOR_CYCLES_mask (uint32_t)(0x01U << BMS_ERRORS1_DCH_CONTACTOR_CYCLES_index)
#define BMS_ERRORS1_DCH_CONTACTOR_CYCLES

#define BMS_ERRORS1_SHUNT_OFFLINE_index 23
#define BMS_ERRORS1_SHUNT_OFFLINE_mask (uint32_t)(0x01U << BMS_ERRORS1_SHUNT_OFFLINE_index)
#define BMS_ERRORS1_SHUNT_OFFLINE

#define BMS_ERRORS1_SHUNT_ERROR_index 24
#define BMS_ERRORS1_SHUNT_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_SHUNT_ERROR_index)
#define BMS_ERRORS1_SHUNT_ERROR

#define BMS_ERRORS1_SETTINGS_ERROR_index 25
#define BMS_ERRORS1_SETTINGS_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_SETTINGS_ERROR_index)
#define BMS_ERRORS1_SETTINGS_ERROR

#define BMS_ERRORS1_WDT_RESET_index 26
#define BMS_ERRORS1_WDT_RESET_mask (uint32_t)(0x01U << BMS_ERRORS1_WDT_RESET_index)
#define BMS_ERRORS1_WDT_RESET

#define BMS_ERRORS1_NO_TEMP_SENSOR_index 27
#define BMS_ERRORS1_NO_TEMP_SENSOR_mask (uint32_t)(0x01U << BMS_ERRORS1_NO_TEMP_SENSOR_index)
#define BMS_ERRORS1_NO_TEMP_SENSOR

#define BMS_ERRORS1_SHORTED_TEMP_SENSOR_index 28
#define BMS_ERRORS1_SHORTED_TEMP_SENSOR_mask (uint32_t)(0x01U << BMS_ERRORS1_SHORTED_TEMP_SENSOR_index)
#define BMS_ERRORS1_SHORTED_TEMP_SENSOR

#define BMS_ERRORS1_SPIRIT_TRUCK_ERROR_index 29
#define BMS_ERRORS1_SPIRIT_TRUCK_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS1_SPIRIT_TRUCK_ERROR_index)
#define BMS_ERRORS1_SPIRIT_TRUCK_ERROR BMS_ERRORS1_SPIRIT_TRUCK_ERROR_mask

#define BMS_ERRORS1_FORKLIFT_mask (BMS_ERRORS1_COMBILIFT_FORKLIFT_mask | BMS_ERRORS1_HYG_FORKLIFT_mask | BMS_ERRORS1_CROWN_FORKLIFT_mask | BMS_ERRORS1_SPIRIT_TRUCK_ERROR_mask |)





//bits 30 and 31 are reserved
#define BMS_ERRORS1_RESERVED_index
#define BMS_ERRORS1_RESERVED_mask 0xC000000

//start errors 2

#define BMS_ERRORS2_LOW_CH_TEMP_index 0
#define BMS_ERRORS2_LOW_CH_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS2_LOW_CH_TEMP_index)
#define BMS_ERRORS2_LOW_CH_TEMP BMS_ERRORS2_LOW_CH_TEMP_mask

#define BMS_ERRORS2_HIGH_CH_TEMP_index 1
#define BMS_ERRORS2_HIGH_CH_TEMP_mask (uint32_t)(0x01U << BMS_ERRORS2_HIGH_CH_TEMP_index)
#define BMS_ERRORS2_HIGH_CH_TEMP BMS_ERRORS2_HIGH_CH_TEMP_mask

#define BMS_ERRORS2_SD_MOUNT_ERROR_index 2
#define BMS_ERRORS2_SD_MOUNT_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS2_SD_MOUNT_ERROR_index)
#define BMS_ERRORS2_SD_MOUNT_ERROR BMS_ERRORS2_SD_MOUNT_ERROR_mask

#define BMS_ERRORS2_SD_RW_ERROR_index 3
#define BMS_ERRORS2_SD_RW_ERROR_mask (uint32_t)(0x01U << BMS_ERRORS2_SD_RW_ERROR_index)
#define BMS_ERRORS2_SD_RW_ERROR BMS_ERRORS2_SD_RW_ERROR_mask

#define BMS_ERRORS2_FORBIDDEN_CHARGING_index 4
#define BMS_ERRORS2_FORBIDDEN_CHARGING_mask (uint32_t)(0x01U << BMS_ERRORS2_FORBIDDEN_CHARGING_index)
#define BMS_ERRORS2_FORBIDDEN_CHARGING BMS_ERRORS2_FORBIDDEN_CHARGING_mask

#define BMS_ERRORS2_STUCK_CONTACTOR_index 5
#define BMS_ERRORS2_STUCK_CONTACTOR_mask (uint32_t)(0x01U << MS_ERRORS2_STUCK_CONTACTOR_index)
#define BMS_ERRORS2_STUCK_CONTACTOR BMS_ERRORS2_STUCK_CONTACTOR_mask

#define BMS_ERRORS2_CH_CONTACTOR_FDBCK_index 6
#define BMS_ERRORS2_CH_CONTACTOR_FDBCK_mask (uint32_t)(0x01U << BMS_ERRORS2_CH_CONTACTOR_FDBCK_index)
#define BMS_ERRORS2_CH_CONTACTOR_FDBCK BMS_ERRORS2_CH_CONTACTOR_FDBCK_mask

#define BMS_ERRORS2_DCH_CONTACTOR_FDBCK_index 7
#define BMS_ERRORS2_DCH_CONTACTOR_FDBCK_mask (uint32_t)(0x01U << BMS_ERRORS2_DCH_CONTACTOR_FDBCK_index)
#define BMS_ERRORS2_DCH_CONTACTOR_FDBCK BMS_ERRORS2_DCH_CONTACTOR_FDBCK_mask

#define BMS_ERRORS2_ISOLATION_FAULT_index 8
#define BMS_ERRORS2_ISOLATION_FAULT_mask (uint32_t)(0x01U << BMS_ERRORS2_ISOLATION_FAULT_index)
#define BMS_ERRORS2_ISOLATION_FAULT BMS_ERRORS2_ISOLATION_FAULT_mask

#define BMS_ERRORS2_PCH_CONTACTOR_FDBCK_index 9
#define BMS_ERRORS2_PCH_CONTACTOR_FDBCK_mask (uint32_t)(0x01U << BMS_ERRORS2_PCH_CONTACTOR_FDBCK_index)
#define BMS_ERRORS2_PCH_CONTACTOR_FDBCK BMS_ERRORS2_PCH_CONTACTOR_FDBCK_mask

#define BMS_ERRORS2_CH_DCH_CONTACTOR_FDBCK_index 10
#define BMS_ERRORS2_CH_DCH_CONTACTOR_FDBCK_mask (uint32_t)(0x01U << BMS_ERRORS2_CH_DCH_CONTACTOR_FDBCK_index)
#define BMS_ERRORS2_CH_DCH_CONTACTOR_FDBCK BMS_ERRORS2_CH_DCH_CONTACTOR_FDBCK_mask

#define BMS_ERRORS2_MAIN_CONTACTOR_FDBCK_index 11
#define BMS_ERRORS2_MAIN_CONTACTOR_FDBCK_mask (uint32_t)(0x01U << BMS_ERRORS2_MAIN_CONTACTOR_FDBCK_index)
#define BMS_ERRORS2_MAIN_CONTACTOR_FDBCK BMS_ERRORS2_MAIN_CONTACTOR_FDBCK_mask

//yeah idek what happens over here, idk if theres forklift errors here
#define BMS_ERRORS2_FORKLIFT_mask (uint32_t)()

//Masks for cool and fun things :)
//Determines whether errors warrant a full shutdown or just a temporary pause

#define BMS_ERRORS1_CHRG_SHTDWN_mask 0x3FFFFFFF
#define BMS_ERRORS1_VEHICLE_SHTDWN_mask 0x3FFFFFFF

#define BMS_ERRORS1_CHRG_SUSPEND_mask 0x00000000
#define BMS_ERRORS1_VEHICLE_SUSPEND_mask 0x00000000

#define BMS_ERRORS2_CHRG_SHTDWN_mask 0x00000FFF
#define BMS_ERRORS2_VEHICLE_SHTDWN_mask 0x00000FFF

#define BMS_ERRORS2_CHRG_SUSPEND_mask 0x00000000
#define BMS_ERRORS2_VEHICLE_SUSPEND_mask 0x00000000





#endif

#ifndef __BMS_INTERNAL_STATE__
#define __BMS_INTERNAL_STATE__

#define BMS_STATE_LOW_SOC_index 0
#define BMS_STATE_LOW_SOC_mask (uint32_t)(0x01U << BMS_STATE_LOW_SOC_index)
#define BMS_STATE_LOW_SOC

#define BMS_STATE_HIGH_CHRG_CURRENT_index 1
#define BMS_STATE_HIGH_CHRG_CURRENT_mask (uint32_t)(0x01U << BMS_STATE_HIGH_CHRG_CURRENT_index)
#define BMS_STATE_HIGH_CHRG_CURRENT

#define BMS_STATE_CHRG_CONTACTOR_STATE_index 2
#define BMS_STATE_CHRG_CONTACTOR_STATE_mask (uint32_t)(0x01U << BMS_STATE_CHRG_CONTACTOR_STATE_index)
#define BMS_STATE_CHRG_CONTACTOR_STATE

#define BMS_STATE_ALLOW_CHARGING_index 3
#define BMS_STATE_ALLOW_CHARGING_mask (uint32_t)(0x01U << BMS_STATE_ALLOW_CHARGING_index)
#define BMS_STATE_ALLOW_CHARGING

#define BMS_STATE_CHRG_CURRENT_PRESENT_index 4
#define BMS_STATE_CHRG_CURRENT_PRESENT_mask (uint32_t)(0x01U << BMS_STATE_CHRG_CURRENT_PRESENT_index)
#define BMS_STATE_CHRG_CURRENT_PRESENT

#define BMS_STATE_DCH_CONTACTOR_STATE_index 5
#define BMS_STATE_DCH_CONTACTOR_STATE_mask (uint32_t)(0x01U << BMS_STATE_DCH_CONTACTOR_STATE_index)
#define BMS_STATE_DCH_CONTACTOR_STATE

#define BMS_STATE_DCH_CURRENT_PRESENT_index 6
#define BMS_STATE_DCH_CURRENT_PRESENT_mask (uint32_t)(0x01U << BMS_STATE_DCH_CURRENT_PRESENT_index)
#define BMS_STATE_DCH_CURRENT_PRESENT

#define BMS_STATE_INCREASED_VOLTAGE_index 7
#define BMS_STATE_INCREASED_VOLTAGE_mask (uint32_t)(0x01U << BMS_STATE_INCREASED_VOLTAGE_index)
#define BMS_STATE_INCREASED_VOLTAGE

#define BMS_STATE_HIGH_DCH_TEMPERATURE_index 8
#define BMS_STATE_HIGH_DCH_TEMPERATURE_mask (uint32_t)(0x01U << BMS_STATE_HIGH_DCH_TEMPERATURE_index)
#define BMS_STATE_HIGH_DCH_TEMPERATURE

#define BMS_STATE_COOLER_index 9
#define BMS_STATE_COOLER_mask (uint32_t)(0x01U << BMS_STATE_COOLER_index)
#define BMS_STATE_COOLER

#define BMS_STATE_HYG_SHUTDOWN_index 10
#define BMS_STATE_HYG_SHUTDOWN_mask (uint32_t)(0x01U << BMS_STATE_HYG_SHUTDOWN_index)
#define BMS_STATE_HYG_SHUTDOWN

#define BMS_STATE_INIT_index 11
#define BMS_STATE_INIT_mask (uint32_t)(0x01U << BMS_STATE_INIT_index)
#define BMS_STATE_INIT

#define BMS_STATE_PRECHARGE_STATE_index 12
#define BMS_STATE_PRECHARGE_STATE_mask (uint32_t)(0x01U << BMS_STATE_PRECHARGE_STATE_index)
#define BMS_STATE_PRECHARGE_STATE

#define BMS_STATE_COMBILIFT_SHUTDOWN_index 13
#define BMS_STATE_COMBILIFT_SHUTDOWN_mask (uint32_t)(0x01U << BMS_STATE_COMBILIFT_SHUTDOWN_index)
#define BMS_STATE_COMBILIFT_SHUTDOWN

#define BMS_STATE_CELL_ANALYSIS_index 14
#define BMS_STATE_CELL_ANALYSIS_mask (uint32_t)(0x01U << BMS_STATE_CELL_ANALYSIS_index)
#define BMS_STATE_CELL_ANALYSIS

#define BMS_STATE_BALANCING_SERIES1_index 15
#define BMS_STATE_BALANCING_SERIES1_mask (uint32_t)(0x01U << BMS_STATE_BALANCING_SERIES1_index)
#define BMS_STATE_BALANCING_SERIES1

#define BMS_STATE_BALANCING_SERIES2_index 16
#define BMS_STATE_BALANCING_SERIES2_mask (uint32_t)(0x01U << BMS_STATE_BALANCING_SERIES2_index)
#define BMS_STATE_BALANCING_SERIES2

#define BMS_STATE_AUX_CONTACTOR_STATE_index 17
#define BMS_STATE_AUX_CONTACTOR_STATE_mask (uint32_t)(0x01U << BMS_STATE_AUX_CONTACTOR_STATE_index)
#define BMS_STATE_AUX_CONTACTOR_STATE

#define BMS_STATE_ACK_POWER_DOWN_index 18
#define BMS_STATE_ACK_POWER_DOWN_mask (uint32_t)(0x01U << BMS_STATE_ACK_POWER_DOWN_index)
#define BMS_STATE_ACK_POWER_DOWN

#define BMS_STATE_CROWN_EWS_index 19
#define BMS_STATE_CROWN_EWS_mask (uint32_t)(0x01U << BMS_STATE_CROWN_EWS_index)
#define BMS_STATE_CROWN_EWS

#define BMS_STATE_MAIN_CONTACTOR_STATE_index 20
#define BMS_STATE_MAIN_CONTACTOR_STATE_mask (uint32_t)(0x01U << BMS_STATE_MAIN_CONTACTOR_STATE_index)
#define BMS_STATE_MAIN_CONTACTOR_STATE

#define BMS_STATE_SERVICE_RESET_index 21
#define BMS_STATE_SERVICE_RESET_mask (uint32_t)(0x01U << BMS_STATE_SERVICE_RESET_index)
#define BMS_STATE_SERVICE_RESET

#define BMS_STATE_CH_DCH_CONTACTOR_STATE_index 22
#define BMS_STATE_CH_DCH_CONTACTOR_STATE_mask (uint32_t)(0x01U << BMS_STATE_CH_DCH_CONTACTOR_STATE_index)
#define BMS_STATE_CH_DCH_CONTACTOR_STATE

#define BMS_STATE_READY_TO_CHARGE_index 23
#define BMS_STATE_READY_TO_CHARGE_mask (uint32_t)(0x01U << BMS_STATE_READY_TO_CHARGE_index)
#define BMS_STATE_READY_TO_CHARGE

#define BMS_STATE_READY_TO_DISCHARGE_index 24
#define BMS_STATE_READY_TO_DISCHARGE_mask (uint32_t)(0x01U << BMS_STATE_READY_TO_DISCHARGE_index)
#define BMS_STATE_READY_TO_DISCHARGE

#define BMS_STATE_RESERVED_mask 0xFE000000
#define BMS_ILLEGAL_STATES (BMS_STATE_HIGH_DCH_TEMPERATURE_mask)




#endif

#ifndef __BMS_DISCRETE_INPUTS__
#define __BMS_DISCRETE_INPUTS__

#define BMS_DI1_BATTERY_COVER_index 0
#define BMS_DI1_BATTERY_COVER_mask (uint8_t)(0x01U << BMS_DI1_BATTERY_COVER_index)
#define BMS_DI1_BATTERY_COVER BMS_DI1_BATTERY_COVER_mask

#define BMS_DI1_CHARGER_CONNECTED_index 1
#define BMS_DI1_CHARGER_CONNECTED_mask (uint8_t)(0x01U << BMS_DI1_CHARGER_CONNECTED_index)
#define BMS_DI1_CHARGER_CONNECTED BMS_DI1_CHARGER_CONNECTED_mask

#define BMS_DI1_PWR_DOWN_REQ_index 2
#define BMS_DI1_PWR_DOWN_REQ_mask (uint8_t)(0x01U << BMS_DI1_PWR_DOWN_REQ_index)
#define BMS_DI1_PWR_DOWN_REQ BMS_DI1_PWR_DOWN_REQ_mask

#define BMS_DI1_INHIBIT_CHRG_index 3
#define BMS_DI1_INHIBIT_CHRG_mask (uint8_t)(0x01U << BMS_DI1_INHIBIT_CHRG_index)
#define BMS_DI1_INHIBIT_CHRG MS_DI1_INHIBIT_CHRG_mask

#define BMS_DI1_INHIBIT_DISCHRG_index 4
#define BMS_DI1_INHIBIT_DISCHRG_mask (uint8_t)(0x01U << BMS_DI1_INHIBIT_DISCHRG_index)
#define BMS_DI1_INHIBIT_DISCHRG BMS_DI1_INHIBIT_DISCHRG_mask

#define BMS_DI1_CH_CONTACTOR_FDBCK_index 5
#define BMS_DI1_CH_CONTACTOR_FDBCK_mask (uint8_t)(0x01U << BMS_DI1_CH_CONTACTOR_FDBCK_index)
#define BMS_DI1_CH_CONTACTOR_FDBCK BMS_DI1_CH_CONTACTOR_FDBCK_mask

#define BMS_DI1_DCH_CONTACTOR_FDBCK_index 6
#define BMS_DI1_DCH_CONTACTOR_FDBCK_mask (uint8_t)(0x01U << BMS_DI1_DCH_CONTACTOR_FDBCK_index)
#define BMS_DI1_DCH_CONTACTOR_FDBCK BMS_DI1_DCH_CONTACTOR_FDBCK_mask

#define BMS_DI1_ISOLATION_STATUS_index 7
#define BMS_DI1_ISOLATION_STATUS_mask (uint8_t)(0x01U << BMS_DI1_ISOLATION_STATUS_index)
#define BMS_DI1_ISOLATION_STATUS MS_DI1_ISOLATION_STATUS_mask

//Discrete inputs 2

#define BMS_DI2_CHARGE_REQUEST_index 0
#define BMS_DI2_CHARGE_REQUEST_mask (uint8_t)(0x01U << BMS_DI2_CHARGE_REQUEST_mask)
#define BMS_DI2_CHARGE_REQUEST

#define BMS_DI2_PRECHARGE_REQUEST_index 1
#define BMS_DI2_PRECHARGE_REQUEST_mask (uint8_t)(0x01U << BMS_DI2_PRECHARGE_REQUEST_index)
#define BMS_DI2_PRECHARGE_REQUEST

#define BMS_DI2_DISCHARGE_REQUEST_index 2
#define BMS_DI2_DISCHARGE_REQUEST_mask (uint8_t)(0x01U << BMS_DI2_DISCHARGE_REQUEST_index)
#define BMS_DI2_DISCHARGE_REQUEST

#define BMS_DI2_PCH_CONTACTOR_FDBCK_index 3
#define BMS_DI2_PCH_CONTACTOR_FDBCK_mask (uint8_t)(0x01U << BMS_DI2_PCH_CONTACTOR_FDBCK_index)
#define BMS_DI2_PCH_CONTACTOR_FDBCK

#define BMS_DI2_CH_DCH_CONTACTOR_FDBCK_index 4
#define BMS_DI2_CH_DCH_CONTACTOR_FDBCK_mask (uint8_t)(0x01U << BMS_DI2_CH_DCH_CONTACTOR_FDBCK_index)
#define BMS_DI2_CH_DCH_CONTACTOR_FDBCK

#define BMS_DI2_MAIN_CONTACTOR_FDBCK_index 5
#define BMS_DI2_MAIN_CONTACTOR_FDBCK_mask (uint8_t)(0x01U << BMS_DI2_MAIN_CONTACTOR_FDBCK_index)
#define BMS_DI2_MAIN_CONTACTOR_FDBCK


#endif

#ifndef __BMS_DATA__
#define __BMS_DATA__

/* **IMPORTANT!**
 * HAL will write incoming can messages to an array of 8 bytes.
 * The following defines are used to get useful data from it
 *
 * Data can be accessed by one of two methods.
 * 1) use the BMS_MSGx_xxxx_index define as the index of the array. I.E.
 * rdata[BMS_MSG1_BATTERY_CURRENT_index]
 *
 * 2) use the BMS_MSGx_xxxx(msg) macro to get the actual value, and handle any nasty typecasting things that
 * may arise. This will return that data type required.
 * for instance
 * uint16_t batt_voltage
 *
 */

//BMS Message 1 contents:
#define BMS_MSG1_DISCRETE_INPUTS_1(msg) (uint8_t) *(msg + BMS_MSG1_DISCRETE_INPUTS_1_index)
#define BMS_MSG1_BATTERY_CURRENT(msg) (int16_t) *((int16_t*) (msg + BMS_MSG1_BATTERY_CURRENT_index))
#define BMS_MSG1_MIN_BATTERY_TEMP(msg) (int8_t) *((int8_t*) (msg + BMS_MSG1_MIN_BATTERY_TEMP_index))
#define BMS_MSG1_MAX_BATTERY_TEMP(msg) (int8_t) *((int8_t*) (msg + BMS_MSG1_MAX_BATTERY_TEMP_index))
#define BMS_MSG1_SOC(msg) (uint8_t) *((uint8_t*) (msg + BMS_MSG1_SOC_index))
#define BMS_MSG1_BATTERY_VOLTAGE(msg) (uint16_t) *((uint16_t*) (msg + BMS_MSG1_BATTERY_VOLTAGE_index))


//Indices of array:
#define BMS_MSG1_DISCRETE_INPUTS_1_index 0
#define BMS_MSG1_BATTERY_CURRENT_index 1
#define BMS_MSG1_MIN_BATTERY_TEMP_index 3
#define BMS_MSG1_MAX_BATTERY_TEMP_index 4
#define BMS_MSG1_SOC_index 5
#define BMS_MSG1_BATTERY_VOLTAGE_index 6



//BMS Message 2 contents:
#define BMS_MSG2_BMS_INTERNAL_STATE(msg) (uint32_t) *((uint32_t*) (msg + BMS_MSG2_BMS_INTERNAL_STATE_index))
#define BMS_MSG2_ERRORS_REGISTER_1(msg) (uint32_t) *((uint32_t*) (msg + BMS_MSG2_ERRORS_REGISTER_1_index))

//Indices of array:
#define BMS_MSG2_BMS_INTERNAL_STATE_index 0
#define BMS_MSG2_ERRORS_REGISTER_1_index 4

//BMS Message 3 contents:
#define BMS_MSG3_ERRORS_REGISTER_2(msg) (uint32_t) *((uint32_t*) (msg + BMS_MSG3_ERRORS_REGISTER_2_index))
#define BMS_MSG3_DISCRETE_INPUTS_2(msg) (uint16_t) *((uint16_t*) (msg + BMS_MSG3_DISCRETE_INPUTS_2_index))


//Indices of array:
#define BMS_MSG3_ERRORS_REGISTER_2_index 0
#define BMS_MSG3_DISCRETE_INPUTS_2_index 4



#endif


#endif
