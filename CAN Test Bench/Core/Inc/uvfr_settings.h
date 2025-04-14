/*
 * uvfr_settings.h
 *
 *  Created on: Oct 10, 2024
 *      Author: byo10
 */

#ifndef INC_UVFR_SETTINGS_H_
#define INC_UVFR_SETTINGS_H_


//#include "motor_controller.h"
#include "driving_loop.h"
#include "uvfr_utils.h"
#include "main.h"
#include "daq.h"
#include "bms.h"



#define ENABLE_FLASH_SETTINGS 0

#ifndef SIZEOF_SETTINGS_BLOCK
#define SIZEOF_SETTINGS_BLOCK sizeof(uv_vehicle_settings);
#endif

// Default settings for the motor controller
#define DEFAULT_CAN_ID_TX              0x201
#define DEFAULT_CAN_ID_RX              0x181
#define DEFAULT_MC_CAN_TIMEOUT         2     // seconds or your time unit
#define DEFAULT_PROPORTIONAL_GAIN      10    // RegID 0x2C
#define DEFAULT_INTEGRAL_TIME_CONSTANT 400   // RegID 0x2D
#define DEFAULT_INTEGRAL_MEMORY_MAX    60    // %

#define DEFAULT_MAX_SPEED       24575   // 75% of full-scale (32767)
#define DEFAULT_MAX_CURRENT     100     // Example
#define DEFAULT_CONT_CURRENT    60      // Example
#define DEFAULT_MAX_TORQUE      3000    // If using torque register (optional)


//typedef struct motor_controller_settings motor_controller_settings;

typedef struct motor_controller_settings{
	uint32_t can_id_tx;
	    uint32_t can_id_rx;
	    uint32_t mc_CAN_timeout;
	    uint8_t  proportional_gain;
	    uint32_t integral_time_constant;
	    uint8_t  integral_memory_max;

	    // extra
	    uint16_t max_speed;    // e.g., RPM in register units (0x34)
	    uint16_t max_current;  // e.g., 0x4D
	    uint16_t cont_current; // e.g., 0x4E
	    uint16_t max_torque;   // if using 0x90 or similar torque command
}motor_controller_settings;


typedef struct uv_vehicle_settings{
	SemaphoreHandle_t settings_mutex;
	struct motor_controller_settings mc_settings;
	bms_settings_t bms_settings;
	driving_loop_args driving_loop_settings;
	daq_loop_args daq_settings;
	void* imd_settings;
	void* pdu_settings;
	//motor_controller_settings mc_settings;


}uv_vehicle_settings;


void nukeSettings(uv_vehicle_settings** settings_to_delete);

enum uv_status_t uvSettingsInit();

#ifndef SRC_UVFR_SETTINGS_C_
//extern includes

extern uv_vehicle_settings* current_vehicle_settings;

#endif


#endif /* INC_UVFR_SETTINGS_H_ */



