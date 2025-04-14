/*
 * uvfr_settings.c
 *
 *  Created on: Oct 10, 2024
 *      Author: byo10
 */
#define SRC_UVFR_SETTINGS_C_

#include "uvfr_utils.h"
#include "main.h"
#include "stdlib.h"


//global variables for all to enjoy
uv_vehicle_settings* current_vehicle_settings = NULL;

#if ENABLE_FLASH_SETTINGS
enum uv_status_t getSettingsFromFlash(){
	//if there exists settings in flash somewhere, then get them. Otherwise we use the defaults
	return UV_ABORTED;
}


bool uvAreCustomSettingsActive(){
	return false;
}

uv_status saveSettings(){
	return UV_ABORTED;
}


#endif


/** @brief Function that allocates the neccessary space for all the vehicle settings, and
 * handles sets all of the settings structs to defaults
 *
 */
void setupDefaultSettings(){
	//real trap shit
	current_vehicle_settings = uvMalloc(sizeof(uv_vehicle_settings));

	 if (!current_vehicle_settings) return;

	    // Motor controller defaults (set from macros)
	    current_vehicle_settings->mc_settings.can_id_tx              = DEFAULT_CAN_ID_TX;
	    current_vehicle_settings->mc_settings.can_id_rx              = DEFAULT_CAN_ID_RX;
	    current_vehicle_settings->mc_settings.mc_CAN_timeout         = DEFAULT_MC_CAN_TIMEOUT;
	    current_vehicle_settings->mc_settings.proportional_gain      = DEFAULT_PROPORTIONAL_GAIN;
	    current_vehicle_settings->mc_settings.integral_time_constant = DEFAULT_INTEGRAL_TIME_CONSTANT;
	    current_vehicle_settings->mc_settings.integral_memory_max    = DEFAULT_INTEGRAL_MEMORY_MAX;

	    current_vehicle_settings->mc_settings.max_speed    = DEFAULT_MAX_SPEED;
	    current_vehicle_settings->mc_settings.max_current  = DEFAULT_MAX_CURRENT;
	    current_vehicle_settings->mc_settings.cont_current = DEFAULT_CONT_CURRENT;
	    current_vehicle_settings->mc_settings.max_torque   = DEFAULT_MAX_TORQUE;
}

void nukeSettings(uv_vehicle_settings** settings_to_delete){


	uvFree(*settings_to_delete);
	*settings_to_delete = NULL;


}

/** @brief this function does one thing, and one thing only, it checks if we have custom settings, then it attempts to get them.
 * If it fails, then we revert to factory defaults.
 *
 */
enum uv_status_t uvSettingsInit(){
#if ENABLE_FLASH_SETTINGS
	uv_status setting_success = get_settings_from_flash();
	if(setting_success == UV_ABORTED){
		setting_success = setupDefaultSettings();
	} else if(setting_success == UV_ERROR){
		return UV_ERROR;
	}

	if(setting_success == UV_OK){
		return UV_OK;
	}else{
		//handle the resulting error, maybe soft reboot ngl
	}

#else
	setupDefaultSettings();
#endif


	return UV_OK;

}




