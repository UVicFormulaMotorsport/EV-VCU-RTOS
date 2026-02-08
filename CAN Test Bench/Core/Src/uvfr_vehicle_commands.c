#define __UV_FILENAME__ "uvfr_vehicle_commands.c"

#include "uvfr_utils.h"

typedef struct output_channel_settings output_channel_settings;

void BeepBeepMotherFucker(){
	coniferEnChannel(HORN);
	vTaskDelay(300);
	coniferToggleChannel(HORN);
	vTaskDelay(300);
	coniferToggleChannel(HORN);
	vTaskDelay(300);
	coniferDisChannel(HORN);
}


/** @brief Function to put vehicle into safe state.
 *
 * Should perform the following functions in order:
 * - Prevent new MC torque or speed requests
 * - Open shutdown cct
 *
 */
void uvSecureVehicle(){
	//Stop MCU Torque requests
	MC_Shutdown();

	coniferDisChannel(BAMO_RFE);
	vTaskDelay(2);
	//open SDC >:)
	uvOpenSDC();

}

void uvStopHornCallbackFunc(TimerHandle_t xTim){
	coniferDisChannel(HORN);
}

#define RTD_SOUND_PERIOD 3000

static inline void abortEnergization(){

}

uv_status uvEnergizeTractiveSystem(){
	StaticTimer_t horn_tim_buf;
	TimerHandle_t htim;
	//Final error check

	if(0){
		return UV_ERROR;
	}


	//BEEP BEEP MOTHERFUCKER LMAO
	if(coniferEnChannel(HORN)!= UV_OK){
		//This means that it would be rules compliant for us to start up the car
		return UV_ERROR;
	}

	htim = xTimerCreateStatic("horn",RTD_SOUND_PERIOD,pdFALSE,NULL,uvStopHornCallbackFunc,&horn_tim_buf);
	if(htim == NULL){
		//Could not create software timer
		coniferDisChannel(HORN);
		return UV_ERROR;
	}

	if(xTimerStart(htim,2)!=pdTRUE){
		coniferDisChannel(HORN);
		return UV_ERROR;
	}


	uvCloseSDC();

	//Wait 5ms, so things can settle down
	vTaskDelay(5);

	//Check for SDC in OK state (should actually activate)

	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4) != 1){
		//SDC ERROR, SDC ERROR SDC ERROR! SOUND THE ALARM!!!
		//This will require additional handling to determine the cause of the error
		//It is likely a result of something being unplugged, BMS faults should be noticeable, possibly the BSPD is the issue?
		//IDRK what this might be a result of
		uvOpenSDC();
		return UV_ERROR;
	}


	//Await pre-charge success

	vTaskDelay(5000); //I think 5 Seconds is enough time but IDRK

	//How?

	//Cycle RFE and RUN

	if(coniferEnChannel(BAMO_RFE)!=UV_OK){
		//ERROR
		uvPanic("RFE ERR",0);
	}

	vTaskDelay(75); //I made this number up ngl

	if(coniferEnChannel(BAMO_RUN)!=UV_OK){
		(void) coniferDisChannel(BAMO_RFE);
		vTaskDelay(10);
		(void) uvOpenSDC();
		uvPanic("RUN ERR",0);
	}

	vTaskDelay(15);

	//Re-enable motor controller OC/UC errors
	MC_setErrorMask(0);//Enable all MC ERRORs
	return UV_OK;

}
