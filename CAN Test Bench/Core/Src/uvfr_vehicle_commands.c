#define __UV_FILENAME__ "uvfr_vehicle_commands.c"

#include "uvfr_utils.h"

typedef struct output_channel_settings output_channel_settings;




static uint8_t TS_status = TS_OFF;
static SemaphoreHandle_t TS_stat_mutex = NULL;

static uint8_t sdc_rb_pwr = 0;

static uint8_t traction_active = 0;
static SemaphoreHandle_t traction_mutex = NULL;

#ifdef VIBECHECK
//Special code here

void vibeCheckInit(){

}

void vCTimCallbackFunc(){

}

void stuckButtonCallBackFunc(){

}

#endif

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

void __attribute__((constructor)) createTSMutexes(){
	TS_stat_mutex = xSemaphoreCreateMutex();
	traction_mutex = xSemaphoreCreateMutex();

}

void uvStopHornCallbackFunc(TimerHandle_t xTim){
	coniferDisChannel(HORN);
}

#define RTD_SOUND_PERIOD 2000

static inline void abortEnergization(){

}



static void uvSetTractiveStatus(uint8_t status){
	if(xSemaphoreTake(TS_stat_mutex,2) == pdTRUE){
		TS_status = status;
		xSemaphoreGive(TS_stat_mutex);
	}
}

static void uvSetTractionFlagStatus(uint8_t status){
	if(xSemaphoreTake(TS_stat_mutex,2) == pdTRUE){
		TS_status = status;
		xSemaphoreGive(TS_stat_mutex);
	}
}

uint8_t uvGetTractiveStatus(){
	uint8_t retval = 0;
	if(xSemaphoreTake(TS_stat_mutex,2) == pdTRUE){
		retval = TS_status;
		xSemaphoreGive(TS_stat_mutex);
	}

	return retval;
}

uint8_t uvGetTractionFlagStatus(){
	uint8_t retval = 0;
	if(xSemaphoreTake(TS_stat_mutex,2) == pdTRUE){
		retval = TS_status;
		xSemaphoreGive(TS_stat_mutex);
	}

	return retval;
}


/** Enables driving, and if needed, it will enable the tractive system
 *
 */
uv_status uvEnableTraction(){
	StaticTimer_t horn_tim_buf;
	TimerHandle_t htim;

	uint8_t ts_stat = uvGetTractiveStatus();

	if(ts_stat == TS_OFF){
		if(uvEnergizeTractiveSystem() != UV_OK){
			return UV_ERROR;
		}
	}else if(ts_stat == TS_ERROR){
		return UV_ERROR;
	}

	ts_stat = uvGetTractiveStatus(); //Did it actually turn on properly?

	if(ts_stat != TS_ACTIVE){
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

	vTaskDelay(RTD_SOUND_PERIOD);


	if(coniferEnChannel(BAMO_RFE)!=UV_OK){
		//ERROR
		uvPanic("RFE ERR",0);
		coniferDisChannel(HORN);
	}

	vTaskDelay(75); //I made this number up ngl

	if(coniferEnChannel(BAMO_RUN)!=UV_OK){
		(void) coniferDisChannel(BAMO_RFE);
		vTaskDelay(10);
		(void) uvOpenSDC();
		uvPanic("RUN ERR",0);
		coniferDisChannel(HORN);
	}

	vTaskDelay(15);

	uvSetTractionFlagStatus(1);




	MC_setErrorMask(0);//Enable all MC ERRORs


	return UV_OK;

}

uv_status uvDisableTraction(){
	coniferDisChannel(BAMO_RUN);
	coniferDisChannel(BAMO_RFE);
	vTaskDelay(5);
	uvSetTractionFlagStatus(0);
	return UV_OK;
}

uv_status uvEnergizeTractiveSystem(){
//	StaticTimer_t horn_tim_buf;
//	TimerHandle_t htim;
	//Final error check
	if(sdc_rb_pwr == 0){
		coniferEnChannel(SDC_BOARD_PWR);
		sdc_rb_pwr = 1;
	}

	vTaskDelay(250);

	if(0){
		return UV_ERROR;
	}


	//BEEP BEEP MOTHERFUCKER LMAO
//	if(coniferEnChannel(HORN)!= UV_OK){
//		//This means that it would be rules compliant for us to start up the car
//		return UV_ERROR;
//	}
//
//	htim = xTimerCreateStatic("horn",RTD_SOUND_PERIOD,pdFALSE,NULL,uvStopHornCallbackFunc,&horn_tim_buf);
//	if(htim == NULL){
//		//Could not create software timer
//		coniferDisChannel(HORN);
//		return UV_ERROR;
//	}
//
//	if(xTimerStart(htim,2)!=pdTRUE){
//		coniferDisChannel(HORN);
//		return UV_ERROR;
//	}


	uvCloseSDC();

	//Wait 5ms, so things can settle down
	vTaskDelay(5);

	//Check for SDC in OK state (should actually activate)

	if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4) != 1){
		//SDC ERROR, SDC ERROR SDC ERROR! SOUND THE ALARM!!!
		//This will require additional handling to determine the cause of the error
		//It is likely a result of something being unplugged, BMS faults should be noticeable, possibly the BSPD is the issue?
		//IDRK what this might be a result of
		//uvOpenSDC();
		//coniferDisChannel(HORN);
		//return UV_ERROR;
	}


	//Await pre-charge success

	vTaskDelay(5000); //I think 5 Seconds is enough time but IDRK


	//Mark as energized


	//How?
	//TODO close the loop here

	//Cycle RFE and RUN

//	if(coniferEnChannel(BAMO_RFE)!=UV_OK){
//		//ERROR
//		uvPanic("RFE ERR",0);
//		coniferDisChannel(HORN);
//	}
//
//	vTaskDelay(75); //I made this number up ngl
//
//	if(coniferEnChannel(BAMO_RUN)!=UV_OK){
//		(void) coniferDisChannel(BAMO_RFE);
//		vTaskDelay(10);
//		(void) uvOpenSDC();
//		uvPanic("RUN ERR",0);
//		coniferDisChannel(HORN);
//	}
//
//	vTaskDelay(15);

	//Re-enable motor controller OC/UC errors


	coniferDisChannel(HORN);
	return UV_OK;

}





uv_status uvDeEnergizeTractiveSystem(){
	coniferDisChannel(BAMO_RUN);
	coniferDisChannel(BAMO_RFE);


	vTaskDelay(10);
	coniferDisChannel(HVIL_PWR);
	MC_setErrorMask(mains_voltage_min_limit|
				rotate_field_enable_not_present_run|
				AC_current_offset_fault);
}
