/** @file driving_loop.c
 *  @brief File containing the meat and potatoes driving loop thread, and all supporting functions
 *
 */



#include "main.h"
#include "uvfr_utils.h"
#include "can.h"
#include "motor_controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h> // move somewhere else
#include <stdio.h>
#include <math.h>

#include "driving_loop.h"

//External Variables:
extern uint16_t adc1_APPS1; //These are the locations for the sensor inputs for APPS and BPS
extern uint16_t adc1_APPS2;
extern uint16_t adc1_BPS1;
extern uint16_t adc1_BPS2; // Brake


driving_loop_args default_dl_settings;//TODO DECIDE WHAT YOU WANT DEFAULT SETTINGS TO BE

driving_loop_args* driving_args = NULL;


bool is_accelerating = false;
float T_PREV = 0;
float T_REQ = 0;

static bool torque_inhibit_active = false; //track state of torque acceptance
float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2);
float calculateBrakePercentage(uint16_t bps1);
bool performSafetyChecks(driving_loop_args* dl_params, uint16_t apps1_value,uint16_t apps2_value, uint16_t bps1_value, uint16_t bps2_value, enum DL_internal_state* dl_status);

//timeout thresholds
#define INPUT_TIMEOUT_MS 500
#define THROTTLE_CHANGE_THRESHOLD 5   // percentage
#define BRAKE_CHANGE_THRESHOLD 5      // percentage

//driving loop state tracking
static TickType_t last_input_change_time = 0;
static float last_throttle_percent = 0.0f;
static float last_brake_percent = 0.0f;

#define TORQUE_DECAY_STEP 2.5f //// Nm per loop step (adjust as needed)
#define THROTTLE_ZERO_THRESHOLD 0.01f // // Below this % throttle, we consider "off"
static bool sent_zero_torque = false; //// Track if we already dropped torque to 0

//define default driving loop settings
driving_loop_args default_dl_settings = {
    .absolute_max_acc_pwr = 10,       // Set appropriate default max power in watts
    .absolute_max_motor_torque = 230,       // Nm
    .absolute_max_accum_current = 200,      // Amps
    .max_accum_current_5s = 200,            // Amps for 5s burst

    .absolute_max_motor_rpm = 6500,        // Max RPM
    .regen_rpm_cutoff = 1000,               // Below this, regen is off

	.min_apps_offset = 0,					 /**<minimum APPS offset */
	.max_apps_offset = 0,					 /**< maximum APPS offset */
	.min_apps_value = 0,	/**< for detecting disconnects and short circuits*/

	.apps1_abs_max_val = 0x10C4,					/**< for detecting disconnects and short circuits*/
	.apps1_abs_min_val = 0x0200,					/**< for detecting disconnects and short circuits*/
	.apps2_abs_min_val = 0x0202,					/**< for detecting disconnects and short circuits*/
	.apps2_abs_max_val = 0x1029,

	.min_BPS_value = 0x0106,						/**< are the brakes valid?*/
	.max_BPS_value = 0x0B7E,						 /**< are the brakes valid?*/

    .apps1_top = 0x0993,                       //idk
    .apps1_bottom = 0x0570,
	.apps2_top = 0x0347,
	.apps2_bottom = 0x02B0, //idk

    .apps_plausibility_check_threshold = 200,	//idk
    .bps_plausibility_check_threshold = 500,	//idk
    .bps_implausibility_recovery_threshold = 300,	//idk
    .apps_implausibility_recovery_threshold = 100,	//idk

    .num_driving_modes = 1,					//idk
    .period = 10,                           // ms, how often loop runs
    .accum_regen_soc_threshold = 90,        // Above this SOC, regen disabled

    .dmodes ={0}                          // Set default driving modes (all 0 for now)
};


//driving_loop_args* driving_args = NULL;

//bool is_accelerating = false;
//float T_PREV = 0;
//float T_REQ = 0;

//allows torque filter to use getKvalue even if it's defined after
static inline float getKValue(int raceMode);

bool performSafetyChecks(driving_loop_args* dl_params, uint16_t apps1_value,uint16_t apps2_value, uint16_t bps1_value, uint16_t bps2_value, enum DL_internal_state* dl_status);

//define diff chennels for adcs
enum uv_status_t initDrivingLoop(void *argument){
	associateDaqParamWithVar(APPS1_ADC_VAL, &adc1_APPS1);
	associateDaqParamWithVar(APPS2_ADC_VAL, &adc1_APPS2);
	associateDaqParamWithVar(BPS1_ADC_VAL, &adc1_BPS1);
	associateDaqParamWithVar(BPS2_ADC_VAL, &adc1_BPS2);

	//allocate memory for the task
	uv_task_info* dl_task = uvCreateTask();

	if(dl_task == NULL){
		//Oh dear lawd if allocation fails return error
		return UV_ERROR;
	}

	//grab those driving loop settings
	driving_args = current_vehicle_settings->driving_loop_settings;

	//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.

	dl_task->task_name = "Driving_Loop"; //assign names
	dl_task->task_function = StartDrivingLoop; //defining the function that the task will run
	dl_task->task_priority = osPriorityHigh; // assigns a high priority in FreeRTOS
	dl_task->stack_size = 256; // memory allocation for task execution
	dl_task->active_states = UV_DRIVING; // Specifies when the task should be active
	dl_task->suspension_states = 0x00;


	dl_task->deletion_states = UV_INIT|UV_READY | PROGRAMMING | UV_SUSPENDED | UV_LAUNCH_CONTROL | UV_ERROR_STATE;
	dl_task->task_period = 100; //runs every 100ms or 0.1 seconds
	dl_task->task_args = NULL;

	return UV_OK; //
}
// function to map throttle percent to torque value
inline static float mapThrottleToTorque(float throttle_percent) {
		static float T_prev = 0.0f; // Stores the last filtered torque value. for filtering to check if decelerating
	    //float throttle_percent = calculateThrottlePercentage(apps1, apps2);
	    if(throttle_percent == 0.0f){
	    	return 0.0f;
	      }
	    double T_MAX = driving_args->absolute_max_motor_torque; // got this from driving_loop.h file
	    float torque_request_current =  (throttle_percent / 100.0f) * T_MAX;
	    float torque_request = T_prev;
	    T_prev = torque_request_current;
	    return torque_request;
}

//race modes
#define ACCELERATION 0
#define AUTOCROSS 1
#define ENDURANCE 2

//function to get K value aka what mode we are in
inline float getKValue(int raceMode) {
		float kVal = 0.3; //Default for if racemode Does not exist
		if(raceMode == ACCELERATION) {
			return kVal = 0.7;
		}else if (raceMode == AUTOCROSS){
			return kVal = 0.4;
		}else if (raceMode == ENDURANCE) {
			return kVal = 0.2;
		}
		return kVal;
}

//function to calculate throttle percentage
float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2) {
	    // Ensure both sensor values are within the valid range
	    //if (apps1 < driving_args->min_apps1_value || apps1 > driving_args->apps1_abs_max_val || apps2 < driving_args->apps2_abs_min_val || apps2 > driving_args->apps2_abs_max_val) return 0.0f;
	    if(apps1 < driving_args->apps1_bottom){
	    	return 0;
	    }

	    if(apps1 > driving_args->apps1_top){
	    	return 100.0f;
	    }
	    // Compute throttle percentage using linear interpolation
	    float throttle_percent = ((float)(apps1 - driving_args->apps1_bottom) / (driving_args->apps1_top - driving_args->apps1_bottom)) * 100.0f;

	    // SAFETY CHECK: Verify APPS1 and APPS2 values are within 10% of each other
	    float apps_diff = fabs((float)apps1 - (float)apps2) / (float)apps1;
	    if (apps_diff > 0.1f) {
	        // Sensors are out of sync, return 0% to prevent errors
	    	//printf("WARNING: APPS sensors out of sync! Returning 0%% throttle.\n");
	    	//uvPanic("idek",0);
	        //return 0.0f;
	    }
	    //else if (apps_diff <= 0.1f){
	    	// Next function call calcThrottlePercentage(param)
	    	 //* OR return something
	    	// *
	    return throttle_percent;
}

//function to calculate throttle percentage
float calculateBrakePercentage(uint16_t bps1) {
	    // Ensure both sensor values are within the valid range
	    if (bps1 < driving_args->min_BPS_value || bps1 > driving_args->max_BPS_value ) return 0.0f;

	    // Compute brake percentage using linear interpolation
	    float brake_percent = ((float)(bps1 - driving_args->min_BPS_value) / (driving_args->max_BPS_value - driving_args->min_BPS_value)) * 100.0f;

	    return brake_percent;
}


/**
	 * @brief  Applies filtering to smooth torque transitions.
	 *
	 * @param  T_req: Requested torque before filtering.
	 * @param  T_prev: Previous filtered torque value.
	 *
	 * @return Smoothed torque value.
* */
inline static float applyTorqueFilter(float T_req, float T_prev, bool is_accelearting) {
	    // Filtering formula: T_filtered = T_prev + (T_req - T_prev) * k
	    //return T_prev + (T_req - T_prev) * FILTER_K;
		// except hehehehhe were gonna create a get_k function based on race modes
		float FILTER_K = getKValue(0);
		if (is_accelerating) {
			FILTER_K = 0.4; // smooth acceleration (adjustable for endurance vs sport mode

		}else{
			FILTER_K = 1.0; // INSTANT to
		}

		// Calculate filtered torque
		float T_filtered =  T_prev + (T_req - T_prev) * FILTER_K;

		// Ensures we do not hold residual torque when stopping
		if (T_req == 0) {
			T_filtered = 0; // If stopping, is requested torque
		}
		return T_filtered;

}



/** Rachan
	 * @brief  Sends the filtered torque value to the motor controller.
	 *
	 * @param  T_filtered: Final torque value after filtering.
*/

// Start of Driving Loop
void StartDrivingLoop(void * argument){
	//Initialize driving loop now

	//extracting task arguments, and gets parameters like min/max allowed values for the APPS and BPS
	uv_task_info* params = (uv_task_info*) argument;

	enum DL_internal_state dl_status = Plausible; // no issues are detected

	/** This line extracts the specific driving loop parameters as specified in the
	 * vehicle settings
	 @code*/
	driving_loop_args* dl_params = current_vehicle_settings->driving_loop_settings;

	//Timeout values
	//static TickType_t last_input_change_time = 0;
	//static float last_throttle_percent = 0.0f;
	//static float last_brake_percent = 0.0f;

	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = xTaskGetTickCount();
	last_input_change_time = last_time;


	/**@endcode */
	for(;;){ // enters infinite loop

		if(params->cmd_data == UV_KILL_CMD){ // to perform task control (suspend/kill)

			killSelf(params);

		}else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params); // if _UV_SUSPEND_CMD received pause the task
		}
		vTaskDelayUntil( &last_time, tick_period); //Me and the boys on our way to wait for a set period every 100ms

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); //Blink and LED (for debugging)

		//Copy the values over into new local variables, in order to avoid messing up the APPS
		uint16_t apps1_value = adc1_APPS1; // reading sensor values
		uint16_t apps2_value = adc1_APPS2;

		uint16_t bps1_value = adc1_BPS1;
		uint16_t bps2_value = adc1_BPS2;
		float T_filtered = 0;

		bool safe = performSafetyChecks(dl_params, apps1_value, apps2_value, bps1_value, bps2_value, &dl_status);

		if(!safe) {
			// if safety check fails, handle error (stop?)
			//continue;
			//MC_Shutdown();
			T_filtered = 0;
			sendTorqueToMotorController(T_filtered);
			continue;
		}

//		if(dl_status == Plausible){

			//implement motor control logic here

			//Compute throttle %
			float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value);
			float brake_percent = calculateBrakePercentage(bps1_value);


			// ---------------------- not used atm
			// How much did the throttle or brake change since last time?
			float throttle_delta = fabs(throttle_percent - last_throttle_percent);
			float brake_delta = fabs(brake_percent - last_brake_percent);

			//TickType_t now = xTaskGetTickCount();
			TickType_t timeout_ticks = pdMS_TO_TICKS(INPUT_TIMEOUT_MS);

			// If either % changed significantly, update the "last time the driver did something"
			// Only update the time if there's meaningful input change
			if (throttle_delta > THROTTLE_CHANGE_THRESHOLD || brake_delta > BRAKE_CHANGE_THRESHOLD) {
			    last_driver_input_time = xTaskGetTickCount();
			    last_throttle_percent = throttle_percent;
			    last_brake_percent = brake_percent;
			}

			// Check for input timeout
			// If it's been too long since the last meaningful input
			//bool input_timeout = (last_driver_input_time - last_input_change_time) > timeout_ticks;

			bool input_timeout = (xTaskGetTickCount() - last_input_change_time) > timeout_ticks;

			//-------------------

			//APPS and Brake Pedal Plausibility Check
			// 1. Trigger torque inhibit if APPS > 25% and Brake is pressed
			if (throttle_percent > 25.0f && brake_percent > 5.0f) {
			    torque_inhibit_active = true;
			}
			// 2. Do NOT reset torque inhibit until BOTH are released to < thresholds
			if (torque_inhibit_active && throttle_percent < 5.0f && brake_percent < 5.0f) {
			    torque_inhibit_active = false;
			}
			// 3. If inhibit is active, override torque request
			if (torque_inhibit_active) {
			    T_filtered = 0.0f;
			}
			// Enforce torque cutoff if:
			// 4. No driver input detected for too long (input_timeout)
			// 5. Torque inhibit is active due to APPS >25% and Brake >5% (per FSAE Rule T.4.2.5)
			//    → Torque must remain zero until APPS <5% and Brake <5% to clear inhibit
			//if (input_timeout || torque_inhibit_active) { //this might cause the inverter to break bc it'll cut torque after 500ms of no pedal?
			if (torque_inhibit_active){
			    T_filtered = 0.0f;
			    sent_zero_torque = true;
			}else{ //normal driving logic
				// 2. Map to torque request
				T_REQ = mapThrottleToTorque(throttle_percent);

				// 3. Determine acceleration status
				is_accelerating = (T_REQ >= T_PREV);

				// 4. Apply filtering to smooth torque
				T_filtered = applyTorqueFilter(T_REQ, T_PREV, is_accelerating);
				// 6. Update previous torque
				//todo: fix T_filter scaling
				//half the requested torque
				T_filtered = T_filtered/2;
			}

			// 5. Send torque to motor controller via motor_controller.c
			if(vehicle_state == UV_DRIVING){
				sendTorqueToMotorController(T_filtered);
			}else{
				sendTorqueToMotorController(0.0);
			}


			T_PREV = T_filtered;

	//}

	}





}



/**
	 * @brief  Performs safety checks on APPS (Throttle) and BPS (Brake) sensors.
	 *
	 * This function ensures that:
	 *  - Throttle position sensors (APPS1 & APPS2) are within 10% of each other.
	 *  - Brake pressure sensors (BPS1 & BPS2) are within 5% of each other.
	 *  - Sensors are within their expected min/max ranges.
	 *  - Brake and throttle are not pressed at the same time.
	 *
	 * If a **fatal error** is detected (e.g., sensor out of range), the function:
	 *  - **Stops the motor**.
	 *  - **Kills the task execution** (`killSelf()`).
	 *
	 * If a **non-fatal error** occurs (e.g., sensor mismatch exceeding the limit):
	 *  - **Stops the motor**.
	 *  - **Suspends the task temporarily** (`suspendSelf()`).
	 *
	 * If safety conditions return to normal, the function:
	 *  - **Restarts the motor**.
	 *
	 * @param dl_params Pointer to the driving loop parameters.
	 * @param apps1_value Raw sensor reading from APPS1.
	 * @param apps2_value Raw sensor reading from APPS2.
	 * @param bps1_value Raw sensor reading from BPS1.
	 * @param bps2_value Raw sensor reading from BPS2.
	 * @param params Pointer to the current task information.
	 * @param dl_status Pointer to the driving loop internal state.
	 *
	 * @retval true  All safety checks passed.
	 * @retval false One or more safety checks failed.
	 */
bool performSafetyChecks(driving_loop_args* dl_params,uint16_t apps1_value,uint16_t apps2_value,uint16_t bps1_value,uint16_t bps2_value,enum DL_internal_state* dl_status){

    //sensor scaling
    float apps1_ratio = 0;
    float apps2_ratio = 0;

    // Normalize APPS1
    if (apps1_value < dl_params->apps1_bottom){
    	apps1_ratio = 0;
    }
    else if (apps1_value > dl_params->apps1_top){
    	apps1_ratio = 1.0f;
    }
    else{
    	apps1_ratio = ((float)(apps1_value - dl_params->apps1_bottom)) / (dl_params->apps1_top - dl_params->apps1_bottom);
    }

    // Normalize APPS2
    if (apps2_value < dl_params->apps2_bottom) {
    	apps2_ratio = 0;
    }
    else if (apps2_value > dl_params->apps2_top){
    	apps2_ratio = 1.0f;
    }
    else {
    	apps2_ratio = ((float)(apps2_value - dl_params->apps2_bottom)) / (dl_params->apps2_top - dl_params->apps2_bottom);
    }
    // Calculate the absolute difference between APPS1 and APPS2 as a percentage
    // (FSAE Rule T.4.2.4: Implausibility is deviation >10% for >100ms)
    float apps_diff_percent = fabsf(apps1_ratio - apps2_ratio) * 100.0f;


    //FATAL RANGE ERRORS
    //APPS1 Range
    if (apps1_value < dl_params->apps1_abs_min_val || apps1_value > dl_params->apps1_abs_max_val) {
        MC_Shutdown();
        return false;
    }
    //APPS2 Range
    if (apps2_value < dl_params->apps2_abs_min_val || apps2_value > dl_params->apps2_abs_max_val) {
        MC_Shutdown();
        return false;
    }
    //BPS1 Range
    if (bps1_value < dl_params->min_BPS_value || bps1_value > dl_params->max_BPS_value) {
        MC_Shutdown();
        return false;
    }
    //BPS2 Range
    if (bps2_value < dl_params->min_BPS_value || bps2_value > dl_params->max_BPS_value) {
        MC_Shutdown();
        return false;
    }

    //APPS MISMATCH CHECK. If greater then 10% then shutdown.
    //if (apps_diff_percent > 25.0f) {
    if (apps_diff_percent > dl_params->apps_plausibility_check_threshold){

        //*dl_status = Implausible;
    	MC_Shutdown();
        return false;
    }

    //THROTTLE + BRAKE Percent
    float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value);
    float brake_percent = calculateBrakePercentage(bps1_value);

    //RECOVERY CHECK. If implausible on return to original state after both brake and pedal released
//    if (*dl_status == Implausible &&
//        throttle_percent < dl_params->apps_implausibility_recovery_threshold &&
//        brake_percent < dl_params->bps_implausibility_recovery_threshold)
//    {
//        *dl_status = Plausible;
//    }

    if (torque_inhibit_active && throttle_percent < dl_params->apps_implausibility_recovery_threshold && brake_percent < dl_params->bps_implausibility_recovery_threshold){
        torque_inhibit_active = false;
    }


    return true; // All good
}

