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


float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2);
bool performSafetyChecks(driving_loop_args* dl_params, uint16_t apps1_value,uint16_t apps2_value, uint16_t bps1_value, uint16_t bps2_value, enum DL_internal_state* dl_status);

enum uv_status_t initDrivingLoop(void *argument){
	uv_task_info* dl_task = uvCreateTask(); // allocate memory for the task

	if(dl_task == NULL){
		//Oh dear lawd if allocation fails return error
		return UV_ERROR;
	}

	driving_args = current_vehicle_settings->driving_loop_settings;


	//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.

	//dl_task->task_name = malloc(16*sizeof(char));
	//if(dl_task->task_name == NULL){
		//return UV_ERROR; //failed to malloc the name of the thing
	//}
	dl_task->task_name = "Driving_Loop"; // assign name


	dl_task->task_function = StartDrivingLoop; // defining the function the task will run
	dl_task->task_priority = osPriorityHigh; // assigns a high priority in FreeRTOS

	dl_task->stack_size = 256; // memory allocation for task execution

	dl_task->active_states = UV_DRIVING; // Specifies when the task should be active
	dl_task->suspension_states = 0x00;


	dl_task->deletion_states = UV_INIT|UV_READY | PROGRAMMING | UV_SUSPENDED | UV_LAUNCH_CONTROL | UV_ERROR_STATE;


	dl_task->task_period = 100; //runs every 100ms or 0.1 seconds

	dl_task->task_args = NULL; //TODO: Add actual settings dipshit

	return UV_OK; //
}

inline static float mapThrottleToTorque(float throttle_percent) {
		static float T_prev = 0.0f; // Stores the last filtered torque value. for filterting to check if decelearting
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

#define ACCELERATION 0
#define AUTOCROSS 1
#define ENDURANCE 2


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

	float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2) {
	    // Ensure both sensor values are within the valid range
	    if (apps1 < driving_args->min_apps1_value || apps1 > driving_args->max_apps1_value || apps2 < driving_args->min_apps2_value || apps2 > driving_args->max_apps2_value) return 0.0f;

	    // Compute throttle percentage using linear interpolation
	    float throttle_percent = ((float)(apps1 - driving_args->min_apps1_value) / (driving_args->max_apps1_value - driving_args->min_apps1_value)) * 100.0f;

	    // SAFETY CHECK: Verify APPS1 and APPS2 values are within 10% of each other
	    float apps_diff = fabs((float)apps1 - (float)apps2) / (float)apps1;
	    if (apps_diff > 0.1f) {
	        // Sensors are out of sync, return 0% to prevent errors
	    	printf("WARNING: APPS sensors out of sync! Returning 0%% throttle.\n");
	    	uvPanic("idek",0);
	        return 0.0f;
	    }
	    //else if (apps_diff <= 0.1f){
	    	// Next function call calcThrottlePercentage(param)
	    	 //* OR return something
	    	// *
	    return throttle_percent;
	}


	/** Rachan
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

	    //if tnext > treq {tnext = treq}

	    //send value to sent
	}



	/** Rachan
	 * @brief  Sends the filtered torque value to the motor controller.
	 *
	 * @param  T_filtered: Final torque value after filtering.
	*/
	static void sendTorqueToMotorController(float T_filtered) {
	    // Placeholder function to send torque command to motor controller
		MotorControllerSpinTest(T_filtered);
	}

/**@brief  Function implementing the ledTask thread.
 * @param  argument: Not used for now. Will have configuration settings later.
 * @retval None
 *
 * This function is made to be the meat and potatoes of the entire vehicle.
 *
 */
void StartDrivingLoop(void * argument){
	//Initialize driving loop now

	/** The first thing we do here is create some local variables here, to cache whatever variables need cached.
	 *	We will be caching variables that are used very frequently in every single loop iteration, and are not
	 */


	//extracting task arguments, and gets parameters like min/max allowed values for the APPS and BPS
	uv_task_info* params = (uv_task_info*) argument;
	/*
	uint16_t min_apps_value;
	uint16_t max_apps_value;

	uint16_t min_apps_offset;
	uint16_t max_apps_offset;
	*/
	enum DL_internal_state dl_status = Plausible; // no issues are detected

	/** This line extracts the specific driving loop parameters as specified in the
	 * vehicle settings
	 @code*/
	driving_loop_args* dl_params = (driving_loop_args*) params->task_args;
	/**@endcode*/

	/*
	min_apps_value = dl_params->min_apps_value;
	max_apps_value = dl_params->max_apps_value;

	min_apps_offset = dl_params->min_apps_offset; //minimum APPS offset
	max_apps_offset = dl_params->max_apps_offset;
	*/

	/**These here lines set the delay. This task executes exactly at the period specified, regardless of how long the task
		 * execution actually takes
		 * rachan: ensures the function runs exactly 100ms, regardless of execution time.
		 *
		 @code*/
	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = xTaskGetTickCount();





	/**@endcode */
	for(;;){ // enters infinite loop




		if(params->cmd_data == UV_KILL_CMD){ // to perform task control (suspend/kill)

			killSelf(params);

		}else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params); // if _UV_SUSPEND_CMD received pause the task
		}
		vTaskDelayUntil( &last_time, tick_period); //Me and the boys on our way to wait for a set period every 100ms

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); //Blink and LED (for debugging)
		//Read stuff from the addresses

		//Copy the values over into new local variables, in order to avoid messing up the APPS
		uint16_t apps1_value = adc1_APPS1; // reading sensor values
		uint16_t apps2_value = adc1_APPS2;

		uint16_t bps1_value = adc1_BPS1;
		uint16_t bps2_value = adc1_BPS2;


		// SAFETY CHECKS
		// Rachan
		// APPS1/APPS2 CHECK (Throttle Position Sensors)
		// NEW function for safety checks

		bool safe = performSafetyChecks(dl_params, apps1_value, apps2_value, bps1_value, bps2_value, &dl_status);

		if(!safe) {
			// if safety check fails, handle error (stop?)
			continue;
		}

		if(dl_status == Plausible){
			//TODO This.
			//implement motor control logic here
			// dont have to start as an if statement just idea
		}

		 // **Determine if accelerating or decelerating**
		is_accelerating = (T_REQ >= T_PREV);


		// if vehicle state is equal to drivibg and only oif its equal to dribvibng
		// change states
		// to avoid edge case

		//Command the motor controller to do the thing
		if(1){//if(1) reminds me that there should be a

		}

		//DL_end:
		//Set loose the ADC, and have it DMA the result into the variables


		//Wait until next D.L. occurance
		//osDelay(DEFAULT_PERIOD);
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


}




		/*
			// ** Throttle (APPS) Checks**
			if(apps1_value < dl_params->min_apps_value){
				//APPS1 is probably unplugged. Act accordingly
				return false; // stop
			}else if(apps1_value > dl_params->max_apps_value){
				//indicative of APPS1 short (APPS1 short circuit detected)
				//goto DL_end;
				return false;
			}


			if(apps2_value < dl_params->max_apps_value){
				//APPS2 is probably unplugged. Act accordingly.
				return false;
			}else if(apps2_value > dl_params->max_apps_value){
				//indicative of APPS2 short circuit detected
				return false;
			}


			// ** Check APPS sensor synchronization
			//uint16_t apps_diff = apps1_value - apps2_value;
			uint16_t apps_diff = (apps1_value > apps2_value)
						? (apps1_value - apps2_value)	// IF apps1 > apps2, do apps1 -apps2
						: (apps2_value - apps1_value); // IF apps2 > apps1, do apps2 - apps1

			if((apps_diff > dl_params->max_apps_offset)||(apps_diff < dl_params->min_apps_offset)){
				//APPS1 and APPS2 are no longer in sync with each other
				return false;
			}

			// BPS1/BPS2 Check (Brake Pressure Sensors)
			//  ** Throttle (BPS) Checks**
			if(bps1_value < dl_params->min_BPS_value){
				// Possible BPS1 disconnection
				return false;
			}else if(bps1_value > dl_params->max_BPS_value){
				// Possible BPS1 short circuit
				return false;
			}

			//Repeat these safety checks for BPS 2.
			if(bps2_value < dl_params->min_BPS_value){
				return false;

			}else if(bps2_value > dl_params->max_BPS_value){
				return false;

			}

			//Brake Plausibility Check

			/** Brake Plausibility Check
			 *
			 * The way that this works is that if the brake pressure is greater than some threshold,
			 * and the accelerator pedal position is also greater than some threshold, the thing will
			 * register that a brake implausibility has occurred. This is not very cash money.
			 *
			 * If this happens, we want to set the torque/speed output to zero. This will only reset itself once
			 * the brakes are set to less than a certain threshold. Honestly evil.
			 *
			 * Rachan: IF both accelerator and brake are pressed at the same time, the system registers a fault and cuts motor power.


			if((bps1_value > dl_params->bps_plausibility_check_threshold) && (apps1_value > dl_params->apps_plausibility_check_threshold)){
				//This means that the pedal and brake are checked simultaneously
				dl_status = Implausible; // set status to error
				// error handler
				return false;
			}

			//possible return to plausibility
			if((dl_status == Implausible) && (1)){
				dl_status = Plausible;
			}

			return true; // All checks passed

			//Compute torque/velocity or whatever
			//if(dl_status == Plausible){
				// Compute motor output
			}
	}


	*/


	/**rachan
	 * @brief Converts APPS sensor readings into a throttle percentage
	 *
	 * @param apps1: Raw voltage value from APPS1 sensor
	 * @param apps2: Raw voltage value from APPS2 sensor
	 *
	 * TO DO: Set RPM_MAX & RPM_MIN
	 * @return Throttle percentage (0% to 100%).
	 *
	 */


	/** AMMAR

	@brief  Maps Throttle Percentage to Torque Request.
	Call the calculateThrottlePercentage function to get the value
	@param  throttle_percent: Throttle percentage (0% - 100%).
	*
	@return Requested torque in Nm.
	*/
bool performSafetyChecks(driving_loop_args* dl_params, uint16_t apps1_value,uint16_t apps2_value, uint16_t bps1_value, uint16_t bps2_value, enum DL_internal_state* dl_status) {
//Perform input validation and ensures values are within the expected range

	// Convert APPS values to a 0-1 float scale
	// This helps us in calculating the percentage difference between the two throttle sensors
	// can do this as a function call
	float apps1_ratio = ((float)(apps1_value - dl_params->min_apps1_value)/(dl_params->max_apps1_value - dl_params->min_apps1_value));
	float apps2_ratio = ((float)(apps2_value - dl_params->min_apps2_value)/(dl_params->max_apps2_value - dl_params->min_apps2_value));
	float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value);

	// Convert BPS values to a 0-1 float scale
	float bps1_ratio = ((float)(bps1_value - dl_params->min_BPS_value) / (dl_params->max_BPS_value - dl_params->min_BPS_value));
	float bps2_ratio = ((float)(bps2_value - dl_params->min_BPS_value) / (dl_params->max_BPS_value - dl_params->min_BPS_value));

	// Compute percentage differences since 2 different spots
	float apps_percentage_diff = fabs(apps1_ratio - apps2_ratio) * 100.0f; // APP1 and APP2
	float bps_percentage_diff = fabs(bps1_ratio - bps2_ratio) * 100.0f; // BPS1 and BPS2

	// Debugging Log Sensor Values
	//printf("APPS1: %f, APPS2: %f, Diff: %f\n", apps1_ratio, apps2_ratio, apps_percentage_diff);
	//printf("BPS1: %.2f, BPS2: %.2f, Diff: %.2f%%\n", bps1_ratio, bps2_ratio, bps_percentage_diff);


	// Fatal Errors APPS Sensors are out of Range shut down motor
	if (apps1_value < dl_params->min_apps1_value || apps1_value > dl_params->max_apps2_value){
		printf("ERROR: APPS1: out of range! Stopping motor.\n");
		//stop_commmand();
		//killself(params);
		uvPanic("idek",0);
		return false;
	}
	if (apps2_value < dl_params->min_apps2_value || apps2_value > dl_params->max_apps2_value) {
	    printf("ERROR: APPS2 out of range! Stopping motor.\n");
	    //stop_command();
	    //killSelf(params); // combination of these two can be replaced with UV_panic
	    uvPanic("idek",0);
	    return false;
	    }

	// Non-fatal Errors : APPS sensors mismatch greater than 10%, suspend task, stop motor
	if (apps_percentage_diff > 10.0f){
		//printf("WARNING: APPS sensors out of sync (%.2f%%)! Suspending task. \n", apps_percentage_diff);
		// output 0 as in no torque request
		// stop_command();
		//suspendSelf(params);
		uvPanic("idek",0);
		return false;
	}

	// Fatal Errors BPS sensor our of range
	if (bps1_value < dl_params->min_BPS_value || bps1_value > dl_params->max_BPS_value) { // BPS1
		//printf("ERROR: BPS1 out of range! Stopping motor.\n");
		//stop_command();
		//killself(params);
		uvPanic("idek",0);
		return false;
	}

	if (bps2_value < dl_params->min_BPS_value || bps2_value > dl_params->max_BPS_value) { // BPS1
		printf("ERROR: BPS2 out of range! Stopping motor.\n");
		//stop_command();
		//killself(params); // uv_panic
		uvPanic("idek",0);
		return false;
	}

	// Non fatal errors: BPS sensors mismatch greater than 5%, suspend tasks, stop motor
	if (bps_percentage_diff > 5.0f){
		//printf("WARNING: BPS sensors are out of sync (%.2f%%)! Suspending task.\n", bps_percentage_diff);
		// output 0 as in no  request
		//stop_command();
		//suspendSelf(params);
		uvPanic("idek",0);
		return false;
	}


	// Brake Plausibility Check: Prevent simultaneous throttle and brake
	 if ((bps1_value > dl_params->bps_plausibility_check_threshold) && (apps1_value > dl_params->apps_plausibility_check_threshold)) {
		 printf("WARNING: Brake and accelerator pressed simultaneously! Suspending task.\n");
	     //stop_command(); // return 0
	     *dl_status = Implausible;
	     //suspendSelf(params);
	     return false;
	    }


	 // System Recovery: resume motor if previously in implausible state
	 if (*dl_status == Implausible) {
		 if(1){ //NOTE THE RULE ON WHERE THE PEDAL MUST BE FOR THIS TO HAPPEN
	     *dl_status = Plausible;
		 }
	     printf("INFO: Safety conditions normal. Motor can resume.\n");
	     //spin_motor(); output torque request to motor_controller
	     }

	     return true; //All checks passed, motor remains active





}


