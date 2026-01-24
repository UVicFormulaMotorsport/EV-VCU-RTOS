/*
 * odometer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: byo10
 */

#define __UV_FILENAME__ "odometer.c"

#include "uvfr_utils.h"

extern volatile float wheel_speed[];
float distance_travelled;

uv_status initOdometer(void* args){

	uv_task_info* odom_task = uvCreateTask();

	if(odom_task == NULL){
				//Oh dear lawd
		return UV_ERROR;
	}

	distance_travelled = 0; // resets distance traveled to 0 when odometer inits, for now..


			//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
	odom_task->task_name = "Odometer";


	odom_task->task_function = odometerTask;
	odom_task->task_priority = osPriorityNormal;


	odom_task->stack_size = _UV_DEFAULT_TASK_STACK_SIZE;

	odom_task->active_states = UV_READY | UV_DRIVING;
	odom_task->suspension_states = UV_ERROR_STATE;
	odom_task->deletion_states = PROGRAMMING | UV_LAUNCH_CONTROL ;

	odom_task->task_period = 100;

	odom_task->task_args = NULL; //TODO: Add actual settings dipshit

	return UV_OK;
}


/** @brief, gotta know what the distance travelled is fam
 *
 */
void odometerTask(void* args){

	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast

	float total_distance_m = distance_travelled;
	TickType_t ticks_now; // variable to hold current ticks since beginning.
	TickType_t ticks_difference = 0; // variable to hold difference between last time and now.
	TickType_t ticks_last_time_d; // variable holds last time that ticks_difference was updated.

		/**These here lines set the delay. This task executes exactly at the period specified, regardless of how long the task
		 * execution actually takes
		 *
		 @code*/
	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = xTaskGetTickCount();
		/**@endcode */
	for(;;){
		if(params->cmd_data == UV_KILL_CMD){
			distance_travelled = total_distance_m; // Final save to persistent variable
			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){
			distance_travelled = total_distance_m; // Save before suspending
			suspendSelf(params);
		}

		vTaskDelayUntil( &last_time, tick_period);

		ticks_now = xTaskGetTickCount(); // gets current ticks since scheduler began
		ticks_difference = ticks_now - ticks_last_time_d; // calculates time interval for calculating distance
		delta_t = ticks_difference / (configTICK_RATE_HZ / 1000.0f);

		if (delta_t > 100){
			// average the front 2 wheels
			float avg_speed_ms = (wheel_speed[0] + wheel_speed[1]) / 2.0f;
			// calculate total distance
			total_distance_m += (avg_speed_ms * delta_t);
			// keep persistent variable updated
			distance_travelled = total_distance_m;
			// convert to km/h
			float speed_kmh = avg_speed_ms * 3.6f;
			// TODO: Send speed_kmh over CANbus here
			// idk.

			// set ticks_last_time_d to current tick count
			ticks_last_time_d = xTaskGetTickCount();
		}

		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);

	}

}
