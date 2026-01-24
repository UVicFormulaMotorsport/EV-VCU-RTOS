/*
 * odometer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: byo10
 */

#define __UV_FILENAME__ "odometer.c"

#include "uvfr_utils.h"

SemaphoreHandle_t xWheelSpeedSem = NULL;
extern volatile float wheel_speed[];
float distance_travelled = 0;

uv_status initOdometer(void* args){

	uv_task_info* odom_task = uvCreateTask();

	if(odom_task == NULL){
				//Oh dear lawd
		return UV_ERROR;
	}

	xWheelSpeedSem = xSemaphoreCreateBinary();

	if (xWheelSpeedSem == NULL) {
	        // Handle error: memory allocation failed
	}

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

	float delta_t = (float)params->task_period / 1000.0f;
	float total_distance_m = distance_travelled;

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

		if (xSemaphoreTake(xWheelSpeedSem, portMAX_DELAY) == pdTRUE){
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
		}

		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);

	}

}
