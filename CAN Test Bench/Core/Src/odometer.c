/*
 * odometer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: byo10
 */

#define __UV_FILENAME__ "odometer.c"

#include "uvfr_utils.h"

QueueHandle_t wheel_speed_queue = NULL;

float odom_distance_m = 0.0f;  /* registered with DAQ as VEH_DISTANCE_RUN */
float odom_speed_m_s  = 0.0f;  /* registered with DAQ as VEH_SPEED */


uv_status initOdometer(void* args){

	wheel_speed_queue = xQueueCreate(1, sizeof(WheelSpeedData));
	if (wheel_speed_queue == NULL) return UV_ERROR;

	associateDaqParamWithVar(VEH_DISTANCE_RUN, &odom_distance_m);
	associateDaqParamWithVar(VEH_SPEED,        &odom_speed_m_s);

	uv_task_info* odom_task = uvCreateTask();

	if(odom_task == NULL){
				//Oh dear lawd
		return UV_ERROR;
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


/** @brief Integrates wheel speed readings into a running odometer distance.
 *
 * Calls WheelSpeed_UpdateAll() each period, which posts the latest four wheel
 * speeds to wheel_speed_queue via xQueueOverwrite.  The task then reads that
 * snapshot and accumulates distance using the fixed 100 ms task period.
 */
void odometerTask(void* args){

	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast

	TickType_t tick_period = pdMS_TO_TICKS(params->task_period);
	TickType_t last_time = xTaskGetTickCount();

	const float dt_s = params->task_period / 1000.0f;

	for(;;){
		if(params->cmd_data == UV_KILL_CMD){
			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params);
		}
		vTaskDelayUntil(&last_time, tick_period);

		WheelSpeed_UpdateAll();

		WheelSpeedData reading;
		if (xQueueReceive(wheel_speed_queue, &reading, 0) == pdTRUE) {
			odom_speed_m_s = (reading.wheel_speed[0] + reading.wheel_speed[1] +
			                  reading.wheel_speed[2] + reading.wheel_speed[3]) / 4.0f;
			odom_distance_m += odom_speed_m_s * dt_s;
		}
	}

}
