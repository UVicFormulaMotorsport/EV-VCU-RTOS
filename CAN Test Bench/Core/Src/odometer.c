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


/** @brief,
 *
 */
void odometerTask(void* args){

	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast

	float total_distance_m = distance_travelled;
	float avg_speed = 0.0f;
	float speed_kmh = 0.0f;
	uv_CAN_msg msg = {0};
	uint8_t* byte_ptr;

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

		// average the front 2 wheels
		avg_speed = (wheel_speed[0] + wheel_speed[1]) / 2.0f;
		// calculate total distance
		total_distance_m += (avg_speed * 0.001f);
		// keep persistent variable updated
		distance_travelled = total_distance_m;
		// convert to km/h
		speed_kmh = avg_speed * 3.6f;
		// Send speed_kmh over CANbus here
		byte_ptr = (uint8_t*)&speed_kmh;
		msg.data[0] = byte_ptr[0];
		msg.data[1] = byte_ptr[1];
		msg.data[2] = byte_ptr[2];
		msg.data[3] = byte_ptr[3];
		msg.msg_id = 0x500;
		msg.dlc = 4;
		msg.flags = 0x01;
		uvSendCanMSG(&msg);



		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);

	}

}
