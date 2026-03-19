/*
 * ready_to_drive.c
 *
 *  Created on: Apr 28, 2025
 *      Author: karni
 */
#define __UV_FILENAME__ "ready_to_drive.c"
#include "uvfr_utils.h"

void rtdTask(void* args);

extern uint16_t adc1_BPS1;

uv_status initRTDtask(void* args){
	(void)args;
	uv_task_info* rtd_task = uvCreateTask();

		if(rtd_task == NULL){
						//Oh dear lawd
			return UV_ERROR;
		}


					//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
		rtd_task->task_name = "rtd";

		rtd_task->task_function = rtdTask;
		rtd_task->task_priority = 2; //Slightly more important than the children tasks

		rtd_task->stack_size = 128;

		rtd_task->active_states = UV_READY;
		rtd_task->suspension_states = 0x00;
		rtd_task->deletion_states = PROGRAMMING | UV_DRIVING | UV_ERROR_STATE;

		rtd_task->task_period = 20; //measured in ms

		rtd_task->task_args = NULL; //TODO: Add actual settings dipshit

		rtd_task->task_flags = 0x0000;


		return UV_OK;
}

void rtdTask(void* args){
	uv_task_info* params = (uv_task_info*)args;

	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)){
		vTaskDelay(10);
	}

	for(;;){

		if(params->cmd_data == UV_KILL_CMD){ // to perform task control (suspend/kill)

					killSelf(params);

				}else if(params->cmd_data == UV_SUSPEND_CMD){
					suspendSelf(params); // if _UV_SUSPEND_CMD received pause the task
				}

		vTaskDelay(20);
		//Check brake pressure above threshold

		//Are they pushing the start button?

		float brake_percent = calculateBrakePercentage(adc1_BPS1);

		if((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))&&(brake_percent > 10.0)){
			vTaskDelay(10);



			if(vehicle_state != UV_DRIVING){
				changeVehicleState(UV_DRIVING);
			}
			vTaskDelay(100);

		}

		//if both, change vehicle state to driving

		//changeVehicleState(UV_DRIVING);



	}
}
