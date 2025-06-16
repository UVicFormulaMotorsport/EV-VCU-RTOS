/*
 * ready_to_drive.c
 *
 *  Created on: Apr 28, 2025
 *      Author: karni
 */

#include "uvfr_utils.h"

void rtdTask(void* args);

uv_status initRTDtask(void* args){
	uv_task_info* rtd_task = uvCreateTask();

		if(rtd_task == NULL){
						//Oh dear lawd
			return UV_ERROR;
		}


					//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
		rtd_task->task_name = "rtd";

		rtd_task->task_function = rtdTask;
		rtd_task->task_priority = 2; //Slightly more important than the children tasks

		rtd_task->stack_size = 64;

		rtd_task->active_states = UV_READY;
		rtd_task->suspension_states = 0x00;
		rtd_task->deletion_states = PROGRAMMING | UV_DRIVING | UV_ERROR_STATE;

		rtd_task->task_period = 20; //measured in ms

		rtd_task->task_args = NULL; //TODO: Add actual settings dipshit


		return UV_OK;
}

void rtdTask(void* args){
	uv_task_info* params = (uv_task_info*)args;

	for(;;){

		if(params->cmd_data == UV_KILL_CMD){ // to perform task control (suspend/kill)

					killSelf(params);

				}else if(params->cmd_data == UV_SUSPEND_CMD){
					suspendSelf(params); // if _UV_SUSPEND_CMD received pause the task
				}

		vTaskDelay(20);
		//Check brake pressure above threshold

		//Are they pushing the start button?

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
			vTaskDelay(500);




			changeVehicleState(UV_DRIVING);

		}

		//if both, change vehicle state to driving

		//changeVehicleState(UV_DRIVING);



	}
}
