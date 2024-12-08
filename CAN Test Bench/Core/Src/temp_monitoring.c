/*
 * temp_monitoring.c
 *
 *  Created on: Oct 31, 2024
 *      Author: byo10
 */


#include "uvfr_utils.h"
#include "gpio.h"

uv_status initTempMonitor(void * arguments){
	uv_task_info* tm_task = uvCreateTask();

	if(tm_task == NULL){
			//Oh dear lawd
		return UV_ERROR;
	}


		//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
	tm_task->task_name = "Temp_monitor";


	tm_task->task_function = tempMonitorTask;
	tm_task->task_priority = osPriorityNormal;

	tm_task->max_instances = 1;
	tm_task->stack_size = _UV_DEFAULT_TASK_STACK_SIZE;

	tm_task->active_states = UV_READY | UV_DRIVING | UV_ERROR_STATE;
	tm_task->suspension_states = 0x00;
	tm_task->deletion_states = PROGRAMMING | UV_LAUNCH_CONTROL ;

	tm_task->task_period = 200;

	tm_task->task_args = NULL; //TODO: Add actual settings dipshit

	return UV_OK;
}


/** @brief Monitors the temperatures of various points in the tractive system, and activates various cooling systems and such accordingly
 *
 * Atm, this is mostly serving as an example of a task
 *
 */
void tempMonitorTask(void* args){
	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast

	/**These here lines set the delay. This task executes exactly at the period specified, regardless of how long the task
	 * execution actually takes
	 *
	 @code*/
	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = xTaskGetTickCount();
	/**@endcode */
	for(;;){
		/** This is an example of a task control point, which is the spot in the task where the task
		 * decides what needs to be done, based on the commands it has received from the task manager and the SCD
		 *
		 */
		if(params->cmd_data == UV_KILL_CMD){
			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params);
		}
		vTaskDelayUntil( &last_time, tick_period);

		//Mohak code here
		TxData[0] = 0b10101010;
		TxData[1] = 0b10101010;
		TxData[2] = 0b10101010;
		TxData[3] = 1;
		TxData[4] = 2;
		TxData[5] = 3;
		TxData[6] = 0b10101010;
		TxData[7] = 0b10101010;


		HAL_StatusTypeDef can_send_status;

					//vTaskDelay(400);



		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = 0x1234;


		TxHeader.DLC = 8;


		//taskENTER_CRITICAL();
		can_send_status = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
		//taskEXIT_CRITICAL();

		if (can_send_status != HAL_OK){
			/* Transmission request Error */
			//uvPanic("Unable to Transmit CAN msg",can_send_status);
			handleCANbusError(&hcan2, 0);
		}


		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); //BLUE

	}
}
