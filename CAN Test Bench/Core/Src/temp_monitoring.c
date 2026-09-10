/*
 * temp_monitoring.c
 *
 *  Created on: Oct 31, 2024
 *      Author: byo10
 */

#define __UV_FILENAME__ "temp_monitoring.c"

#include "can.h"
#include "uvfr_utils.h"
#include "gpio.h"

#define USE_DEVICE_TEMP_FOR_FANS

//Currently every 60s (ish)
#define ROTATION_PERIOD 60000

uint16_t adc_coolant_temp1 = 0;
uint16_t adc_coolant_temp2 = 0;
extern int16_t mc_motor_temp;
extern int16_t mc_igbt_temp;



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


	tm_task->stack_size = _UV_DEFAULT_TASK_STACK_SIZE;

	tm_task->active_states = UV_READY | UV_DRIVING | UV_ERROR_STATE;
	tm_task->suspension_states = 0x00;
	tm_task->deletion_states = PROGRAMMING | UV_LAUNCH_CONTROL ;

	tm_task->task_period = 200;

	tm_task->task_args = NULL; //TODO: Add actual settings dipshit

	return UV_OK;
}

void testfunc(uv_CAN_msg* msg){
	if(vehicle_state == UV_READY){
		changeVehicleState(UV_DRIVING);
	}else if(vehicle_state == UV_DRIVING){
		changeVehicleState(UV_ERROR_STATE);
	}else if(vehicle_state == UV_ERROR_STATE){
		changeVehicleState(UV_READY);
	}
}

void testfunc2(uv_CAN_msg* msg){
	static uv_CAN_msg response = {
		.msg_id = 0x65,
		.dlc = 1


	};

	response.data[0] = msg->data[0] + msg->data[1];
	response.flags = 0;
	uvSendCanMSG(&response);
}

void testfunc3(uv_CAN_msg* msg){
	static uv_CAN_msg response = {
		.msg_id = 0x65,
		.dlc = 1



	};

	response.data[0] = msg->data[0] + msg->data[1];
	response.flags = CAN_BUS_1;
	uvSendCanMSG(&response);
}

/** @brief Monitors the temperatures of various points in the tractive system, and activates various cooling systems and such accordingly
 *
 * Atm, this is mostly serving as an example of a task
 *
 */

void CAN_1_TEST_msg(uv_CAN_msg* msg){


	//


}


void CAN_2_TEST_msg2(uv_CAN_msg* msg){


//


}






void tempMonitorTask(void* args){
	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast

	motor_controller_settings* t_settings = current_vehicle_settings->mc_settings;

	insertCANMessageHandler(0x86, testfunc, CAN_BUS_1);
	insertCANMessageHandler(0x87, testfunc, CAN_BUS_1);
	insertCANMessageHandler(0x64, testfunc3, CAN_BUS_1);

	/**These here lines set the delay. This task executes exactly at the period specified, regardless of how long the task
	 * execution actually takes
	 *
	 @code*/



	insertCANMessageHandler(0x64, testfunc2, CAN_BUS_2);
	//insertCANMessageHandler(0x6B1, BMS_msg2, CAN_BUS_2);


//	uv_CAN_msg temp1;
//	uv_CAN_msg temp2;

//	temp1.data[0] = 1;
//	temp1.data[1] = 2;
//	temp1.data[2] = 3;
//	temp1.dlc = 3;
//	temp1.msg_id = 0x85;
//	temp1.flags = CAN_BUS_1;
//
//	temp2.data[0] = 9;
//	temp2.data[1] = 10;
//	temp2.data[2] = 11;
//	temp2.dlc = 3;
//	temp2.msg_id = 0xF5;
//	temp2.flags = 0x00;


//	uvSendCanMSG(&temp1);
//	uvSendCanMSG(&temp2);




	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = 0;

	uint8_t pump_on = 0;
	uint8_t fana_on = 0;
	uint8_t fanb_on = 0;

	uint8_t fans_rotated = 0;

	uint16_t k;

	int16_t pump_src_temp = 0;
	int16_t fan_src_temp = 0;

	conifer_output_channel afans = RAD_FANS1;
	conifer_output_channel bfans = RAD_FANS2;


	/**@endcode */
	for(;;){
		/** This is an example of a task control point, which is the spot in the task where the task
		 * decides what needs to be done, based on the commands it has received from the task manager and the SCD
		 *
		 */

		//vTaskDelayUntil( &last_time, tick_period);
		if(params->cmd_data == UV_KILL_CMD){

			//TASK DESTRUCTOR: CLEAN UP ANY RESOURCES USED BY THE TASK HERE

			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){

			//TASK SUSPENSION DESTRUCTOR: RELEASE THINGS LIKE MUTICES OR SEMAPHORES, BUT NO NEED TO DEALLOCATE ANY MEMORY

			suspendSelf(params);
		}

		last_time = xTaskGetTickCount();



		if(params->cmd_data == UV_KILL_CMD || params->cmd_data == UV_SUSPEND_CMD){
			continue; // The idea here is to skip the delay
		}


		//Periodically rotate fans to prevent wear on just one side. ROTATION_PERIOD is given in ms
		if(k*params->task_period >= ROTATION_PERIOD){
			fans_rotated = !fans_rotated;
			k = 0;
		}

		//Actually do the rotation
		if(fans_rotated){
			afans = RAD_FANS2;
			bfans = RAD_FANS1;
		}else{
			afans = RAD_FANS1;
			bfans = RAD_FANS2;
		}

		pump_src_temp = (mc_igbt_temp > mc_motor_temp)? mc_igbt_temp : mc_motor_temp;

#ifdef USE_DEVICE_TEMP_FOR_FANS
		fan_src_temp = (mc_igbt_temp > mc_motor_temp)? mc_igbt_temp : mc_motor_temp;
#else
		//This is where we would use the coolant temp, however we dont actually have that setup for conversion atm, so will need to ponder
#error "NEED TO POPULATE THIS CODE"
#endif

		//hysterisis for pump fans
		if((pump_src_temp > t_settings->pump_on_threshold_c)&&(pump_on == 0)){
			//activate pumps
			pump_on = 1;
			coniferEnChannel(COOLANT_PUMP1);
		}else if((pump_src_temp < t_settings->pump_off_threshold_c )&&(pump_on == 1)){
			//deactivate pumps
			pump_on = 0;
			coniferDisChannel(COOLANT_PUMP1);
		}


		//hysterisis for group a
		if((fan_src_temp > t_settings->fana_on_threshold_c)&&(fana_on == 0)){
			//activate pumps
			fana_on = 1;
			//confiferEnChannel(COOLANT_PUMP1);
		}else if((fan_src_temp < t_settings->fana_off_threshold_c )&&(fana_on == 1)){
					//deactivate pumps
			fana_on = 0;

		}

		//hysterisis for group b fans
		if((fan_src_temp > t_settings->fanb_on_threshold_c)&&(fanb_on == 0)){
					//activate pumps
			fanb_on = 1;
				//confiferEnChannel(COOLANT_PUMP1);
		}else if((fan_src_temp < t_settings->fanb_off_threshold_c )&&(fanb_on == 1)){
							//deactivate pumps
			fanb_on = 0;
			//confiferDisChannel(COOLANT_PUMP1);
		}

		//Physically update fan power irl
		if(fana_on == 1){
			coniferEnChannel(afans);
		}else{
			coniferDisChannel(afans);
		}

		if(fanb_on == 1){
			coniferEnChannel(bfans);
		}else{
			coniferDisChannel(bfans);
		}



		k++;

		uvTaskDelayUntil(params, last_time, tick_period); //The delay that keeps task on period theoretically
	}
}
