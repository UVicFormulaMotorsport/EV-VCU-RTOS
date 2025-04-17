#define _SRC_UVFR_DAQ

#include "uvfr_utils.h"
#include "daq.h"





typedef struct daq_param_list_node{
	struct daq_param_list_node* next;

	uint32_t can_id;
	uint16_t param;

	uint8_t* size;
}daq_param_list_node;

typedef struct daq_child_task{
	struct daq_child_task* next_task;
	TaskHandle_t meta_task_handle;
	uint32_t period;
	daq_param_list_node* param_list;


}daq_child_task;

daq_loop_args* curr_daq_settings = NULL;

daq_loop_args default_daq_settings = {
	.total_params_logged = 4,
	.throttle_daq_to_preserve_performance = 1,
	.minimum_daq_period = 10,
	.can_channel = 1,
	.daq_child_priority = 1
};

daq_datapoint default_datapoints[] ={
	{.can_id = 0x530,
	.param = APPS1_ADC_VAL,
	.period = 100,
	.type = UV_UINT16},

	{.can_id = 0x531,
	.param = APPS2_ADC_VAL,
	.period = 100,
	.type = UV_UINT16},

	{.can_id = 0x532,
	.param = BPS1_ADC_VAL,
	.period = 100,
	.type = UV_UINT16},

	{.can_id = 0x533,
	.param = BPS2_ADC_VAL,
	.period = 100,
	.type = UV_UINT16}

};

static void* param_ptrs[MAX_LOGGABLE_PARAMS];

static daq_child_task* daq_tlist = NULL;

static daq_datapoint* datapoints = NULL;

static daq_param_list_node* param_bank = NULL;

void daqSubTask(void* args);

/**
 *
 */
uv_status associateDaqParamWithVar(uint16_t paramID, void* var){
	if(uvIsPTRValid(var)!=UV_OK){
		return UV_ERROR;
	}

	if(paramID >= MAX_LOGGABLE_PARAMS){
		return UV_ERROR;
	}

	param_ptrs[paramID] = var;

	return UV_OK;
}

/** @brief
 *
 */
static inline void insertParamToParamList(daq_param_list_node* node, daq_param_list_node** list){
	if(*list == NULL){
		*list = node;
		return;
	}

	daq_param_list_node* tmp = *list; //Sets a temporary var as the first list entry

	while(tmp->next != NULL){
		tmp = tmp->next;
	}

	//Now this is a situation where tmp->next MUST be NULL
	tmp->next = node; // Set it's next as a node

}

/** @brief
 *
 */
uv_status insertParamToRegister(daq_param_list_node* node, daq_datapoint* datapoint){
	//Step 1: find which task this will be assigned to. If no tasks have been created yet, then create them lol
	if(daq_tlist == NULL){
		daq_tlist = uvMalloc(sizeof(daq_child_task));

		if(daq_tlist == NULL){
			//Oopsies
		}

		daq_tlist->period = datapoint->period;
	}

	daq_child_task* list_tmp = daq_tlist;
	node->can_id = datapoint->can_id;

	while(1){
		//Keep going until we find one with a period that matches
		if(list_tmp->period == datapoint->period){
			//Insert to list
			insertParamToParamList(node, &(list_tmp->param_list));

			break;
		}

		if(list_tmp->next_task == NULL){
			list_tmp->next_task = uvMalloc(sizeof(daq_child_task));

			if(list_tmp->next_task == NULL){
				//Another big oopsie
			}

			list_tmp->next_task->period = datapoint->period;

		}

		list_tmp = list_tmp->next_task;
	}

	return UV_OK;
}

/** @brief This pre-allocates parameters to one of the daq subtasks
 *
 */
uv_status configureDaqSubTasks(){
	uint16_t n_logged_params = curr_daq_settings->total_params_logged;
	//uint16_t n; // Will use this to track number of params used as we iterate

	daq_datapoint* master_param_list = current_vehicle_settings->daq_param_list;

	daq_param_list_node* param_bank = uvMalloc(n_logged_params*sizeof(daq_param_list_node));

	if(param_bank == NULL){
		uvPanic("outofmem",0);
	}

	for(int i = 0; i<n_logged_params ; i++){
		//We now go through all of the parameters.
		if(insertParamToRegister(&(param_bank[i]),&(master_param_list[i])) != UV_OK){
			//commit a warcrime
		}
	}



}

/** @brief Function that starts up all the subtasks
 *
 */
uv_status startDaqSubTasks(){
	daq_child_task* tmp = daq_tlist;
	BaseType_t retval = 0;
	while(tmp != NULL){
		retval = xTaskCreate(daqSubTask,
				"DaqSub",64,tmp,
				curr_daq_settings->daq_child_priority,
				&(tmp->meta_task_handle));


	}
	return UV_OK;
}

/** @brief Function that shuts down the subtasks
 *
 */
uv_status stopDaqSubTasks(){
	daq_child_task* tmp = daq_tlist;
	BaseType_t retval = 0;
	while(tmp!=NULL){
		vTaskDelete(tmp->meta_task_handle);
	}
	return UV_ERROR;
}

/** @brief initializes the master DAQ task, all that fun stuff. This task probably manages a while plethora of smaller tasks
 *
 * This is a fairly standard function. Here are the things that it does in order:
 *
 * Step 1: Get Daq settings.
 *
 * Step 2: Create and configure DAQ task.
 *
 * Step 3: Read which parameters we want to read.
 *
 * Step 4: Generate Subtask Metadata
 *
 * Step 5: Assign params to subtasks
 *
 */
uv_status initDaqTask(void * args){
	curr_daq_settings = current_vehicle_settings->daq_settings;
	//datapoints = current_vehicle_settings->daq_param_list;



	uv_task_info* daq_task = uvCreateTask();

	if(daq_task == NULL){
					//Oh dear lawd
		return UV_ERROR;
	}


				//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
	daq_task->task_name = "daq";

	daq_task->task_function = daqMasterTask;
	daq_task->task_priority = curr_daq_settings->daq_child_priority + 1; //Slightly more important than the children tasks

	daq_task->stack_size = _UV_DEFAULT_TASK_STACK_SIZE;

	daq_task->active_states = UV_READY | UV_DRIVING | UV_ERROR_STATE;
	daq_task->suspension_states = 0x00;
	daq_task->deletion_states = PROGRAMMING | UV_LAUNCH_CONTROL ;

	daq_task->task_period = 20; //measured in ms

	daq_task->task_args = NULL; //TODO: Add actual settings dipshit


	return UV_OK;
}

/** @brief Controls the Daq
 *
 */
void daqMasterTask(void* args){
	uv_task_info* params = (uv_task_info*) args; //Evil pointer typecast



		/**These here lines set the delay. This task executes exactly at the period specified, regardless of how long the task
		 * execution actually takes
		 *
		 @code*/
	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	//TickType_t last_time = xTaskGetTickCount();		/**@endcode */
	for(;;){
		if(params->cmd_data == UV_KILL_CMD){
			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params);
		}
		vTaskDelay(tick_period);

		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
//		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
//			vTaskDelay(25);
//			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
//
//			}
//
//
//			if(vehicle_state == UV_READY){
//				changeVehicleState(UV_DRIVING);
//			}else if(vehicle_state == UV_DRIVING){
//				changeVehicleState(UV_ERROR_STATE);
//			}else if(vehicle_state == UV_ERROR_STATE){
//				changeVehicleState(UV_READY);
//			}
//
//
//		}


	}
}


uv_CAN_msg tmp_daq_msg;

/** @brief
 *
 */
static inline void sendDaqMsg(daq_param_list_node* param){
	if(param_ptrs[param->param] == NULL){ //We simply ignore nulls instead of say, crashing
		return;
	}

	tmp_daq_msg.msg_id = param->can_id;
	tmp_daq_msg.data[0] = *((uint8_t*)(param_ptrs[param->param]));
	tmp_daq_msg.data[1] = *((uint8_t*)(param_ptrs[param->param]+1));
	tmp_daq_msg.data[2] = *((uint8_t*)(param_ptrs[param->param]+2));
	tmp_daq_msg.data[3] = *((uint8_t*)(param_ptrs[param->param]+3));
	tmp_daq_msg.dlc = param->size;

	uvSendCanMSG(&tmp_daq_msg);

}

/** @brief
 *
 */
static inline void sendAllParamsFromList(daq_param_list_node* list){
	if(list == NULL){
		return;
	}

	while(list != NULL){
		sendDaqMsg(list);

		list = list->next;
	}
}

/** @brief
 *
 */
void daqSubTask(void* args){
	daq_child_task* params = (daq_child_task*)args;

	TickType_t curr_time = xTaskGetTickCount();
	for(;;){
		sendAllParamsFromList(params->param_list);
		vTaskDelayUntil(&curr_time,pdMS_TO_TICKS(params->period));
	}
}


