/*
 * uvfr_external_devices.c
 *
 *  Created on: Dec 6, 2025
 *      Author: byo10
 */

#include "uvfr_utils.h"

static xdev_info xdev_registry[FINAL_XDEV];

typedef enum uv_status_t uv_status;

/** Linked list of CAN messages
 *
 */
typedef struct xdev_poll_msg{
	struct xdev_poll_msg* next;
	uv_CAN_msg pmsg;
}xdev_poll_msg;

/** @brief Initializes the XDEV registry to be in a normal state
 *
 */
__attribute__((constructor)) void __cfgDefaultXdevRegistry(){
	for(int i = 0; i< FINAL_XDEV; i++){
		xdev_registry[i].activation_time = 0;
		xdev_registry[i].ecode1 = 0;
		xdev_registry[i].ecode2 = 0;
		xdev_registry[i].flags = 0;
		xdev_registry[i].period = 0;
		xdev_registry[i].peripheral_status = XDEV_NC;
		xdev_registry[i].xdev_mutex = NULL;
		xdev_registry[i].xdev_rx_smphr = NULL;
		xdev_registry[i].xdev_poll_msgs = NULL;

	}
}

/**@brief Function called to wait on an external device.
 *
 * Accepts an ID and a maximum wait time. Returns UV_OK if it returned within the specified time.
 * Returns UV_ERROR if the device times out.
 *
 */
uv_status uvWaitOnExternalDevice(uint8_t device_id, TickType_t time_to_wait){
	if(xSemaphoreTake(xdev_registry[device_id].xdev_rx_smphr,time_to_wait) == pdTRUE){
		return UV_OK;
	}
	return UV_ABORTED;
}

/** @brief Creates the mutex that protects the params of each xDEV
 *
 */
static inline uv_status createXdevMutex(uint8_t xdev){

	xdev_registry[xdev].xdev_mutex = xSemaphoreCreateMutex();
	if (xdev_registry[xdev].xdev_mutex == NULL){
		return UV_ERROR;
	}
	return UV_OK;
}

static inline uv_status createXdevSemphr(uint8_t xdev){
	xdev_registry[xdev].xdev_rx_smphr = xSemaphoreCreateBinary();
	if (xdev_registry[xdev].xdev_rx_smphr == NULL){
		return UV_ERROR;
	}

	return UV_OK;
}

/** @brief Registers an external device with XDevMon, and allows the thing to actually work as expected
 *
 */
uv_status uvRegisterExternalDevice(uint8_t device_id, TickType_t period, uint16_t xd_flags, char* name){
	if(xdev_registry[device_id].xdev_mutex == NULL){
		if(createXdevMutex(device_id) != UV_OK){
			return UV_ERROR;
		}
	}

	if(xdev_registry[device_id].xdev_rx_smphr == NULL){
		if(createXdevSemphr(device_id) != UV_OK){
			return UV_ERROR;
		}
	}

	//xdev_registry[device_id].

	//Take Mutex

	//set attributes
	memcpy(xdev_registry[device_id].name,name,8); //At most 7 chars of name

	xdev_registry[device_id].period = period;
	xdev_registry[device_id].flags = xd_flags;
	xdev_registry[device_id].activation_time = 0xFFFFFFFF;//Idunno what to set this as

	//Release Mutex

	return UV_OK;
}


/** @brief Internal function to poll an external device.
 *
 */
static uv_status pollXdev(uint8_t xdev){
	if((xdev_registry[xdev].flags & XDEV_POLLING_REQUIRED)==0){
		return UV_OK;
	}

	uint8_t is_unchill = 0;

	xdev_poll_msg* tmp = (xdev_poll_msg*) xdev_registry[xdev].xdev_poll_msgs;
	if(tmp == NULL){
		return UV_ERROR;
	}

	while(tmp != NULL){
		uvSendCanMSG(&(tmp->pmsg));
		if(uvWaitOnExternalDevice(xdev,2)!=UV_OK){
			//ERROR!!!
			is_unchill = 1;
		}
		tmp = tmp->next;
	}

	if(is_unchill){
		return UV_ERROR;
	}

	return UV_OK;
}



//POLL ALL EXTERNAL DEVICES - prevent timeout bullshit on startup - prevents erroneous faulting
static uv_status pollAllXdevs(){
	uv_status retval = UV_OK;
	for(int i = 0 ;i<FINAL_XDEV;i++){
		uv_status retval2 = pollXdev(i);
		if(retval2 != UV_OK){
			retval = retval2;
		}

	}
	return retval;
}

/** @brief Task responsible for monitoring external devices for timeouts, and ensuring that we can poll all the random ass devices n shi
 *
 */
void xDevMon(void* args){
	//Initialization stuff
	uv_task_info* params = (uv_task_info*)args;

	//Initial poll of external devices

	char estring[16];

	if(pollAllXdevs() != UV_OK){

	}

	vTaskDelay(10);

	int k = 0;
	TickType_t tick_period = pdMS_TO_TICKS(params->task_period); //Convert ms of period to the RTOS ticks
	TickType_t last_time = 0;
	for(;;){
		if(params->cmd_data == UV_KILL_CMD){

			//TASK DESTRUCTOR: CLEAN UP ANY RESOURCES USED BY THE TASK HERE

			killSelf(params);
		}else if(params->cmd_data == UV_SUSPEND_CMD){

			//TASK SUSPENSION DESTRUCTOR: RELEASE THINGS LIKE MUTICES OR SEMAPHORES, BUT NO NEED TO DEALLOCATE ANY MEMORY

			suspendSelf(params);
		}

		last_time = xTaskGetTickCount();

		//Check timeouts

		for(int i = 0 ;i<FINAL_XDEV;i++){
			if((xdev_registry[i].flags & (XDEV_DEVICE_EXPECTED | XDEV_CHECK_TIMEOUT_BIT))!=(XDEV_DEVICE_EXPECTED | XDEV_CHECK_TIMEOUT_BIT)){
				continue; //Not checking timeout on these badboys
			}

			TickType_t t_since = xdev_registry[i].last_heard_from;
			TickType_t tolerance = (xdev_registry[i].period * 6)/5; //20% margin allowed

			if(t_since > tolerance){
				//timeout
			}

		}

		//Handle whatever errors just so happen to arise

		for(int i = 0; i<FINAL_XDEV;i++){

		}

		//Poll tasks
		for(int i = 0; i<FINAL_XDEV;i++){
			uint32_t per = (xdev_registry[i].period)/10;
			if(k%per == 0){
				if(pollXdev(i)==UV_ERROR){
					//HANDLE ERROR HERE
				}
			}

		}

		k = (k+1)%100;
		//Delay
		if(params->cmd_data == UV_KILL_CMD || params->cmd_data == UV_SUSPEND_CMD){
			continue; // The idea here is to skip the delay
		}
		uvTaskDelayUntil(params, last_time, tick_period); //The delay that keeps t
	}
}

/** @brief Function to setup all external devices
 *
 *
 *
 */
uv_status uvSetupXdevs(){





	uv_task_info* xdt = uvCreateTask();

	if(xdt == NULL){
				//Oh dear lawd
		return UV_ERROR;
	}


			//DO NOT TOUCH ANY OF THE FIELDS WE HAVENT ALREADY MENTIONED HERE. FOR THE LOVE OF GOD.
	xdt->task_name = "xDevMon";
	xdt->task_function = xDevMon;
	xdt->task_priority = osPriorityNormal;


	xdt->stack_size = 256;

	xdt->active_states = UV_READY | UV_DRIVING | UV_ERROR_STATE | PROGRAMMING | UV_LAUNCH_CONTROL ;
	xdt->suspension_states = 0x00;
	xdt->deletion_states = 0x00;

	xdt->task_period = 10;

	xdt->task_args = NULL; //TODO: Add actual settings dipshit

	return UV_OK;
}

/** @brief Function called from the can message handler for an external device
 *
 * This function records the timestamp to allow for timeout computation, and gives the semaphore,
 * so that tasks waiting on this specific external device are able to resume execution normally
 */
void externalDeviceRxHandler(uint8_t device_id){
	xSemaphoreTake(xdev_registry[device_id].xdev_mutex, 0);
	xdev_registry[device_id].last_heard_from = xTaskGetTickCount();
	if(xSemaphoreGive(xdev_registry[device_id].xdev_mutex) != pdTRUE){
		//UNABLE TO RELEASE MUTEX

		//uvPanic()??
	}

	if(xSemaphoreGive(xdev_registry[device_id].xdev_rx_smphr) != pdTRUE){
			//UNABLE TO GIVE SEMAPHORE
	}
}



void externalDeviceStatUpdate(uint8_t device_id){

}

xdev_status getXdevStatus(uint8_t device_id){
	return xdev_registry[device_id].peripheral_status;
}

void getAllXdevStatus(xdev_status* arr){
	for(int i = 0;i<FINAL_XDEV;i++){
		arr[i] = getXdevStatus(i);
	}

}

/** @brief adds a CAN message to the polling of the task
 *
 */
uv_status uvAddPollMsgToXdev(uint8_t xdev_id, uv_CAN_msg* msg){
	if(xdev_id >= FINAL_XDEV){
		return UV_ERROR;
	}
	if(msg == NULL){
		return UV_ERROR;
	}

	xdev_poll_msg* pmsg = (xdev_poll_msg*) uvMalloc(sizeof(xdev_poll_msg));
	pmsg->next = NULL;
	pmsg->pmsg.dlc = msg->dlc;
	pmsg->pmsg.flags = msg->flags;
	pmsg->pmsg.msg_id = msg->msg_id;
	for(int i = 0;i<8;i++){
		pmsg->pmsg.data[i] = msg->data[i];
	}


	if(pmsg == NULL){
		return UV_ERROR;
	}

	if(xdev_registry[xdev_id].xdev_poll_msgs == NULL){
		xdev_registry[xdev_id].xdev_poll_msgs = (xdev_poll_msg*)pmsg;
		return UV_OK;
	}

	xdev_poll_msg* tmp = (xdev_poll_msg*)xdev_registry[xdev_id].xdev_poll_msgs;

	while(tmp->next!=NULL){
		tmp = tmp->next;
	}

	tmp->next = pmsg;
	return UV_OK;

}

//void
