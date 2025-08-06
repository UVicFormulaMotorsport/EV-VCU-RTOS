// This where the code to handle switching channels with the PDU will go


#include "pdu.h"
#include "uvfr_utils.h"
#include "can.h"
#include "main.h"
#include "constants.h"

uv19_pdu_settings default_pdu_settings = {
	.PDU_rx_addr = 0x310,
	.PDU_tx_addr = 0x311,
	.sdc_channel = 0x0C
};



void initPDU(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;
	uv_init_task_response response = {UV_OK,PDU,0,NULL};
	vTaskDelay(102); //Pretend to be doing something for now

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
				//OOPS
		uvPanic("Failed to enqueue PDU OK Response",0);
	}


	vTaskSuspend(params->meta_task_handle);
}
