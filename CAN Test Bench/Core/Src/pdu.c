// This where the code to handle switching channels with the PDU will go
//typedef struct abstract_conifer_channel abstract_conifer_channel;

#include "pdu.h"
#include "uvfr_utils.h"
#include "can.h"
#include "main.h"
#include "constants.h"



uint16_t read_5A_vals; //What the PDU claims its 5A channels are up to
uint16_t set_5A_vals; //What the intended 5A channel outputs are

uint16_t read_20A_vals; //What the PDU claims its 20A channels are up to
uint16_t set_20A_vals; //What the intended 20A channel outputs are

uv_CAN_msg last_received_from_PDU;


uv_CAN_msg msg_to_PDU = {
	.flags = 0x00,
	.dlc = 1,
	.msg_id = 0x710,
	.data = {0,0,0,0,0,0,0,0}
};


#define U19_PDU_20A_BIT 0b00100000
#define U19_PDU_EN_BIT  0b00010000

// PDU commands for 5A Circuit
uv_status u19updatePduChannel(struct abstract_conifer_channel* ch_ptr, uint32_t* ecode){
	//assume that the channel is in fact in use, no need.

	//This PDU is simply a dumb enable/disable signal, nothing fancy going on here.
	//First part is we need to get the channel.
	uint8_t ch = (ch_ptr->hardware_mapping)&0xFF;
	uint8_t msg = ch;

	//Bounds check on ch:
	if((ch > U19_PDU_20A_8)||(ch > U19_PDU_5A_16 && ch < U19_PDU_20A_1)){
		//This is a PDU channel that dont exist
		*ecode = CONIFER_DRV_INVALID_HW_CH_ID;
	}

	if((ch_ptr->status_control_reg & CONIFER_CH_EN_BIT) == CONIFER_CH_EN_BIT){
		//Enable
		if(ch & U19_PDU_20A_BIT){
			set_20A_vals |= 0x01U<<(ch&0x0F); //Indicates what the set 20A channels are
		}else{
			set_5A_vals |= 0x01U<<ch;
		}

		msg |= U19_PDU_EN_BIT; //Set the bit on the msg
	}else{
		//Otherwise disable
		if(ch & U19_PDU_20A_BIT){
			set_20A_vals &= ~(0x01U<<(ch&0x0F)); //Clear the bit because we really dont want them to be like this
		}else{
			set_5A_vals &= ~(0x01U<<ch);
		}
	}

	msg_to_PDU.data[0] = msg;
	return uvSendCanMSG(&msg_to_PDU); //Send the message to the PDU
}

void initPDU(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;
	uv_init_task_response response = {UV_OK,PDU,0,NULL};


	msg_to_PDU.flags = conifer_params->pdu_bus;

	vTaskDelay(10); //Pretend to be doing something for now

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
				//OOPS
		uvPanic("Failed to enqueue PDU OK Response",0);
	}


	vTaskSuspend(params->meta_task_handle);
}
