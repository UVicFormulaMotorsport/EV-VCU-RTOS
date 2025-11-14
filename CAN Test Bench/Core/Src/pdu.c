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

TickType_t pdu_last_Rx_time;
uv_CAN_msg last_received_from_PDU;

SemaphoreHandle_t pdu_tx_mutex = NULL;

uv_CAN_msg msg_to_PDU = {
	.flags = 0x00,
	.dlc = 1,
	.msg_id = 0x710,
	.data = {0,0,0,0,0,0,0,0}
};


#define U19_PDU_20A_BIT 0b00100000
#define U19_PDU_EN_BIT  0b00010000
#define HIGHEST_CH 0x28
#define FIRST_20A_CH 0x21

// PDU commands
uv_status u19updatePduChannel(struct abstract_conifer_channel* ch_ptr, uint32_t* ecode){
	//assume that the channel is in fact in use, no need.

	//This PDU is simply a dumb enable/disable signal, nothing fancy going on here.
	//First part is we need to get the channel.
	uint8_t ch = (ch_ptr->hardware_mapping)&0xFF;
	uint8_t msg = ch;
	uint8_t en = 0;

	//Bounds check on ch:
	if((ch > HIGHEST_CH)||(ch > 0x0F && ch < FIRST_20A_CH)){
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

		en = 1; //Set the bit on the msg
	}else{
		//Otherwise disable
		if(ch & U19_PDU_20A_BIT){
			set_20A_vals &= ~(0x01U<<(ch&0x0F)); //Clear the bit because we really dont want them to be like this
		}else{
			set_5A_vals &= ~(0x01U<<ch);
		}
	}

	//Prevent the channel from turning on if load shedding is currently active
	if(ch_ptr->status_control_reg&(CONIFER_CH_LS_ACTIVE|CONIFER_CH_FLT_BIT)){
		en = 0;
	}

	if(en){
		msg |= U19_PDU_EN_BIT;
	}

	uv_status retval;

	msg_to_PDU.data[0] = msg;
	if(xSemaphoreTake(pdu_tx_mutex,2) == pdTRUE){
		//handle error here please :)
		retval =  uvSendCanMSG(&msg_to_PDU); //Send the message to the PDU
		if(xSemaphoreGive(pdu_tx_mutex)!=pdTRUE){
				//Unable to release mutex, next PDU message will not correctly send. Ohh god.
				//Probably one of the worst situations one could possibly end up in from a safety standpoint ngl.
		}
	}


	if(retval != UV_OK){
		//UHH OHH
		return retval;
	}

	return UV_OK;
}

uv_status initPDU(uint32_t* ecode){
	pdu_tx_mutex = xSemaphoreCreateMutex();

	if(pdu_tx_mutex == NULL){
		//UHH OHH
		if(ecode!= NULL){
			*ecode = CONIFER_DRV_INIT_ERR;
		}
		return UV_ERROR;
	}


	msg_to_PDU.flags = conifer_params->pdu_bus;

	vTaskDelay(10); //Pretend to be doing something for now




	return UV_OK;
}
