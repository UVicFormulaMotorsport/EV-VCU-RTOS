/*
 * uvfr_conifer.c
 *
 *  Created on: Aug 14, 2025
 *      Author: byo10
 */

#include"uvfr_utils.h"

//This is the default configuration for conifer
struct conifer_settings default_conifer_config;

static abstract_conifer_channel ch_table[FINAL_CONIFER_OUTPUT];

uv_status coniferInit(){
	//STEP 1: Init the ch_table to something normal. Set all outputs as "not in use"
	for(int i = 0; i < FINAL_CONIFER_OUTPUT;i++){
		abstract_conifer_channel* ch = ch_table + i*sizeof(abstract_conifer_channel);
		ch->hardware_mapping = 0x0700; //Makes channel as not in use and sets location as invalid
		ch->status_control_reg = 0;
	}

	//STEP 2: Pull up the relevant settings into the table


	//STEP 3: Set system initial conditions to turn on/off the relevant channels
	return UV_OK;
}

uv_status coniferDeInit(){
	return UV_OK;
}


uv_status coniferEnChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	//uint8_t src = 7;
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
	uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
	switch(src){
	case LOCAL_CH:

		break;
	case UV19_PDU_CH:

		break;
	case ECUMASTER_PMU16_CH:

		break;
	default:

		break;
	}


	return UV_ERROR;
}

uv_status coniferDisChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}
	return UV_ERROR;
}

uv_status coniferToggleChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}
	return UV_ERROR;
}

uv_status coniferSetDutyCycle(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}

	return UV_ERROR;
}

uv_status coniferSetDirection(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}
	return UV_ERROR;
}

uv_status coniferSetDutyCycleAndDirection(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}
	return UV_ERROR;
}


uint16_t coniferGetChannelCurrent(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
		switch(src){
		case LOCAL_CH:

			break;
		case UV19_PDU_CH:

			break;
		case ECUMASTER_PMU16_CH:

			break;
		default:

			break;
		}
	return 0;
}

uint16_t coniferGetChannelFbck(conifer_output_channel ch){
	uint8_t src = (ch_ptr->hardware_mapping&HW_CH_LOC_MASK)>>8;
		uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
	switch(src){
	case LOCAL_CH:

		break;
	case UV19_PDU_CH:

		break;
	case ECUMASTER_PMU16_CH:

		break;
	default:

		break;
	}
	return 0;
}

uint16_t coniferGetChannelFaults(conifer_output_channel ch){
	return 0;
}

