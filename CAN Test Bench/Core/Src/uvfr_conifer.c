/*
 * uvfr_conifer.c
 *
 *  Created on: Aug 14, 2025
 *      Author: byo10
 */

#include"uvfr_conifer.h"
#include"uvfr_utils.h"


//typedef enum CONIFER_OUTPUT conifer_output_channel;
struct conifer_settings;
//This is the default configuration for conifer
struct conifer_settings default_conifer_config = {

};
static abstract_conifer_channel ch_table[FINAL_CONIFER_OUTPUT];

//Driver function lookup tables
uv_status (*ch_updaters[8])(abstract_conifer_channel*, uint32_t*) = {0,0,0,0,0,0,0,0};
uv_status (*current_getters[8]) (abstract_conifer_channel*, uint32_t*) = {0,0,0,0,0,0,0,0};

//typedef enum uv_status_t uv_status;

uv_status coniferInvalidChLocCallback(abstract_conifer_channel*, uint8_t){
	return UV_ERROR;
}

/** @brief This is meant to update the hardware to match the hardware abstraction.
 *
 * This extracts the hardware mapping of the abstract channel, and then invokes the drivers for the relevant hardware depending
 * on what is mapped in the driver function lookup tables.
 *
 */
static inline uv_status coniferUpdateChannel(abstract_conifer_channel* ch_ptr){
	uint8_t src = (ch_ptr->hardware_mapping&CONIFER_CH_LOC_MASK)>>8;
	uint8_t ch_id = ch_ptr->hardware_mapping & 0x00FF;
	uv_status (*update_func)(abstract_conifer_channel*, uint8_t) = ch_updaters[src];

	if(update_func == NULL){
		return UV_ABORTED;
	}

	uv_status retval = update_func(ch_ptr,0);//Me executing the random ass callback func
	return retval;

}

uv_status coniferInit(){
	//STEP 1: Init the ch_table to something normal. Set all outputs as "not in use"
	for(int i = 0; i < FINAL_CONIFER_OUTPUT;i++){
		abstract_conifer_channel* ch = ch_table + i*sizeof(abstract_conifer_channel);
		ch->hardware_mapping = 0x0700; //Makes channel as not in use and sets location as invalid
		ch->status_control_reg = 0;
	}


	//STEP 2: Pull up the relevant settings into the table


	//STEP 3: Configure Driver LUTs

	ch_updaters[UV19_PDU_CH] = u19updatePduChannel;

	//STEP 4: Set system initial conditions to turn on/off the relevant channels
	return UV_OK;
}

uv_status coniferDeInit(){
	return UV_OK;
}

/** @brief Enables a conifer output channel
 *
 */
uv_status coniferEnChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	//uint8_t src = 7;
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	coniferUpdateChannel(ch_ptr);


	return UV_ERROR;
}

/** @brief Disables a conifer output channel
 *
 */
uv_status coniferDisChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	coniferUpdateChannel(ch_ptr);
	return UV_ERROR;
}

/** @brief Toggles a conifer output channel
 *
 */
uv_status coniferToggleChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	coniferUpdateChannel(ch_ptr);
}

/** @brief Sets Duty Cycle of a conifer managed PWM output
 *
 */
uv_status coniferSetDutyCycle(conifer_output_channel ch, uint8_t duty){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	coniferUpdateChannel(ch_ptr);
	return UV_ERROR;
}

/** @brief Sets direction of a conifer managed H-bridge output.
 *
 */
uv_status coniferSetDirection(conifer_output_channel ch, uint8_t dir){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	coniferUpdateChannel(ch_ptr);
	return UV_ERROR;
}

/** @brief Set duty cycle and direction of a conifer managed
 *
 */
uv_status coniferSetDutyCycleAndDirection(conifer_output_channel ch, uint8_t duty, uint8_t dir){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	coniferUpdateChannel(ch_ptr);
	return UV_ERROR;
}


uint16_t coniferGetChannelCurrent(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}
	uint8_t src = (ch_ptr->hardware_mapping&CONIFER_CH_LOC_MASK)>>8;
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
	abstract_conifer_channel* ch_ptr = ch_table + ch*sizeof(abstract_conifer_channel);
	uint8_t src = (ch_ptr->hardware_mapping&CONIFER_CH_LOC_MASK)>>8;
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

