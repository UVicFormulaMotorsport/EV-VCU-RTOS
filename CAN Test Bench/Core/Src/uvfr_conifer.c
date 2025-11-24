/*
 * uvfr_conifer.c
 *
 *  Created on: Aug 14, 2025
 *      Author: byo10
 */
#define __UV_FILENAME__ "uvfr_conifer.c"
#include"uvfr_conifer.h"
#include"uvfr_utils.h"


//typedef enum CONIFER_OUTPUT conifer_output_channel;
struct conifer_settings;
//This is the default configuration for conifer
struct conifer_settings default_conifer_settings = {
	.conif_flags = EXPECT_UV19PDU|EN_DYNAMIC_LOAD_SHEDDING,
	.OCP_flt_time = 0,
	.OCP_load_shed_time = 0,

};

#define __dcs default_conifer_settings
#define __ch_list default_conifer_settings.ch_list

conifer_state cstate = {
	.status_flags = 0,
	.load_shedding = 0
};

static abstract_conifer_channel ch_table[FINAL_CONIFER_OUTPUT];

//Driver function lookup tables
uv_status (*ch_updaters[8])(abstract_conifer_channel*, uint32_t*) = {0,0,0,0,0,0,0,0};
uv_status (*current_getters[8]) (abstract_conifer_channel*, uint32_t*) = {0,0,0,0,0,0,0,0};

//typedef enum uv_status_t uv_status;

/** @brief Generates the default channel list at runtime, because this is less painful than trying to statically declare it
 *
 */
__attribute__((constructor)) void coniferGenerateDefaults(){
	uint8_t n = 0;

	//Accumulator power
	__ch_list[n].ch = GENERAL_ACCU_PWR1;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_5;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_EN_BIT|CONIFER_CH_IS_CRIT_BIT;
	n++;

	//Motor controller power
	__ch_list[n].ch = BAMO_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_7;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_EN_BIT|CONIFER_CH_IS_CRIT_BIT;
	n++;

	//RFE
	__ch_list[n].ch = BAMO_RFE;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_11;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//RUN
	__ch_list[n].ch = BAMO_RUN;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_15;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//BSPD
	__ch_list[n].ch = BSPD_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_13;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_EN_BIT|CONIFER_CH_IS_CRIT_BIT;
	n++;

	//DASH + Steering wheel
	__ch_list[n].ch = DASH_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_10;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_EN_BIT|CONIFER_CH_IS_CRIT_BIT;
	n++;

	//VCU
	__ch_list[n].ch = VCU_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_6;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_EN_BIT|CONIFER_CH_IS_CRIT_BIT;
	n++;

	//BRAKE LIGHT
	__ch_list[n].ch = BRAKE_LIGHT;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_12;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//RTD BUZZER
	__ch_list[n].ch = HORN;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_9;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//COOLANT PUMP
	__ch_list[n].ch = COOLANT_PUMP1;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_1;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//RAD FAN 1
	__ch_list[n].ch = RAD_FANS1;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_2;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//RAD FAN 2
	__ch_list[n].ch = RAD_FANS2;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_20A_3;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_ALLOW_LOAD_SHED;
	n++;

	//SDC BOARD
	__ch_list[n].ch = SDC_BOARD_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_14;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;

	//SDC ORIGIN
	__ch_list[n].ch = HVIL_PWR;
	__ch_list[n].ch_dat.hardware_mapping = CONIFER_CH_IN_USE|(UV19_PDU_CH<<8)|U19_PDU_5A_16;
	__ch_list[n].ch_dat.status_control_reg = CONIFER_CH_IS_CRIT_BIT;
	n++;


	__dcs.n_ch = n;
}

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
	uint32_t ecode = CONIFER_DRV_OK;
	uv_status (*update_func)(abstract_conifer_channel*, uint32_t*) = ch_updaters[src];

	if(update_func == NULL){
		return UV_ABORTED;
	}

	uv_status retval = update_func(ch_ptr,&ecode);//Me executing the random ass callback func
	if(retval != UV_OK){
		if(ch_ptr->status_control_reg & CONIFER_CH_IS_CRIT_BIT){
			uvPanic("FATAL CONIFER FAULT",0);
			return UV_ERROR;
		}else{
			return UV_WARNING;
		}
	}
	return UV_OK;

}

void* theScrimDonguloder(uv_task_info* task_to_scrimdongulode){
	if(task_to_scrimdongulode == NULL){

	}
}

/** @brief Initializes the conifer library.
 *
 */
uv_status coniferInit(){
	uint32_t init_ecode;

	vTaskDelay(1);
	//STEP 1: Init the ch_table to something normal. Set all outputs as "not in use"
	for(int i = 0; i < FINAL_CONIFER_OUTPUT;i++){
		abstract_conifer_channel* ch = &ch_table[i];
		ch->hardware_mapping = 0x0700; //Makes channel as not in use and sets location as invalid
		ch->status_control_reg = 0;
	}


	//STEP 2: Pull up the relevant settings into the table
	for(int i = 0; i < conifer_params->n_ch; i++){
		uint16_t chID = conifer_params->ch_list[i].ch;
		ch_table[chID].hardware_mapping = conifer_params->ch_list[i].ch_dat.hardware_mapping;
		ch_table[chID].status_control_reg = conifer_params->ch_list[i].ch_dat.status_control_reg;
	}

	vTaskDelay(1);
	//STEP 3: Configure Driver LUTs

	//Channel updaters
	ch_updaters[UV19_PDU_CH] = u19updatePduChannel;
	vTaskDelay(1);
	//Current getters

	//Step 4: Initialize and confirm existance of relevant external devices
	uv_status retval = UV_OK;

	//Does the PDU exist?
	if(conifer_params->conif_flags & EXPECT_UV19PDU){
		//retval
		retval = initPDU(&init_ecode);
//		goto CONIFER_INIT_ERR;
	}

	if(retval != UV_OK){
		//Bad bad not good
	}

	//STEP 5: Set system initial conditions to turn on/off the relevant channels
	for(int i = 0; i < FINAL_CONIFER_OUTPUT; i++){
		abstract_conifer_channel* ch_ptr = &ch_table[i];
		if(!IS_CONIFER_CH_USED(ch_ptr)){
			continue;
		}

		//We will update all of the conifer channels now
		if(coniferUpdateChannel(ch_ptr) != UV_OK){
			//Handle the error
		}


	}
	return UV_OK;

}

uv_status coniferDeInit(){
	return UV_OK;
}

uv_status coniferEnLoadShedding(){
	if(cstate.load_shedding != 0){
		return UV_ABORTED;
	}


	cstate.load_shedding = 1;
	for(int i = 0; i< FINAL_CONIFER_OUTPUT;i++){
		abstract_conifer_channel* ch_ptr = &ch_table[i];
		if((ch_ptr->status_control_reg&CONIFER_CH_ALLOW_LOAD_SHED)){
			ch_ptr->status_control_reg |= CONIFER_CH_LS_ACTIVE;
		}else{
			continue;
		}

		if(coniferUpdateChannel(ch_ptr) == UV_ERROR){
			//Handle the error
		}
	}
	return UV_OK;
}

uv_status coniferDisLoadShedding(){
	if(cstate.load_shedding == 0){
		return UV_ABORTED;
	}


	cstate.load_shedding = 0;
	for(int i = 0; i< FINAL_CONIFER_OUTPUT;i++){
		abstract_conifer_channel* ch_ptr = &ch_table[i];

		if(!(ch_ptr->status_control_reg&CONIFER_CH_LS_ACTIVE)){
			continue;
		}

		ch_ptr->status_control_reg&= (~CONIFER_CH_LS_ACTIVE);

		if(coniferUpdateChannel(ch_ptr) == UV_ERROR){
			//Handle the error
		}
	}

	return UV_OK;
}

/** @brief Enables a conifer output channel
 *
 */
uv_status coniferEnChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = &ch_table[ch];
	//uint8_t src = 7;
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	ch_ptr->status_control_reg |= CONIFER_CH_EN_BIT;

	return coniferUpdateChannel(ch_ptr);
}

/** @brief Disables a conifer output channel
 *
 */
uv_status coniferDisChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = &ch_table[ch];
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	ch_ptr->status_control_reg &= ~CONIFER_CH_EN_BIT;

	return coniferUpdateChannel(ch_ptr);
}

/** @brief Toggles a conifer output channel
 *
 */
uv_status coniferToggleChannel(conifer_output_channel ch){
	abstract_conifer_channel* ch_ptr = &ch_table[ch];
	if(!IS_CONIFER_CH_USED(ch_ptr)){
		return UV_ABORTED;
	}

	ch_ptr->status_control_reg ^= CONIFER_CH_EN_BIT;

	return coniferUpdateChannel(ch_ptr);
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

