// This where the code to handle BMS errors and such will go
//Quazi Heider

#include "main.h"
#include "bms.h"
#include "constants.h"
#include "pdu.h"
#include "can.h"
#include "tim.h"
#include "dash.h"
#include <string.h>
#include <stdint.h>



//two IDs for bms can message
// 0x6B0
// 0x6B1

volatile bms_state_t g_bms_state = {0};

#define curr_bms_settings (*(current_vehicle_settings->bms_settings))

bms_settings_t default_bms_settings = {
		.BMS_CAN_timeout = 100,

		// Temp ranges
		.discharge_cold_fault_c = -20,
		.discharge_cold_warn_c  = -10,

		.charge_cold_fault_c    = 0,
		.charge_hot_fault_c     = 45,

		.discharge_hot_warn_c             = 50,
		.discharge_hot_derate_c           = 55,
		.discharge_hot_fault_c            = 60,

		// temp sensor plausibility checks
		.temp_plaus_min_c       = -40,
		.temp_plaus_max_c       = 100,

		// SOC threshoolds
		.soc_low_warn_pct       = 20,
		.soc_regen_disable_pct  = 95,

		//derate start point
		.derate_start_c         = 55,


		// Plausibility ranges
		// These values may need to be changed
		.temp_plaus_min_c        = -40,
		.temp_plaus_max_c        = 100,
		.current_plaus_min_dA    = -20000,   // -2000 A
		.current_plaus_max_dA    =  20000,   // +2000 A
		.voltage_plaus_min_dV    = 0,        // 0 V
		.voltage_plaus_max_dV    = 8000,     // 800 V (adjust to your pack)

		//Temps (need to be tuned to cell datasheet and comp-specific rules)


};
void BMS_msg1(uv_CAN_msg* msg){ // msg is raw CAN msg, gets processed in voltageCANdata
	// msg1 can handle current, voltage, charge, relay, checksum (corruption)
	g_bms_state.pack_current_dA = (msg->data[0]<<8 | msg->data[1]); // x 0.1A
	g_bms_state.pack_voltage_dV = (msg->data[2]<<8 | msg->data[3]); // x 0.1V

	g_bms_state.soc_pct = (msg->data[4])*2; // x2; this is an int

	g_bms_state.relayState = (msg->data[5]<<8 | msg ->data[6]);
	g_bms_state.msg1corrupt = (msg->data[7]);


	/*
	 * Chatgpt suggested code for handling corrupt byte
	 *     // CRC / corruption (example XOR; replace with real CRC if you have one)
			uint8_t rx_crc = msg->data[7];
			uint8_t calc   = crc8_simple(msg->data, 7);
	 *
	 */

	//current and voltage plausibility checks

	//corrupt message
	if (g_bms_state.msg1corrupt){
		//uvPanic? corrupt bms message
		return; // kill process as bad msg
	}

	// --- PLAUSIBILITY CHECKS ---
	// Current plausibility - verify we are getting clean values before they get used
	// values like amp, voltage to compute power will be needed when creating torque reduction system for TCS
	if (g_bms_state.pack_current_dA < curr_bms_settings.current_plaus_min_dA ||
		g_bms_state.pack_current_dA > curr_bms_settings.current_plaus_max_dA) {
		// raise fault for sensor range
		// log warning for implausible amp range
		return;  // fail-fast
	}

	// Voltage plausibility
	if (g_bms_state.pack_voltage_dV < curr_bms_settings.voltage_plaus_min_dV ||
		g_bms_state.pack_voltage_dV > curr_bms_settings.voltage_plaus_max_dV) {
		// raise fault for sensor range
		// log warning for implausible voltage value
		return;
	}

	// SOC checks
	// Optional: SOC plausibility (0–100%)
	if (g_bms_state.soc_pct > 100) {
		// raise fault for sensor range
		// log warning for implausible SOC
		//chatgpt says we should clamp this value but i dont think we should because the battery should never reach over 100% anyway
		// if it does reach over 100 this condition will hit and we will just reset the pct value, does not make sense to me
		// if its over 100 its bad
		//g_bms_state.soc_pct = 100; clamp it instead of using invalid value
	}
	if (g_bms_state.soc_pct < curr_bms_settings.soc_low_warn_pct) { // if battery pct is lower than settings warning threshold
		// flash low battery warning to screen
	}

	else if (g_bms_state.soc_pct >= curr_bms_settings.soc_regen_disable_pct) { // if battery pct is higher than regen warning thresholh
		// disable regenerative breaking
	}

	else {
		// regenerative braking should be on as battery within conditionsw
	}
	// in all other cases battery should be functioning as normal

}

void BMS_msg2(uv_CAN_msg* msg){
	// msg2 can handle DCL (Max current output), tempurature, checksum
	g_bms_state.dcl_dA = (msg->data[0]<<8 | msg->data[3]);
	// this is the derate current limit - how much power is the motor allowed to send out at this given moment
	// determined within the incoming msg?? i actually dont understand why, shouldnt we be the one telling the battery how much it can ouput
	// this value would be useful in creating limp mode, can be static
	// could also be useful in creating TCS but would have to be dynamic (gradually allow more power respective to tire slip)
	// might need to extern this to use in msg1
	// however right now i have no clue what to do with this lol

	g_bms_state.min_cell_temp = (msg->data[4]); // celsius
	g_bms_state.max_cell_temp = (msg->data[5]); // celsius

	g_bms_state.msg2corrupt = (msg->data[7]);

	/*
	 * Chatgpt suggested code for handling corrupt byte
	 *     // CRC / corruption (example XOR; replace with real CRC if you have one)
    		uint8_t rx_crc = msg->data[7];
    		uint8_t calc   = crc8_simple(msg->data, 7);
	 *
	 */


	// incoming can msg plausibility
	if (g_bms_state.msg2corrupt){
		//uvPanic?
		return;
	}

	//tempurature value plausibilty checks
	if (g_bms_state.min_cell_temp < curr_bms_settings.temp_plaus_min_c || g_bms_state.max_cell_temp > curr_bms_settings.temp_plaus_max_c) {
		/*
		 * checking validity of sensor values
		 * if out of range then raise error (sensor/comms fault)
		 */
	}

	//charger sanity checks (only need to run when charger is connected
	if (g_bms_state.max_cell_temp >= curr_bms_settings.charge_hot_fault_c){
		//STOP charging - battery is too hot to accept charge
	} else if (g_bms_state.min_cell_temp <= curr_bms_settings.charge_cold_fault_c){
		//STOP charging - battery is too cold to accept charge
	}


	// HOT SIDE
	if (g_bms_state.max_cell_temp >= curr_bms_settings.discharge_hot_fault_c) {
				 //hot temp hard fault
	}
	else if (g_bms_state.max_cell_temp >= curr_bms_settings.discharge_hot_derate_c){
		// approaching hot temps, derate motor or do something here
		// okay to keep running
		// to derate limit amps / power output (like a limp mode in a real car)
	}
	else if (g_bms_state.max_cell_temp >= curr_bms_settings.discharge_hot_warn_c){
		// dont need to do anything crazy here just flash dash warning that pack is hot
	}
	else {
		// clear any warnings
		// normal operability
		// reset any invoked conditions (eg limp mode - make sure car does not stay in limp mode after throwing it and recovering)
	}

	// COLD SIDE
	if (g_bms_state.min_cell_temp <= curr_bms_settings.discharge_cold_fault_c){
		//cold temp hard fault
	} else if (g_bms_state.min_cell_temp <= curr_bms_settings.discharge_cold_warn_c){
		//cold temp warning
		// need to derate battery here too as overpowering the battery at too low temps is harmful

	} else {
		// clear any warnings
		//normal operation
	}

}



void BMS_Init(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;

	osDelay(200);

	uv_init_task_response response = {UV_OK,BMS,0,NULL};

	if(xQueueSendToBack(params->init_info_queue,&response,100) != pdPASS){
			//OOPS
	}

	insertCANMessageHandler(0x6B0, BMS_msg1);
	insertCANMessageHandler(0x6B1, BMS_msg2);

		//Kill yourself
	vTaskSuspend(params->meta_task_handle);
}







