/*
 * uvfr_diagnostics_system.c
 *
 *  Created on: Jun 14, 2025
 *      Author: byo10
 */
#define __UV_FILENAME__ "uvfr_diagnostics.c"
#include "uvfr_utils.h"
#include "bms.h"
#include "imd.h"

extern HeapStats_t xHeapStats;

#define DEBUG_PORT_GENERAL 0
#define DEBUG_PORT_OS 1
#define DEBUG_PORT_TRACTIVE_SYSTEM 2
#define DEBUG_PORT_XDEV 3
#define DEBUG_PORT_CONIFER 4
#define DEBUG_PORT_STATE_ENGINE 5
#define DEBUG_PORT_CSV 6



void dispWheelSpeeds();

void dispStateEngineStatus(){
	return;
}

/* =========================================================
 * NOTE:
 * printf("%f") doesn't work on our target, so we print fixed-point values
 * by splitting integer/decimal manually.
 * ========================================================= */
static void print_fixed_d(const char* label, int32_t value, int decimals, const char* unit)
{
	int32_t scale = 1;
	for(int i = 0; i < decimals; i++) scale *= 10;

	int32_t whole = value / scale;
	int32_t frac  = value % scale;
	if(frac < 0) frac = -frac;

	printf("%s: %ld", label, (long)whole);
	if(decimals > 0){
		printf(".%0*ld", decimals, (long)frac);
	}
	if(unit){
		printf(" %s", unit);
	}
	printf("\n");
}

static void dispBMSStatus(void)
{
	printf("\nBMS STATUS\n");
	printf("----------\n");

	// msg1 can handle current, voltage, charge, relay, checksum (corruption)
	// g_bms_state.pack_current_dA  : 0.1 A
	// g_bms_state.pack_voltage_dV  : 0.1 V
	// g_bms_state.soc_pct          : percent (int)
	// g_bms_state.min/max_cell_temp: Celsius

	print_fixed_d("Pack Voltage", (int32_t)g_bms_state.pack_voltage_dV, 1, "V");
	print_fixed_d("Pack Current", (int32_t)g_bms_state.pack_current_dA, 1, "A");

	printf("SOC: %u %%\n", (unsigned)g_bms_state.soc_pct);

	printf("Min Cell Temp: %d C\n", (int)g_bms_state.min_cell_temp);
	printf("Max Cell Temp: %d C\n", (int)g_bms_state.max_cell_temp);

	printf("Relay State: 0x%04X\n", (unsigned)g_bms_state.relayState);

	printf("MSG1 Corrupt: %u\n", (unsigned)g_bms_state.msg1corrupt);
	printf("MSG2 Corrupt: %u\n", (unsigned)g_bms_state.msg2corrupt);
}

static void dispIMDStatus(void)
{
	printf("\nIMD STATUS\n");
	printf("----------\n");

	// These getters must exist in imd.c / imd.h
	// (keeps diagnostics cohesive without poking IMD internals directly)
	uint8_t  serial_ok   = IMD_GetSerial0Valid();
	uint32_t serial_word = IMD_GetSerial0Word();

	printf("Serial Valid: %u\n", (unsigned)serial_ok);
	printf("Serial Word:  0x%08lX\n", (unsigned long)serial_word);

	// If you add more getters later, you can print them here too:
	// printf("Status Bits:  0x%02X\n", (unsigned)IMD_GetStatusBits());
	// printf("Error Flags:  0x%04X\n", (unsigned)IMD_GetErrorFlags());

	printf("IMD ONLINE: %s\n", serial_ok ? "YES" : "NO");
}

void dispExtDeviceStatus(){
	//BMS
	dispBMSStatus();

	//PDU

	//Motor Controller

	//IMD
	dispIMDStatus();

	//DCDC

	//Steering wheel

	//Whatever else

}

void dispTractiveSystemStatus(){
	printf("TRACTIVE SYSTEM STATUS\n");
	printf("----------------------\n");
	printf("Contactor Status:\n");
	printf("Pack Voltage: %d.%d",0,0);
	printf("Pack Current: %d.%d\n",0,0);
	//printf("MC Warning + ERRORS: %d\n",0,0);
	printf("Motor RPM: %d.%d RPM",0,0);
	printf("Motor Phase current: %d.%d A \n",0,0);

	printf("Motor Temp: %d.%d",0,0);
	printf("Inverter Temp: %d.%d \n",0,0);

	printf("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");

}

void dispVehicleStatusReport(){
	uint32_t systime = xTaskGetTickCount();
	char* str = NULL;
	str = uvGetStateString();


	printf("VEHICLE_STATUS:\n --------------------------------------- \n");
	printf("System Time: [ %lu ] \t",(unsigned long)systime);
	printf("Vehicle State %s \n",str);


	dispExtDeviceStatus();
	dispTractiveSystemStatus();
	dispStateEngineStatus();


	return;
}

/** @brief Background task responsible for much of our live telemtry and fault detection capabilies
 *
 */
void uvBackgroundDiagnosticsDaemon(void* args){
	uv_task_info* params = (uv_task_info*) args;
	int k = 0;
	for(;;){
		vTaskDelay(100);

#ifdef DEBUG
	if(k%10 == 0){
		dispVehicleStatusReport();
	}
#endif

		vPortGetHeapStats(&xHeapStats);
		//dispWheelSpeeds();
		k = (k + 1)%100;
		if(params->cmd_data == UV_KILL_CMD){
			killSelf(params);
		} else if(params->cmd_data == UV_SUSPEND_CMD){
			suspendSelf(params);
		}

	}
}

uv_status uvEnterDiagnosticMode(){
	return UV_OK;
}

uv_status uvExitDiagnosticMode(){
	return UV_OK;
}

/** @brief Initialize the diagnostics of the vehicle
 *
 */
uv_status uvInitDiagnostics(){
	uint32_t var = 0;
	uv_task_info* diag_task = uvCreateServiceTask();
	diag_task->task_function = uvBackgroundDiagnosticsDaemon;
	diag_task->active_states = 0xFFFF;
	diag_task->task_name = "diagDaemon";
	diag_task->stack_size = 1024;


	uvStartTask(&var,diag_task);
	return UV_OK;
}



void handleDiagnosticMsg(uv_CAN_msg* msg){

	uint8_t cmd_byte = msg->data[0];
	switch(cmd_byte){
	case ENTER_DIAGNOSTICS_MODE:

		break;
	case EXIT_DIAGNOSTICS_MODE:

		break;
	case REQUEST_STATE_CHANGE:
		if(changeVehicleState(msg->data[1]<<8 | msg->data[2]) != UV_OK){

		}
		break;
	case FORCE_STATE_CHANGE:

		break;
	case dCLEAR_FAULTS:

		break;
	default:

		break;
	}
}


/** overrides the weak function prototype of __io_putchar so that we can use the
 * ITM registers to send diagnostic data back to a debugger
 *
 */
int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
}

#ifdef DEBUG
#define MAX_DEBUGSTRING_LENGTH 256
//Basically like ITM_SendChar, however you can send to one of several registers
uint32_t ITM_SendCharToReg (uint32_t ch,uint32_t port)
{
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    while (ITM->PORT[port].u32 == 0UL)
    {
      __NOP();
    }
    ITM->PORT[port].u8 = (uint8_t)ch;
  }
  return (ch);
}

uv_status __debugWrite(char* str,uint32_t port){
	int i = 0;
	while(i<MAX_DEBUGSTRING_LENGTH){
		if(ITM_SendCharToReg(str[i],port)!=str[i]){
			//UHH OHH
		}

		if(str[i] == '\0'){
			break;
		}
		i++;
	}

	return UV_OK;
}




void uvAssertFailed(char* file, uint16_t line, TaskHandle_t task, char* condition){

}

#endif

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ){
	//This is where we end up if one of the tasks has a stack overflow

	//STOP THE CAR

	//LOG WHAT HAPPENED

	//What task did it

	//How badly did it overflow?

	//Try to get a trace!

	//Hang:

	for(;;){

	}
}

void vApplicationMallocFailedHook(){
	//pvPortMalloc has failed

	//STOP THE CAR

	//LOG WHAT HAPPENED

	//What task did it

	//Hang
}


void vApplicationTickHook( void ){
	//This is not used but it makes the compiler STFU
}

void __tic();

uint32_t __toc();
