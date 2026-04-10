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
//TaskStatus_t task_stats[MAX_NUM_MANAGED_TASKS+5];

//#define DEBUG_PORT_GENERAL 0
//#define DEBUG_PORT_OS 1
//#define DEBUG_PORT_TRACTIVE_SYSTEM 2
//#define DEBUG_PORT_XDEV 3
//#define DEBUG_PORT_CONIFER 4
//#define DEBUG_PORT_STATE_ENGINE 5
//#define DEBUG_PORT_CSV 6

#define DEBUG_LOG_TASK_RUNTIME_STATS

typedef struct{
	uint32_t port;
	char* str;
}uv_print_request;

QueueHandle_t print_queue;

void dispWheelSpeeds();

static uint32_t ITM_SendCharToReg (char ch,uint32_t port);
static uv_status __debugWriteInternal(char* str,uint32_t port);

int32_t sprint_fixed_d(char* buf, const char* label, int32_t value, int decimals, const char* unit)
{
	int32_t nchars_written = 0;
	int32_t tmp = 0;
	int32_t scale = 1;
	for(int i = 0; i < decimals; i++) scale *= 10;

	int32_t whole = value / scale;
	int32_t frac  = value % scale;
	if(frac < 0) frac = -frac;

	tmp = sprintf(buf,"%s: %ld", label, (long)whole);
	if(tmp < 0){
		return -1;
	}else{
		nchars_written += tmp;
		buf += tmp;
	}
	if(decimals > 0){
		tmp = sprintf(buf,".%0*ld", decimals, (long)frac);
		if(tmp < 0){
			return -1;
		}
		nchars_written += tmp;
		buf += tmp;
	}
	if(unit){
		tmp = sprintf(buf," %s", unit);
		if(tmp < 0){
			return -1;
		}
		nchars_written += tmp;
		buf += tmp;
	}
	tmp = sprintf(buf,"\n");
	nchars_written += tmp;
	buf += tmp;


	return nchars_written;
}

void print_fixed_d(const char* label, int32_t value, int decimals, const char* unit)
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

	// From bms.c:
	// pack_voltage_dV is 0.1V
	// pack_current_dA is 0.1A (signed)
	// soc_pct is % int
	// min/max temps in C

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

	char buf[256] = {0};
	printf("\nIMD STATUS\n");
	printf("----------\n");

	uint8_t serial_ok = IMD_GetSerial0Valid();
	uint32_t serial_word = IMD_GetSerial0Word();
	uint8_t get_status_bits = IMD_GetStatusBits();
	uint16_t get_rp_raw = IMD_GetRpRaw();
	uint16_t get_rn_raw = IMD_GetRnRaw();
	uint16_t get_errors = IMD_GetErrorFlagsRaw();
	uint16_t get_safety_touch_current = IMD_GetSafetyTouchCurrent();

	printf("Serial Valid: %u\n", (unsigned)serial_ok);
	printf("Serial Word:  0x%08lX\n", (unsigned long)serial_word);
	printf("Isolation Resistance (Rp): %u\n", (unsigned)get_rp_raw);
	printf("Isolation Resistance (Rn): %u\n", (unsigned)get_rn_raw);
	printf("Error Flags: %u\n", (unsigned)get_errors);
	printf("Safe to touch? (Current Value): %u\n", (unsigned)get_safety_touch_current);
	printf("IMD ONLINE: %s\n", IMD_IsOnline() ? "YES" : "NO");
}

void dispStateEngineStatus(){
	return;
}


void dispExtDeviceStatus(){
	//BMS
	dispBMSStatus();
	//IMD
	dispIMDStatus();

	//PDU

	//Motor Controller

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
	printf("System Time: [ %d ] \t",systime);
	printf("Vehicle State %s \n",str);


	dispExtDeviceStatus();
	//dispTractiveSystemStatus();
	//dispStateEngineStatus();


	return;
}

/** @brief Background task responsible for much of our live telemtry and fault detection capabilies
 *
 */
void uvBackgroundDiagnosticsDaemon(void* args){
	uv_task_info* params = (uv_task_info*) args;
	int k = 0;
	UBaseType_t n_active_tasks = 0;

	for(;;){
		vTaskDelay(100);



#ifdef DEBUG
	if(k%10 == 0){
		//dispVehicleStatusReport();
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
#ifdef DEBUG
		printf("Initializing Diagnostics\n");
#endif
	uint32_t var = 0;
	uv_task_info* diag_task = uvCreateServiceTask();
	diag_task->task_function = uvBackgroundDiagnosticsDaemon;
	diag_task->active_states = 0xFFFF;
	diag_task->task_name = "diagDaemon";
	diag_task->stack_size = 1024;
	diag_task->task_priority = 1;





	//TestPorts lol;
	if(__debugWriteInternal("Testing Port 0\n \0",0)!=UV_OK){
		uvPanic("ITM_FAIL",0);
	}

	if(__debugWriteInternal("Testing Port 1\n \0",1)!=UV_OK){
		uvPanic("ITM_FAIL",0);
	}

	if(__debugWriteInternal("Testing Port 2\n \0",2)!=UV_OK){
			uvPanic("ITM_FAIL",0);
	}

	if(__debugWriteInternal("Testing Port 3\n \0",3)!=UV_OK){
				uvPanic("ITM_FAIL",0);
	}
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
static uint32_t ITM_SendCharToReg (char ch, uint32_t port)
{
    if (port < 32) { // ITM has 32 ports (0-31)
        // Check if ITM is enabled and port stimulus is enabled
        if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port))) {
            while (ITM->PORT[port].u32 == 0); // Wait for port to be ready
            ITM->PORT[port].u8 = (uint8_t)ch;
        }
    }
}

static uv_status __debugWriteInternal(char* str,uint32_t port){
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

//TODO RTOSiffy this to delegate printing to the actual task responsible for that
uv_status __debugWrite(char* str, uint32_t port){
	return __debugWriteInternal(str,port);
}


void uvAssertFailed(char* file, uint16_t line, TaskHandle_t task, char* condition){

}

#endif

void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ){
	//This is where we end up if one of the tasks has a stack overflow

	//STOP THE CAR
	uvPanic("Stack_overflow",0);
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
	uvPanic("Failed Malloc",0);
	//LOG WHAT HAPPENED

	//What task did it

	//Hang

	for(;;){

	}
}


void vApplicationTickHook( void ){
	//This is not used but it makes the compiler STFU
}

static uint32_t tictime_us = 0xFFFFFFFF;

void __tic(){
	tictime_us = __HAL_TIM_GET_COUNTER(&htim5);
}

uint32_t __toc(){
	uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
	if (tictime_us > now) return 0;
	return now - tictime_us;
}
