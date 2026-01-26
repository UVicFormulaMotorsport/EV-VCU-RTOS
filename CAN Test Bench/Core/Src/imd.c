// This where the code to handle IMD errors and such will go
// Jan 2026
// Rachan Grewal and Quazi Heider

// personal note for myself: this does NOT handle logic for actually shutting down car
// it logs data specifically for something like "What happened RIGHT BEFORE we shut down"
// IMD itself handles all the safety logic at a hardware level, not software controlled.
// it is less safe to handle shutdown logic in CAN because of noise and signal degradation

#define __UV_FILENAME__ "imd.c"

#include "imd.h"

// We need to include can.h because we will send CAN messages through the functions in that file
// When a CAN message comes it will throw an interrupt can.c deals with the incoming message
// the function in can.c gets the ID and sends the data to  the functions here
#include "can.h"
#include "main.h"
#include "constants.h"
#include "uvfr_utils.h"

// We need to include pdu.h for the shutdown circuit
#include "pdu.h"

#include <string.h>
#include <stdint.h>

/* =========================================================
 * CAN IDs / Request MUX codes
 * ========================================================= */

//TODO: set these the values as coressponding

#ifndef IMD_CAN_ID_Tx
#define IMD_CAN_ID_Tx  0x0A100101UL
// Request from host is structured as:
// 		0xA100101 + Operand Bit
#endif

#ifndef IMD_CAN_ID_Rx
#define IMD_CAN_ID_Rx  0x0A100100UL
#endif

// this is  IMD request code for serial
#ifndef RequestMUX_serial_number_0
#define RequestMUX_serial_number_0  0x08
#endif

//the data length code im seeing for every MUX in the ref manual is 3
// this is a result of id + 1 byte operator (read write etc) + 2 bytes data
#ifndef uv_imd_standard_dlc
#define uv_imd_standard_dlc 3
#endif

// These are all the valid Request_mux parameters we want to consistently poll
#ifndef RequestMUX_isolation_state
#define RequestMUX_isolation_state 0xE0
#endif

#ifndef RequestMUX_isolation_resistances
#define RequestMUX_isolation_resistances 0xE1
#endif

#ifndef RequestMUX_isolation_capacitances
#define RequestMUX_isolation_capacitances 0xE2
#endif

#ifndef RequestMUX_battery_voltage_vb
#define RequestMUX_battery_voltage_vb 0xE4
#endif

#ifndef RequestMUX_error_flags
#define RequestMUX_error_flags 0xE5
#endif

#ifndef RequestMUX_dynamic_iso_state
#define RequestMUX_dynamic_iso_state 0xE7
#endif


// single-word expected serial chunk for a “simple check”
static const uint32_t IMD_EXPECTED_SERIAL0 = 0xB8DD9AF9U;


/* =========================================================
 * Local state
 * ========================================================= */

typedef struct {
	uint8_t online;

	// Status bits returned in byte 1 on status replies
	uint8_t status_bits;

	// Parsed values (raw, units depend on IMD manual)
	uint16_t iso_state_raw;   // E0: bytes 2..3
	uint16_t rp_raw;          // E1: bytes 2..3
	uint16_t rn_raw;          // E1: bytes 5..6
	uint16_t cp_nf;           // E2: bytes 2..3
	uint16_t cn_nf;           // E2: bytes 5..6
	uint16_t glv_raw;         // E4: bytes 2..3
	uint16_t error_flags;     // E5: bytes 2..3

	// “simple check” serial chunk
	uint8_t  serial0_valid;
	uint32_t serial0_word;
} imd_state_t;

static volatile imd_state_t g_imd = {0};


// Mutex for IMD state
static SemaphoreHandle_t imd_mutex = NULL;

static uv_status IMD_EnsureMutex(void) {
	if (imd_mutex == NULL) {
		imd_mutex = xSemaphoreCreateMutex();
		if (imd_mutex == NULL) return UV_ERROR;
	}
	return UV_OK;
}

static inline void IMD_Lock(void)   { xSemaphoreTake(imd_mutex, portMAX_DELAY); }
static inline void IMD_Unlock(void) { xSemaphoreGive(imd_mutex); }

static inline uint16_t u16_be(const uint8_t *p) {
	return (uint16_t)((p[0] << 8) | p[1]);
}


// This is for sending an invdividual request, NOT polling consistently
// Building a 1-byte request frame
static void IMD_SendRequest(uint8_t code) {
	uv_CAN_msg msg;
	memset(&msg, 0, sizeof(msg));

	msg.msg_id = IMD_CAN_ID_Tx;
	msg.dlc    = 1;                  // 1 byte request: MUX only
	msg.flags  = UV_CAN_EXTENDED_ID;  // need to integrate with can.c
	msg.data[0] = code;

	uvSendCanMSG(&msg);
}


// XDevMon integration

static uv_status IMD_RegisterWithXDevMon(void) {
	// Polling every 100ms is a good starting point i think
	TickType_t period_ms = 100;

	uint16_t flags = XDEV_DEVICE_EXPECTED | XDEV_CHECK_TIMEOUT_BIT | XDEV_POLLING_REQUIRED;

	if (uvRegisterExternalDevice(IMD, period_ms, flags, "IMD") != UV_OK) {
		return UV_ERROR;
	}

	// messages that will be constantly polled and taken data from are here
	// the uv_CAN_msg is being constructed and sent here

	//All isolation related MUX
	// edit all these later to use header file

	//poll for electrical isolation in bytes 2 and 3
	uv_CAN_msg poll_isolation_state;
	memset(&poll_isolation_state, 0, sizeof(poll_isolation_state));
	poll_isolation_state.msg_id  = IMD_CAN_ID_Tx;
	poll_isolation_state.dlc     = uv_imd_standard_dlc;                  // request is 1 byte (MUX)
	poll_isolation_state.flags   = UV_CAN_EXTENDED_ID | CAN_BUS_1;
	poll_isolation_state.data[0] = uv_request_mux_isolation_state; // edit everything to use this

	//poll for resistance from postive of HV to chassis (Rp) and negative of HV to chassis (Rn)
	uv_CAN_msg poll_isolation_resistance;
	memset(&poll_isolation_resistance, 0, sizeof(poll_isolation_resistance));
	poll_isolation_resistance.msg_id  = IMD_CAN_ID_Tx;
	poll_isolation_resistance.dlc     = uv_imd_standard_dlc;
	poll_isolation_resistance.flags   = UV_CAN_EXTENDED_ID;
	poll_isolation_resistance.data[0] = uv_request_mux_isolation_resistances;

	//poll for capacitance from from HV positive to chassis (Cp) and capacitance from HV negatie to chassis (Cn)
	uv_CAN_msg poll_isolation_capacitances;
	memset(&poll_isolation_capacitances, 0, sizeof(poll_isolation_capacitances));
	poll_isolation_capacitances.msg_id  = IMD_CAN_ID_Tx;
	poll_isolation_capacitances.dlc     = uv_imd_standard_dlc;
	poll_isolation_capacitances.flags   = UV_CAN_EXTENDED_ID;
	poll_isolation_capacitances.data[0] = uv_request_mux_isolation_capacitances;

	// Poll for "safe to touch" aspect
	// It does this calculation on its own, and we can log this to see what caused it to go out of sepc
	uv_CAN_msg poll_safety_touch_energy;
	memset(&poll_safety_touch_energy, 0, sizeof(poll_safety_touch_energy));
	poll_safety_touch_energy.msg_id  = IMD_CAN_ID_Tx;
	poll_safety_touch_energy.dlc     = uv_imd_standard_dlc;
	poll_safety_touch_energy.flags   = UV_CAN_EXTENDED_ID;
	poll_safety_touch_energy.data[0] = uv_request_mux_safety_touch_energy;
	uv_CAN_msg poll_safety_touch_current;
	memset(&poll_dynamic_iso_state, 0, sizeof(poll_dynamic_iso_state));
	poll_dynamic_iso_state.msg_id  = IMD_CAN_ID_Tx;
	poll_dynamic_iso_state.dlc     = uv_imd_standard_dlc;
	poll_dynamic_iso_state.flags   = UV_CAN_EXTENDED_ID;
	poll_dynamic_iso_state.data[0] = uv_request_mux_safety_touch_current;

	//battery voltage MUX
	uv_CAN_msg poll_battery_voltage_vb;
	memset(&poll_battery_voltage_vb, 0, sizeof(poll_battery_voltage_vb));
	poll_battery_voltage_vb.msg_id  = IMD_CAN_ID_Tx;
	poll_battery_voltage_vb.dlc     = uv_imd_standard_dlc;
	poll_battery_voltage_vb.flags   = UV_CAN_EXTENDED_ID;
	poll_battery_voltage_vb.data[0] = uv_request_mux_battery_voltage;

	//error flag MUX
	uv_CAN_msg poll_error_flags;
	memset(&poll_error_flags, 0, sizeof(poll_error_flags));
	poll_error_flags.msg_id  = IMD_CAN_ID_Tx;
	poll_error_flags.dlc     = 1;
	poll_error_flags.flags   = UV_CAN_EXTENDED_ID;
	poll_error_flags.data[0] = uv_request_mux_Error_flags;


	//Add all current MUX to xdevmon, these are the values from the IMD we'll constantly be polling

	if (uvAddPollMsgToXdev(IMD, &poll_isolation_state) != UV_OK) {
		return UV_ERROR;
	}

	if (uvAddPollMsgToXdev(IMD, &poll_isolation_resistance) != UV_OK) {
		return UV_ERROR;
	}

	if (uvAddPollMsgToXdev(IMD, &poll_isolation_capacitances) != UV_OK) {
		return UV_ERROR;
	}

	if (uvAddPollMsgToXdev(IMD, &poll_safety_touch_current) != UV_OK) {
		return UV_ERROR;
	}

	if (uvAddPollMsgToXdev(IMD, &poll_error_flags) != UV_OK) {
		return UV_ERROR;
	}

	if (uvAddPollMsgToXdev(IMD, &poll_battery_voltage_vb) != UV_OK) {
		return UV_ERROR;
	}

	return UV_OK;
}

// send message once,


// init task

void initIMD(void *args) {
	uv_init_task_args *params = (uv_init_task_args *)args;

	uv_init_task_response resp;
	memset(&resp, 0, sizeof(resp));
	resp.device = IMD;
	resp.status = UV_ERROR;
	resp.errmsg = "IMD init fail";
	resp.nchar  = 12;

	if (!params || !params->init_info_queue) {
		vTaskDelete(NULL);
	}

	if (IMD_EnsureMutex() != UV_OK) {
		resp.errmsg = "IMD mutex";
		resp.nchar  = 9;
		goto done;
	}

	// Reset state
	IMD_Lock();
	memset((void*)&g_imd, 0, sizeof(g_imd));
	IMD_Unlock();

	// register with XDevMon and install ping poll message
	if (IMD_RegisterWithXDevMon() != UV_OK) {
		resp.errmsg = "IMD xdev reg";
		resp.nchar  = 12;
		goto done;
	}

	// optional: send one immediate request (so init doesn't wait for next poll tick)
	// NOTE: using your enum name from imd.h here
	IMD_SendRequest(Serial_number_0);

	// wait for a response (requires CAN RX handler to call externalDeviceRxHandler(IMD))
	if (uvWaitOnExternalDevice(IMD, pdMS_TO_TICKS(300)) != UV_OK) {
		resp.errmsg = "IMD no resp";
		resp.nchar  = 11;
		goto done;
	}

	// simple “serial0” validation
	IMD_Lock();
	uint8_t ok = g_imd.serial0_valid && (g_imd.serial0_word == IMD_EXPECTED_SERIAL0);
	IMD_Unlock();

	if (!ok) {
		resp.errmsg = "IMD serial bad";
		resp.nchar  = 14;
		goto done;
	}

	resp.status = UV_OK;
	resp.errmsg = NULL;
	resp.nchar  = 0;

done:
	xQueueSendToBack(params->init_info_queue, &resp, 0);

	// Starts up, does what it needs to do, then sleeps
	vTaskSuspend(NULL);
}


// CAN RX hook
// Call this from can.c when IMD_CAN_ID_Rx arrives
void IMD_CanRxHandler(uint32_t can_id, const uint8_t data[8], uint8_t dlc) {
	if (can_id != IMD_CAN_ID_Rx) return;
	if (dlc < 2) return; // need at least mux + status bits (or mux + data for serial)

	uint8_t mux = data[0];

	if (IMD_EnsureMutex() != UV_OK) return;

	IMD_Lock();
	g_imd.online = 1;
	IMD_Unlock();

	// we need to add functionality here to handle all the different incoming messages
	switch (mux) {

		// only handle our "ping" response (serial check)
		case Serial_number_0: {
			if (dlc < 5) break;

			// imd.c (original) used [1..4] as the 32-bit chunk (keep consistent)
			uint32_t word =
				((uint32_t)data[1] << 24) |
				((uint32_t)data[2] << 16) |
				((uint32_t)data[3] <<  8) |
				((uint32_t)data[4] <<  0);

			IMD_Lock();
			g_imd.serial0_word  = word;
			g_imd.serial0_valid = 1;
			IMD_Unlock();
			break;
		}

		case RequestMUX_isolation_state: {
			if (dlc < 4) break;
			IMD_Lock();
			g_imd.status_bits   = data[1];
			g_imd.iso_state_raw = u16_be(&data[2]);
			IMD_Unlock();
			break;
		}

		case RequestMUX_isolation_resistances: {
			if (dlc < 7) break;
			IMD_Lock();
			g_imd.status_bits = data[1];
			g_imd.rp_raw      = u16_be(&data[2]);
			g_imd.rn_raw      = u16_be(&data[5]);
			IMD_Unlock();
			break;
		}

		case RequestMUX_isolation_capacitances: {
			if (dlc < 7) break;
			IMD_Lock();
			g_imd.status_bits = data[1];
			g_imd.cp_nf       = u16_be(&data[2]);
			g_imd.cn_nf       = u16_be(&data[5]);
			IMD_Unlock();
			break;
		}

		case RequestMUX_battery_voltage_vb: {
			if (dlc < 4) break;
			IMD_Lock();
			g_imd.status_bits = data[1];
			g_imd.glv_raw     = u16_be(&data[2]);
			IMD_Unlock();
			break;
		}

		case RequestMUX_error_flags: {
			if (dlc < 4) break;
			IMD_Lock();
			g_imd.status_bits  = data[1];
			g_imd.error_flags  = u16_be(&data[2]);
			IMD_Unlock();
			break;
		}

		case RequestMUX_dynamic_iso_state: {
			// You can decode this once you confirm payload layout
			// For now, still record status bits so you can see it in diagnostics
			IMD_Lock();
			g_imd.status_bits = data[1];
			IMD_Unlock();
			break;
		}

		default:
			// unhandled mux
			break;
	}

	// CRITICAL: tells XDevMon / init waiters that IMD responded IMPORTNAT
	externalDeviceRxHandler(IMD);
}
