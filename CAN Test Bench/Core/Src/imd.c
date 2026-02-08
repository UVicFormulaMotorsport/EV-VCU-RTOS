// This where the code to handle IMD errors and such will go
// Jan 2026
// Rachan Grewal and Quazi Heider
//
// personal note for myself: this does NOT handle logic for actually shutting down car
// it logs data specifically for something like "What happened RIGHT BEFORE we shut down"
// IMD itself handles all the safety logic at a hardware level, not software controlled.
// it is less safe to handle shutdown logic in CAN because of noise and signal degradation

#define __UV_FILENAME__ "imd.c"

#include "imd.h"
#include "can.h"
#include "main.h"
#include "constants.h"
#include "uvfr_utils.h"
#include "uvfr_external_devices.h"

#include <string.h>
#include <stdint.h>

/* =========================================================
 * CAN IDs / bus
 * ========================================================= */

// TODO: replace these CAN IDs using the IMD datasheet
#ifndef IMD_CAN_ID_Tx
#define IMD_CAN_ID_Tx  0x0A100101UL
#endif

#ifndef IMD_CAN_ID_Rx
#define IMD_CAN_ID_Rx  0x0A100100UL
#endif

// TODO: pick the real bus the IMD is wired to
#ifndef IMD_CAN_BUS
#define IMD_CAN_BUS CAN_BUS_1
#endif

// Requests are 1 byte: MUX only
#define IMD_REQ_DLC 1

/* =========================================================
 * Optional: simple "ping" serial check
 * ========================================================= */

// If you don't want the init to fail based on serial, set this to 0
#ifndef IMD_ENABLE_SERIAL_CHECK
#define IMD_ENABLE_SERIAL_CHECK 1
#endif

// Replace it with real serial chunk OR disable serial check.
static const uint32_t IMD_EXPECTED_SERIAL0 = 0xB8DD9AF9U;

// Default IMD settings stored in flash SBLOCK (used by uvfr_settings.c)
const uv_imd_settings default_imd_settings = {
    .min_isolation_resistances       = 500,  // TODO pick real threshold (kOhm? raw?)
    .expected_isolation_capacitances = 0,    // TODO (nF? raw?)
    .max_imd_temperature             = 105,  // degC (if that’s what datasheet uses)
};


/* =========================================================
 * Local state
 * ========================================================= */

typedef struct {
	uint8_t online;

	// status bits byte from IMD replies
	uint8_t status_bits;

	// parsed raw values (exact units depend on datasheet)
	uint16_t iso_state_raw;      // uv_request_mux_isolation_state (E0)
	uint16_t rp_raw;             // uv_request_mux_isolation_resistances (E1) bytes 2..3
	uint16_t rn_raw;             // uv_request_mux_isolation_resistances (E1) bytes 5..6
	uint16_t cp_nf;              // uv_request_mux_isolation_capacitances (E2) bytes 2..3
	uint16_t cn_nf;              // uv_request_mux_isolation_capacitances (E2) bytes 5..6
	uint16_t glv_raw;            // uv_request_mux_battery_voltage (E4) bytes 2..3
	uint16_t error_flags_raw;    // uv_request_mux_Error_flags (E5) bytes 2..3
	uint16_t temp_raw;           // uv_request_mux_Temperature (0x80) bytes 2..3 (if used)
	uint16_t safety_touch_current; //uv_request_mux_safety_touch_current( (0xE6??) bytes something something
									//i will fix this later -quazi from byrons computer

	// ping / identity check
	uint8_t  serial0_valid;
	uint32_t serial0_word;
} imd_state_t;

static volatile imd_state_t g_imd_state = {0};


/* =========================================================
 * Helpers
 * ========================================================= */

static inline uint16_t u16_be(const uint8_t *p) {
	return (uint16_t)((p[0] << 8) | p[1]);
}

// Sending a one-byte request frame (MUX only)
static void IMD_SendRequest(uint8_t mux_code) {
	uv_CAN_msg msg;
	memset(&msg, 0, sizeof(msg));

	msg.msg_id  = IMD_CAN_ID_Tx;
	msg.dlc     = IMD_REQ_DLC;
	msg.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	msg.data[0] = mux_code;

	uvSendCanMSG(&msg);
}


/* =========================================================
 * XDevMon integration
 * ========================================================= */

static uv_status IMD_RegisterWithXDevMon(void) {
	TickType_t period_ms = 100;

	uint16_t flags = XDEV_DEVICE_EXPECTED | XDEV_CHECK_TIMEOUT_BIT | XDEV_POLLING_REQUIRED;

	if (uvRegisterExternalDevice(IMD, period_ms, flags, "IMD") != UV_OK) {
		return UV_ERROR;
	}

	// Add poll messages (DLC MUST BE 1 in your system)
	uv_CAN_msg poll;

	// --- ping / serial chunk (optional but useful) ---
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = Serial_number_0;
	if (uvAddPollMsgToXdev(IMD, &poll) != UV_OK) return UV_ERROR;

	// --- isolation resistance (your requested “certain value”) ---
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_isolation_resistances;
	if (uvAddPollMsgToXdev(IMD, &poll) != UV_OK) return UV_ERROR;

	// --- error flags (super useful to log) ---
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_Error_flags;
	if (uvAddPollMsgToXdev(IMD, &poll) != UV_OK) return UV_ERROR;

	// Optional extra polls (enable as you want)
	// isolation state
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_isolation_state;
	(void)uvAddPollMsgToXdev(IMD, &poll);

	// capacitances
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_isolation_capacitances;
	(void)uvAddPollMsgToXdev(IMD, &poll);

	// temperature
	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_Temperature;
	(void)uvAddPollMsgToXdev(IMD, &poll);

	memset(&poll, 0, sizeof(poll));
	poll.msg_id  = IMD_CAN_ID_Tx;
	poll.dlc     = 1;
	poll.flags   = UV_CAN_EXTENDED_ID | IMD_CAN_BUS;
	poll.data[0] = uv_request_mux_safety_touch_current;
	(void)uvAddPollMsgToXdev(IMD, &poll);

	return UV_OK;
}


/* =========================================================
 * CAN RX handler (BMS style)
 * ========================================================= */

void IMD_CanRxHandler(uv_CAN_msg* msg) {
	if (!msg) return;
	if (msg->msg_id != IMD_CAN_ID_Rx) return;
	if (msg->dlc < 1) return;

	// data[0] is always the returned MUX
	uint8_t mux = msg->data[0];

	// mark online every time we get something valid
	g_imd_state.online = 1;

	switch (mux) {

		// -------------------------------------------------
		// Manufacturer serial chunk 0: bytes [1..4]
		// -------------------------------------------------
		case Serial_number_0: {
			if (msg->dlc < 5) break;

			uint32_t word =
				((uint32_t)msg->data[1] << 24) |
				((uint32_t)msg->data[2] << 16) |
				((uint32_t)msg->data[3] <<  8) |
				((uint32_t)msg->data[4] <<  0);

			g_imd_state.serial0_word  = word;
			g_imd_state.serial0_valid = 1;
			break;
		}

		// -------------------------------------------------
		// Isolation state (E0): status bits at [1], value at [2..3]
		// -------------------------------------------------
		case uv_request_mux_isolation_state: {
			if (msg->dlc < 4) break;
			g_imd_state.status_bits  = msg->data[1];
			g_imd_state.iso_state_raw = u16_be(&msg->data[2]);
			break;
		}

		// -------------------------------------------------
		// Isolation resistances (E1):
		// Rp: bytes [2..3]
		// Rn: bytes [5..6]
		// -------------------------------------------------
		case uv_request_mux_isolation_resistances: {
			if (msg->dlc < 7) break;
			g_imd_state.status_bits = msg->data[1];
			g_imd_state.rp_raw      = u16_be(&msg->data[2]);
			g_imd_state.rn_raw      = u16_be(&msg->data[5]);
			break;
		}

		// -------------------------------------------------
		// Isolation capacitances (E2):
		// Cp: bytes [2..3]
		// Cn: bytes [5..6]
		// -------------------------------------------------
		case uv_request_mux_isolation_capacitances: {
			if (msg->dlc < 7) break;
			g_imd_state.status_bits = msg->data[1];
			g_imd_state.cp_nf       = u16_be(&msg->data[2]);
			g_imd_state.cn_nf       = u16_be(&msg->data[5]);
			break;
		}

		// -------------------------------------------------
		// GLV battery voltage (E4): value at [2..3]
		// -------------------------------------------------
		case uv_request_mux_battery_voltage: {
			if (msg->dlc < 4) break;
			g_imd_state.status_bits = msg->data[1];
			g_imd_state.glv_raw     = u16_be(&msg->data[2]);
			break;
		}

		// -------------------------------------------------
		// Error flags (E5): value at [2..3]
		// -------------------------------------------------
		case uv_request_mux_Error_flags: {
			if (msg->dlc < 4) break;
			g_imd_state.status_bits    = msg->data[1];
			g_imd_state.error_flags_raw = u16_be(&msg->data[2]);
			break;
		}

		// -------------------------------------------------
		// Temperature (0x80): value at [2..3] (confirm in datasheet)
		// -------------------------------------------------
		case uv_request_mux_Temperature: {
			if (msg->dlc < 4) break;
			g_imd_state.status_bits = msg->data[1];
			g_imd_state.temp_raw    = u16_be(&msg->data[2]);
			break;
		}

		// Check safety touch -quazi from byrons computer
		case uv_request_mux_safety_touch_current: {
			if (msg->dlc < 4) break;
			g_imd_state.safety_touch_current = msg->data[1];
			g_imd_state.safety_touch_current = u16_be(&msg->data[2]);
			break;
		}

		default:
			// unhandled mux — ignore
			break;
	}

	// CRITICAL: tells XDevMon / init waiters that IMD responded
	externalDeviceRxHandler(IMD);
}


/* =========================================================
 * init task (like BMS_Init but with ping check)
 * ========================================================= */

void initIMD(void* args) {
	uv_init_task_args* params = (uv_init_task_args*) args;

	// small delay like the BMS does (optional)
	//osDelay(200);

	uv_init_task_response resp;
	memset(&resp, 0, sizeof(resp));
	resp.device = IMD;
	resp.status = UV_ERROR;
	resp.errmsg = "IMD init fail";
	resp.nchar  = 12;

	if (!params || !params->init_info_queue) {
		vTaskDelete(NULL);
	}

	// clear state
	memset((void*)&g_imd_state, 0, sizeof(g_imd_state));

	// register rx handler first (so the “ping” can be received)
	insertCANMessageHandler(IMD_CAN_ID_Rx, IMD_CanRxHandler, IMD_CAN_BUS);

	// register with xdevmon + add poll list
	if (IMD_RegisterWithXDevMon() != UV_OK) {
		resp.errmsg = "IMD xdev reg";
		resp.nchar  = 12;
		goto done;
	}

	// send one immediate ping (don’t wait for next poll tick)
	IMD_SendRequest(Serial_number_0);

	// wait for response (externalDeviceRxHandler(IMD) will release semaphore)
	if (uvWaitOnExternalDevice(IMD, pdMS_TO_TICKS(300)) != UV_OK) {
		resp.errmsg = "IMD no resp";
		resp.nchar  = 11;
		goto done;
	}

#if IMD_ENABLE_SERIAL_CHECK
	// basic “did we talk to the right device” check
	if (!(g_imd_state.serial0_valid && (g_imd_state.serial0_word == IMD_EXPECTED_SERIAL0))) {
		resp.errmsg = "IMD serial bad";
		resp.nchar  = 14;
		goto done;
	}
#endif

	resp.status = UV_OK;
	resp.errmsg = NULL;
	resp.nchar  = 0;

done:
	(void)xQueueSendToBack(params->init_info_queue, &resp, 100);

	// same style as BMS: init task suspends itself
	vTaskSuspend(params->meta_task_handle);
}


/* =========================================================
 * Getter functions (simple + safe for other modules)
 * ========================================================= */

// NOTE: These return RAW values. Once you confirm the datasheet scaling,
// we can make these return real units (kOhm, nF, V, etc).

uint8_t IMD_IsOnline(void) {
	return g_imd_state.online;
}

uint8_t IMD_GetSerial0Valid(void) {
	return g_imd_state.serial0_valid;
}

uint32_t IMD_GetSerial0Word(void) {
	return g_imd_state.serial0_word;
}

uint8_t IMD_GetStatusBits(void) {
	return g_imd_state.status_bits;
}

// “certain value”: isolation resistance (Rp/Rn)
uint16_t IMD_GetRpRaw(void) {
	return g_imd_state.rp_raw;
}

uint16_t IMD_GetRnRaw(void) {
	return g_imd_state.rn_raw;
}

uint16_t IMD_GetErrorFlagsRaw(void) {
	return g_imd_state.error_flags_raw;
}

uint16_t IMD_GetSafetyTouchCurrent(void){
	return g_imd_state.safety_touch_current;
}
