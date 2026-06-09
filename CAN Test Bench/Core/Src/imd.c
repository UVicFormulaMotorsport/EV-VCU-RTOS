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

// TODO: replace these CAN IDs using the IMD datasheet - Rachan done

#define IMD_CAN_ID_REQUEST   0x22U
#define IMD_CAN_ID_RESPONSE  0x23U
#define IMD_CAN_ID_INFO      0x37U

#define IMD_CMD_DUMMY        0x00U
#define IMD_CMD_GET_R_ISO    0x35U
#define IMD_CMD_GET_STATUS   0x37U
#define IMD_CMD_GET_HV1      0x36U
#define IMD_CMD_GET_HV2      0x3AU
#define IMD_CMD_SET_HV_RELAY 0xD2U

// TODO: pick the real bus the IMD is wired to
#ifndef IMD_CAN_BUS
#define IMD_CAN_BUS CAN_BUS_2
#endif

// Requests are 5  bytes: CMD + DataWord1 + DataWord2
#define IMD_REQ_DLC 5

/* =========================================================
 * Optional: simple "ping" serial check
 * ========================================================= */

// If you don't want the init to fail based on serial, set this to 0
#ifndef IMD_ENABLE_SERIAL_CHECK
#define IMD_ENABLE_SERIAL_CHECK 0
#endif

// Replace it with real serial chunk OR disable serial check.- Rachan Disabled for now 
// static const uint32_t IMD_EXPECTED_SERIAL0 = 0xB8DD9AF9U;

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

	// parsed raw values (exact units depend on datasheet) updated - Rachan
	uint16_t r_iso_kohm;       // Bender D_IMC_R_ISO insulation resistance
	uint16_t imc_status;       // Bender D_IMC_STATUS
	uint16_t imc_status_ext;   // Bender D_IMC_STATUS_EXT
	uint16_t vifc_status;      // Bender D_VIFC_STATUS
	uint16_t hv1_voltage;      // Bender D_IMC_HV_1
	uint16_t hv2_voltage;      // Bender D_IMC_HV_2
									//i will fix this later -quazi from byrons computer

	// ping / identity check
	uint8_t  serial0_valid;
	uint32_t serial0_word;
} imd_state_t;

static volatile imd_state_t g_imd_state = {0};


/* =========================================================
 * Helpers
 * ========================================================= */
// if we are not using this can we remove?
static inline uint16_t u16_be(const uint8_t *p) {
	return (uint16_t)((p[0] << 8) | p[1]);
}


// Sending a Bender IMD request frame: CMD + DataWord1 + DataWord2
static void IMD_SendRequest(uint8_t cmd, uint16_t data_word1, uint16_t data_word2) {
	uv_CAN_msg msg;
	memset(&msg, 0, sizeof(msg));

	msg.msg_id = IMD_CAN_ID_REQUEST;
	msg.dlc    = IMD_REQ_DLC;
	msg.flags  = IMD_CAN_BUS;   // standard CAN ID

	msg.data[0] = cmd;
	msg.data[1] = (uint8_t)(data_word1 & 0xFF);
	msg.data[2] = (uint8_t)((data_word1 >> 8) & 0xFF);
	msg.data[3] = (uint8_t)(data_word2 & 0xFF);
	msg.data[4] = (uint8_t)((data_word2 >> 8) & 0xFF);

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

	uv_CAN_msg poll;
	memset(&poll, 0, sizeof(poll));

	poll.msg_id  = IMD_CAN_ID_REQUEST;
	poll.dlc     = IMD_REQ_DLC;
	poll.flags   = IMD_CAN_BUS;   // standard CAN, not extended

	poll.data[0] = IMD_CMD_GET_R_ISO;
	poll.data[1] = 0;
	poll.data[2] = 0;
	poll.data[3] = 0;
	poll.data[4] = 0;

	if (uvAddPollMsgToXdev(IMD, &poll) != UV_OK) {
		return UV_ERROR;
	}

	return UV_OK;
}


/* =========================================================
 * CAN RX handler (BMS style)
 * ========================================================= */
void IMD_CanRxHandler(uv_CAN_msg* msg) {
	if (!msg) return;

	if (msg->msg_id == IMD_CAN_ID_INFO) {
		if (msg->dlc < 6) return;

		g_imd_state.online = 1;

		g_imd_state.r_iso_kohm  = (uint16_t)(msg->data[0] | (msg->data[1] << 8));
		g_imd_state.imc_status  = (uint16_t)(msg->data[2] | (msg->data[3] << 8));
		g_imd_state.vifc_status = (uint16_t)(msg->data[4] | (msg->data[5] << 8));

		g_imd_state.status_bits = (uint8_t)(g_imd_state.imc_status & 0xFF);

		externalDeviceRxHandler(IMD);
		return;
	}

	if (msg->msg_id == IMD_CAN_ID_RESPONSE) {
		if (msg->dlc < 5) return;

		g_imd_state.online = 1;

		uint8_t cmd = msg->data[0];

		switch (cmd) {
			case IMD_CMD_GET_R_ISO:
				g_imd_state.r_iso_kohm = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
				break;

			case IMD_CMD_GET_STATUS:
				g_imd_state.imc_status     = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
				g_imd_state.imc_status_ext = (uint16_t)(msg->data[3] | (msg->data[4] << 8));
				g_imd_state.status_bits    = (uint8_t)(g_imd_state.imc_status & 0xFF);
				break;

			case IMD_CMD_GET_HV1:
				g_imd_state.hv1_voltage = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
				break;

			case IMD_CMD_GET_HV2:
				g_imd_state.hv2_voltage = (uint16_t)(msg->data[1] | (msg->data[2] << 8));
				break;

			default:
				break;
		}

		externalDeviceRxHandler(IMD);
	}
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
	insertCANMessageHandler(IMD_CAN_ID_RESPONSE, IMD_CanRxHandler, IMD_CAN_BUS);
	insertCANMessageHandler(IMD_CAN_ID_INFO, IMD_CanRxHandler, IMD_CAN_BUS);

	// register with xdevmon + add poll list
	if (IMD_RegisterWithXDevMon() != UV_OK) {
		resp.errmsg = "IMD xdev reg";
		resp.nchar  = 12;
		goto done;
	}

	// send one immediate ping (don’t wait for next poll tick)
	IMD_SendRequest(IMD_CMD_DUMMY, 0, 0);
	// wait for response (externalDeviceRxHandler(IMD) will release semaphore)
	if (uvWaitOnExternalDevice(IMD, pdMS_TO_TICKS(300)) != UV_OK) { // change to 1000 if to slow this can cause issue testing if it slow to respond
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

uint16_t IMD_GetRisoKohm(void) {
	return g_imd_state.r_iso_kohm;
}

uint16_t IMD_GetIMCStatus(void) {
	return g_imd_state.imc_status;
}

uint16_t IMD_GetVIFCStatus(void) {
	return g_imd_state.vifc_status;
}

/* Backwards-compatible old getter names */
uint16_t IMD_GetRpRaw(void) {
	return g_imd_state.r_iso_kohm;
}

uint16_t IMD_GetRnRaw(void) {
	return g_imd_state.r_iso_kohm;
}

uint16_t IMD_GetErrorFlagsRaw(void) {
	return g_imd_state.imc_status;
}

uint16_t IMD_GetSafetyTouchCurrent(void) {
	return 0;
}