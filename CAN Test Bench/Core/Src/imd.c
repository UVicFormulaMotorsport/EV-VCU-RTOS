// This where the code to handle IMD errors and such will go
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


//TODO: set these the values as coressponding

#ifndef IMD_CAN_ID_Tx
#define IMD_CAN_ID_Tx  0xA100101
// Request from host is structured as:
// 		0xA100101 + Operand Bit

#ifndef IMD_CAN_ID_Rx
#define IMD_CAN_ID_Rx  0xA100100
#endif

// this is  IMD request code for serial
#ifndef RequestMUX_serial_number_0
#define RequestMUX_serial_number_0  0x08
#endif

//the data length code im seeing for every MUX in the ref manual is 3
// this is a result of id + 1 byte operator (read write etc) + 2 bytes data
#ifndef standard_dlc
#define standard_dlc 3

// These are all the valid Request_mux parameters we want to consistently poll
#ifndef RequestMUX_isolation_state
#define RequestMUX_isolation_state 0xE0

#ifndef RequestMUX_isolation_resistances
#define RequestMUX_isolation_resistances 0xE1

#ifndef RequestMUX_isolation_capacitances
#define RequestMUX_isolation_capacitances 0xE2

#ifndef RequestMUX_battery_voltage_vb
#define RequestMUX_battery_voltage_vb 0xE4

#ifndef RequestMUX_error_flags
#define RequestMUX_error_flags 0xE5

#ifndef RequestMUX_dynamic_iso_state
#define RequestMUX_dynamic_iso_state 0xE7


// single-word expected serial chunk for a “simple check”
static const uint32_t IMD_EXPECTED_SERIAL0 = 0xB8DD9AF9U;


// Local state

static SemaphoreHandle_t imd_mutex = NULL;
static uint8_t imd_serial0_valid = 0;
static uint32_t imd_serial0_word = 0;

static uv_status IMD_EnsureMutex(void) {
    if (imd_mutex == NULL) {
        imd_mutex = xSemaphoreCreateMutex();
        if (imd_mutex == NULL) return UV_ERROR;
    }
    return UV_OK;
}

static inline void IMD_Lock(void)   { xSemaphoreTake(imd_mutex, portMAX_DELAY); }
static inline void IMD_Unlock(void) { xSemaphoreGive(imd_mutex); }


// Building a 1-byte request frame
static void IMD_SendRequest(uint8_t code) {
    uv_CAN_msg msg;
    memset(&msg, 0, sizeof(msg));

    msg.msg_id = IMD_CAN_ID_Tx;
    msg.dlc    = 1;
    msg.flags  = 0;          // need to integrate with can.c
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
    uv_CAN_msg poll_isolation_state;
    memset(&poll_isolation_state, 0, sizeof(poll_isolation_state));
    poll_isolation_state.msg_id  = IMD_CAN_ID_Tx;
    poll_isolation_state.dlc     = standard_dlc;
    poll_isolation_state.flags   = UV_CAN_EXTENDED_ID;
    poll_isolation_state.data[0] = RequestMUX_isolation_state;

    uv_CAN_msg poll_isolation_resistance;
	memset(&poll_isolation_resistance, 0, sizeof(poll_isolation_resistance));
	poll_isolation_resistance.msg_id  = IMD_CAN_ID_Tx;
	poll_isolation_resistance.dlc     = standard_dlc;
	poll_isolation_resistance.flags   = UV_CAN_EXTENDED_ID;
	poll_isolation_resistance.data[0] = RequestMUX_isolation_resistance;

	uv_CAN_msg poll_isolation_capacitances;
	memset(&poll_isolation_capacitances, 0, sizeof(poll_isolation_capacitances));
	poll_isolation_capacitances.msg_id  = IMD_CAN_ID_Tx;
	poll_isolation_capacitances.dlc     = standard_dlc;
	poll_isolation_capacitances.flags   = UV_CAN_EXTENDED_ID;
	poll_isolation_capacitances.data[0] = RequestMUX_isolation_capacitance;

	uv_CAN_msg poll_dynamic_iso_state;
	memset(&poll_dynamic_iso_state, 0, sizeof(poll_dynamic_iso_state));
	poll_poll_dynamic_iso_state.msg_id  = IMD_CAN_ID_Tx;
	poll_dynamic_iso_state.dlc     = standard_dlc;
	poll_dynamic_iso_state.flags   = UV_CAN_EXTENDED_ID;
	poll_dynamic_iso_state.data[0] = RequestMUX_isolation_capacitance;

	//battery voltage MUX
	uv_CAN_msg poll_battery_voltage_vb;
	memset(&poll_battery_voltage_vb, 0, sizeof(poll_battery_voltage_vb));
	poll_battery_voltage_vb.msg_id  = IMD_CAN_ID_Tx;
	poll_battery_voltage_vb.dlc     = standard_dlc;
	poll_battery_voltage_vb.flags   = UV_CAN_EXTENDED_ID;
	poll_battery_voltage_vb.data[0] = RequestMUX_battery_voltage_vb;

	//error flag MUX
	uv_CAN_msg poll_error_flags;
	memset(&poll_error_flags, 0, sizeof(poll_error_flags));
	poll_error_flags.msg_id  = IMD_CAN_ID_Tx;
	poll_error_flags.dlc     = standard_dlc;
	poll_error_flags.flags   = UV_CAN_EXTENDED_ID;
	poll_error_flags.data[0] = RequestMUX_error_flags;


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

    if (uvAddPollMsgToXdev(IMD, &poll_dynamic_iso_state) != UV_OK) {
            return UV_ERROR;
     }

    if (uvAddPollMsgToXdev(IMD, &poll_error_flags) != UV_OK) {
            return UV_ERROR;
     }

    if (uvAddPollMsgToXdev(IMD, &poll_batter_voltage_vb) != UV_OK) {
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
    imd_serial0_valid = 0;
    imd_serial0_word  = 0;
    IMD_Unlock();

    // register with XDevMon and install ping poll message
    if (IMD_RegisterWithXDevMon() != UV_OK) {
        resp.errmsg = "IMD xdev reg";
        resp.nchar  = 12;
        goto done;
    }

    // optional: send one immediate request (so init doesn't wait for next poll tick)
    IMD_SendRequest(Serial_number_0);

    // wait for a response (requires CAN RX handler to call externalDeviceRxHandler(IMD))
    if (uvWaitOnExternalDevice(IMD, pdMS_TO_TICKS(300)) != UV_OK) {
        resp.errmsg = "IMD no resp";
        resp.nchar  = 11;
        goto done;
    }

    // simple “serial0” validation
    IMD_Lock();
    uint8_t ok = imd_serial0_valid && (imd_serial0_word == IMD_EXPECTED_SERIAL0);
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
    vTaskDelete(NULL);
}


// CAN RX hook
// Call this from can.c when IMD_CAN_ID_Rx arrives
void IMD_CanRxHandler(uint32_t can_id, const uint8_t data[8], uint8_t dlc) {
    if (can_id != IMD_CAN_ID_Rx) return;
    if (dlc < 5) return; // need at least code + 4 bytes

    // only handle our "ping" response
    // we need to add functionality here to handle all the different incoming messages
    if (data[0] != Serial_number_0) return;

    // imd.c (original) used [1..4] as the 32-bit chunk (keep consistent)
    uint32_t word =
        ((uint32_t)data[1] << 24) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] <<  8) |
        ((uint32_t)data[4] <<  0);

    if (IMD_EnsureMutex() == UV_OK) {
        IMD_Lock();
        imd_serial0_word  = word;
        imd_serial0_valid = 1;
        IMD_Unlock();
    }

    // CRITICAL: tells XDevMon / init waiters that IMD responded IMPORTNAT
    externalDeviceRxHandler(IMD);
}
