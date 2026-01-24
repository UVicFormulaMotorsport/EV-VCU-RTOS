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
#define IMD_CAN_ID_Tx  0x18FF50E5U   // REPALCE

#ifndef IMD_CAN_ID_Rx
#define IMD_CAN_ID_Rx  0x18FF50E6U   // REPLACE
#endif

// this is  IMD request code for serial
#ifndef Serial_number_0
#define Serial_number_0  0xA0        // REPLACE
#endif

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

    // Add ONE poll message: serial0 request (acts like a ping)
    uv_CAN_msg ping;
    memset(&ping, 0, sizeof(ping));
    ping.msg_id  = IMD_CAN_ID_Tx;
    ping.dlc     = 1;
    ping.flags   = 0;        // TODO set EXT flag if needed
    ping.data[0] = Serial_number_0;

    if (uvAddPollMsgToXdev(IMD, &ping) != UV_OK) {
        return UV_ERROR;
    }

    return UV_OK;
}


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
