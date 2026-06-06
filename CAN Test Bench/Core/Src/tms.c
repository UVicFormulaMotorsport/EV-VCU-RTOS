/*
 * tms.c
 *
 * VCU-side Thermal Management System (TMS) tap. See tms.h for the full rundown.
 *
 * In short: the TMS broadcasts battery-pack temperatures as a J1939 thermistor
 * frame (0x1839F380) aimed at the BMS, but mirrored onto both buses. The BMS
 * handles its own copy; we just listen in on the dash bus, sanity-check the
 * frame, and drop the numbers into g_tms_state so the dash/DAQ can display them.
 * Nothing here is safety-critical -- it is display/telemetry only.
 */

#define __UV_FILENAME__ "tms.c"

#include "main.h"
#include "tms.h"
#include "can.h"
#include "uvfr_utils.h"
#include "uvfr_external_devices.h"
#include <stdint.h>

volatile tms_state_t g_tms_state = {0};

/** @brief CAN RX callback for the TMS thermistor-data frame (0x1839F380).
 *
 * Validates the checksum, then copies the pack temperatures into g_tms_state.
 * A corrupt frame is flagged and dropped so the dash keeps showing the last
 * good reading instead of garbage.
 */
void TMS_CANRxHandler(uv_CAN_msg* msg){
	if(msg == NULL){
		return;
	}

	/* Checksum: byte7 = low 8 bits of (sum of bytes 0..6 + 0x39 + DLC(8)).
	 * This mirrors can_tms_send_module_data() on the TMS side exactly. */
	uint16_t sum = (uint16_t)msg->data[0] + msg->data[1] + msg->data[2]
	             + msg->data[3] + msg->data[4] + msg->data[5] + msg->data[6]
	             + 0x39u + 0x08u;

	if((uint8_t)(sum & 0xFFu) != msg->data[7]){
		g_tms_state.msg_corrupt = 1; // keep the last good values, drop this frame
		return;
	}

	g_tms_state.msg_corrupt = 0;
	g_tms_state.low_c       = (int8_t)msg->data[1];
	g_tms_state.high_c      = (int8_t)msg->data[2];
	g_tms_state.avg_c       = (int8_t)msg->data[3];
	g_tms_state.therm_count = msg->data[4];
	g_tms_state.high_id     = msg->data[5];
	g_tms_state.low_id      = msg->data[6];

	/* Let XDEVMON know the TMS is alive (freshness tracking only). */
	externalDeviceRxHandler(TMS);
}

/** @brief Init task: installs the RX handler and registers the TMS with XDEVMON.
 *
 * Mirrors BMS_Init. The TMS is a display-only tap, so it is registered WITHOUT
 * XDEV_DEVICE_EXPECTED -- a missing TMS must never fault the vehicle or block
 * boot. Likewise, a failed xdev registration is non-fatal here.
 */
void TMS_Init(void* args){
	uv_init_task_args* params = (uv_init_task_args*) args;

	vTaskDelay(20);

	uv_init_task_response response = {UV_OK, TMS, 0, NULL};

	insertCANMessageHandler(TMS_THERM_DATA_CAN_ID, TMS_CANRxHandler, TMS_CAN_BUS);

	if(uvRegisterExternalDevice(TMS, 200, XDEV_CHECK_TIMEOUT_BIT, "TMS") != UV_OK){
		/* Display-only device: carry on regardless. The RX handler is already
		 * installed, so temps will still flow even without xdev monitoring. */
	}

	if(xQueueSendToBack(params->init_info_queue, &response, 100) != pdPASS){
		//OOPS
	}

	//Kill yourself
	vTaskSuspend(params->meta_task_handle);
}
