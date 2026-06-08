/*
 * tms.h
 *
 * VCU-side handler for the Thermal Management System (TMS).
 *
 * The TMS impersonates an Orion external thermistor expansion module and talks
 * to the BMS over J1939 entirely on its own -- the BMS ingests those frames
 * directly, so the VCU is NOT part of that thermal safety loop and does not need
 * to handle the TMS->BMS path. The TMS happens to publish the same frames on
 * both CAN buses, so VCU-side we simply listen in on the dash bus (CAN2) and
 * stash the pack temperatures in g_tms_state. This is a display/telemetry tap
 * only -- it exists so the temps can be shown on the dash and logged via DAQ.
 *
 * Frame we care about (see EV26-TMS-NEW/TMS_MAIN/Core/Src/can_tms.c):
 *   0x1839F380  Thermistor module -> BMS  @ ~100 ms  (extended ID, DLC=8)
 *     byte0 = module#        byte4 = therm_count (always 6)
 *     byte1 = low_c  (int8)  byte5 = high_id (pack 0..5)
 *     byte2 = high_c (int8)  byte6 = low_id  (pack 0..5)
 *     byte3 = avg_c  (int8)  byte7 = checksum
 *
 * Temperatures are int8 degrees C (1 C / LSB, signed). Each pack value is itself
 * a hottest-cell reading, so low_c/avg_c run warm by design.
 */

#ifndef INC_TMS_H_
#define INC_TMS_H_

#include "main.h"
#include "uvfr_utils.h"
#include <stdint.h>

/* CAN ID of the thermistor-data frame the TMS broadcasts (extended ID). */
#define TMS_THERM_DATA_CAN_ID   0x1839F380u

/* Bus the VCU listens for the TMS on. The dash + DAQ live on CAN2, and the TMS
 * mirrors its frames onto both buses, so we tap it here. */
#define TMS_CAN_BUS             CAN_BUS_2

/* Number of packs the TMS reports (its 6 "thermistors"). */
#define TMS_NUM_PACKS           6u

/* Runtime state (telemetry). Mirrors can_tms_module_data_t on the TMS side. */
typedef struct {
	int8_t  low_c;        // lowest pack reading, signed degrees C
	int8_t  high_c;       // highest pack reading, signed degrees C
	int8_t  avg_c;        // mean of the pack readings, signed degrees C
	uint8_t therm_count;  // number of packs reported (expected 6)
	uint8_t high_id;      // pack index (0..5) reporting high_c
	uint8_t low_id;       // pack index (0..5) reporting low_c
	uint8_t msg_corrupt;  // 1 if the last frame failed its checksum
} tms_state_t;

extern volatile tms_state_t g_tms_state; // volatile: updated from the CAN RX path

/* Registers the CAN RX handler + the (optional) xdev monitor entry. Mirrors
 * BMS_Init. Spawned as an init task from uvfr_utils.c. */
void TMS_Init(void* args);

/* CAN RX callback for TMS_THERM_DATA_CAN_ID. */
void TMS_CANRxHandler(uv_CAN_msg* msg);

#endif /* INC_TMS_H_ */
