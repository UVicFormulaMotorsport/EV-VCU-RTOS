// Code to make human readable CAN messages for the device

// This file will define message data as human readable stuff
#ifndef __IMD_H__
#define __IMD_H__

#include "main.h"
#include "uvfr_utils.h"
#include <stdint.h>



// Bender-Standard default IDs from iso175 manual chapter 4.11.2 (11-bit standard IDs)
enum imd_can_ids{
	IMD_CAN_ID_Request        = 0x22,
	IMD_CAN_ID_Response       = 0x23,
	IMD_CAN_ID_Info_General   = 0x37, // sent every 100 ms by default (sec 4.11.1 "Cycle time periodic messages", p.22)
	IMD_CAN_ID_Info_IsoDetail = 0x38, // off by default (sec 4.11.1, p.22)
	IMD_CAN_ID_Info_Voltage   = 0x39, // off by default. NOTE: sec 10.1 table, p.32 lists this as 0x38 - looks like a
	                                  // typo since 0x38 is IsoDetail (sec 4.11.2 says 0x39). Confirm on the bus
	IMD_CAN_ID_INFO_ITSystem  = 0x3A, // off by default (sec 4.11.1, p.22)
};


//
// NOT from the manual - assumptions to confirm with the Bender iso175 CAN spec / DBC:
//  - Hex values are just "bit N" converted to (1 << N)
//  - 16-bit width is assumed: the highest bit the manual mentions is bit 10
//  - Bit 0 = least significant bit is assumed; byte position/endianness in the CAN frame is unknown
//
// Any bit set -> OKHS status output goes low (sec 4.10.3 "Status output OKHS", p.21)
enum imd_warnings_alarms{
	WA_Device_error     = 0x0001, // bit 0  - sec 4.10.1 "Device error", p.19
	WA_HV_pos_conn_fail = 0x0002, // bit 1  - sec 4.4.2 "Offline self test", p.15 ("Bit 1: true = HV+ connection failure")
	WA_HV_neg_conn_fail = 0x0004, // bit 2  - sec 4.4.2 "Offline self test", p.15 ("Bit 2: true = HV- connection failure")
	WA_Earth_conn_fail  = 0x0008, // bit 3  - sec 4.4.1 "Online self test", p.14 + sec 4.10.2 "Chassis-ground connection faults", p.20
	                              //          WARNING: sec 8.3, p.28 calls the chassis-ground fault "bit 4", which contradicts
	                              //          sec 4.4.1 and sec 4.10.2. Verify on the bench (disconnect LV pin 3 from chassis)
	WA_Iso_alarm        = 0x0010, // bit 4  - sec 4.10.2 "Insulation resistance fault (alarm)", p.20
	                              //          R_iso < Threshold_Error (default 100 kOhm, sec 10.1 "Factory settings", p.32)
	WA_Iso_warning      = 0x0020, // bit 5  - sec 4.10.2 "Insulation resistance warning (prewarning)", p.20
	                              //          R_iso < Threshold_Warning (default 500 kOhm, sec 10.1 "Factory settings", p.32)
	WA_Iso_outdated     = 0x0040, // bit 6  - sec 4.10.2 "Insulation resistance measured value no longer up to date", p.20
	WA_Unbalance        = 0x0080, // bit 7  - sec 4.10.2 "HV voltage unbalance faults", p.21
	WA_Undervoltage     = 0x0100, // bit 8  - sec 4.10.2 "Undervoltage error", p.21
	WA_Unsafe_to_start  = 0x0200, // bit 9  - sec 4.3.4 "Fast start measurement", p.13 + sec 4.10.2 "System condition unstable", p.21
	WA_Earthlift_open   = 0x0400, // bit 10 - sec 4.10.2 "'Earth' disconnector open", p.21
};

enum imd_r_iso_status{
	Riso_estimate = 0xFC, // sec 4.3.4 "Fast start measurement", p.13 ("estimated insulation value during startup")
	Riso_first_measured  = 0xFD, // ONLY appears in the tolerance table, sec 10.2 "Tabular data", p.34 - meaning is NOT
	                      // described in the manual. Name "interim" is a guess; confirm with the CAN spec
	Riso_normal   = 0xFE, // sec 4.3.1, p.12 + sec 4.4.2, p.15 ("normal insulation measurement")

	Riso_SNV   = 0xFF    // sec 4.3.1, p.12 + sec 4.4.2, p.15 ("no insulation measurement possible")
};
// ---------------------------------------------------------------
// Function declarations
uint8_t  IMD_IsOnline(void);
uint8_t  IMD_GetStatusBits(void);

uint16_t IMD_GetRpRaw(void);
uint16_t IMD_GetRnRaw(void);
uint16_t IMD_GetErrorFlagsRaw(void);

uint8_t  IMD_GetSerial0Valid(void);
uint32_t IMD_GetSerial0Word(void);
uint16_t IMD_GetSafetyTouchCurrent(void);



// High resolution measurements


//Incoming can message reciever handler
void IMD_CanRxHandler(uv_CAN_msg* msg);

void initIMD(void* args);



#endif
