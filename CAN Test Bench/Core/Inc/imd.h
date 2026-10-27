// Code to make human readable CAN messages for the device

// This file will define message data as human readable stuff
#ifndef __IMD_H__
#define __IMD_H__

#include "main.h"
#include "uvfr_utils.h"
#include <stdint.h>

// ---------------------------------------------------------------
// Bender ISOMETER iso175 - Status: Warnings_and_Alarms
// Source: iso175 operating manual, iso175_D00415_03_M_XXEN (07.2025)
// The manual has NO single table for these bits. Each bit is mentioned in the
// section listed next to it (page numbers = printed page numbers in the manual).

// Bender-Standard default IDs from iso175 manual chapter 4.11.2 (11-bit standard IDs)
enum imd_can_ids{
	IMD_CAN_ID_response        = 0x22, 
	IMD_CAN_ID_request         = 0x23,
	IMD_CAN_ID_Info_General    = 0x37, // broadcast every 100 ms by default
	IMD_CAN_ID_Info_IsoDetaul  = 0x38, // off by default
	IMD_CAN_ID_Info_Temperature = 0x39  // off by default
	


}


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
