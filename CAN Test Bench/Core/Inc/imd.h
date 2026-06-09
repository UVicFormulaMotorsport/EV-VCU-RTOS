// Code to make human readable CAN messages for the IMD

#ifndef __IMD_H__
#define __IMD_H__

#include "main.h"
#include "uvfr_utils.h"
#include <stdint.h>

/* Bender iso165C / iso165C-1 settings */
typedef struct uv_imd_settings {
	uint16_t min_isolation_resistances;        // kOhm threshold
	uint16_t expected_isolation_capacitances;  // unused for Bender right now
	uint16_t max_imd_temperature;              // unused for Bender right now
} uv_imd_settings;

/* Temporary generic status masks until D_IMC_STATUS is fully mapped */
enum imd_status_bits {
	IMD_STATUS_BIT_0 = 0x0001,
	IMD_STATUS_BIT_1 = 0x0002,
	IMD_STATUS_BIT_2 = 0x0004,
	IMD_STATUS_BIT_3 = 0x0008,
	IMD_STATUS_BIT_4 = 0x0010,
	IMD_STATUS_BIT_5 = 0x0020,
	IMD_STATUS_BIT_6 = 0x0040,
	IMD_STATUS_BIT_7 = 0x0080,
};

/* Function declarations */
uint8_t  IMD_IsOnline(void);
uint8_t  IMD_GetStatusBits(void);

/* Bender-specific getters */
uint16_t IMD_GetRisoKohm(void);
uint16_t IMD_GetIMCStatus(void);
uint16_t IMD_GetVIFCStatus(void);

/* Backwards-compatible getters for old code paths */
uint16_t IMD_GetRpRaw(void);
uint16_t IMD_GetRnRaw(void);
uint16_t IMD_GetErrorFlagsRaw(void);
uint16_t IMD_GetSafetyTouchCurrent(void);

uint8_t  IMD_GetSerial0Valid(void);
uint32_t IMD_GetSerial0Word(void);

void IMD_CanRxHandler(uv_CAN_msg* msg);
void initIMD(void* args);

#endif