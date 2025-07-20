// Code to make human readable CAN messages for the device



#ifndef _BMS_H_
#define _BMS_H_

#include "main.h"
#include "uvfr_utils.h"

#define DEFAULT_BMS_CAN_TIMEOUT ((uv_timespan_ms)200)

typedef struct bms_settings_t{ //TODO Needs populating
	uint32_t BMS_CAN_timeout;
	uint16_t max_cell_temp;
	uint16_t min_cell_temp;
	uint16_t min_soc;
	uint16_t min_cell_voltage;
	uint16_t max_cell_voltage;
	uint16_t max_pack_voltage;
	uint16_t min_pack_voltage;
	uint16_t max_variance_between_cells;

}bms_settings_t;

void BMS_Init(void* args);

#endif



