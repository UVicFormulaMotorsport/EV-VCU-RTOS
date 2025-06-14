/*
 * odometer.h
 *
 *  Created on: Nov 7, 2024
 *      Author: byo10
 */

#ifndef INC_ODOMETER_H_
#define INC_ODOMETER_H_

#include"uvfr_utils.h"

typedef struct uv_persistant_data_frame{
	uint64_t total_vehicle_uptime;
	uint64_t total_time_driving;
	uint64_t total_distance_cm;
}uv_persistant_data_frame;

uv_status initOdometer(void* args);

void odometerTask(void* args);


#endif /* INC_ODOMETER_H_ */
