/*
 * odometer.h
 *
 *  Created on: Nov 7, 2024
 *      Author: byo10
 */

#ifndef INC_ODOMETER_H_
#define INC_ODOMETER_H_

#include "uvfr_utils.h"

typedef struct uv_persistant_data_frame{
	uint64_t total_vehicle_uptime;
	uint64_t total_time_driving;
	uint64_t total_distance_cm;
}uv_persistant_data_frame;

/* Snapshot of all four wheel speeds sent through the IPC queue. */
typedef struct {
	float wheel_speed[4];   /* m/s, one per wheel */
	TickType_t timestamp;   /* xTaskGetTickCount() at time of measurement */
} WheelSpeedData;

/* Queue handle - depth 1, always holds the latest reading.
 * Created by initOdometer(); written by WheelSpeed_UpdateAll(). */
extern QueueHandle_t wheel_speed_queue;

uv_status initOdometer(void* args);

void odometerTask(void* args);


#endif /* INC_ODOMETER_H_ */
