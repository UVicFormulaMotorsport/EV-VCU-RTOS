/*
 * uvfr_external_devices.h
 *
 *  Created on: Dec 6, 2025
 *      Author: byo10
 */

#ifndef INC_UVFR_EXTERNAL_DEVICES_H_
#define INC_UVFR_EXTERNAL_DEVICES_H_

typedef enum uv_xdev_status xdev_status;

#include "uvfr_utils.h"

uv_status uvSetupXdevs();

uv_status uvAddPollMsgToXdev(uint8_t xdev_id, uv_CAN_msg* msg);

uv_status uvStartExDevMonitoring();

uv_status uvRegisterExternalDevice(uint8_t device_id, TickType_t period, uint16_t xd_flags, char* name);

void externalDeviceRxHandler(uint8_t device_id);

uv_status uvWaitOnExternalDevice(uint8_t device_id, TickType_t time_to_wait);

void externalDeviceStatUpdate(uint8_t device_id);

xdev_status getXdevStatus(uint8_t device_id);

void getAllXdevStatus(xdev_status* arr);

#endif /* INC_UVFR_EXTERNAL_DEVICES_H_ */
