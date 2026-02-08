/*
 * uvfr_vehicle_logger.h
 *
 *  Created on: Jul 13, 2025
 *      Author: rachangrewal
 *
 *  Header for the vehicle logger system.
 *  Tracks and stores faults in a small log buffer,
 *  and uses a bitfield to flag which types of faults have occurred.
 */

#ifndef INC_UVFR_VEHICLE_LOGGER_H_
#define INC_UVFR_VEHICLE_LOGGER_H_

#include <stdint.h>
#include "uvfr_utils.h"
#include "uvfr_state_engine.h"  // for uv_task_info, uv_task_status, etc.


//typedef uint8_t bool;

// types of faults we can log, this might be over kill

typedef enum {
    FAULT_PANIC = 0,
    FAULT_TASK_FAIL,
    FAULT_TASK_LATE,
    FAULT_STATE_ILLEGAL,
    FAULT_SCD_TIMEOUT,
    FAULT_SENSOR_ERROR,
    FAULT_STATE_TRANSITION,
    FAULT_MANUAL,
    FAULT_FLASH_ERROR,
    FAULT_SETTINGS_CORRUPT,
    FAULT_CAN_MSG_FAIL,
    // Add more if needed?
} fault_event_type_e;


// log entry format

typedef struct {
    fault_event_type_e type;     // what type  of error (enum)
    const char* task_name;       // name of the task (if any)
    const char* msg;             // error message
    uint32_t timestamp;          // time of event (xTaskGetTickCount)
    const char* file;            // source file
    int line;                    // line number
    const char* func;
    uint8_t is_panic;            // was this triggered by a panic?
    uv_task_id task_id;          // task ID (or 0xFF if none)
    uv_vehicle_state vehicle_state; // vehicle state at time of error
    uv_task_status task_state;   // what state the task was in (if known)
} vehicle_log_entry;



// public logger functions

// logs a fault or panic
void logVehicleFault(fault_event_type_e type,
                     uv_task_info* task,
                     const char* msg,
                     const char* file,
                     int line,
                     const char* func,
                     bool is_panic);

// returns number of log entries stored
uint8_t getLogCount(void);

// returns a pointer to a log entry by index
const vehicle_log_entry* getLogEntry(uint8_t index);

// prints all logs to UART
void dumpLogsToUART(void);

// returns current fault bitfield
uint32_t getSystemFaultFlags(void);

// clears fault bits (bitmask of flags to clear)
void clearSystemFaultFlags(uint32_t flags_to_clear);

// Sends all stored log entries as formatted CAN messages.
// Intended for post-panic diagnostics.
void flushLogsToCAN(void);


#endif // UVFR_VEHICLE_LOGGER_H
