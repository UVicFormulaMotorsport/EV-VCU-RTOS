/*
 * uvfr_vehicle_logger.c
 *
 *  Created on: Jul 6, 2025
 *  Author: Rachan Grewal
 *
 *  This file logs vehicle faults, panics, and task errors.
 *  It keeps a short history in memory (circular buffer),
 *  and uses a bitfield to track which errors have ever happened.
 */

#include "uvfr_utils.h"
#include "uvfr_state_engine.h"
#include "uvfr_vehicle_logger.h"

#include "can.h" // for uv_can_transmit()


// Logger Configuration

#define MAX_LOG_ENTRIES 32  // max # of stored
#define LOG_DUMP_CAN_ID 0x666 // CAN ID for diagnostic dump


// Logger Storage
static vehicle_log_entry log_buffer[MAX_LOG_ENTRIES]; // log storage
static uint8_t log_write_index = 0;  // where to write next
static bool has_wrapped = false;     // has buffer wrapped?

static uint32_t system_error_flags = 0; // Bitfield: which faults have occurred?


// Logger APIs


/**
 * returns the current bitfield (1 bit per fault type)
 */
uint32_t getSystemFaultFlags(void) {
    return system_error_flags;
}

/**
 * clears specific fault flags (set bits) from the bitfield
 */
void clearSystemFaultFlags(uint32_t flags_to_clear) {
    system_error_flags &= ~flags_to_clear;
}

/**
 * returns how many logs are stored right now
 */
uint8_t getLogCount(void) {
    return has_wrapped ? MAX_LOG_ENTRIES : log_write_index;
}

/**
 * returns a specific log entry by index
 */
const vehicle_log_entry* getLogEntry(uint8_t index) {
    if (index >= getLogCount()) return NULL;
    return &log_buffer[index];
}

/**
 * prints all stored logs to UART (e.x, for debugging)
 */
void dumpLogsToUART(void) {
    uint8_t count = getLogCount();
    for (uint8_t i = 0; i < count; i++) {
        const vehicle_log_entry* entry = getLogEntry(i);
        if (entry != NULL) {
            printf("[LOG %02d] Time=%lu ticks | Type=%d | Task=%s\n"
                   "         Msg=\"%s\"\n"
                   "         File: %s:%d (%s)\n"
                   "         Panic=%s | State=%d | ID=%d | TaskState=%d\n\n",
                   i,
                   entry->timestamp,
                   entry->type,
                   entry->task_name ? entry->task_name : "None",
                   entry->msg,
                   entry->file,
                   entry->line,
                   entry->func,
                   entry->is_panic ? "YES" : "NO",
                   entry->vehicle_state,
                   entry->task_id,
                   entry->task_state
            );
        }
    }
}

/**
 * adds a new log entry for a fault or error.
 */
void logVehicleFault(fault_event_type_e type,
                     uv_task_info* task,
                     const char* msg,
                     const char* file,
                     int line,
                     const char* func,
                     bool is_panic)
{
    // Mark this fault type as "occurred"
    system_error_flags |= (1U << type);

    // Write into next log slot
    vehicle_log_entry* entry = &log_buffer[log_write_index];

    entry->type = type;
    entry->msg = msg;
    entry->file = file;
    entry->line = line;
    entry->func = func;
    entry->is_panic = is_panic;
    entry->timestamp = xTaskGetTickCount();
    entry->vehicle_state = vehicle_state;

    // Add task info (if task is valid)
    if (task != NULL) {
        entry->task_name = task->task_name;
        entry->task_id = task->task_id;
        entry->task_state = task->task_state;
    } else {
        entry->task_name = "None";
        entry->task_id = 0xFF;
        entry->task_state = UV_TASK_NOT_STARTED;
    }

    // Move to next slot
    log_write_index++;
    if (log_write_index >= MAX_LOG_ENTRIES) {
        log_write_index = 0;
        has_wrapped = true;
    }
}

/**
 * Flush all current log entries out over CAN for diagnostic laptop tools.
 * Each entry is sent as a single compact CAN frame.
 *
 * Frame layout:
 *   ID  = LOG_DUMP_CAN_ID
 *   DLC = 8
 *   Byte 0 : fault type
 *   Byte 1 : timestamp LSB
 *   Byte 2 : timestamp MSB
 *   Byte 3 : task ID
 *   Byte 4 : is_panic flag
 *   Byte 5 : vehicle_state
 *   Byte 6 : reserved (0)
 *   Byte 7 : reserved (0)
 */
void flushLogsToCAN(void)
{
    uint8_t count = getLogCount();

    for (uint8_t i = 0; i < count; i++)
    {
        const vehicle_log_entry* entry = getLogEntry(i);
        if (entry == NULL) continue;

        uv_CAN_msg msg;
        msg.msg_id = LOG_DUMP_CAN_ID;
        msg.dlc    = 8;

        msg.data[0] = (uint8_t)entry->type;
        msg.data[1] = (uint8_t)(entry->timestamp & 0xFF);
        msg.data[2] = (uint8_t)((entry->timestamp >> 8) & 0xFF);
        msg.data[3] = (uint8_t)entry->task_id;
        msg.data[4] = (uint8_t)entry->is_panic;
        msg.data[5] = (uint8_t)entry->vehicle_state;
        msg.data[6] = 0;
        msg.data[7] = 0;

        uvSendCanMSG(&msg);  // <-- Use your existing CAN transmit wrapper
        osDelay(2);          // small gap between packets
    }
}
