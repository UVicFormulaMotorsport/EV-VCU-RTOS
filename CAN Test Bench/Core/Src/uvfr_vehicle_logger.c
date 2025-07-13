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


// Logger Configuration

#define MAX_LOG_ENTRIES 32  // max # of stored


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
 *
 * @param type      what type of fault occurred (enum value)
 * @param task      pointer to the task that caused it (can be NULL)
 * @param msg       message or reason for the error
 * @param file      source file (use __FILE__)
 * @param line      line number (use __LINE__)
 * @param func      function name (use __func__)
 * @param is_panic  true if this was caused by a system panic
 */
void logVehicleFault(fault_event_type_e type,
                     uv_task_info* task,
                     const char* msg,
                     const char* file,
                     int line,
                     const char* func,
                     bool is_panic)
{
    // mark this fault type as "occurred"
    system_error_flags |= (1U << type);

    // write into next log slot
    vehicle_log_entry* entry = &log_buffer[log_write_index];

    entry->type = type;
    entry->msg = msg;
    entry->file = file;
    entry->line = line;
    entry->func = func;
    entry->is_panic = is_panic;
    entry->timestamp = xTaskGetTickCount();
    entry->vehicle_state = vehicle_state;

    // add task info (if task is valid)
    if (task != NULL) {
        entry->task_name = task->task_name;
        entry->task_id = task->task_id;
        entry->task_state = task->task_state;
    } else {
        entry->task_name = "None";
        entry->task_id = 0xFF;
        entry->task_state = UV_TASK_NOT_STARTED;
    }

    // move to next slot
    log_write_index++;
    if (log_write_index >= MAX_LOG_ENTRIES) {
        log_write_index = 0;
        has_wrapped = true;
    }
}
