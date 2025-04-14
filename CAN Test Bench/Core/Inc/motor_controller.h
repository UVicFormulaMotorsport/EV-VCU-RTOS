#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "main.h"
#include "uvfr_utils.h"
#include "uvfr_settings.h"
#include "can.h"

typedef struct motor_controller_settings motor_controller_settings;
typedef struct uv_CAN_msg uv_CAN_msg;

/* Enums for CAN register IDs and other constants */

/* Speed parameters: */
enum motor_controller_speed_parameters {
    N_actual = 0x30,  // actual motor speed in rpm (16-bit, little-endian)
    N_set    = 0x31,  // setpoint (used for torque command in our case)
    N_cmd    = 0x32,  // command speed after ramp
    N_error  = 0x33   // speed error
};

enum motor_controller_status {
    LOGIMAP_ERRORS = 0x82,  // Error bitfield
    LOGIMAP_IO     = 0x83,  // I/O status bitfield
    POS_ACTUAL     = 0x86,  // 32-bit position
    //motor_controller_errors_warnings = 0x8F // Some controllers use 0x8F for combined errors/warnings
};

/* Current parameters */
enum motor_controller_current_parameters {
    CURRENT_ACTUAL = 0x69
};

/* Motor constants */
enum motor_controller_motor_constants {
    nominal_motor_frequency = 0x05,
    nominal_motor_voltage   = 0x06,
    power_factor            = 0x0E,
    motor_max_current       = 0x4D,
    motor_continuous_current= 0x4E,
    motor_pole_number       = 0x4F,
    motor_kt_constant       = 0x87,  // low
    motor_ke_constant       = 0x87,  // high, back emf
    rated_motor_speed       = 0x59,
    motor_temperature_switch_off_point = 0xA3,
    stator_leakage_inductance = 0xB1,
    nominal_magnitizing_current = 0xB2,
    motor_magnetising_inductance = 0xB3,
    rotor_resistance        = 0xB4,
    minimum_magnetising_current = 0xB5,
    time_constant_rotor     = 0xB6,
    leakage_inductance_ph_ph = 0xBB,
    stator_resistance_ph_ph = 0xBC,
    time_constant_stator    = 0xBD
};

/* Temperatures */
enum motor_controller_temperatures {
    igbt_temperature       = 0x4A,
    motor_temperature      = 0x49,
    air_temperature        = 0x4B,
    current_derate_temperature = 0x4C,
    temp_sensor_pt1        = 0x9C,
    temp_sensor_pt2        = 0x9D,
    temp_sensor_pt3        = 0x9E,
    temp_sensor_pt4        = 0x9F
};

/* Measurements */
enum motor_controller_measurements {
    DC_bus_voltage = 0xEB
};

/* Status, error, and warning flags */
enum motor_controller_status_information_errors_warnings {
    motor_controller_errors_warnings = 0x8F, // used as register ID for error/warning response


    /* Errors (assume low 16 bits) */
    eprom_read_error                   = 1 << 8,
    hardware_fault                     = 1 << 9,
    rotate_field_enable_not_present_run= 1 << 10,
    CAN_timeout_error                  = 1 << 11,
    feedback_signal_error              = 1 << 12,
    mains_voltage_min_limit            = 1 << 13,
    motor_temp_max_limit               = 1 << 14,
    IGBT_temp_max_limit                = 1 << 15,

    /* Additional errors/warnings (often in high 16 bits) */
    mains_voltage_max_limit            = 1,
    critical_AC_current                = 1 << 1,
    race_away_detected                 = 1 << 2,
    ecode_timeout_error                = 1 << 3,
    watchdog_reset                     = 1 << 4,
    AC_current_offset_fault            = 1 << 5,
    internal_hardware_voltage_problem  = 1 << 6,
    bleed_resistor_overload            = 1 << 7,
    parameter_conflict_detected        = 1 << 8,
    special_CPU_fault                  = 1 << 9,
    rotate_field_enable_not_present_norun = 1 << 10,
    auxiliary_voltage_min_limit        = 1 << 11,
    feedback_signal_problem            = 1 << 12,
    warning_5                          = 1 << 13,
    motor_temperature_warning          = 1 << 14,
    IGBT_temperature_warning           = 1 << 15,
    Vout_saturation_max_limit          = 1,
    warning_9                          = 1 << 1,
    speed_actual_resolution_limit      = 1 << 2,
    check_ecode_ID                     = 1 << 3,
    tripzone_glitch_detected           = 1 << 4,
    ADC_sequencer_problem              = 1 << 5,
    ADC_measurement_problem            = 1 << 6,
    bleeder_resistor_warning           = 1 << 7
};

/* I/O enum (placeholder) */
enum motor_controller_io {
    todo6969 = 6969
};

/* PI control values */
enum motor_controller_PI_values {
    accelerate_ramp       = 0x35,
    dismantling_ramp      = 0xED,
    recuperation_ramp     = 0xC7,
    proportional_gain     = 0x1C,  // may change based on mode
    integral_time_constant= 0x1D,
    integral_memory_max   = 0x2B,
    proportional_gain_2   = 0xC9,
    current_feed_forward  = 0xCB,
    ramp_set_current      = 0x25
};

/* Repeating time values */
enum motor_controller_repeating_time {
    none            = 0,
    one_hundred_ms  = 0x64
};

/* Limp mode values */
enum motor_controller_limp_mode {
    N_lim       = 0x34,
    N_lim_plus  = 0x3F,
    N_lim_minus = 0x3E
};

/* Startup-related registers */
enum motor_controller_startup {
    clear_errors     = 0x8E,
    firmware_version = 0x1B,
//	serial_number = 0x62
};

/* Additional register IDs for requests */
//#define SERIAL_NUMBER_REGISTER     0x1B  // adjust if needed
//#define FIRMWARE_VERSION_REGISTER  0x1E  // adjust if needed
#define SERIAL_NUMBER_REGISTER     0x62  // adjust if needed
#define FIRMWARE_VERSION_REGISTER  0x1B  // adjust if needed

/* Command identifier used for torque command.
   (For example, you might use N_set from motor_controller_speed_parameters.) */
#define N_set_cmd  N_set

/*
 * Declare an extern for your global mc_default_settings so other files can use it.
 * The actual definition will be in motor_controller.c.
 */
extern motor_controller_settings mc_default_settings;


/* Function prototypes that will be called by other modules (driving_loop, init, etc.) */
void MC_Startup(void* args);
uint16_t MotorControllerSpinTest(float T_filtered);
void MC_Request_Data(uint8_t RegID);
void ProcessMotorControllerResponse(uv_CAN_msg* msg);
void Parse_Bamocar_Response(uv_CAN_msg* msg);

#endif /* __MOTOR_CONTROLLER_H__ */
