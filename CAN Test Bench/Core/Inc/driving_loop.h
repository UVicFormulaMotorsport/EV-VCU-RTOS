/*
 * driving_loop.h
 *
 *  Created on: Oct 14, 2024
 *      Author: byo10
 *
 *  Updated: keep pedal mapping to LINEAR + ADAPTIVE only.
 */

#ifndef INC_DRIVING_LOOP_H_
#define INC_DRIVING_LOOP_H_

#include <stdint.h>
#include "motor_controller.h"
#include "uvfr_utils.h"

/* Optional typedefs (fine to keep if used elsewhere) */
typedef uint16_t MC_Torque;
typedef uint16_t MC_RPM;
typedef uint16_t MC_POWER;

/* ============================================================================
 * Pedal map selection (keep it simple: linear + adaptive only)
 * NOTE: enums in C can compile to 32-bit depending on compiler.
 * If packing ever becomes annoying, swap this to a uint8_t + defines.
 * ========================================================================== */
typedef enum
{
    DL_MAP_LINEAR   = 0,
    DL_MAP_ADAPTIVE = 1
} dl_map_mode_t;

/* Driving loop internal status */
typedef enum
{
    Plausible   = 0x01,
    Implausible = 0x02,
    Erroneous   = 0x04
} DL_internal_state_t;

/* ============================================================================
 * Map parameter structs
 * ========================================================================== */

/** @brief Parameters for a linear torque map: y = m x + b */
typedef struct linear_torque_map_args
{
    int32_t offset;  /**< output offset (units depend on implementation) */
    float   slope;   /**< gain (units depend on implementation) */
} linear_torque_map_args;

/** @brief Adaptive map parameters
 *
 * Simple adaptive drive map knobs (minimal map explosion).
 */
typedef struct adaptive_torque_map_args
{
    float   base_slope;     /**< baseline slope */
    int32_t offset;         /**< output offset */

    /* Softening at speed (optional) */
    uint16_t soften_rpm;    /**< start softening above this rpm */
    float    soften_gain;   /**< 0..1 fraction to reduce at high speed */

    /* Optional clamp helpers */
    uint16_t max_rpm;       /**< clamp behavior near max rpm */
} adaptive_torque_map_args;

/** @brief Union holding whichever map params are active for a driving mode */
typedef union drivingModeParams
{
    linear_torque_map_args   linear;
    adaptive_torque_map_args adaptive;
} drivingModeParams;

/* ============================================================================
 * Driving Modes (per-mode caps + map selection)
 * ========================================================================== */
typedef struct drivingMode
{
    char dm_name[16];            /**< Name of mode, 15 chars + '\0' */

    /* 32-bit fields */
    uint32_t max_acc_pwr;        /**< mode power cap [W] (0 = disabled) */
    uint32_t max_motor_torque;   /**< mode torque cap [Nm] (0 = disabled) */
    uint32_t max_current;        /**< mode current cap [A] (0 = disabled) */

    /* 16-bit fields */
    uint16_t flags;

    /* map selection */
    dl_map_mode_t control_map_fn;  /**< which mapping mode to use */

    /* union last */
    drivingModeParams map_fn_params; /**< parameters for the selected map */
} drivingMode;

/* ============================================================================
 * Driving loop configuration parameters
 * (flash / desktop-tunable)
 * ========================================================================== */
typedef struct driving_loop_args
{
    /* ========================= 32-bit fields ========================= */

    /* HARD PHYSICAL LIMITS */
    uint32_t absolute_max_acc_pwr;        // Max accumulator power [W]
    uint32_t absolute_max_motor_torque;   // Max allowed motor torque [Nm]
    uint32_t absolute_max_accum_current;  // Max accumulator current [A]
    uint32_t max_accum_current_5s;        // Short-term accumulator current [A]

    /* DERATING / LIMITING */
    uint32_t torque_limit_source_mask;    // Active torque limit source bitmask
    uint32_t default_power_limit_w;       // Fallback power limit [W]
    uint32_t default_current_limit_a;     // Fallback current limit [A]

    /* TORQUE INHIBIT (floats) */
    float torque_inhibit_apps_percent;          // Throttle % above which inhibit triggers
    float torque_inhibit_bps_percent;           // Brake % above which inhibit triggers
    float torque_inhibit_recover_apps_percent;  // Throttle % below which inhibit clears
    float torque_inhibit_recover_bps_percent;   // Brake % below which inhibit clears

    /* DRIVER FEEL / TORQUE SHAPING */
    float throttle_deadband_percent;     // Throttle below this % treated as zero
    float torque_zero_threshold_nm;      // Torque below this snaps to zero [Nm]
    float filter_k_accel;                // Torque rise filter gain (0–1)
    float filter_k_decel;                // Torque drop filter gain (0–1)
    float torque_rate_up_nm_per_s;       // Max torque increase rate [Nm/s]
    float torque_rate_down_nm_per_s;     // Max torque decrease rate [Nm/s]
    float derate_rate_nm_per_s;          // Torque derate rate [Nm/s]

    /* REGEN (floats) */
    float regen_max_torque_nm;           // Max regen torque magnitude [Nm]
    float regen_blend_start_bps_percent; // Brake % where regen blending begins
    float regen_blend_end_bps_percent;   // Brake % where full regen is requested

    /* INPUT TIMEOUT (floats) */
    float input_change_threshold_percent; // Pedal delta to count as activity [%]

    /* ========================= 16-bit fields ========================= */

    /* MOTOR LIMITS */
    uint16_t absolute_max_motor_rpm;      // Absolute motor speed limit [RPM]
    uint16_t regen_rpm_cutoff;            // Regen disabled below this RPM

    /* APPS / BPS RAW LIMITS */
    uint16_t min_apps_offset;
    uint16_t max_apps_offset;
    uint16_t min_apps_value;

    uint16_t apps1_abs_min_val;
    uint16_t apps1_abs_max_val;
    uint16_t apps2_abs_min_val;
    uint16_t apps2_abs_max_val;

    uint16_t min_BPS_value;
    uint16_t max_BPS_value;

    /* APPS / BPS SCALING */
    uint16_t apps1_top;
    uint16_t apps1_bottom;
    uint16_t apps2_top;
    uint16_t apps2_bottom;

    /* PLAUSIBILITY & SAFETY */
    uint16_t apps_plausibility_check_threshold;      // Allowed APPS1/APPS2 mismatch [%]
    uint16_t apps_mismatch_time_ms;                   // Time mismatch must persist [ms]
    uint16_t bps_plausibility_check_threshold;        // Brake plausibility threshold (reserved)
    uint16_t bps_implausibility_recovery_threshold;   // Brake signal recovery threshold
    uint16_t apps_implausibility_recovery_threshold;  // Throttle signal recovery threshold

    /* INPUT TIMEOUT */
    uint16_t input_timeout_ms;

    /* ========================= 8-bit fields ========================= */

    /* REGEN */
    uint8_t regen_enable;                 // Enable / disable regen
    uint8_t accum_regen_soc_threshold;    // Regen disabled above this SOC [%]

    /* INPUT TIMEOUT */
    uint8_t input_timeout_enable;         // Enable / disable input timeout

    /* DRIVING MODES */
    uint8_t num_driving_modes;            // Number of populated modes
    uint8_t period;                       // Driving loop period [ms]

    /* ========================= Arrays / structs ========================= */

    drivingMode dmodes[8];                // Per-mode caps + map params

} driving_loop_args;

/* ============================================================================
 * Public API
 * ========================================================================== */
enum uv_status_t initDrivingLoop(void *argument);
void StartDrivingLoop(void *argument);

#endif /* INC_DRIVING_LOOP_H_ */
