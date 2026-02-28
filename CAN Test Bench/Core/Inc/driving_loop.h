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
    DL_MAP_CUBIC = 1,
	DL_MAP_EXP = 2
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
    int32_t offset;         /**< output offset */ //4

    /* Softening at speed (optional) */
    float    soften_gain;   /**< 0..1 fraction to reduce at high speed */ //8
    uint16_t soften_rpm;    /**< start softening above this rpm */ //12


    /* Optional clamp helpers */
    uint16_t max_rpm;       /**< clamp behavior near max rpm */ //14
    //16
} adaptive_torque_map_args;

typedef struct scurve_map_args{
	uint32_t dummy;

}scurve_map_args;

typedef struct exponential_map_args{
	float s;

}exponential_map_args;

/** @brief Union holding whichever map params are active for a driving mode */
typedef union drivingModeParams
{
    linear_torque_map_args   linear;
    exponential_map_args exp;
    scurve_map_args scurve;
} drivingModeParams;

/* ============================================================================
 * Driving Modes (per-mode caps + map selection)
 * ========================================================================== */
typedef struct drivingMode
{
    char dm_name[16];            /**< Name of mode, 15 chars + '\0' */

    /* 32-bit fields */
    uint32_t max_acc_pwr;        /**< mode power cap [W] (0 = disabled) */ //16
    uint32_t max_motor_torque;   /**< mode torque cap [Nm] (0 = disabled) */ //20
    //uint32_t max_current;        /**< mode current cap [A] (0 = disabled) */ //24

    float kVal;					 /**< mode K value for filtering */

    /* 16-bit fields */
    uint16_t flags; //28
    uint16_t reserved16b; //30 - maintains bit alignment

    /* map selection */
    dl_map_mode_t control_map_fn;  /**< which mapping mode to use */ //32

    adaptive_torque_map_args adaptive_settings;

    /* union last */
    drivingModeParams map_fn_params; /**< parameters for the selected map */ //36
} drivingMode;

typedef enum dmode_flags{
	DMODE_REGEN_EN,
	DMODE_TC_EN,
	DMODE_FILT_EN
}dmode_flags;


/* ============================================================================
 * Driving loop configuration parameters
 * (flash / desktop-tunable)
 * ========================================================================== */
typedef struct driving_loop_args
{
    /* ========================= 32-bit fields ========================= */

    /* HARD PHYSICAL LIMITS */
    uint32_t absolute_max_acc_pwr;        // Max accumulator power [W] //0
    uint32_t absolute_max_motor_torque;   // Max allowed motor torque [Nm] //4
    uint32_t absolute_max_accum_current;  // Max accumulator current [A] //8
    uint32_t max_accum_current_5s;        // Short-term accumulator current [A] //12

    /* DERATING / LIMITING */
    uint32_t torque_limit_source_mask;    // Active torque limit source bitmask //16
    uint32_t default_power_limit_w;       // Fallback power limit [W] //20
    uint32_t default_current_limit_a;     // Fallback current limit [A] //24

    /* TORQUE INHIBIT (floats) */
    float torque_inhibit_apps_percent;          // Throttle % above which inhibit triggers //28
    float torque_inhibit_bps_percent;           // Brake % above which inhibit triggers //32
    float torque_inhibit_recover_apps_percent;  // Throttle % below which inhibit clears //36
    float torque_inhibit_recover_bps_percent;   // Brake % below which inhibit clears //40

    /* DRIVER FEEL / TORQUE SHAPING */
    float throttle_deadband_percent;     // Throttle below this % treated as zero //44
    float torque_zero_threshold_nm;      // Torque below this snaps to zero [Nm] //48
    float filter_k_accel;                // Torque rise filter gain (0–1) //52
    float filter_k_decel;                // Torque drop filter gain (0–1) //56
    float torque_rate_up_nm_per_s;       // Max torque increase rate [Nm/s] //60
    float torque_rate_down_nm_per_s;     // Max torque decrease rate [Nm/s] //64
    float derate_rate_nm_per_s;          // Torque derate rate [Nm/s] //68

    /* REGEN (floats) */
    float regen_max_torque_nm;           // Max regen torque magnitude [Nm] //72
    float regen_blend_start_bps_percent; // Brake % where regen blending begins //76
    float regen_blend_end_bps_percent;   // Brake % where full regen is requested //80

    /* INPUT TIMEOUT (floats) */
    float input_change_threshold_percent; // Pedal delta to count as activity [%] //84

    /* ========================= 16-bit fields ========================= */

    /* MOTOR LIMITS */
    uint16_t absolute_max_motor_rpm;      // Absolute motor speed limit [RPM] //88
    uint16_t regen_rpm_cutoff;            // Regen disabled below this RPM //90

    /* APPS / BPS RAW LIMITS */
    uint16_t min_apps_offset; //92
    uint16_t max_apps_offset; //94
    uint16_t min_apps_value; //96

    uint16_t apps1_abs_min_val; //98
    uint16_t apps1_abs_max_val; //100
    uint16_t apps2_abs_min_val; //102
    uint16_t apps2_abs_max_val; //104

    uint16_t min_BPS_value; //106
    uint16_t max_BPS_value; //108

    /* APPS / BPS SCALING */
    uint16_t apps1_top; //110
    uint16_t apps1_bottom; //112
    uint16_t apps2_top; //114
    uint16_t apps2_bottom; //116

    /* PLAUSIBILITY & SAFETY */
    uint16_t apps_plausibility_check_threshold;      // Allowed APPS1/APPS2 mismatch [%] //118
    uint16_t apps_mismatch_time_ms;                   // Time mismatch must persist [ms] //120
    uint16_t bps_plausibility_check_threshold;        // Brake plausibility threshold (reserved) - unused //122
    uint16_t bps_implausibility_recovery_threshold;   // Brake signal recovery threshold - unused //124
    uint16_t apps_implausibility_recovery_threshold;  // Throttle signal recovery threshold - unused //126

    /* INPUT TIMEOUT */
    uint16_t input_timeout_ms; //128

    /* ========================= 8-bit fields ========================= */

    /* REGEN */
    uint8_t regen_enable;                 // Enable / disable regen //130
    uint8_t accum_regen_soc_threshold;    // Regen disabled above this SOC [%] //131

    /* INPUT TIMEOUT */
    uint8_t input_timeout_enable;         // Enable / disable input timeout //132

    /* DRIVING MODES */
    uint8_t num_driving_modes;            // Number of populated modes //133
    uint8_t period;                       // Driving loop period [ms] //134
    uint8_t reserved8b_1; //135
	//136

    /* ========================= Arrays / structs ========================= */

    drivingMode dmodes[4];                // Per-mode caps + map params

} driving_loop_args;

/* ============================================================================
 * Public API
 * ========================================================================== */
enum uv_status_t initDrivingLoop(void *argument);
void StartDrivingLoop(void *argument);
float calculateBrakePercentage(uint16_t bps1);

#endif /* INC_DRIVING_LOOP_H_ */
