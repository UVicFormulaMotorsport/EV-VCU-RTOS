/** @file driving_loop.c
 *  @brief Main driving loop task: reads APPS/BPS, runs plausibility/safety checks,
 *         maps throttle -> torque request, applies a torque filter, and sends to motor controller.
 *
 *  Units convention (used throughout this file):
 *   - Raw sensor inputs: ADC counts (uint16_t)
 *   - Pedal positions: percent [%] (float, 0.0–100.0)
 *   - Torque values: [Nm] (float)
 *   - Speed feedback: [RPM] (int16_t)
 *   - Power limits: [W] (float / uint32_t)
 *   - Time: [ms] in settings, RTOS ticks internally
 *   - Omega: [rad/s] when converting RPM for power-based torque caps
 */
#define __UV_FILENAME__ "driving_loop.c"


#include "main.h"
#include "uvfr_utils.h"
#include "can.h"
#include "motor_controller.h"
#include "uvfr_state_engine.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "driving_loop.h"
#include "../FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h"
#include "../FreeRTOS/Source/include/FreeRTOS.h"
#include "../FreeRTOS/Source/include/task.h"

// -----------------------------------------------------------------------------
// External sensor variables (populated by ADC/DAQ code elsewhere)
// -----------------------------------------------------------------------------
extern uint16_t adc1_APPS1;   // [ADC counts]
extern uint16_t adc1_APPS2;   // [ADC counts]
extern uint16_t adc1_BPS1;    // [ADC counts]
extern uint16_t adc1_BPS2;    // [ADC counts]

// Vehicle state comes from the state machine.
//extern uv_vehicle_state_t vehicle_state;  // [enum state], e.g. UV_DRIVING

// Extern settings pointer (flash-configurable vehicle settings)
//extern uv_vehicle_settings* current_vehicle_settings; // [ptr]
//extern int vehicle_state;
extern enum uv_vehicle_state_t vehicle_state;

// BMS health / freshness
extern TickType_t bms_last_msg_time;  // [RTOS ticks]
extern uint8_t    is_bms_connected;   // [bool-like], 0/1

// BMS telemetry (verify scaling here matches pack message definitions)
extern uint16_t packCurrent;   // [0.1 A]  => packCurrent * 0.1f = [A]
extern uint16_t packVoltage;   // [0.1 V]  => packVoltage * 0.1f = [V]
extern uint16_t packDCL;       // [0.1 A] or [A]? comment says "max discharge current" (assumed 0.1A below)
extern uint16_t stateOfCharge; // [%] (often 0–100)

extern uint16_t msg1corrupt;   // [bool-like], 0/1
extern uint16_t msg2corrupt;   // [bool-like], 0/1

// -----------------------------------------------------------------------------
// Driving loop settings
// -----------------------------------------------------------------------------
driving_loop_args* driving_args = NULL;  // [ptr] active DL settings

// -----------------------------------------------------------------------------
// Default driving loop settings.
// These can be overwritten from flash / vehicle settings / desktop tuner.
// -----------------------------------------------------------------------------
driving_loop_args default_dl_settings = {

    /* ========================= 32-bit fields (float / uint32_t) ========================= */

    /* TORQUE INHIBIT (floats, % thresholds) */
    .torque_inhibit_apps_percent         = 25.0f, // [%] throttle
    .torque_inhibit_bps_percent          = 15.0f, // [%] brake
    .torque_inhibit_recover_apps_percent = 5.0f,  // [%] throttle
    .torque_inhibit_recover_bps_percent  = 5.0f,  // [%] brake

    /* DRIVER FEEL / TORQUE SHAPING */
    .throttle_deadband_percent = 0.0f,  // [%] throttle deadband
    .torque_zero_threshold_nm  = 0.0f,  // [Nm] snap-to-zero threshold
    .filter_k_accel            = 0.4f,  // [0–1] accel filter gain
    .filter_k_decel            = 1.0f,  // [0–1] decel filter gain
    .torque_rate_up_nm_per_s   = 1e9f,  // [Nm/s] (1e9 disables effectively)
    .torque_rate_down_nm_per_s = 1e9f,  // [Nm/s]
    .derate_rate_nm_per_s      = 1e9f,  // [Nm/s]

    /* HARD PHYSICAL LIMITS */
    .absolute_max_acc_pwr       = 10,   // [W] placeholder bring-up
    .absolute_max_motor_torque  = 230,  // [Nm]
    .absolute_max_accum_current = 200,  // [A]
    .max_accum_current_5s       = 200,  // [A]
    .absolute_max_motor_rpm     = 6500, // [RPM]
    .regen_rpm_cutoff           = 1000, // [RPM]

    /* PLAUSIBILITY & SAFETY */
    .apps_mismatch_time_ms = 100, // [ms] mismatch persistence time (currently not enforced in logic below)

    /* DERATING / LIMITING */
    .default_power_limit_w   = 0, // [W] (0 = unused)
    .default_current_limit_a = 0, // [A] (0 = unused)

    /* ========================= 16-bit fields (uint16_t) ========================= */

    /* APPS / BPS RAW + BOUNDS */
    .min_apps_offset  = 0, // [ADC counts]
    .max_apps_offset  = 0, // [ADC counts]
    .min_apps_value   = 0, // [ADC counts]

    .apps1_abs_min_val = 0x0200, // [ADC counts]
    .apps1_abs_max_val = 0x10C4, // [ADC counts]
    .apps2_abs_min_val = 0x0202, // [ADC counts]
    .apps2_abs_max_val = 0x1029, // [ADC counts]

    .min_BPS_value = 0x0106, // [ADC counts]
    .max_BPS_value = 0x0B7E, // [ADC counts]

    /* APPS / BPS SCALING */
    .apps1_top    = 0x09F9, // [ADC counts] 100% throttle
    .apps1_bottom = 0x0570, // [ADC counts] 0% throttle
    .apps2_top    = 0x0999, // [ADC counts] 100% throttle
    .apps2_bottom = 0x02B0, // [ADC counts] 0% throttle

    /* PLAUSIBILITY & SAFETY */
    .apps_plausibility_check_threshold       = 10,  // [%] allowed APPS mismatch
    .bps_plausibility_check_threshold        = 500, // [%] reserved (not used)
    .bps_implausibility_recovery_threshold   = 5, //300, // [%] NOTE: currently treated like % but value looks like ADC-era legacy
    .apps_implausibility_recovery_threshold  = 5, //100, // [%] same note as above

    /* ========================= 8-bit fields ========================= */
    .torque_limit_source_mask = 0, // [bitmask]
    .num_driving_modes        = 1, // [count]
    .period                   = 10, // [ms] DL period setting (task_period currently used separately)

    .dmodes = {0}, // [struct array] mode table (optional / future)
};

// -----------------------------------------------------------------------------
//State variables used by the driving loop / filter
// -----------------------------------------------------------------------------
bool  is_accelerating = false; // [bool]
float T_PREV = 0.0f;           // [Nm] previous torque actually sent (post-limits)
float T_REQ  = 0.0f;           // [Nm] torque request from pedal map (pre-filter/limits)

static bool torque_inhibit_active = false; // [bool] latched inhibit

// -----------------------------------------------------------------------------
// Driver inactivity tracking (currently informational / future use)
// -----------------------------------------------------------------------------
#define INPUT_TIMEOUT_MS 500
#define THROTTLE_CHANGE_THRESHOLD 5.0f  // [%]
#define BRAKE_CHANGE_THRESHOLD    5.0f  // [%]

TickType_t last_driver_input_time = 0; // [RTOS ticks]
static float last_throttle_percent = 0.0f; // [%]
static float last_brake_percent    = 0.0f; // [%]

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2);
static float calculateBrakePercentage(uint16_t bps1);

static bool  performSafetyChecks(driving_loop_args* dl_params,
                                 uint16_t apps1_value,
                                 uint16_t apps2_value,
                                 uint16_t bps1_value,
                                 uint16_t bps2_value,
                                 DL_internal_state_t* dl_status);

static inline float getKValue(int raceMode);
static inline float applyTorqueFilter(float T_req, float T_prev, bool is_accelerating);

static inline float dl_clampf(float x, float lo, float hi);
static inline float dl_slewLimit(float target, float prev, float rate_nm_per_s, float dt_s);

static bool  bms_is_ok(void);
static float torqueCapFromBMS(float omega_rad_s);
static float limitTorque(float T_cmd, float T_prev, const driving_loop_args* dl, float dt_s);

// -----------------------------------------------------------------------------
// initDrivingLoop(): sets up DAQ associations and creates the driving loop task
// -----------------------------------------------------------------------------
enum uv_status_t initDrivingLoop(void *argument)
{
    (void)argument;

    // Associate DAQ parameters with live ADC variables (ADC counts)
    associateDaqParamWithVar(APPS1_ADC_VAL, &adc1_APPS1); // [ADC counts]
    associateDaqParamWithVar(APPS2_ADC_VAL, &adc1_APPS2); // [ADC counts]
    associateDaqParamWithVar(BPS1_ADC_VAL,  &adc1_BPS1);  // [ADC counts]
    associateDaqParamWithVar(BPS2_ADC_VAL,  &adc1_BPS2);  // [ADC counts]

    uv_task_info* dl_task = uvCreateTask(); // [ptr]
    if (dl_task == NULL) {
        return UV_ERROR;
    }

    // Pull settings from global vehicle settings (flash-configurable)
    driving_args = current_vehicle_settings->driving_loop_settings; // [ptr]

    dl_task->task_name          = "Driving_Loop";
    dl_task->task_function      = StartDrivingLoop;
    dl_task->task_priority      = osPriorityHigh;
    dl_task->stack_size         = 256;         // [words/bytes depends on wrapper]
    dl_task->active_states      = UV_DRIVING;  // [state bit(s)]
    dl_task->suspension_states  = 0x00;        // [state bitmask]

    dl_task->deletion_states = UV_INIT | UV_READY | PROGRAMMING | UV_SUSPENDED |
                               UV_LAUNCH_CONTROL | UV_ERROR_STATE; // [state bitmask]

    dl_task->task_period = 100; // [ms] RTOS scheduling period used by wrapper
    dl_task->task_args   = NULL;

    return UV_OK;
}

// -----------------------------------------------------------------------------
// Adaptive pedal map (DRIVE ONLY – NO REGEN)
// throttle_percent: [%] 0..100
// returns: torque request [Nm]
// -----------------------------------------------------------------------------
static float mapThrottleToTorqueAdaptive(float throttle_percent, const driving_loop_args* dl)
{
    // Normalize throttle to 0..1
    float apps = dl_clampf(throttle_percent / 100.0f, 0.0f, 1.0f); // [unitless]

    // Deadband normalized to 0..1
    const float dead = dl_clampf(dl->throttle_deadband_percent / 100.0f, 0.0f, 0.9f); // [unitless]

    // x = clamp((apps - dead)/(1 - dead))
    float x = (apps - dead) / (1.0f - dead); // [unitless]
    x = dl_clampf(x, 0.0f, 1.0f);            // [unitless]

    if (x <= 0.0f) {
        return 0.0f; // [Nm]
    }

    // smoothstep shaping: 3x^2 - 2x^3 (unitless)
    float f_drive = (3.0f * x * x) - (2.0f * x * x * x); // [unitless]

    float T_max = (float)dl->absolute_max_motor_torque; // [Nm]
    float T_req = T_max * f_drive;                      // [Nm]

    return T_req; // [Nm]
}

// -----------------------------------------------------------------------------
// “Race mode” filter shaping (placeholder)
// -----------------------------------------------------------------------------
#define ACCELERATION 0
#define AUTOCROSS    1
#define ENDURANCE    2

static inline float getKValue(int raceMode)
{
    float kVal = 0.3f; // [0–1] default smoothing gain

    if (raceMode == ACCELERATION) {
        kVal = 0.7f; // [0–1]
    } else if (raceMode == AUTOCROSS) {
        kVal = 0.4f; // [0–1]
    } else if (raceMode == ENDURANCE) {
        kVal = 0.2f; // [0–1]
    }

    return kVal; // [0–1]
}

// -----------------------------------------------------------------------------
// APPS percent calculation
// apps1/apps2: [ADC counts]
// return: throttle [%] 0..100
// -----------------------------------------------------------------------------
static float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2)
{
    (void)apps2; // apps2 used in plausibility checks elsewhere

    // Clamp 0..100% based on APPS1 bottom/top calibration points (ADC counts)
    if (apps1 <= driving_args->apps1_bottom) { // [ADC counts]
        return 0.0f; // [%]
    }
    if (apps1 >= driving_args->apps1_top) { // [ADC counts]
        return 100.0f; // [%]
    }

    float throttle_percent =
        ((float)(apps1 - driving_args->apps1_bottom) /
         (float)(driving_args->apps1_top - driving_args->apps1_bottom)) * 100.0f; // [%]

    return throttle_percent; // [%]
}

// -----------------------------------------------------------------------------
// Brake percent calculation
// bps1: [ADC counts]
// return: brake [%] 0..100
// -----------------------------------------------------------------------------
static float calculateBrakePercentage(uint16_t bps1)
{
    // Sanity bounds (ADC counts)
    if (bps1 < driving_args->min_BPS_value || bps1 > driving_args->max_BPS_value) { // [ADC counts]
        return 0.0f; // [%]
    }

    float brake_percent =
        ((float)(bps1 - driving_args->min_BPS_value) /
         (float)(driving_args->max_BPS_value - driving_args->min_BPS_value)) * 100.0f; // [%]

    return brake_percent; // [%]
}

// -----------------------------------------------------------------------------
// Torque filter
// T_req/T_prev: [Nm]
// returns: filtered torque [Nm]
// -----------------------------------------------------------------------------
static inline float applyTorqueFilter(float T_req, float T_prev, bool is_accelerating)
{
    float FILTER_K = getKValue(ACCELERATION); // [0–1] accel smoothing gain

    // Never allow “lag” when torque is dropping (pedal lift / brake)
    if (!is_accelerating) {
        FILTER_K = 1.0f; // [0–1] immediate drop
    }

    // First-order filter: T_prev + (T_req - T_prev) * K
    float T_filtered = T_prev + (T_req - T_prev) * FILTER_K; // [Nm]

    // Snap-to-zero when request is exactly zero
    if (T_req == 0.0f) {
        T_filtered = 0.0f; // [Nm]
    }

    return T_filtered; // [Nm]
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static inline float dl_clampf(float x, float lo, float hi)
{
    // x/lo/hi are unit-consistent (unitless, %, Nm, etc.)
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float dl_slewLimit(float target, float prev, float rate_nm_per_s, float dt_s)
{
    // target/prev: [Nm]
    // rate_nm_per_s: [Nm/s]
    // dt_s: [s]
    if (rate_nm_per_s <= 0.0f || rate_nm_per_s > 1e8f) {
        return target; // [Nm] treat as disabled
    }

    float max_step = rate_nm_per_s * dt_s; // [Nm]
    float delta    = target - prev;        // [Nm]

    if (delta >  max_step) return prev + max_step; // [Nm]
    if (delta < -max_step) return prev - max_step; // [Nm]
    return target;                                  // [Nm]
}

// -----------------------------------------------------------------------------
// BMS health gate
// return: true if BMS fresh + valid
// -----------------------------------------------------------------------------
static bool bms_is_ok(void)
{
    const TickType_t now     = xTaskGetTickCount();  // [RTOS ticks]
    const TickType_t timeout = pdMS_TO_TICKS(200);   // [RTOS ticks] (200ms)

    if (!is_bms_connected) return false;            // [bool]
    if ((now - bms_last_msg_time) > timeout) return false; // [tick delta]

    if (msg1corrupt || msg2corrupt) return false;   // [bool-like]

    return true;
}

// -----------------------------------------------------------------------------
// Power-based torque cap from BMS
// omega_rad_s: [rad/s]
// return: torque cap [Nm]
// -----------------------------------------------------------------------------
static float torqueCapFromBMS(float omega_rad_s)
{
    // packVoltage [0.1 V] -> [V]
    float V = packVoltage * 0.1f; // [V]

    // packDCL assumed [0.1 A] -> [A] (verify scaling from BMS message)
    float I = packDCL * 0.1f;     // [A]

    float P_max = V * I;          // [W] electrical power cap

    // Avoid nonsense at low speeds
    if (omega_rad_s < 10.0f) {    // [rad/s]
        return 1e9f;              // [Nm] effectively uncapped here
    }

    return P_max / omega_rad_s;   // [Nm] (since W / (rad/s) = N·m)
}

// -----------------------------------------------------------------------------
// Apply all torque limiting/derating in one place
// T_cmd/T_prev: [Nm]
// dt_s: [s]
// return: limited torque [Nm]
// -----------------------------------------------------------------------------
static float limitTorque(float T_cmd, float T_prev, const driving_loop_args* dl, float dt_s)
{
    // 0) Hard BMS safety gate
    if (!bms_is_ok()) {
        return 0.0f; // [Nm]
    }

    float T = T_cmd; // [Nm]

    // 1) Absolute motor torque clamp (hard cap)
    T = dl_clampf(T, 0.0f, (float)dl->absolute_max_motor_torque); // [Nm]

    // 2) Optional slew-rate limiting (Nm/s)
    float rate = (T >= T_prev) ? dl->torque_rate_up_nm_per_s
                               : dl->torque_rate_down_nm_per_s; // [Nm/s]

    T = dl_slewLimit(T, T_prev, rate, dt_s); // [Nm]

    // 3) Power envelope from BMS: T <= P_max / omega
    extern int16_t mc_speed_rpm; // [RPM] from motor_controller.c feedback
    float omega = ((float)mc_speed_rpm * 2.0f * 3.1415926f) / 60.0f; // [rad/s]
    float T_cap_bms = torqueCapFromBMS(omega); // [Nm]

    T = fminf(T, T_cap_bms); // [Nm]

    return T; // [Nm]
}

// -----------------------------------------------------------------------------
// Driving Loop Task
// -----------------------------------------------------------------------------
void StartDrivingLoop(void *argument)
{
    uv_task_info* params = (uv_task_info*)argument; // [ptr task metadata]

    DL_internal_state_t dl_status = Plausible;   // [enum] plausibility state

    // Active driving-loop parameters (flash-configurable)
    driving_loop_args* dl_params = current_vehicle_settings->driving_loop_settings; // [ptr]

    // Period handling
    TickType_t tick_period = pdMS_TO_TICKS(params->task_period); // [RTOS ticks] from [ms]
    TickType_t last_time   = xTaskGetTickCount();                // [RTOS ticks]
    last_driver_input_time = last_time;                          // [RTOS ticks]

    for (;;)
    {
        // Task control (kill/suspend)
        if (params->cmd_data == UV_KILL_CMD) {
            killSelf(params);
        } else if (params->cmd_data == UV_SUSPEND_CMD) {
            suspendSelf(params);
        }

        // Run at fixed interval
        vTaskDelayUntil(&last_time, tick_period); // [ticks]

        // Snapshot ADC values so mid-loop changes don’t produce mixed samples
        const uint16_t apps1_value = adc1_APPS1; // [ADC counts]
        const uint16_t apps2_value = adc1_APPS2; // [ADC counts]
        const uint16_t bps1_value  = adc1_BPS1;  // [ADC counts]
        const uint16_t bps2_value  = adc1_BPS2;  // [ADC counts]

        float T_filtered = 0.0f; // [Nm]

        // 1) Safety checks first (plausibility / bounds / inhibit logic)
        bool safe = performSafetyChecks(dl_params,
                                        apps1_value, apps2_value, // [ADC counts]
                                        bps1_value, bps2_value,   // [ADC counts]
                                        &dl_status);              // [enum out]

        if (!safe) {
            T_filtered = 0.0f;                   // [Nm]
            sendTorqueToMotorController(T_filtered); // expects torque [Nm] at API boundary
            continue;
        }

        // 2) Convert raw ADC -> pedal percentages
        float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value); // [%]
        float brake_percent    = calculateBrakePercentage(bps1_value);                  // [%]

        // 3) Track significant driver input changes (used for future watchdog / inactivity)
        float throttle_delta = fabsf(throttle_percent - last_throttle_percent); // [%]
        float brake_delta    = fabsf(brake_percent - last_brake_percent);       // [%]

        if (throttle_delta > THROTTLE_CHANGE_THRESHOLD || brake_delta > BRAKE_CHANGE_THRESHOLD) {
            last_driver_input_time = xTaskGetTickCount(); // [ticks]
            last_throttle_percent  = throttle_percent;    // [%]
            last_brake_percent     = brake_percent;       // [%]
        }

        // 4) Pedal map: throttle [%] -> torque request [Nm]
        T_REQ = mapThrottleToTorqueAdaptive(throttle_percent, dl_params); // [Nm]

        // Determine ramp direction for filter selection
        is_accelerating = (T_REQ >= T_PREV); // [bool]

        // Apply filter: keeps drop instant and rise smoothed
        T_filtered = applyTorqueFilter(T_REQ, T_PREV, is_accelerating); // [Nm]

        // Bring-up scaling: halves torque before limits (temporary)
        T_filtered = T_filtered / 2.0f; // [Nm]

        // dt from task period (ms -> s)
        float dt_s = (float)params->task_period / 1000.0f; // [s]

        // Apply limit stack (BMS gate, hard torque cap, slew, power envelope)
        float T_to_send = limitTorque(T_filtered, T_PREV, dl_params, dt_s); // [Nm]

        // Only allow torque output in the driving state
        if (vehicle_state == UV_DRIVING) {
            sendTorqueToMotorController(T_to_send); // [Nm]
        } else {
            sendTorqueToMotorController(0.0f);      // [Nm]
        }

        // Save post-limit torque for next loop iteration
        T_PREV = T_to_send; // [Nm]
    }
}

// -----------------------------------------------------------------------------
// Safety checks / plausibility checks
// Returns true if torque is allowed, false if torque must be inhibited
// -----------------------------------------------------------------------------
static bool performSafetyChecks(driving_loop_args* dl_params,
                                uint16_t apps1_value, uint16_t apps2_value, // [ADC counts]
                                uint16_t bps1_value,  uint16_t bps2_value,  // [ADC counts]
                                DL_internal_state_t* dl_status)
{
    // Normalize APPS1 to 0..1 using calibration points (ADC counts)
    float apps1_ratio = 0.0f; // [unitless]
    if (apps1_value <= dl_params->apps1_bottom) {
        apps1_ratio = 0.0f; // [unitless]
    } else if (apps1_value >= dl_params->apps1_top) {
        apps1_ratio = 1.0f; // [unitless]
    } else {
        apps1_ratio = ((float)(apps1_value - dl_params->apps1_bottom)) /
                      (float)(dl_params->apps1_top - dl_params->apps1_bottom); // [unitless]
    }

    // Normalize APPS2 to 0..1 using calibration points (ADC counts)
    float apps2_ratio = 0.0f; // [unitless]
    if (apps2_value <= dl_params->apps2_bottom) {
        apps2_ratio = 0.0f; // [unitless]
    } else if (apps2_value >= dl_params->apps2_top) {
        apps2_ratio = 1.0f; // [unitless]
    } else {
        apps2_ratio = ((float)(apps2_value - dl_params->apps2_bottom)) /
                      (float)(dl_params->apps2_top - dl_params->apps2_bottom); // [unitless]
    }

    // APPS mismatch percent (0..100%)
    float apps_diff_percent = fabsf(apps1_ratio - apps2_ratio) * 100.0f; // [%]

    // Pedal percentages used for torque inhibit logic
    float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value); // [%]
    float brake_percent    = calculateBrakePercentage(bps1_value);                  // [%]

    // --- Absolute bounds checks (raw ADC safety) ---
    // If any sensor violates its absolute bounds, torque is inhibited immediately.
    if (apps1_value < dl_params->apps1_abs_min_val || apps1_value > dl_params->apps1_abs_max_val) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        return false;
    }
    if (apps2_value < dl_params->apps2_abs_min_val || apps2_value > dl_params->apps2_abs_max_val) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        return false;
    }
    if (bps1_value < dl_params->min_BPS_value || bps1_value > dl_params->max_BPS_value) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        return false;
    }
    if (bps2_value < dl_params->min_BPS_value || bps2_value > dl_params->max_BPS_value) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        return false;
    }

    // --- APPS plausibility mismatch check ---
    // apps_diff_percent is in [%], threshold is also expected [%]
    if (apps_diff_percent > (float)dl_params->apps_plausibility_check_threshold) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Implausible;     // [enum]
        return false;
    }

    // --- Brake + Throttle torque inhibit ---
    // throttle_percent and brake_percent are in [%]
    // NOTE: these values should ideally use dl_params->torque_inhibit_* thresholds
    //if (throttle_percent > 25.0f && brake_percent > 5.0f) {
    if (throttle_percent > dl_params->torque_inhibit_apps_percent && brake_percent    > dl_params->torque_inhibit_bps_percent){
        torque_inhibit_active = true; // [bool]
        *dl_status = Implausible;     // [enum]
        return false;
    }

    // --- Recovery hysteresis ---
    // Once inhibited, allow recovery only after both inputs are back below recovery thresholds.
    // NOTE: apps_implausibility_recovery_threshold / bps_implausibility_recovery_threshold should be [%]
    if (torque_inhibit_active &&
        throttle_percent < (float)dl_params->apps_implausibility_recovery_threshold && // [%]
        brake_percent    < (float)dl_params->bps_implausibility_recovery_threshold)    // [%]
    {
        torque_inhibit_active = false; // [bool]
    }

    if (torque_inhibit_active) {
    	*dl_status = Implausible; // [enum]
        return false;
    }

    *dl_status = Plausible; // [enum]
    return true;
}
