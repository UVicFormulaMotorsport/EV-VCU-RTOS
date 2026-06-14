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
//extern uint8_t    is_bms_connected;   // [bool-like], 0/1
#define is_bms_connected 1

// BMS telemetry (verify scaling here matches pack message definitions)
extern volatile bms_state_t g_bms_state;


//extern uint16_t packCurrent;   // [0.1 A]  => packCurrent * 0.1f = [A]
#define packCurrent g_bms_state.pack_current_dA
//extern uint16_t packVoltage;   // [0.1 V]  => packVoltage * 0.1f = [V]
#define packVoltage g_bms_state.pack_voltage_dV
//extern uint16_t packDCL;       // [0.1 A] or [A]? comment says "max discharge current" (assumed 0.1A below)
#define packDCL g_bms_state.dcl_dA
//extern uint16_t stateOfCharge; // [%] (often 0–100)
#define stateOfCharge g_bms_state.soc_pct

//extern uint16_t msg1corrupt;   // [bool-like], 0/1
#define msg1corrupt 0
//extern uint16_t msg2corrupt;   // [bool-like], 0/1
#define msg2corrupt 0

// -----------------------------------------------------------------------------
// Throttle/Brake pct values (global)
// -----------------------------------------------------------------------------
uint16_t g_throttle_percent = 0;
uint16_t g_brake_percent = 0;
// -----------------------------------------------------------------------------
// Driving loop settings
// -----------------------------------------------------------------------------
driving_loop_args* driving_args = NULL;  // [ptr] active DL settings

// -----------------------------------------------------------------------------
// Default driving loop settings.
// These can be overwritten from flash / vehicle settings / desktop tuner.
// -----------------------------------------------------------------------------
driving_loop_args default_dl_settings =
{

    /* ========================= 32-bit fields (float / uint32_t) ========================= */

    /* TORQUE INHIBIT (floats, % thresholds) */
    .torque_inhibit_apps_percent         = 25.0f, // [%] throttle
    .torque_inhibit_bps_percent          = 15.0f, // [%] brake
    .torque_inhibit_recover_apps_percent = 5.0f,  // [%] throttle
    .torque_inhibit_recover_bps_percent  = 5.0f,  // [%] brake

    /* DRIVER FEEL / TORQUE SHAPING */
    .throttle_deadband_percent = 0.0f,  // [%] throttle deadband
    .torque_zero_threshold_nm  = 0.0f,  // [Nm] snap-to-zero threshold
    .torque_rate_up_nm_per_s   = 1e9f,  // [Nm/s] (1e9 disables effectively)
    .torque_rate_down_nm_per_s = 1e9f,  // [Nm/s]
    .derate_rate_nm_per_s      = 1e9f,  // [Nm/s]


    /* HARD PHYSICAL LIMITS */
    .absolute_max_acc_pwr       = 75000,   // [W] placeholder bring-up
    .absolute_max_motor_torque  = 140,  // [Nm]
    .absolute_max_accum_current = 100,  // [A]
    .max_accum_current_5s       = 200,  // [A]
    .absolute_max_motor_rpm     = 6500, // [RPM]
    .regen_rpm_cutoff           = 1000, // [RPM]

    /* PLAUSIBILITY & SAFETY */
    //.apps_mismatch_time_ms = 100, // [ms] mismatch persistence time (currently not enforced in logic below)

    /* DERATING / LIMITING */
   // .default_power_limit_w   = 0, // [W] (0 = unused)
   // .default_current_limit_a = 0, // [A] (0 = unused)

    /* ========================= 16-bit fields (uint16_t) ========================= */

    /* APPS / BPS RAW + BOUNDS */
    .min_apps_offset  = 0, // [ADC counts]
    .max_apps_offset  = 0, // [ADC counts]
    .min_apps_value   = 0, // [ADC counts]

    .apps1_abs_min_val = 0x0020, // [ADC counts]
    .apps1_abs_max_val = 0x10C4, // [ADC counts]
    .apps2_abs_min_val = 0x0000, // [ADC counts]
    .apps2_abs_max_val = 0x1029, // [ADC counts]

    .min_BPS_value = 0x00120, // [ADC counts]
    .max_BPS_value = 0x0490, // [ADC counts]

    /* APPS / BPS SCALING */
    .apps1_top    = 2000, //0x09F9, // [ADC counts] 100% throttle
    .apps1_bottom = 1055, // [ADC counts] 0% throttle
    .apps2_top    = 1540, // [ADC counts] 100% throttle
    .apps2_bottom = 675, // [ADC counts] 0% throttle

    /* PLAUSIBILITY & SAFETY */
    .apps_plausibility_check_threshold       = 30,  // [%] allowed APPS mismatch
    .bps_plausibility_check_threshold        = 500, // [%] reserved (not used)
    .bps_implausibility_recovery_threshold   = 5, //300, // [%] NOTE: currently treated like % but value looks like ADC-era legacy
    .apps_implausibility_recovery_threshold  = 5, //100, // [%] same note as above

    /* ========================= 8-bit fields ========================= */
    .torque_limit_source_mask = 0, // [bitmask]
    .num_driving_modes        = 3,// 3, // [count]
    .period                   = 10, // [ms] DL period setting (task_period currently used separately)
	/* =============================================================================
	 * Adaptive Pedal Map Tuning Guide (Per Driving Mode)
	 *
	 * rpm_fade        : [low → high]
	 *                   low  → aggressive launch, little low-speed smoothing
	 *                   high → softer launch, more low-speed torque reduction
	 *
	 * coast_p_low     : [0.0 → 0.2]
	 *                   low  → torque engages immediately at standstill
	 *                   high → larger “coast” region at low speed
	 *
	 * coast_p_high    : [0.0 → 0.1]
	 *                   low  → responsive once rolling
	 *                   high → softer pedal at moderate speed
	 *
	 * coast_rpm_start : [rpm]
	 *                   speed below which maximum coast window is applied
	 *
	 * coast_rpm_end   : [rpm]
	 *                   speed above which minimum coast window is applied
	 *
	 * soften_gain     : [0.0 → 1.0]
	 *                   low  → no high-RPM torque reduction
	 *                   high → stronger torque reduction near max_rpm
	 *
	 * soften_rpm      : [rpm]
	 *                   RPM where high-speed softening begins
	 *
	 * max_rpm         : [rpm]
	 *                   RPM where softening reaches full effect
	 *
	 * base_slope      : [<1.0 → >1.0]
	 *                   low  → globally softer pedal
	 *                   high → globally more aggressive pedal
	 *
	 * kVal            : [0.0 → 1.0]
	 *                   low  → smoother torque rise (more filtering)
	 *                   high → faster torque response
	 * =============================================================================
	 */
    .dmodes = {
                [0] = {
                	.dm_name = "Cool EXP 1",
                .control_map_fn = DL_MAP_EXP,
                    .kVal = 0.85f,
                    .max_acc_pwr = 75000,      // [W] mode operating ceiling (active cap uses min(global, mode))
                    .max_motor_torque = 220,   // [Nm] mode operating ceiling (active cap uses min(global, mode))
                    .adaptive_settings = {
                            .soften_gain = 0.08f,
                            .soften_rpm  = 5200,
                            .max_rpm     = 6500,
                            .offset      = 0,
                            .base_slope  = 1.0f, // currently unused by active map code (reserved for future shaping)
                            .coast_rpm_start = 0,
                            .coast_rpm_end   = 700,
                            .coast_p_low     = 0.01f,
                            .coast_p_high    = 0.00f,
                    },
                    .map_fn_params.exp = {
                        .s = 0.90f, // exponent used by EXP map: f(x) = x^s
                    },
            },

                [1] = {
                	.dm_name = "Cooler then your EX",
                .control_map_fn = DL_MAP_EXP,
                    .kVal = 0.60f,
                    .max_acc_pwr = 40000,      // [W] mode operating ceiling (active cap uses min(global, mode))
                    .max_motor_torque = 160,   // [Nm] mode operating ceiling (active cap uses min(global, mode))
                    .adaptive_settings = {
                            .soften_gain = 0.20f,
                            .soften_rpm  = 4800,
                            .max_rpm     = 6500,
                            .offset      = 0,
                            .base_slope  = 1.0f, // currently unused by active map code (reserved for future shaping)
                            .coast_rpm_start = 0,
                            .coast_rpm_end   = 1400,
                            .coast_p_low     = 0.03f,
                            .coast_p_high    = 0.015f,
                    },
                    .map_fn_params.exp = {
                        .s = 1.0f, // exponent used by EXP map: f(x) = x^s
                    },
            },

                [2] = {
                	.dm_name = "THE CUBE",
                	.control_map_fn = DL_MAP_CUBIC,
                    .kVal = 0.45f,
                    .max_acc_pwr = 10000,      // [W] mode operating ceiling (active cap uses min(global, mode))
                    .max_motor_torque = 100,   // [Nm] mode operating ceiling (active cap uses min(global, mode))
                    .adaptive_settings = {
                        .soften_gain = 0.35f,
                        .soften_rpm  = 4200,
                        .max_rpm     = 6500,
                        .offset      = 0,
                        .base_slope  = 1.0f, // currently unused by active map code (reserved for future shaping)
                        .coast_rpm_start = 0,
                        .coast_rpm_end   = 2200,
                        .coast_p_low     = 0.05f,
                        .coast_p_high    = 0.025f,
                    },
                    .map_fn_params.exp = {
                        .s = 1.5f, // ignored in CUBIC mode; only used when control_map_fn = DL_MAP_EXP
                    },

                },
				[3]= {
                    .dm_name = "THE CUBE",
                	.control_map_fn = DL_MAP_CUBIC,
                    .kVal = 0.45f,
                    .max_acc_pwr = 30000,      // [W] mode operating ceiling (active cap uses min(global, mode))
                    .max_motor_torque = 150,   // [Nm] mode operating ceiling (active cap uses min(global, mode))
                    .adaptive_settings = {
                        .soften_gain = 0.35f,
                        .soften_rpm  = 4200,
                        .max_rpm     = 6500,
                        .offset      = 0,
                        .base_slope  = 1.0f, // currently unused by active map code (reserved for future shaping)
                        .coast_rpm_start = 0,
                        .coast_rpm_end   = 2200,
                        .coast_p_low     = 0.05f,
                        .coast_p_high    = 0.025f,
                    },
                    .map_fn_params.exp = {
                        .s = 1.5f, // ignored in CUBIC mode; only used when control_map_fn = DL_MAP_EXP
                    },
                }
				//[0] = {
//				.control_map_fn = DL_MAP_LINEAR,
//				.kVal = 0.65f,                 // quicker response than 0.5
//				.adaptive_settings = {
//				.rpm_fade        = 250,       // mild launch softening (or set 1 to “almost off”)
//				.coast_rpm_start = 0,
//				.coast_rpm_end   = 0,         // disable moving coast band
//				.coast_p_low     = 0.00f,     // no coast at any speed
//				.coast_p_high    = 0.00f,
//				 .soften_gain     = 0.00f,
//				                         .soften_rpm      = 4500,
//				                         .max_rpm         = 6500,
//				                         .offset          = 0,
//				                         .base_slope      = 1.0f,
//				                       },
//				                       .map_fn_params.linear = { .slope = 1.0f, .offset = 0,
//				                     },
				                 // optional c
		},
     // [struct array] mode table (optional / future)
};

// -----------------------------------------------------------------------------
//State variables used by the driving loop / filter
// -----------------------------------------------------------------------------
bool  is_accelerating = false; // [bool]
float T_PREV = 0.0f;           // [Nm] previous torque actually sent (post-limits)
float T_REQ  = 0.0f;           // [Nm] torque request from pedal map (pre-filter/limits)

//uint16_t throttle_percent_i = 0;
//uint16_t brake_percent_i = 0;

#define TORQUE_DECAY_STEP 2.5f //// Nm per loop step (adjust as needed)
#define THROTTLE_ZERO_THRESHOLD 0.01f // // Below this % throttle, we consider "off"
static bool sent_zero_torque = false; //// Track if we already dropped torque to 0

static bool torque_inhibit_active = false; PRIVILEGED_DATA // [bool] latched inhibit

static uint8_t __current_dmode = 1; PRIVILEGED_DATA//[Unitless] Index of current driving mode
SemaphoreHandle_t dmode_mutex = NULL; PRIVILEGED_DATA

//Macro to make the driving mode seem much simpler
#define driving_mode driving_args->dmodes[__current_dmode]

// -----------------------------------------------------------------------------
// Driver inactivity tracking (currently informational / future use)
// -----------------------------------------------------------------------------
#define INPUT_TIMEOUT_MS 500
#define THROTTLE_CHANGE_THRESHOLD 5.0f  // [%]
#define BRAKE_CHANGE_THRESHOLD    5.0f  // [%]

TickType_t last_driver_input_time = 0; // [RTOS ticks]
static float last_throttle_percent = 0.0f; // [%]
static float last_brake_percent    = 0.0f; // [%]

#ifdef DEBUG_DL
char dl_debug_printbuf[512] = {0};
char* dl_cur_printbuf = dl_debug_printbuf;
#endif

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static float calculateThrottlePercentage(uint16_t apps1, uint16_t apps2);
//static float calculateBrakePercentage(uint16_t bps1);

void print_fixed_d(const char* label, int32_t value, int decimals, const char* unit);
static inline float dl_getOmegaRadS(int16_t rpm);

static bool  performSafetyChecks(driving_loop_args* dl_params,
                                 uint16_t apps1_value,
                                 uint16_t apps2_value,
                                 uint16_t bps1_value,
                                 uint16_t bps2_value,
                                 DL_internal_state_t* dl_status);

static inline float getKValue();
static inline float applyTorqueFilter(float T_req, float T_prev, bool is_accelerating);

static inline float dl_clampf(float x, float lo, float hi);
static inline float dl_slewLimit(float target, float prev, float rate_nm_per_s, float dt_s);

static bool  bms_is_ok(void);
static float torqueCapFromBMS(float omega_rad_s);
static float limitTorque(float T_cmd, float T_prev, const driving_loop_args* dl, float dt_s, drivingMode* dm);
static float torqueCapFromAbsPower(float omega_rad_s, const driving_loop_args* dl,const drivingMode* dm);


// -----------------------------------------------------------------------------
// initDrivingLoop(): sets up DAQ associations and creates the driving loop task
// -----------------------------------------------------------------------------
enum uv_status_t initDrivingLoop(void *argument)
{
    (void)argument;
    extern int16_t mc_speed_rpm;

    dmode_mutex = xSemaphoreCreateMutex();

    // Associate DAQ parameters with live ADC variables (ADC counts)
    associateDaqParamWithVar(APPS1_ADC_VAL, &adc1_APPS1); // [ADC counts]
    associateDaqParamWithVar(APPS2_ADC_VAL, &adc1_APPS2); // [ADC counts]
    associateDaqParamWithVar(BPS1_ADC_VAL,  &adc1_BPS1);  // [ADC counts]
    associateDaqParamWithVar(BPS2_ADC_VAL,  &adc1_BPS2);  // [ADC counts]

    associateDaqParamWithVar(MOTOR_RPM, &mc_speed_rpm);

    associateDaqParamWithVar(APPS_PERCENT, &g_throttle_percent);
    associateDaqParamWithVar(BPS_PERCENT, &g_brake_percent);

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

uv_status cycleDmode(){
	if(dmode_mutex == NULL){
		return UV_ERROR;
	}

	if(xSemaphoreTake(dmode_mutex,2) == pdTRUE){
		__current_dmode = (__current_dmode + 1)%(current_vehicle_settings->driving_loop_settings->num_driving_modes);
		xSemaphoreGive(dmode_mutex);
	}else{
		return UV_ABORTED;
	}

	return UV_OK;
}

// -----------------------------------------------------------------------------
// Linear Pedal map (DRIVE ONLY – NO REGEN)
// throttle_percent: [%] 0..100
// returns: torque request [Nm]
// -----------------------------------------------------------------------------
//static float mapThrottleToTorqueLinear(float throttle_percent,
//                                       float T_max,
//                                       const driving_loop_args* dl)
//{
//    float apps = dl_clampf(throttle_percent / 100.0f, 0.0f, 1.0f);
//
//    const float dead = dl_clampf(dl->throttle_deadband_percent / 100.0f, 0.0f, 0.9f);
//
//    float x = (apps - dead) / (1.0f - dead);
//    x = dl_clampf(x, 0.0f, 1.0f);
//
//    return T_max * x; // [Nm]
//}

/*
 * Adaptive Torque Map (Drive Only)
 *
 * 0) Normalize throttle (0–100% → 0–1) and remove deadband.
 *
 * 1a) Speed-dependent pedal shift ("moving map"):
 *      - Compute a zero-torque pedal threshold based on RPM.
 *      - Low RPM  → larger zero-torque region.
 *      - High RPM → smaller zero-torque region.
 *      - If pedal <= threshold → 0 Nm.
 *      - Otherwise shift + rescale remaining pedal travel to 0–1.
 *
 * 1b) Apply pedal shaping (Linear / Exp / Cubic)
 *      - Shapes driver feel only.
 *      - Does NOT enforce torque limits.
 *
 * 3) Optional high-RPM softening:
 *      - Above soften_rpm, torque is gradually reduced toward max_rpm.
 *
 * 4) Final torque request:
 *      T_req = T_allow * shaped_pedal
 *
 * Result:
 *      Speed-aware pedal behavior with mode-dependent feel,
 *      while torque ceilings are enforced upstream.
 */
static float mapAdaptiveFromMode(float throttle_percent, float T_max,
                                 const driving_loop_args* dl,
                                 const drivingMode* dm)
{
    /*
     * STEP 0: Normalize throttle input to 0–1 with deadband
     * apps = throttle as fraction (0–1)
     * dead = deadband fraction (0–1)
     * After deadband removal:
     *   x = 0     at bottom of pedal
     *   x = 1     at full pedal
     */
    float apps = dl_clampf(throttle_percent / 100.0f, 0.0f, 1.0f);
    const float dead = dl_clampf(dl->throttle_deadband_percent / 100.0f, 0.0f, 0.9f);
    float x = (apps - dead) / (1.0f - dead);
    x = dl_clampf(x, 0.0f, 1.0f);
    // If below deadband, request zero torque immediately
    if (x <= 0.0f) return 0.0f;

    /* =======================================================================
     * STEP 1A: Speed-Dependent Pedal Map Shift ("Moving Map")
     *
     * The pedal has a coasting region (zero torque zone) that depends on speed.
     *
     *  - At low RPM → larger zero-torque region (more pedal travel = coast)
     *  - At higher RPM → smaller zero-torque region (torque engages sooner)
     *
     * We:
     *   1) Compute the speed-dependent zero-torque threshold
     *   2) If pedal is inside that region → return 0 torque
     *   3) Otherwise shift and rescale pedal so remaining travel maps 0–1
     * ======================================================================= */

    extern int16_t mc_speed_rpm;

    /* Use absolute RPM (forward/reverse behave the same for shaping) */
    float vehicle_rpm = (float)mc_speed_rpm;
    if (vehicle_rpm < 0.0f)
        vehicle_rpm = -vehicle_rpm;


    /* === Per-mode tuning parameters === */

        // [Ratio 0.0 to 1.0] The "Deadzone" size at 0 RPM (e.g., 0.05 = 5%).
        // This provides a safety buffer so the car doesn't "creep" at a standstill.
        float zero_torque_at_low_speed = dm->adaptive_settings.coast_p_low;

        // [Ratio 0.0 to 1.0] The "Deadzone" size at high speed (e.g., 0.02 = 2%).
        // Shrinking the deadzone here makes the car feel more responsive once moving.
        float zero_torque_at_high_speed = dm->adaptive_settings.coast_p_high;

        // [RPM] The motor speed where the map starts moving (usually 0 RPM).
        float low_speed_rpm =(float)dm->adaptive_settings.coast_rpm_start;

        // [RPM] The motor speed where the map reaches its final "high speed" shape (e.g., 1200 RPM).
        float high_speed_rpm =(float)dm->adaptive_settings.coast_rpm_end;

        /* Keep values safe (Clamps to 0.0-0.9 range to ensure the pedal always works) */
        zero_torque_at_low_speed  = dl_clampf(zero_torque_at_low_speed,  0.0f, 0.9f);
        zero_torque_at_high_speed = dl_clampf(zero_torque_at_high_speed, 0.0f, 0.9f);

        /* === Compute speed-dependent zero-torque threshold === */

        /* THE ZERO TORQUE THRESHOLD:
         * This is the "Live Edge." It is the specific point on the pedal travel where
         * the motor transitions from doing nothing (coasting) to applying torque.
         */
        float zero_torque_threshold = zero_torque_at_high_speed;

        /* CHECK: (high_speed_rpm > low_speed_rpm)
         * This ensures the tuning range is valid to avoid dividing by zero in the math below.
         */
        if (high_speed_rpm > low_speed_rpm){

            /* THE SPEED FRACTION (The "Where am I?" Variable):
             * Tells us how far we are in the transition from 0 to 1200 RPM.
             * 0.0 = At/below Start RPM. 0.5 = Halfway (600 RPM). 1.0 = At/above End RPM.
             */
            float speed_fraction =(vehicle_rpm - low_speed_rpm) /(high_speed_rpm - low_speed_rpm);
            speed_fraction = dl_clampf(speed_fraction, 0.0f, 1.0f);

            /* INTERPOLATE:
             * Slides the threshold between Low (5%) and High (2%) based on the speed_fraction.
             * As you speed up, the threshold (deadzone) gets smaller.
             */
            zero_torque_threshold =(1.0f - speed_fraction) * zero_torque_at_low_speed + speed_fraction * zero_torque_at_high_speed;
        }


        /* === Apply coasting region === */

        /* THE COAST GATE:
         * If current pedal position (x) is below the threshold we just calculated, force 0 Nm.
         * This is the "Electronic Coast" zone where the car rolls freely.
         */
        if (x <= zero_torque_threshold)
            return 0.0f; // [Nm] Return zero torque


        /* THE SHIFT AND RESCALE:
         * Stretches the remaining pedal travel (from threshold to 100%) back to a 0.0-1.0 range.
         * This prevents a "jump" in torque when you cross the threshold.
         * Formula: (Foot_Position - Deadzone) / (Available_Pedal_Range)
         */
        float shifted_pedal = (x - zero_torque_threshold) / (1.0f - zero_torque_threshold);


        shifted_pedal = dl_clampf(shifted_pedal, 0.0f, 1.0f);

        /* Use shifted_pedal (x) for the final torque curves (Linear, Exponential, etc.) */
        x = shifted_pedal;

    /*
     * STEP 1B : Pedal curve shaping  f(x)
     * curve_type selects how throttle maps to torque fraction:
     * 0 = Linear        f(x) = x
     * 1 = Power-law     f(x) = x^s
     * 2 = Smoothstep    f(x) = 3x^2 - 2x^3
     * These only shape driver "feel".
     * They do NOT enforce limits.
     */

    float f = x;  // default linear behavior

    //pick your fighter
    const uint8_t curve_type = dm->control_map_fn;

    if (curve_type == DL_MAP_LINEAR) {
        // Linear: direct proportional mapping
        // f(x) = x
        // Already assigned above
        /* =======================================================================
         * STEP 2: TU/e Paper Adaptive Low-Speed Fade
         * * Instead of reducing torque at high speed (softening), this reduces
         * sensitivity at low speed to prevent "jerky" launches.
         * ======================================================================= */

    //    // 1. Get the tuning constant (in RPM)
    //    float v_fade = (float)dm->adaptive_settings.rpm_fade;
    //
    //    // 2. Use absolute RPM for the calculation
    //    float vehicle_rpm = fabsf((float)mc_speed_rpm);
    //
    //    // 3. The "Launch Torque" Epsilon
    //    // In the paper's theory, if RPM=0, Torque=0 (the car would never move).
    //    // We use a small epsilon to ensure the car can actually start.
    //    const float v_epsilon = 10.0f; // [rpm] Adjust this for "bite" off the line
    //    float v_for_calc = (vehicle_rpm < v_epsilon) ? v_epsilon : vehicle_rpm;
    //
    //    // 4. Calculate the Paper's Fade Factor: v / (v + v_fade)
    //    // Example: if v_fade = 250, at 250 RPM you get 50% torque. At 2500 RPM you get 91%.
    //    float paper_fade = v_for_calc / (v_for_calc + v_fade);
    //
    //    // 5. Final Safety Clamp (0.0 to 1.0)
    //    paper_fade = dl_clampf(paper_fade, 0.0f, 1.0f);
    //
    //    // 6. Apply to the torque request
    //    T_req *= paper_fade;
    }
    else if (curve_type == DL_MAP_EXP) {
        // Power-law: softens low pedal, ramps harder near the top
        // s > 1.0 → softer initial response
        // s = 1.0 → linear
        float s = dm->map_fn_params.exp.s;
        // Prevent degenerate exponent
        if (s < 0.1f) s = 0.1f;
        f = powf(x, s);
    }
    else {
        // Smooth step cubic: OEM-style S-curve
        // f(x) = 3x^2 - 2x^3
        // - zero slope at x=0
        // - zero slope at x=1
        // - smooth, progressive feel
        f = (3.0f * x * x) - (2.0f * x * x * x);

        /*
         * Optional true sigmoid shape (commented out on purpose):
         * - Uses base_slope as "steepness" (k)
         * - k ~ 4..12 is a practical tuning range
         * - Normalized so f(0)=0 and f(1)=1
         *
         * float k = dl_clampf(dm->adaptive_settings.base_slope, 4.0f, 12.0f);
         * float s0 = 1.0f / (1.0f + expf( 0.5f * k));
         * float s1 = 1.0f / (1.0f + expf(-0.5f * k));
         * float sx = 1.0f / (1.0f + expf(-(x - 0.5f) * k));
         * f = (sx - s0) / (s1 - s0);
         */
    }

#ifdef DEBUG_DL
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf, "DL Power Proportion:",f*1000,3,"");
#endif

    /* =======================================================================
     * STEP 2: Optional Adaptive Low-Speed Fade
     * Research Paper equation:
     *     T_req(p, v) = p * T_max(v) * ( v / (v + v_fade) )
     Implementation:
     *     p        → f      (pedal fraction after shaping, 0..1)
     *     T_max(v) → T_max  (torque ceiling already computed upstream)
     *     v        → rpm    (using motor speed as velocity proxy)
     *     v_fade   → rpm_fade (tuning constant in rpm)
     *
     * Meaning of rpm_fade:
     *     rpm = rpm_fade  → torque is reduced to 50%
     *     rpm >> rpm_fade → fade ≈ 1 (no reduction)
     *     rpm ≈ 0         → fade ≈ 0 (strong reduction)
     *
     * Purpose:
     *     Reduce torque sensitivity at very low speed to prevent
     *     aggressive launch jerk or wheelspin.
     *
     * Important edge case:
     *     If rpm == 0 exactly, the equation gives fade = 0,
     *     which would command zero torque regardless of pedal.
     *
     *     To prevent a “dead pedal at standstill” condition,
     *     we clamp the speed used in the fade equation to a
     *     small epsilon value.
     *
     *     This preserves the shape of the paper equation while
     *     guaranteeing nonzero launch torque.
     * ======================================================================= */

//    extern int16_t mc_speed_rpm;
//
//    /* Use magnitude of speed (reverse should behave same as forward) */
//    rpm = (float)mc_speed_rpm;
//    if (rpm < 0.0f)
//        rpm = -rpm;
//
//    /* Tuning constant:
//     * rpm_fade defines the speed where torque = 50% of requested value.
//     */
//    float rpm_fade = (float)dm->adaptive_settings.rpm_fade;
//
//    /* Prevent divide-by-zero or unstable behavior */
//    if (rpm_fade < 1.0f)
//        rpm_fade = 1.0f;
//
//    /* Small epsilon to prevent exact-zero speed from killing torque */
//    const float rpm_epsilon = 1.0f;  // [rpm]
//
//    /* Use epsilon-clamped speed for fade calculation */
//    float v = (rpm < rpm_epsilon) ? rpm_epsilon : rpm;
//
//    /* Paper fade function: v / (v + v_fade) */
//    float fade = v / (v + rpm_fade);
//
//    /* Safety clamp to valid 0..1 range */
//    fade = dl_clampf(fade, 0.0f, 1.0f);
//
//    /* Apply adaptive scaling to torque request */
//    float T_req = T_max * f * fade;

    float T_req = T_max * f;

    /*
     * STEP 3: Optional high-RPM softening
     Separate from adaptive fade.
     If rpm > soften_rpm:
     gradually reduce torque as rpm approaches max_rpm.
     Used to:
     - reduce harshness near top speed
     */

    float soften_gain = dm->adaptive_settings.soften_gain;
    float soften_rpm  = (float)dm->adaptive_settings.soften_rpm;
    float max_rpm     = (float)dm->adaptive_settings.max_rpm;

    if (soften_gain > 0.0f &&
        max_rpm > soften_rpm &&
        vehicle_rpm > soften_rpm)
    {
        float t = ((float)mc_speed_rpm - soften_rpm) / (max_rpm - soften_rpm);
        t = dl_clampf(t, 0.0f, 1.0f);

        T_req *= (1.0f - soften_gain * t);
    }

    /* Final safety clamp to allowed torque ceiling */
    float retval = dl_clampf(T_req, 0.0f, T_max);
    return retval;
}

// -----------------------------------------------------------------------------
// Adaptive POWER pedal map (drive only)
// throttle_percent: [%] 0..100
// T_allow:          [Nm] torque ceiling (already includes power caps etc.)
// returns:          [Nm] torque request (pre-filter / pre-slew)
// -----------------------------------------------------------------------------
//static float mapThrottleToTorquePower(float throttle_percent,
//                                      float T_allow,
//                                      const driving_loop_args* dl)
//{
//    // throttle -> 0..1
//    float apps = dl_clampf(throttle_percent / 100.0f, 0.0f, 1.0f);
//
//    // apply deadband
//    const float dead = dl_clampf(dl->throttle_deadband_percent / 100.0f, 0.0f, 0.9f);
//    float x = (apps - dead) / (1.0f - dead);
//    x = dl_clampf(x, 0.0f, 1.0f);
//
//    if (x <= 0.0f) return 0.0f;
//
//    // speed -> omega
//    extern int16_t mc_speed_rpm;
//    //float omega = fabsf(dl_getOmegaRadS(mc_speed_rpm)); // [rad/s]
//    float omega = fabsf(dl_getOmegaRadS(mc_speed_rpm)); // [rad/s]
//
//    // if low speed, behave like torque map (avoid divide by ~0)
//    if (omega < 10.0f) {
//        return T_allow * x; // [Nm]
//    }
//
//    // convert the torque ceiling back into a power ceiling
//    // P_allow = T_allow * omega  [W]
//    float P_allow = T_allow * omega; // [W]
//
//    // pedal requests power
//    float P_req = P_allow * x; // [W]
//
//    // convert requested power back to torque
//    float T_req = P_req / omega; // [Nm]
//
//    // final clamp to ceiling (belt + suspenders)
//    return dl_clampf(T_req, 0.0f, T_allow);
//}

// -----------------------------------------------------------------------------
// “Race mode” filter shaping (placeholder)
// -----------------------------------------------------------------------------
#define ACCELERATION 0
#define AUTOCROSS    1
#define ENDURANCE    2

//TODO should be part of DMODE
static inline float getKValue(){
    return driving_mode.kVal; // [0–1]
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
float calculateBrakePercentage(uint16_t bps1)
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

/**
 * @brief Applies first-order smoothing to torque requests.
 *
 * This function smooths torque rise to reduce driveline shock,
 * while allowing immediate torque drop for safety and responsiveness.
 *
 * Behavior:
 *  - If accelerating: applies configurable low-pass filter gain
 *  - If decelerating: allows immediate torque drop
 *  - Snaps to zero when request is exactly zero
 *
 * This function does NOT:
 *  - Enforce torque ceilings
 *  - Apply slew rate limiting
 *  - Gate torque based on system health
 *
 * @param T_req Current torque request [Nm]
 * @param T_prev Previously commanded torque [Nm]
 * @param is_accelerating True if torque is increasing
 * @return Filtered torque command [Nm]
 */
static inline float applyTorqueFilter(float T_req, float T_prev, bool is_accelerating)
{
    float FILTER_K = getKValue(); // [0–1] accel smoothing gain

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
    if (delta < -max_step) return prev - max_step; // [Nm] //NOTE: This is not rules compliant, I aint sayin shit tho
    return target;                                  // [Nm]
}

// -----------------------------------------------------------------------------
// BMS health gate
// return: true if BMS fresh + valid
// -----------------------------------------------------------------------------
static bool bms_is_ok(void)
{
    //const TickType_t now     = xTaskGetTickCount();  // [RTOS ticks]
    //const TickType_t timeout = pdMS_TO_TICKS(200);   // [RTOS ticks] (200ms)

    if (!is_bms_connected) return false;            // [bool]
    //if ((now - bms_last_msg_time) > timeout) return false; // [tick delta]

    if (msg1corrupt || msg2corrupt) return false;   // [bool-like]

    return true;
}

// -----------------------------------------------------------------------------
// BMS power-based torque cap
// T_cap = (Vpack * DCL) / omega
// omega: [rad/s]
// return: torque cap [Nm]
// -----------------------------------------------------------------------------
static float torqueCapFromBMS(float omega_rad_s)
{
    // packVoltage [0.1 V] -> [V]
    float V = packVoltage * 0.1f; // [V]

    // packDCL assumed [0.1 A] -> [A] (verify scaling from BMS message)
    //float I = packDCL * 0.1f;     // [A]
    float I = (float)packDCL;     // [A]

    float P_max = V * I;          // [W] electrical power cap

    // Avoid nonsense at low speeds
    omega_rad_s = fabsf(omega_rad_s);
    if (omega_rad_s < 10.0f) {    // [rad/s]
        return 1e9f;              // [Nm] effectively uncapped here
    }

    return P_max / omega_rad_s;   // [Nm] (since W / (rad/s) = N·m)
}

// -----------------------------------------------------------------------------
// Final torque conditioning before sending
// Applies:
//   - BMS gate
//   - Hard torque clamp
//   - Slew rate limiting
// return: torque to send [Nm]
// -----------------------------------------------------------------------------
static float limitTorque(float T_cmd, float T_prev, const driving_loop_args* dl, float dt_s, drivingMode* dm)
{
    // 0) Hard BMS safety gate
    if (!bms_is_ok()) {
        return 0.0f; // [Nm]
    }

    float T = T_cmd; // [Nm]

    // 1) Absolute motor torque clamp (hard cap)
    float T_Lim = fminf((float)dl->absolute_max_motor_torque,(float)dm->max_motor_torque);
    T = dl_clampf(T, 0.0f, T_Lim); // [Nm]

    // 2) Optional slew-rate limiting (Nm/s)
    float rate = (T >= T_prev) ? dl->torque_rate_up_nm_per_s
                               : dl->torque_rate_down_nm_per_s; // [Nm/s]

    //Slew rate limiting. Should slew rate be depent on dmode? IDK
    T = dl_slewLimit(T, T_prev, rate, dt_s); // [Nm]

    return T; // [Nm]
}

//Converts between rpm and rad/s
static inline float dl_getOmegaRadS(int16_t rpm)
{
    return ((float)rpm * 2.0f * 3.1415926f) / 60.0f; // [rad/s]
}

// -----------------------------------------------------------------------------
// Absolute software power cap
// T_cap = absolute_max_acc_pwr / omega
// omega: [rad/s]
// return: torque cap [Nm]
// -----------------------------------------------------------------------------
static float torqueCapFromAbsPower(float omega_rad_s, const driving_loop_args* dl, const drivingMode* dm)
{
    omega_rad_s = fabsf(omega_rad_s);

    /* OLD BEHAVIOR (kept as fallback/reference)
     * This selected min(global, mode) but later returned torque using GLOBAL only.
     */
    //uint32_t max_pwr = (dl->absolute_max_acc_pwr > dm->max_acc_pwr)? dm->max_acc_pwr : dl->absolute_max_acc_pwr;

    /* NEW BEHAVIOR
     * 0 means disabled for each source.
     * - only global enabled -> use global
     * - only mode enabled   -> use mode
     * - both enabled        -> use stricter (minimum)
     * - both disabled       -> no cap from this source
     */
    uint32_t global_pwr = dl->absolute_max_acc_pwr;
    uint32_t mode_pwr = (dm != NULL) ? dm->max_acc_pwr : 0u;
    uint32_t max_pwr = 0u;

    if (global_pwr == 0u) {
        max_pwr = mode_pwr;
    } else if (mode_pwr == 0u) {
        max_pwr = global_pwr;
    } else {
        max_pwr = (global_pwr < mode_pwr) ? global_pwr : mode_pwr;
    }
#ifdef DEBUG_DL
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf, "Power Limit:",max_pwr,0,"W");
#endif

    if (omega_rad_s < 10.0f) return 1e9f;          // avoid divide-by-zero region
    if (max_pwr == 0u) return 1e9f; // disabled

    /* OLD BEHAVIOR (kept as fallback/reference)
     * return ((float)dl->absolute_max_acc_pwr) / omega_rad_s;
     */

    return ((float)max_pwr) / omega_rad_s; // [Nm]
}


// -----------------------------------------------------------------------------
// Compute torque ceiling
// Combines:
//   - Absolute motor torque limit
//   - Absolute pack power cap (if enabled)
//   - BMS power cap (if BMS OK)
// return: max allowed torque [Nm]
//TODO make me driving mode!
// -----------------------------------------------------------------------------
static float dl_computeTorqueCeiling(const driving_loop_args* dl, const drivingMode* dm)
{
    extern int16_t mc_speed_rpm;

    // Speed -> omega (use magnitude for power math)
    float omega = fabsf(dl_getOmegaRadS(mc_speed_rpm)); // [rad/s]

#ifdef DEBUG_DL
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"MC Speed RPM: %d \t",mc_speed_rpm);
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf, "Omega",(omega*1000),3,"rad/s");
#endif
    // 0) Start from hard torque ceiling
    float T_allow = (float)dl->absolute_max_motor_torque; // [Nm]

    //Apply torque ceiling for specific driving mode
    if (dm && dm->max_motor_torque > 0u) {
        T_allow = fminf(T_allow, (float)dm->max_motor_torque);
    }



    // If we're basically stopped, stay torque-limited (avoid divide by ~0)
    if (omega < 10.0f) {
        return dl_clampf(T_allow, 0.0f, (float)dl->absolute_max_motor_torque);
    }

        // 1) Absolute/mode pack power cap (whichever is enabled, stricter wins)
        {
        /* OLD BEHAVIOR (kept as fallback/reference)
         * if (dl->absolute_max_acc_pwr > 0u) {
         *     float T_absP = torqueCapFromAbsPower(omega, dl, dm);
         *     T_allow = fminf(T_allow, T_absP);
         * }
         */

        float T_absP = torqueCapFromAbsPower(omega, dl, dm); //Takes into account dmode max power

#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_abs_pwr:",(T_absP*1000),3,"Nm");
#endif

        T_allow = fminf(T_allow, T_absP);
    }

    // 2) BMS power cap (V * DCL) if BMS OK
    if (bms_is_ok()) {
        float T_bms = torqueCapFromBMS(omega); // [Nm]

#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_BMS:",(T_bms*1000),3,"Nm");
#endif
        T_allow = fminf(T_allow, T_bms);
    }

    //TODO: Thermal derate?


    // Final sanity clamp
    return dl_clampf(T_allow, 0.0f, (float)dl->absolute_max_motor_torque);
}

// -----------------------------------------------------------------------------
// Driving Loop Task
// -----------------------------------------------------------------------------
void StartDrivingLoop(void *argument)
{
    uv_task_info* params = (uv_task_info*)argument; // [ptr task metadata]

    xSemaphoreTake(dmode_mutex,2);

    DL_internal_state_t dl_status = Plausible;   // [enum] plausibility state

    // Active driving-loop parameters (flash-configurable)
    driving_loop_args* dl_params = current_vehicle_settings->driving_loop_settings; // [ptr]
    drivingMode* cdm = &driving_mode;

    // Period handling
    TickType_t tick_period = pdMS_TO_TICKS(params->task_period); // [RTOS ticks] from [ms]
    TickType_t last_time   = xTaskGetTickCount();                // [RTOS ticks]
    last_driver_input_time = last_time;
    // [RTOS ticks]

    //brakelight status
    uint8_t bl_on = 0;

#ifdef DEBUG
    //uint32_t exec_time_us = 0;
#endif


    bool safe = true;

    if(uvEnableTraction()!= UV_OK){
    	//Hmmm interesting
    }

    vTaskDelay(500);

    coniferEnChannel(COOLANT_PUMP1);

    for (;;)
    {
        // Task control (kill/suspend)
        if (params->cmd_data == UV_KILL_CMD) {
        	coniferDisChannel(COOLANT_PUMP1);
        	uvDisableTraction();
        	xSemaphoreGive(dmode_mutex);
            killSelf(params);
        } else if (params->cmd_data == UV_SUSPEND_CMD) {
        	coniferDisChannel(COOLANT_PUMP1);
        	uvDisableTraction();
        	xSemaphoreGive(dmode_mutex);
            suspendSelf(params);
        }

#ifdef DEBUG_DL
        dl_cur_printbuf = dl_debug_printbuf;
#endif

        // Run at fixed interval
        vTaskDelayUntil(&last_time, tick_period); // [ticks]


        tic();

        // Snapshot ADC values so mid-loop changes don’t produce mixed samples

        taskENTER_CRITICAL();//Critical section to avoid bullshit interrupting this
        const uint16_t apps1_value = adc1_APPS1; // [ADC counts]
        const uint16_t apps2_value = adc1_APPS2; // [ADC counts]
        const uint16_t bps1_value  = adc1_BPS1;  // [ADC counts]
        const uint16_t bps2_value  = adc1_BPS2;  // [ADC counts]
        taskEXIT_CRITICAL();

        float T_filtered = 0.0f; // [Nm]

        float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value); // [%]
        float brake_percent    = calculateBrakePercentage(bps1_value);  // [%]

        g_throttle_percent = (uint8_t)throttle_percent;
        g_brake_percent    = (uint8_t)brake_percent;


        if((brake_percent > 10) && (bl_on == 0)){
        	coniferEnChannel(BRAKE_LIGHT);
        	bl_on = 1;
        }else if((brake_percent < 8) && (bl_on == 1)){
        	coniferDisChannel(BRAKE_LIGHT);
        	bl_on = 0;
        }


        //This code is responsible for exiting driving mode, and reverting to ready state
        if((brake_percent > 10.0)&&(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0))){
        	extern int16_t mc_speed_rpm;
        	if(mc_speed_rpm == 0){
        		changeVehicleState(UV_READY);
        	}
        }

//        int a = *((int*)0x08200000); //Deliberately trigger hardfault
//        a++;


        if(!safe){
        	T_filtered = 0.0f;                   // [Nm]
        	sendTorqueToMotorController(T_filtered); // expects torque [Nm] at API boundary

        	//Throttle less than 5% -> safe
        	if(throttle_percent < 5.0){
        		safe = true;
        	}else{
        		continue;
        	}

        }

        // 1) Safety checks first (plausibility / bounds / inhibit logic)
        safe = performSafetyChecks(dl_params,
                                        apps1_value, apps2_value, // [ADC counts]
                                        bps1_value, bps2_value,   // [ADC counts]
                                        &dl_status);              // [enum out]

        if (!safe) {
            T_filtered = 0.0f;                   // [Nm]
            sendTorqueToMotorController(T_filtered); // expects torque [Nm] at API boundary
            continue;
        }

#ifdef DEBUG_DL
        debugWrite(dl_debug_printbuf,DEBUG_PORT_TRACTIVE);
        dl_cur_printbuf = dl_debug_printbuf;
#endif

        // 2) Convert raw ADC -> pedal percentages
        //float throttle_percent = calculateThrottlePercentage(apps1_value, apps2_value); // [%]
        //float brake_percent    = calculateBrakePercentage(bps1_value);                  // [%]

        // 3) Track significant driver input changes (used for future watchdog / inactivity)
        float throttle_delta = fabsf(throttle_percent - last_throttle_percent); // [%]
        float brake_delta    = fabsf(brake_percent - last_brake_percent);       // [%]

        if (throttle_delta > THROTTLE_CHANGE_THRESHOLD || brake_delta > BRAKE_CHANGE_THRESHOLD) {
            last_driver_input_time = xTaskGetTickCount(); // [ticks]
            last_throttle_percent  = throttle_percent;    // [%]
            last_brake_percent     = brake_percent;       // [%]
        }

        // 4) Pedal map: throttle [%] -> torque request [Nm]
        //TODO: Tmax should be dependent on speed and such
        //TODO: Does not take into account different Dmodes

        //T_REQ = mapThrottleToTorqueAdaptive(throttle_percent, dl_params); // [Nm]
        float T_allow = dl_computeTorqueCeiling(dl_params,cdm);

#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_allow", (int32_t)(T_allow*1000), 3, "Nm");
#endif

        //T_REQ = mapThrottleToTorqueAdaptive(throttle_percent, T_allow, dl_params); //[Nm]

        //FOR DRIVING MODE
//        const drivingMode* dm = &dl_params->dmodes[__current_dmode];
//        T_REQ = mapThrottleToTorqueFromMode(throttle_percent, dl_params, dm);

        T_REQ = mapAdaptiveFromMode(throttle_percent,T_allow, dl_params,cdm); //[Nm] 

        //Temporary:
        //T_REQ = T_REQ/2.0f;
#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_REQ", T_REQ*1000, 3, "Nm");
#endif
        // Determine ramp direction for filter selection
        is_accelerating = (T_REQ >= T_PREV); // [bool]

        // Apply filter: keeps drop instant and rise smoothed
        T_filtered = applyTorqueFilter(T_REQ, T_PREV, is_accelerating); // [Nm]

#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_filtered", T_filtered*1000, 3, "Nm");
#endif
        // Bring-up scaling: halves torque before limits (temporary)
        //T_filtered = T_filtered / 2.0f; // [Nm]

        // dt from task period (ms -> s)
        float dt_s = (float)params->task_period / 1000.0f; // [s]

        // Apply limit stack (BMS gate, hard torque cap, slew, power envelope)
        float T_to_send = limitTorque(T_filtered, T_PREV, dl_params, dt_s, cdm); // [Nm]

#ifdef DEBUG_DL
        dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"T_to_send", T_to_send*1000, 3, "Nm");
#endif
        // Only allow torque output in the driving state
        if (vehicle_state == UV_DRIVING) {
            sendTorqueToMotorController(T_to_send); // [Nm]
        } else {
            sendTorqueToMotorController(0.0f);      // [Nm]
        }

        // Save post-limit torque for next loop iteration
        T_PREV = T_to_send; // [Nm]

#ifdef DEBUG_DL
        debugWrite(dl_debug_printbuf,DEBUG_PORT_TRACTIVE);
#endif

        //exec_time_us = toc();
        //printf("DL ex time: %d \n",exec_time_us);

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

#ifdef DEBUG_DL
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"APPS1 Value: %d\t",apps1_value);
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"APPS1 Percent",(uint32_t)(apps1_ratio*10000),2,"%");
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"APPS2 Value: %d\t",apps2_value);
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"APPS2 Percent",(uint32_t)(apps2_ratio*10000),2,"%");
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"APPS Delta",(uint32_t)(apps_diff_percent*100),2,"%");
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"BPS1 Value: %d\t",bps1_value);
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"BPS2 Value: %d\t",bps2_value);
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"Brake Percent",(uint32_t)(brake_percent*100),2,"%");
#endif

    // --- Absolute bounds checks (raw ADC safety) ---
    // If any sensor violates its absolute bounds, torque is inhibited immediately.
    if (apps1_value < dl_params->apps1_abs_min_val || apps1_value > dl_params->apps1_abs_max_val) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]

#ifdef DEBUG_DL
        printf("APPS1 Out of bounds\n");
#endif
        return false;
    }
    if (apps2_value < dl_params->apps2_abs_min_val || apps2_value > dl_params->apps2_abs_max_val) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]

#ifdef DEBUG_DL
        printf("APPS2 Out of bounds\n");
#endif
        //printf("APPS2 Value: %d\n",apps2_value);
        return false;
    }
    if (bps1_value < dl_params->min_BPS_value || bps1_value > dl_params->max_BPS_value) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        printf("BPS1 Out of bounds\n");

        return false;
    }
    if (bps2_value < dl_params->min_BPS_value || bps2_value > dl_params->max_BPS_value) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Erroneous;       // [enum]
        printf("BPS2 Out of bounds\n");
        //printf("BPS2 Value: %d\n",bps2_value);
        return false;
    }

    // --- APPS plausibility mismatch check ---
    // apps_diff_percent is in [%], threshold is also expected [%]
    if (apps_diff_percent > (float)dl_params->apps_plausibility_check_threshold) {
        torque_inhibit_active = true; // [bool]
        *dl_status = Implausible;     // [enum]
        printf("APPS1 Percentage Delta Too High\n");
        //printf("APPS1 Value: %d\n",apps_diff_percent);
        return false;
    }

    // --- Brake + Throttle torque inhibit ---
    // throttle_percent and brake_percent are in [%]
    // NOTE: these values should ideally use dl_params->torque_inhibit_* thresholds
    //if (throttle_percent > 25.0f && brake_percent > 5.0f) {
    if (throttle_percent > dl_params->torque_inhibit_apps_percent && brake_percent    > dl_params->torque_inhibit_bps_percent){
        torque_inhibit_active = true; // [bool]
        *dl_status = Implausible;     // [enum]
        printf("Simultaneous throttle and brake inputs\n");
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
