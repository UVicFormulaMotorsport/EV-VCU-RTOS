/* motor_controller.c */

#define __UV_FILENAME__ "motor_controller.c"

//#include "motor_controller.h"
//#include "can.h"           // For uvSendCanMSG, uv_CAN_msg, etc.
//#include "cmsis_os.h"      // For vTaskSuspend
//#include "FreeRTOS.h"
//#include "task.h"
//#include "uvfr_utils.h"    // For uvPanic, etc.
//#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include "uvfr_settings.h"
//#include "cmsis_os.h"      // For vTaskSuspend

#include "uvfr_utils.h"

extern uv_vehicle_settings* current_vehicle_settings;
//extern QueueHandle_t CAN_Rx_Queue;

// Redirect all mc_settings.x to actual config struct
#define mc_settings (current_vehicle_settings->mc_settings)
//register for motor controller errors and warnings so we can make them cyclic
//#define motor_controller_errors_warnings 0x82


//global variable for the last mc responce
uv_CAN_msg last_mc_response;
//global variable for timeout
//TickType_t last_driver_input_time = 0;

//cyclic parameters
int16_t mc_speed_rpm = 0;
int16_t mc_current = 0;
int16_t mc_torque_cmd = 0;
int16_t mc_motor_temp = 0;
int16_t mc_igbt_temp = 0;

//Masks between errors and warnings
uint16_t mc_error_mask = 0;
uint16_t mc_warning_mask = 0;



/* Global default settings variable defined here.
 * This uses the motor_controller_settings definition from uvfr_settings.h.
 */
motor_controller_settings mc_default_settings = {
    .can_id_tx              = 0x201,
    .can_id_rx              = 0x181,
    .mc_CAN_timeout         = 2,
//    .proportional_gain      = 10,   // uint8_t
//    .integral_time_constant = 400,  // uint32_t
//    .integral_memory_max    = 60,    // uint8_t (represents 60%)

    // Scaled values (normalized to 32767)
	//TODO: make these values unscaled, in units
    .max_speed              = 32767, //12357,   // (2457.5 RPM / 6500 RPM) * 32767
    .max_current            = 32767, //2600,   // 140 Nm --> I = T / kt = 140 / 0.94 = 148.9 A --> DIG CURRENT LIMIT (148.9 A / 250 A) * 32767 = 19516
	.iq_fullscale_arms		= 250,	   // FULL ALLOWABLE CURENT [Arms]
    .cont_current           = 32767,    // (60 A / 250 A) * 32767
    .max_torque             = 32767,   // Full scale = 230 Nm = 32767
    .max_motor_temp         = 32767,   // 120 °C → full scale (as per 0xA3 field)
	.warning_motor_temp		= 32767,	//120 °C → full scale (as per 0xA2 field)

	.mc_bus 				= CAN_BUS_1,
	// current control
	.cc_kp    				= 20,		//Kp (0..200) "Num" register 0x1C
	.cc_ti    				= 600,		//Ti (ms) 			register 0x1D
	.cc_tim   				= 100,		//TiM (%) 			register 0x2B
	.cc_xkp2  				= 0,		//xKP2 				register 0xC9
	.cc_kf    				= 0,		//Kf				register 0xCB
	.cc_ramp  				= 2000,		//Ramp (us) 		register 0x25
	// optional derating knobs
	.imax_pk  				= 100,		//I max pk (%) or scaled A  register 0xC4
	.icon_eff 				= 100,		//I con eff (Arms or %) 	register 0xC5
	.t_peak2  				= 5,		//Topeak2 (s) 				register 0xF0
};

void print_fixed_d(const char* label, int32_t value, int decimals, const char* unit);

#ifdef DEBUG_DL
extern char dl_debug_printbuf[256];
extern char* dl_cur_printbuf;
#endif

/**
 * @brief Configure the Bamocar current controller (PI + feedforward + ramp).
 *
 * This function sets all primary current control tuning parameters used by the
 * Bamocar inverter. These parameters define the closed-loop behavior of the
 * motor current controller and directly affect torque response, stability,
 * and transient behavior.
 *
 * Intended use:
 *  - Called during initialization with conservative defaults, OR
 *  - Called at runtime from a desktop tuning application over CAN
 *
 * Parameters (see Unitek documentation – Current Control):
 *  - kp     : Proportional gain (MC_REG_KP)
 *  - ti     : Integral time constant [ms] (MC_REG_TI)
 *  - tim    : Max integral memory [%] (MC_REG_TIM)
 *  - xkp2   : Gain multiplier at high current [%] (MC_REG_XKP2)
 *  - kf     : Current feedforward gain (MC_REG_KF)
 *  - ramp   : Current ramp rate [µs] (MC_REG_RAMP)
 *
 * All values are written directly to the inverter registers using MC_Set_Param().
 * No scaling is performed here — the caller is responsible for providing values
 * in the exact units and ranges expected by the Bamocar.
 *
 * @return UV_OK if all parameters are written successfully,
 *         UV_ERROR if any register write fails.
 *
 * Safety note:
 *  This function does NOT enable the motor or apply torque. It only updates
 *  controller parameters and is safe to call while the motor is disabled.
 */

uv_status MC_SetCurrentControlParams(uint16_t kp, uint16_t ti, uint16_t tim,
                                     uint16_t xkp2, uint16_t kf, uint16_t ramp)
{
    if (MC_Set_Param(MC_REG_KP, kp) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_TI, ti) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_TIM, tim) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_XKP2, xkp2) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_KF, kf) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_RAMP, ramp) != UV_OK) return UV_ERROR;
    return UV_OK;
}

/**
 * @brief Configure current limits and thermal derating behavior.
 *
 * This function sets all current limit and derating parameters that protect
 * the motor, inverter, and wiring from overcurrent and overheating.
 *
 * These parameters define:
 *  - Peak vs continuous current limits
 *  - Allowed duration of overcurrent events
 *  - Automatic current reduction based on speed, motor temperature,
 *    and inverter temperature
 *
 * Intended use:
 *  - Set once during startup based on hardware limits, OR
 *  - Tuned via a desktop application during validation and testing
 *
 * Parameters (see Unitek documentation – Current Derating):
 *  - imax_pk  : Peak current limit [% of device max] (MC_REG_IMAX_PK)
 *  - icon_eff : Continuous current limit [% of device max] (MC_REG_ICON_EFF)
 *  - tpeak2   : Max duration of peak current [s] (MC_REG_TPEAK2)
 *
 *  - ilim_dig : Digital input-based current reduction [%] (MC_REG_ILIM_DIG)
 *  - ired_n   : Speed-based current reduction [%] (MC_REG_IRED_N)
 *
 *  - ired_td  : Start of inverter temperature derating (MC_REG_IRED_TD)
 *  - ired_te  : End of inverter temperature derating (MC_REG_IRED_TE)
 *  - ired_tm  : Start of motor temperature derating (MC_REG_IRED_TM)
 *
 * All values are written directly to the inverter registers using MC_Set_Param().
 * No scaling or validation is performed here — the caller must ensure values
 * are within safe and documented limits.
 *
 * @return UV_OK if all parameters are written successfully,
 *         UV_ERROR if any register write fails.
 *
 * Safety note:
 *  These limits are enforced internally by the inverter and act as a final
 *  layer of protection even if higher-level software misbehaves.
 */

uv_status MC_SetDeratingParams(uint16_t imax_pk, uint16_t icon_eff, uint16_t tpeak2,
                               uint16_t ilim_dig, uint16_t ired_n,
                               uint16_t ired_td, uint16_t ired_te, uint16_t ired_tm)
{
    if (MC_Set_Param(MC_REG_IMAX_PK, imax_pk) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_ICON_EFF, icon_eff) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_TPEAK2, tpeak2) != UV_OK) return UV_ERROR;

    if (MC_Set_Param(MC_REG_ILIM_DIG, ilim_dig) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_IRED_N, ired_n) != UV_OK) return UV_ERROR;

    if (MC_Set_Param(MC_REG_IRED_TD, ired_td) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_IRED_TE, ired_te) != UV_OK) return UV_ERROR;
    if (MC_Set_Param(MC_REG_IRED_TM, ired_tm) != UV_OK) return UV_ERROR;
    return UV_OK;
}



/**
 * @brief Send torque request to Bamocar using M_set (Iq) current control.
 *
 * Input:
 *   T_cmd_nm : requested motor torque [Nm]
 *
 * Output on CAN:
 *   M_set (dig) : digital setpoint for active current Iq
 *                Normalization: 32767 == I_fullscale_pk_A  (or “Imax pk” per manual)
 *
 * Assumptions / required settings:
 *   mc_settings->motor_kt_nm_per_a   = motor torque constant Kt [Nm/A]
 *   mc_settings->i_fullscale_a      = full-scale Iq current that equals 32767 [A]
 *   mc_settings->max_current        = current limit in *digital* units (0..32767)
 *
 * Notes:
 *   - This implements:   Iq_cmd[A] = T_cmd[Nm] / Kt[Nm/A]
 *   - Then converts:     M_set[dig] = (Iq_cmd[A] / I_fullscale[A]) * 32767
 */
uint16_t sendTorqueToMotorController(float T_filtered){

    /* 0) Sanitize */
    if (T_filtered < 0.0f) {
        T_filtered = 0.0f; // [Nm] drive-only
    }

    /* 1) Motor constants (datasheet) */
    //TODO: Make this tunable
    const float Kt_Nm_per_A = 0.94f; // [Nm/A] (assumed consistent with Arms convention)

    /* 2) Full-scale current reference for Bamocar normalization */
    const float I_fs_arms = (float)mc_settings->iq_fullscale_arms; // [Arms] where 32767 == I_fs_arms

    if (Kt_Nm_per_A <= 0.0f || I_fs_arms <= 0.0f) {
        T_filtered = 0.0f;
        uvPanic("Literally undriveable, how the fuck?",0);
    }

    /* 3) Torque -> current (Iq request) */
    float Iq_cmd_arms = (Kt_Nm_per_A > 0.0f) ? (T_filtered / Kt_Nm_per_A) : 0.0f; // [Arms]

#ifdef DEBUG_DL
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"IQ_CMD_RMS: ", (Iq_cmd_arms*1000) , 3, "A");
#endif

    /* 4) Convert digital current limit to Arms and clamp
     * max_current is [dig] where 32767 == I_fs_arms3
     */
    const float I_limit_arms =
        ((float)mc_settings->max_current / 32767.0f) * I_fs_arms; // [Arms]

    if (Iq_cmd_arms > I_limit_arms) Iq_cmd_arms = I_limit_arms;
    if (Iq_cmd_arms < 0.0f)         Iq_cmd_arms = 0.0f;

#ifdef DEBUG_DL
    dl_cur_printbuf += sprint_fixed_d(dl_cur_printbuf,"Ilim_RMS: ", (Iq_cmd_arms*1000) , 3, "A");
#endif

    /* 5) Current -> Bamocar M_set digital
     * trqcmd_dig = (Iq_cmd / I_fs) * 32767
     */
    int16_t trqcmd_dig = (int16_t)((Iq_cmd_arms / I_fs_arms) * 32767.0f);

#ifdef DEBUG_DL
    dl_cur_printbuf += sprintf(dl_cur_printbuf,"digital command %d \n",trqcmd_dig);
#endif


    if (trqcmd_dig >  32767) trqcmd_dig =  32767;
    if (trqcmd_dig < -32768) trqcmd_dig = -32768;

    //pack can message
    static uv_CAN_msg torque_msg;
    memset(&torque_msg, 0, sizeof(torque_msg));

    torque_msg.msg_id = mc_settings->can_id_tx;
    torque_msg.dlc    = 3;
    // Use the N_set command
    //torque_msg.data[0] = N_set; //speed comand
    torque_msg.data[0] = M_Set; //torque command
    // Little-endian: LSB first then MSB
    torque_msg.data[1] = (uint8_t)(trqcmd_dig & 0xFF);
    torque_msg.data[2] = (uint8_t)((trqcmd_dig >> 8) & 0xFF);
    torque_msg.flags   = mc_settings->mc_bus;


    if (uvSendCanMSG(&torque_msg) != UV_OK) {
        uvPanic("Failed to send torque command", 0);
        return 1;
    }
    return 0;
}

/**
 * @brief Sends a CAN request to retrieve a specific register from the motor controller.
 *
 * The request message is formatted as: [0x3D, RegID, 0], which should trigger an immediate reply.
 */
void MC_Request_Data(uint8_t RegID)
{
    uv_CAN_msg request_msg;
    memset(&request_msg, 0, sizeof(request_msg));

    request_msg.msg_id = mc_settings->can_id_tx;
    request_msg.dlc    = 3;
    request_msg.data[0] = 0x3D;   // Request command identifier
    request_msg.data[1] = RegID;    // The register to be requested
    request_msg.data[2] = 0;
    request_msg.flags   = mc_settings->mc_bus;

    if (uvSendCanMSG(&request_msg) != UV_OK) {
        uvPanic("CAN Request Transmission Failed", 0);
    }
}

/** @brief Awaits a specific parameter from the motor controller
 *
 *	Returns UV_OK if it receives one, returns UV_ABORTED if timeout, returns UV_ERROR if something
 *	goes catastrophically wrong.
 */
//uv_status MC_await_param(uint8_t param, TickType_t time_to_wait){
//	TickType_t time_called = xTaskGetTickCount();
//	return UV_ERROR;
//}

/**
 * @brief Sends a parameter write command to the motor controller.
 *
 * Constructs a 3-byte CAN message to set the value of a specific register
 * on the motor controller. The message format is:
 *   [RegID, LSB(data), MSB(data), 0x00]
 *
 * This function does not verify the result—use MC_SetAndVerify_Param()
 * for value confirmation.
 *
 * @param RegID The register address to be written.
 * @param d     The 16-bit value to write to the register.
 * @return UV_OK on success, UV_ERROR if the CAN transmission fails.
 */
uv_status MC_Set_Param(uint8_t RegID,uint16_t d){
    uv_CAN_msg tx_msg;
    tx_msg.msg_id = mc_settings->can_id_tx;
    tx_msg.dlc = 3; //DLC MUST BE 3
    tx_msg.data[0] = RegID;

    tx_msg.data[1] = d & 0xFF;
    tx_msg.data[2] = (d >> 8) & 0xFF;
    tx_msg.flags = 0; //mc_settings->0;


    tx_msg.flags = mc_settings->mc_bus;

    if(uvSendCanMSG(&tx_msg) != UV_OK){
        uvPanic("MC_Param set fail", 0);
        return UV_ERROR;
    }

    return UV_OK;
}

/**
 * @brief Writes a value to a motor controller register and verifies the write.
 *
 * Sends a parameter set command to the specified register, then requests
 * the value back and compares the response to ensure the write succeeded.
 * Uses little-endian 16-bit parsing for verification.
 *
 * This is a safer alternative to MC_Set_Param() when reliability is critical.
 *
 * @param reg_id  The target register address to write.
 * @param set_val The 16-bit value to write to the register.
 * @return UV_OK if the write and verification succeeded, otherwise UV_ERROR.
 */

uv_status MC_SetAndVerify_Param(uint8_t reg_id, uint16_t set_val)
{
	//send parameter to be set
    if (MC_Set_Param(reg_id, set_val) != UV_OK) {
        uvPanic("Set failed", reg_id);
        return UV_ERROR;
    }
    //delay to give time to set
    vTaskDelay(pdMS_TO_TICKS(20));

    // Request the parameter back
    MC_Request_Data(reg_id);

    // Delay for the response to arrive via CAN
    vTaskDelay(pdMS_TO_TICKS(20));

    // Manually call response parser on the updated CAN message
    //ProcessMotorControllerResponse(&last_mc_response);

    // Validate the returned register
    if (last_mc_response.data[0] != reg_id) {
        uvPanic("Mismatched REGID in response", last_mc_response.data[0]);
        return UV_ERROR;
    }

    // Parse 16-bit LE value
    uint16_t returned_val = (last_mc_response.data[2] << 8) | last_mc_response.data[1];

    if (returned_val != set_val) {
        char err_buf[64];
        snprintf(err_buf, sizeof(err_buf),
                 "Mismatch for Reg 0x%02X: set 0x%04X, got 0x%04X",
                 reg_id, set_val, returned_val);
        //uvPanic(err_buf, 0);
        return UV_ERROR;
    }

    return UV_OK;
}

/**
 * @brief Parses a 32-bit value from a CAN message in little-endian format.
 *
 * This example assumes that the data bytes are stored as:
 *   data[0] = LSB, data[3] = MSB.
 */
//void Parse_Bamocar_Response(uv_CAN_msg* msg)
//{
//    if (!msg || msg->dlc < 4) {
//        uvPanic("Invalid motor controller response", 0);
//        return;
//    }
//    uint32_t val = (uint32_t)((msg->data[3] << 24) |
//                              (msg->data[2] << 16) |
//                              (msg->data[1] << 8)  |
//                               msg->data[0]);
//    //printf("Parsed 32-bit LE value: 0x%08X\n", val);
//}

void MC_setErrorMask(uint16_t new_mask){
	mc_error_mask = new_mask;
}


/**
 * @brief Parses and handles a 16-bit error/warning field from the motor controller.
 *
 * This function expects a 2-byte little-endian error field:
 *   data[0] = LSB, data[1] = MSB
 *
 * It checks the resulting bitfield for critical error flags and
 * calls uvPanic() for any fault conditions that require an immediate stop.
 *
 * @param data    Pointer to the error data array (must be at least 2 bytes).
 * @param length  Length of the data array (must be >= 2).
 */
static void MotorControllerErrorHandler_16bitLE(uint8_t *data, uint8_t length)
{
    if (length < 2)
        return;

    uint16_t errors = (uint16_t)((data[0] << 8) | data[1]);

    errors = errors & (~mc_error_mask);


    if (errors & eprom_read_error) {
        uvPanic("EPROM Read Error", 0);
    }
    if (errors & hardware_fault) {
        uvPanic("Hardware Fault", 0);
    }
    if (errors & rotate_field_enable_not_present_run) {
        uvPanic("Rotating Field Enable Not Present (Run Active)", 0);
    }
    if (errors & CAN_timeout_error) {
        uvPanic("CAN Timeout Error", 0);
    }
    if (errors & feedback_signal_error) {
        uvPanic("Feedback Signal Error", 0);
    }
    if (errors & mains_voltage_min_limit) {
        uvPanic("Mains Voltage Below Minimum Limit", 0);
    }
    if (errors & motor_temp_max_limit) {
        uvPanic("Motor Temperature Exceeded Maximum Limit", 0);
    }
    if (errors & IGBT_temp_max_limit) {
        uvPanic("IGBT Temperature Exceeded Maximum Limit", 0);
    }
    if (errors & mains_voltage_max_limit) {
        uvPanic("Mains Voltage Exceeded Maximum Limit", 0);
    }
    if (errors & critical_AC_current) {
        uvPanic("Critical AC Current Detected", 0);
    }
    if (errors & race_away_detected) {
        uvPanic("Race Away Detected", 0);
    }
    if (errors & ecode_timeout_error) {
        uvPanic("Ecode Timeout Error", 0);
    }
    if (errors & watchdog_reset) {
        uvPanic("Watchdog Reset Occurred", 0);
    }
    if (errors & AC_current_offset_fault) {
        uvPanic("AC Current Offset Fault", 0);
    }
    if (errors & internal_hardware_voltage_problem) {
        uvPanic("Internal Hardware Voltage Problem", 0);
    }
    if (errors & bleed_resistor_overload) {
        uvPanic("Bleed Resistor Overload", 0);
    }
    // can add additional error checks as needed.

	//motor_controller_settings* settings = (motor_controller_settings*) params->specific_args;
}


/**
 * @brief Processes a CAN response from the motor controller.
 *
 * This function decodes a received CAN message by examining the register ID
 * in the first byte (data[0]) and handling the response accordingly.
 *
 * It performs little-endian parsing to extract values for speed, current,
 * torque, temperatures, and error flags. Critical errors will trigger
 * uvPanic() via the error handler.
 *
 * The full message is also stored in a global buffer (last_mc_response)
 * for later access and verification routines.
 *
 * @param msg Pointer to the received CAN message from the motor controller.
 */
void ProcessMotorControllerResponse(uv_CAN_msg* msg)
{
//	//every incoming mc message gets stored for use later
//	memcpy(&last_mc_response, msg, sizeof(uv_CAN_msg));
//
//    if (!msg || msg->dlc < 2)
//        return;
    if (!msg || msg->dlc < 2)
        return;

    // every incoming mc message gets stored for use later
    memcpy(&last_mc_response, msg, sizeof(uv_CAN_msg));

    uint8_t reg_id = msg->data[0];

    externalDeviceRxHandler(MOTOR_CONTROLLER);

    switch (reg_id) {
        case N_actual: { // SPEED_ACTUAL (0x30)
            if (msg->dlc >= 3) {
                int16_t speed = (int16_t)((msg->data[2] << 8) | msg->data[1]);
                mc_speed_rpm = (int16_t)(((float)speed/32767.0f)*6500); //Slightly smaller now
                //2457.5 RPM / 6500 RPM) * 32767
                //mc_speed_rpm = (int16_t)((msg->data[2] << 8) | msg->data[1]); //cyclic

            }
            break;
        }

        case CURRENT_ACTUAL:{  // 0x31: 16-bit, little-endian
            if (msg->dlc >= 3) {
                int16_t current_raw = (int16_t)((msg->data[2] << 8) | msg->data[1]);
                //mc_current = (int16_t)((msg->data[2] << 8) | msg->data[1]); //cyclic
                /* CURRENT SCALING (Amps)
                * Logic: (raw_value / 32767) * I_fullscale_arms
                 * Based on your settings: I_fs_arms = 250
                  */
                 float I_fs_arms = (float)mc_settings->iq_fullscale_arms;
                 mc_current = (int16_t)((float)current_raw * (I_fs_arms / 32767.0f));
            }
            break;
        }

        //case LOGIMAP_ERRORS:  // 0x82: error bitfield, little-endian
//        case motor_controller_errors_warnings:
//            if (msg->dlc >= 3) {
//                MotorControllerErrorHandler_16bitLE(&msg->data[1], 2);
//            }
//            break;

        case LOGIMAP_IO: { // 0x83: I/O status, 16-bit, little-endian
            if (msg->dlc >= 3) {
                uint16_t io_flags = (uint16_t)((msg->data[2] << 8) | msg->data[1]);
            }
            break;
        }

        case POS_ACTUAL: { // 0x86: 32-bit value, little-endian
            if (msg->dlc >= 5) {
                int32_t pos = (int32_t)((msg->data[4] << 24) |
                                        (msg->data[3] << 16) |
                                        (msg->data[2] << 8)  |
                                         msg->data[1]);
            }
            break;
        }

        case motor_controller_errors_warnings:{
            // For error/warning responses using this register, assume a 16-bit field
            if (msg->dlc >= 3) {
                MotorControllerErrorHandler_16bitLE(&msg->data[1], 2);
            }
            break;
        }
        case M_out:	{	//0xA0: actual active current scaled
            mc_torque_cmd = (int16_t)((msg->data[2] << 8) | msg->data[1]); //cyclic
            break;
        }

        case motor_temperature:	{	//0x49: motor temperature
            //mc_motor_temp = (int16_t)((msg->data[2] << 8) | msg->data[1]); //cyclic
            int16_t raw_motor_temp = (int16_t)((msg->data[2] << 8) | msg->data[1]);
            lookupMotorTemp(raw_motor_temp, &mc_motor_temp);
            //mc_motor_temp = (int16_t)((float)raw_m_temp / 204.8f);

            /* * T-MOTOR
             * Mapping: 0 to 32000 Num
             * Logic: Based on the "Analog Temperature VdcBus Manual", the motor
             * temperature (using a KTY81-210 sensor
             * To convert this to Celsius, a linear factor of 204.8 is applied,
             * where 32000 represents approximately 156.25°C based off the graph cited below
             * * Citation: "Analog Temperature VdcBus Manual",.
             */
            //mc_motor_temp = (int16_t)((float)raw_motor_temp / 204.8f);
            //int16_t tMotorMapped = (int16_t)raw_motor_temp; // Direct mapping to 0..32000 range


            break;
        }

        case igbt_temperature: {	//0x4A: igbt temperature
            //mc_igbt_temp = (int16_t)((msg->data[2] << 8) | msg->data[1]); //cyclic
            int16_t raw_igbt_temp = (int16_t)((msg->data[2] << 8) | msg->data[1]);

            /* * IGBT (POWER STAGE) TEMPERATURE SCALING
             * Target Mapping: 0 to 32767 Num (REGID 0x4A)
             * Logic: The IGBTs use internal NTC sensors that follow a non-linear curve
             * specific to the Bamocar hardware.
             * * Citation: "Bamocar D3 Manual", Section 6.2 Power Stages - Temperature.
             * Reference Table (examples from manual):
             * 125°C = 28480 | 100°C = 26702 | 25°C = 18797
             */
            // Pass the address of the global mc_igbt_temp to be updated
            lookupIgbtTemp(raw_igbt_temp, &mc_igbt_temp);
            break;
        }

        default: {
            // Handle other responses as needed or call a default parser.
            break;
        }
    }
}

/**
 * @brief Approximate KTY81 motor temperature lookup from Bamocar raw value.
 *
 * Converts the raw Bamocar motor temperature register value into an
 * approximate human-readable temperature in °C using a lookup table
 * derived from the KTY81 graph.
 *
 * @param raw_motor_temp  Raw motor temperature register value from Bamocar.
 * @param result          Pointer to output temperature in °C.
 */
void lookupMotorTemp(int16_t raw_motor_temp, int16_t* result)
{
    /*
     * LUT maps Bamocar raw motor-temperature register units -> degC.
     *
     * x-axis  (raw_units): raw ADC-like units from the inverter register.
     * y-axis  (temp_c)   : human-readable temperature in degrees Celsius.
     *
     * Keep both arrays in the same order and with matching indices.
     * Example: raw_units[i] corresponds to temp_c[i].
     */
    static float temp_c[] = {
        -30, -20, -10,   0,  10,  20,  25,  30,  40,  50,
         60,  70,  80,  90, 100, 110, 120, 130, 140, 150
    };

    static int32_t raw_units[] = {
         7414,8240,8802,9369,9939,10510,10795,11080,11646, 12207,
         12762,13308,13846,14373,14890,15391,15852,16251,16569,16789
    };

        /*
        * FALLBACK (OLD BEHAVIOR, NO DATA-PROCESSING DEPENDENCY)
        * -------------------------------------------------------
        * Keep this block as a quick rollback/reference path.
        *
        * To use it:
        * 1) Comment out the LUT_if_t/xToY_if call below.
        * 2) Uncomment this block.
        */
        //{
        //    const int lut_len = (int)(sizeof(temp_c) / sizeof(temp_c[0]));
        //
        //    if (raw_motor_temp <= raw_units[0]) {
        //        *result = (int16_t)temp_c[0];
        //        return;
        //    }
        //
        //    if (raw_motor_temp >= raw_units[lut_len - 1]) {
        //        *result = (int16_t)temp_c[lut_len - 1];
        //        return;
        //    }
        //
        //    for (int i = 0; i < lut_len - 1; i++) {
        //        if (raw_motor_temp < raw_units[i + 1]) {
        //            float slope = ((float)(temp_c[i + 1] - temp_c[i])) /
        //                          ((float)(raw_units[i + 1] - raw_units[i]));
        //
        //            *result = (int16_t)(temp_c[i] +
        //                                slope * (raw_motor_temp - raw_units[i]));
        //            return;
        //        }
        //    }
        //
        //    *result = (int16_t)temp_c[lut_len - 1];
        //    return;
        //}

    /* Output pointer check so callers can safely pass through error paths. */
    if (result == NULL) {
        return;
    }

    /*
     * Build a LUT_if_t so this function uses the shared data-processing module
     * instead of carrying a custom interpolation implementation locally.
     *
     * Flags:
     * - LUT_LINTERP: linear interpolation between points.
     * - LUT_CAP_AT_MAX_MIN: clamp outside table instead of extrapolating.
     */
    static LUT_if_t motor_temp_lut = {
        .x = raw_units,
        .y = temp_c,
        //.n = (uint8_t)(sizeof(raw_units) / sizeof(raw_units[0])), - BUG - sizeof(raw_units is size of pointer)
		.n = 20,
        .flags = (LUT_LINTERP | LUT_CAP_AT_MAX_MIN)
    };

    /*
     * Validate LUT once per call path to catch malformed table edits.
     * If validation fails, output a safe fallback value and return.
     */
    if (validateLUT_if(&motor_temp_lut) != UV_OK) {
        *result = 0;
        return;
    }

    /*
     * xToY_if performs clamping + interpolation according to LUT flags.
     * Cast back to int16_t because mc_motor_temp is stored as integer degC.
     */
    *result = (int16_t)xToY_if(&motor_temp_lut, (int32_t)raw_motor_temp);
}
/**
 * IGBT (POWER STAGE) TEMPERATURE - VOID VERSION
 * @brief Non-linear lookup for IGBT Temp
 * Citation: "Bamocar D3 Manual", Section 6.2 Power Stages
 */
void lookupIgbtTemp(int16_t T_deg, int16_t* result) {
    // static const: Keeps tables in Flash memory (important for RTOS memory management)
    static const float temps[]   = { -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125 };
    static const int16_t units[] = { 16308, 16387, 16487, 16609, 16757, 16938, 17151, 17400, 17688, 18017, 18387, 18797, 19247, 19733, 20250, 20793, 21357, 21933, 22515, 23097, 23671, 24232, 24775, 25296, 25792, 26261, 26702, 27114, 27497, 27851, 28179, 28480 };

    if (T_deg <= units[0]) {
        *result = (int16_t)temps[0];
        return;
    }
    if (T_deg >= units[31]) {
        *result = (int16_t)temps[31];
        return;
    }

    //TODO use dataprocessing LUT dependency to decrease executable size
    for (int i = 0; i < 31; i++) {
        if (T_deg < units[i + 1]) {
            float slope = (temps[i + 1] - temps[i]) / (float)(units[i + 1] - units[i]);
            *result = (int16_t)(temps[i] + slope * (T_deg - units[i]));
            return;
        }
    }
}

/**
 * @brief Enables or disables cyclic transmission of selected motor controller parameters.
 *
 * Sends a series of CAN messages to configure the Bamocar controller to periodically
 * transmit values such as speed, current, torque, temperatures, and error flags.
 *
 * The message format is:
 *   [0x3D, RegID, interval_ms]
 * Where interval_ms is the repeat interval in milliseconds (1–254).
 * Passing 0xFF disables transmission for that register.
 *
 * @param interval_ms Transmission interval in milliseconds (1–254). Use 0xFF to disable.
 */
void MC_EnableCyclicSpeedTransmission(uint8_t interval_ms)
{
	//TODO: figure out bug
    //if (interval_ms < 1 || interval_ms > 254) return;
    // allow 1..254 for streaming, and 0xFF to disable
    if (!((interval_ms >= 1 && interval_ms <= 254) || interval_ms == 0xFF)) {
        return;
    }


    uint8_t regs[] = {
        N_actual,           // 0x30 — Actual Speed
        CURRENT_ACTUAL,     // 0x69 — Actual Current
        M_out,              // 0xA0 — Actual Active Current Scaled
        motor_temperature,  // 0x49 — Current Motor temperature
        igbt_temperature,    // 0x4A — IGBT temperature
		max_motor_temp, 	//0xa3	- max motor temp
		warning_motor_temp, //0xa2 - warning motor temp
		//LOGIMAP_ERRORS, // 0x8F — ERROR BIT map
		motor_controller_errors_warnings, //Errors and warnings duh
    };
    //this might have a bug
    //YES IT DOES - BYRON sizeof(regs) is the size of the pointer, not the contents of the array!!
    for (int i = 0; i < 8; i++) {
    	//TODO: figure out if this is better
    //for (int i = 0; i < (int)(sizeof(regs)/sizeof(regs[0])); i++){

        uv_CAN_msg tx;
        memset(&tx, 0, sizeof(tx));

        tx.msg_id  = mc_settings->can_id_tx;
        tx.dlc     = 3;
        tx.data[0] = 0x3D;            // Command: Enable cyclic read
        tx.data[1] = regs[i];         // Target register
        tx.data[2] = interval_ms;     // Repeating time (1–254 ms)
        tx.flags   = mc_settings->mc_bus;

        uvSendCanMSG(&tx);
        vTaskDelay(pdMS_TO_TICKS(5));  // delay between messages
    }
}


/**
 * @brief Initializes the motor controller.
 *
 * This function performs all necessary startup steps to prepare the Bamocar
 * motor controller for operation. It is typically called during system
 * initialization from uvfr_utils.c.
 *
 * The routine performs the following actions:
 *  - Toggles a GPIO pin for debug indication
 *  - Registers a CAN RX handler for controller responses
 *  - Enables cyclic transmission of critical feedback parameters
 *  - Sends initialization commands (based on Bamocar Example 11)
 *  - Requests metadata (serial number, firmware version)
 *  - Optionally sets and verifies controller parameters
 *  - Sends status to the system init queue
 *  - Suspends itself after completion
 *
 * @param args Pointer to uv_init_task_args, used to send back init status.
 */
void MC_Startup(void* args)
{
	//toggle pin
    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

	MC_setErrorMask(mains_voltage_min_limit|
			rotate_field_enable_not_present_run|
			AC_current_offset_fault);
	//MC_setErrorMask(0xFFFFFFFF);

    //Register CAN RX handler first and routes eveyrthing though processmotorcontrollerresponse
    //subsequently the motor controller error handler
    insertCANMessageHandler(mc_settings->can_id_rx, ProcessMotorControllerResponse, mc_settings->mc_bus);

    uvRegisterExternalDevice(MOTOR_CONTROLLER, 100, XDEV_DEVICE_EXPECTED|XDEV_CHECK_TIMEOUT_BIT, "Bamocar");
    //start cyclic transmission
    MC_EnableCyclicSpeedTransmission(100); // every 100ms

    // === Bamocar Init Routine (partial from Example 11) ===

    // 1. Request BTB (0xE2) — optional
    MC_Request_Data(0xE2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 2. Disable motor
    //0x51 is mode state
    MC_Set_Param(0x51, 0x0004);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Request enable hardware
    MC_Request_Data(0xE8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 4. Enable controller
    MC_Set_Param(0x51, 0x0000);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 5. Set acceleration ramp
    MC_Set_Param(0x35, 0x01F4);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 6. Set deceleration ramp
    MC_Set_Param(0xED, 0x03E8);
    vTaskDelay(pdMS_TO_TICKS(10));

    //pointer to mc init task
    uv_init_task_args* MC_init_args = (uv_init_task_args*)args;
        QueueHandle_t queue = MC_init_args->init_info_queue;

        uv_init_task_response rx;
        //ID of responding device
        rx.device = MOTOR_CONTROLLER;
        rx.status = UV_OK;

		// Request serial number and firmware version.
		MC_Request_Data(SERIAL_NUMBER_REGISTER);
		vTaskDelay(10); //delay
		//Request device serial number servo
		//MC_Request_Data(0x62);
		//vTaskDelay(10);

		MC_Request_Data(FIRMWARE_VERSION_REGISTER);
		vTaskDelay(10); //

		//clear error list
		MC_Set_Param(0x8E, 0x444d);

		//sendTorqueToMotorController(0); //set initial speed to 0
		vTaskDelay(pdMS_TO_TICKS(10));

		//set and verify tests
		//MC_SetAndVerify_Param(0x31, 0x0CCD);  // 10% speed N_Set
		//MC_SetAndVerify_Param(0x6A, 15);   // Kp
		//MC_SetAndVerify_Param(0x6B, 501);  // Ki

		// Request error data (if desired, e.g., using the error/warning register).
		//MC_Request_Data((uint8_t)motor_controller_errors_warnings);
		//MC_Request_Data(0x82) //check errors should see 82 ## ##

		//Signal success via an RTOS queue here...
		xQueueSend(queue,&rx,0);

		// Suspend this task so it does not run repeatedly.
		vTaskSuspend(NULL);
}

/**
 * @brief Safely shuts down the motor controller.
 *
 * This function stops all periodic CAN feedback, zeroes the torque or speed
 * command (depending on control mode), disables the motor controller, and
 * optionally requests the error/warning register for post-shutdown diagnostics.
 *
 * It ensures the controller is left in a known, safe state after driving stops.
 * Use this before powering down or transitioning to an inactive state.
 */
void MC_Shutdown(void)
{
    	// 1. Stop all cyclic transmission
        MC_EnableCyclicSpeedTransmission(0xFF);  // 0xFF disables periodic streaming
        vTaskDelay(pdMS_TO_TICKS(10));

        // 2. Set speed or torque to zero
        // Use one of these depending on your control mode:
        // --- If using speed mode:
        //MC_Set_Param(0x31, 0x0000);  // N_set = 0
        // --- If using torque mode (comment out one or the other):
        sendTorqueToMotorController(0.0f);

        vTaskDelay(pdMS_TO_TICKS(10));

        // 3. Disable controller
        MC_Set_Param(0x51, 0x0004);  // Disable command
        vTaskDelay(pdMS_TO_TICKS(10));

        // 4. Optional: Request error/warning register after shutdown
        MC_Request_Data(motor_controller_errors_warnings);
        vTaskDelay(pdMS_TO_TICKS(10));
}
