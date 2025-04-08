/* motor_controller.c */

#include "motor_controller.h"
#include "can.h"           // For uvSendCanMSG, uv_CAN_msg, etc.
#include "cmsis_os.h"      // For vTaskSuspend
#include "uvfr_utils.h"    // For uvPanic, etc.
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Global default settings variable defined here.
 * This uses the motor_controller_settings definition from uvfr_settings.h.
 */
motor_controller_settings mc_default_settings = {
    .can_id_tx              = 0x201,
    .can_id_rx              = 0x181,
    .mc_CAN_timeout         = 2,
    .proportional_gain      = 10,   // uint8_t
    .integral_time_constant = 400,  // uint32_t
    .integral_memory_max    = 60    // uint8_t (represents 60%)
};

/**
 * @brief Sends a direct torque command to the motor controller.
 *
 * This function accepts a float for the desired torque (T_filtered),
 * clamps it between 0 and 100, converts it to a 16-bit integer, and then
 * sends it over CAN using uvSendCanMSG.
 */
uint16_t MotorControllerSpinTest(float T_filtered)
{
    if (T_filtered < 0.0f)
        T_filtered = 0.0f;
    if (T_filtered > 100.0f)
        T_filtered = 100.0f;

    uint16_t torque_cmd = (uint16_t) T_filtered;

    uv_CAN_msg torque_msg;
    memset(&torque_msg, 0, sizeof(torque_msg));

    torque_msg.msg_id = mc_default_settings.can_id_tx;
    torque_msg.dlc    = 3;
    // Use the N_set command (from your enum motor_controller_speed_parameters)
    torque_msg.data[0] = N_set;
    // Little-endian: LSB first then MSB
    torque_msg.data[1] = (uint8_t)(torque_cmd & 0xFF);
    torque_msg.data[2] = (uint8_t)((torque_cmd >> 8) & 0xFF);
    torque_msg.flags   = 0;

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

    request_msg.msg_id = mc_default_settings.can_id_tx;
    request_msg.dlc    = 3;
    request_msg.data[0] = 0x3D;   // Request command identifier
    request_msg.data[1] = RegID;    // The register to be requested
    request_msg.data[2] = 0;
    request_msg.flags   = 0;

    if (uvSendCanMSG(&request_msg) != UV_OK) {
        uvPanic("CAN Request Transmission Failed", 0);
    }
}

uv_status MC_Set_Param(uint8_t RegID,uint16_t d){
	uv_CAN_msg tx_msg;
	tx_msg.msg_id = mc_default_settings.can_id_tx;
	tx_msg.dlc = 3; //DLC MUST BE 3
	tx_msg.data[0] = RegID;

	tx_msg.data[1] = d & 0xFF;
	tx_msg.data[2] = (d >> 8) & 0xFF;
	tx_msg.data[3] = 0;

	if(uvSendCanMSG(&tx_msg) != UV_OK){
		uvPanic("MC_Param set fail", 0);
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
void Parse_Bamocar_Response(uv_CAN_msg* msg)
{

    if (!msg || msg->dlc < 4) {
        uvPanic("Invalid motor controller response", 0);
        return;
    }
    uint32_t val = (uint32_t)((msg->data[3] << 24) |
                              (msg->data[2] << 16) |
                              (msg->data[1] << 8)  |
                               msg->data[0]);
    printf("Parsed 32-bit LE value: 0x%08X\n", val);

    uint8_t cmd_byte = msg->data[0];

    uint16_t word = deserializeSmallE16(msg->data,1);

    switch(cmd_byte){
    case 0x1C:

    	if(word != mc_default_settings.proportional_gain);
    	uvPanic("aa",0);

    	break;

    case 2:

    	break;
    default:
    	//Nothing we recognise

    	break;
    }
}

/**
 * @brief Helper function to process a 16-bit error/warning field (little-endian).
 *
 * It expects the error data in 2 bytes where:
 *   data[0] = LSB, data[1] = MSB.
 * It checks the error flags and calls uvPanic for critical errors.
 */
static void MotorControllerErrorHandler_16bitLE(uint8_t *data, uint8_t length)
{
    if (length < 2)
        return;

    uint16_t errors = (uint16_t)((data[1] << 8) | data[0]);

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
    // You can add additional error checks as needed.
}

/**
 * @brief Processes a motor controller response received via CAN.
 *
 * This function examines the first byte as the register ID and then processes the rest
 * of the message using little-endian parsing. For error/warning responses (for example,
 * when reg_id equals motor_controller_errors_warnings), it calls the error handler.
 */
void ProcessMotorControllerResponse(uv_CAN_msg* msg)
{
    if (!msg || msg->dlc < 2)
        return;

    uint8_t reg_id = msg->data[0];

    switch (reg_id) {
        case N_actual:  // SPEED_ACTUAL (0x30)
            if (msg->dlc >= 3) {
                int16_t speed = (int16_t)((msg->data[2] << 8) | msg->data[1]);
                printf("SPEED_ACTUAL: %d rpm\n", speed);
            }
            break;

        case CURRENT_ACTUAL:  // 0x31: 16-bit, little-endian
            if (msg->dlc >= 3) {
                int16_t current_raw = (int16_t)((msg->data[2] << 8) | msg->data[1]);
                printf("CURRENT_ACTUAL: %d (raw units)\n", current_raw);
            }
            break;

        case LOGIMAP_ERRORS:  // 0x82: error bitfield, little-endian
            if (msg->dlc >= 3) {
                MotorControllerErrorHandler_16bitLE(&msg->data[1], 2);
            }
            break;

        case LOGIMAP_IO:  // 0x83: I/O status, 16-bit, little-endian
            if (msg->dlc >= 3) {
                uint16_t io_flags = (uint16_t)((msg->data[2] << 8) | msg->data[1]);
                printf("LOGIMAP_IO flags: 0x%04X\n", io_flags);
            }
            break;

        case POS_ACTUAL:  // 0x86: 32-bit value, little-endian
            if (msg->dlc >= 5) {
                int32_t pos = (int32_t)((msg->data[4] << 24) |
                                        (msg->data[3] << 16) |
                                        (msg->data[2] << 8)  |
                                         msg->data[1]);
                printf("POS_ACTUAL: %ld\n", (long)pos);
            }
            break;

        case motor_controller_errors_warnings:
            // For error/warning responses using this register, assume a 16-bit field
            if (msg->dlc >= 3) {
                MotorControllerErrorHandler_16bitLE(&msg->data[1], 2);
            }
            break;

        default:
            // Handle other responses as needed or call a default parser.
            break;
    }
}

/**
 * @brief Initializes the motor controller.
 *
 * This routine performs the following steps:
 *   1. Requests the serial number and firmware version.
 *   2. Sends a nominal torque command (spin test).
 *   3. Requests error/warning data.
 *   4. Suspends itself after successful initialization.
 */
void MC_Startup(void* args)
{
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

    uv_init_task_args* MC_init_args = (uv_init_task_args*)args;
    QueueHandle_t queue = MC_init_args->init_info_queue;

    uv_init_task_response rx;

    rx.device = MOTOR_CONTROLLER;
    rx.status = UV_OK;

    insertCANMessageHandler(0x181, Parse_Bamocar_Response);

    // Example: Request serial number and firmware version.
    MC_Request_Data(SERIAL_NUMBER_REGISTER);

    vTaskDelay(10);
    MC_Request_Data(FIRMWARE_VERSION_REGISTER);

    // Send a nominal torque command (e.g., 10.0 units).
    if (MotorControllerSpinTest(10.0f) != 0) {
        uvPanic("Motor Spin Test Failed", 0);
    }
    vTaskDelay(10);
    // Request error data (if desired, e.g., using the error/warning register).
    MC_Request_Data((uint8_t)motor_controller_errors_warnings);

    vTaskDelay(10);

    MC_Request_Data(0x1C);

    vTaskDelay(10);

    uint16_t dat = mc_default_settings.proportional_gain;

    MC_Set_Param(0x1C,dat);

    vTaskDelay(30);


    MC_Request_Data(0x1C);
    // Optionally, signal success via an RTOS queue here...

    xQueueSend(queue,&rx,0);

    // Suspend this task so it does not run repeatedly.
    vTaskSuspend(NULL);
}
