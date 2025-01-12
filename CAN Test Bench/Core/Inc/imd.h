// Code to make human-readable CAN messages for the device
// This file defines message data and function declarations

#ifndef __IMD_H__
#define __IMD_H__

#include "main.h"

// ---------------------------------------------------------------
// Constants and Enums

// CAN IDs
#define IMD_CAN_ID_Tx 0x100  // Transmit ID for the IMD
#define IMD_CAN_ID_Rx 0x101  // Receive ID for the IMD

// Faults: Status Bits
enum imd_status_bits {
    Isolation_status_bit0   = 0b00000001, // IS0
    Isolation_status_bit1   = 0b00000010, // IS1
    Low_Battery_Voltage     = 0b00000100, // LV (1 if below 15V threshold)
    High_Battery_Voltage    = 0b00001000, // HV (1 if higher than Max_battery_working_voltage)
    Exc_off                 = 0b00010000, // EO (Excitation pulse 0 for operating)
    High_Uncertainty        = 0b00100000, // HU (1 for uncertainty > 5%)
    Touch_energy_fault      = 0b01000000, // EF (1 for energy exceeds 0.2J)
    Hardware_Error          = 0b10000000, // HE (1 for error)
};

// Requests for Status
enum imd_status_requests {
    isolation_state         = 0xE0,  // Electrical isolation (bytes 2 & 3)
    isolation_resistances   = 0xE1,  // Rp and Rn isolation resistances
    isolation_capacitances  = 0xE2,  // Cp and Cn capacitances (bytes 2, 3, 5, 6)
    voltages_Vp_and_Vn      = 0xE3,  // High voltage battery voltages
    battery_voltage         = 0xE4,  // GLV battery voltage
    Error_flags             = 0xE5,  // Series of error flag bits
    safety_touch_energy     = 0xE6,  // Charge stored in the system
    safety_touch_current    = 0xE7,  // Monitors safe touch current
    Max_battery_working_voltage = 0xF0, // Parameter to set max voltage
    Temperature             = 0x80,  // Board temperature
};

// Error Flags
enum imd_error_flags {
    Err_temp        = 0x0080, // HT: High temperature (>105°C)
    Err_clock       = 0x0100, // CE: Clock error
    Err_Watchdog    = 0x0200, // WD: Watchdog error
    Err_Vpwr        = 0x0400, // V_PWR: Power supply error
    Err_Vexi        = 0x0800, // V_EXI: Excitation voltage error
    Err_VxR         = 0x1000, // VxR: Voltage reference error
    Err_CH          = 0x2000, // CH: Chassis connection error
    Err_Vx1         = 0x4000, // Vx1: HV+ connection error
    Err_Vx2         = 0x8000, // Vx2: HV- connection error
};

// Manufacturer Requests
enum imd_manufacturer_requests {
    Part_name_0     = 0x01,
    Part_name_1     = 0x02,
    Part_name_2     = 0x03,
    Part_name_3     = 0x04,
    Version_0       = 0x05,
    Version_1       = 0x06,
    Version_2       = 0x07,
    Serial_number_0 = 0x08,
    Serial_number_1 = 0x09,
    Serial_number_2 = 0x0A,
    Serial_number_3 = 0x0B,
    Uptime_counter  = 0x0C,
};

// High-Resolution Measurements
enum imd_high_resolution_measurements {
    Vn_hi_res       = 0x60, // High-res negative voltage
    Vp_hi_res       = 0x61, // High-res positive voltage
    Vexc_hi_res     = 0x62, // High-res excitation voltage
    Vb_hi_res       = 0x63, // High-res battery voltage
    Vpwr_hi_res     = 0x65, // High-res power voltage
};

// ---------------------------------------------------------------
// Function Declarations

// Parses the data received from a CAN message
void IMD_Parse_Message(int DLC, uint8_t Data[]);

// Status Checks
void IMD_Check_Status_Bits(uint8_t Data);
void IMD_Check_Error_Flags(uint8_t Data[]);

// Value Checks
void IMD_Check_Isolation_State(uint8_t Data[]);
void IMD_Check_Isolation_Resistances(uint8_t Data[]);
void IMD_Check_Isolation_Capacitances(uint8_t Data[]);
void IMD_Check_Voltages_Vp_and_Vn(uint8_t Data[]);
void IMD_Check_Battery_Voltage(uint8_t Data[]);
void IMD_Check_Safety_Touch_Energy(uint8_t Data[]);
void IMD_Check_Safety_Touch_Current(uint8_t Data[]);
void IMD_Check_Temperature(uint8_t Data[]);

// Startup Configuration Checks
void IMD_Check_Max_Battery_Working_Voltage(uint8_t Data[]);
void IMD_Check_Part_Name(uint8_t Data[]);
void IMD_Check_Version(uint8_t Data[]);
void IMD_Check_Serial_Number(uint8_t Data[]);
void IMD_Check_Uptime(uint8_t Data[]);

// High-Resolution Measurement Requests
void IMD_Request_Status(uint8_t Status);

// Called on startup to initialize IMD checks
void IMD_Startup(void);

// called on startup @deprecated
void IMD_Startup();

void initIMD(void* args);

#endif  // __IMD_H__




