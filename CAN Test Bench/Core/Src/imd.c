#include "imd.h"
#include "can.h"
#include "main.h"
#include "constants.h"
#include "pdu.h"

// Global Variables
uint8_t IMD_status_bits = 0;
uint8_t IMD_High_Uncertainty = 0;

uint32_t IMD_Read_Part_Name[4];
const uint32_t IMD_Expected_Part_Name[4] = {0x12345678, 0x23456789, 0x34567890, 0x45678901}; //TODO: this is a PLACEHOLDER must be checked
uint8_t IMD_Part_Name_Set = 0;

uint32_t IMD_Read_Version[3];
const uint32_t IMD_Expected_Version[3] = {0x01020304, 0x05060708, 0x090A0B0C}; //TODO: this is a PLACEHOLDER must be checked
uint8_t IMD_Version_Set = 0;

uint32_t IMD_Read_Serial_Number[4];
const uint32_t IMD_Expected_Serial_Number[4] = {0xB8DD9AF9, 0x6094F48B, 0x1F1C3794, 0xFCF9A95B}; //TODO: this is a PLACEHOLDER must be checked
uint8_t IMD_Serial_Number_Set = 0;

int32_t IMD_Temperature = 0;
uint32_t IMD_Uptime = 0;
uint8_t IMD_error_flags_requested = 0;

// Function Definitions
void IMD_Parse_Message(int DLC, uint8_t Data[]);
void IMD_Request_Status(uint8_t RequestMux);
void IMD_Check_Status_Bits(uint8_t status_bits);
void IMD_Check_Error_Flags(uint8_t Data[]);
void IMD_Check_Isolation_State(uint8_t Data[]);
void IMD_Check_Isolation_Resistances(uint8_t Data[]);
void IMD_Check_Isolation_Capacitances(uint8_t Data[]);
void IMD_Check_Voltages_Vp_and_Vn(uint8_t Data[]);
void IMD_Check_Battery_Voltage(uint8_t Data[]);
void IMD_Check_Temperature(uint8_t Data[]);
void IMD_Check_Safety_Touch_Energy(uint8_t Data[]);
void IMD_Check_Safety_Touch_Current(uint8_t Data[]);
void IMD_Check_Max_Battery_Working_Voltage(uint8_t Data[]);
void IMD_Check_Part_Name(uint8_t Data[]);
void IMD_Check_Version(uint8_t Data[]);
void IMD_Check_Serial_Number(uint8_t Data[]);
void IMD_Check_Uptime(uint8_t Data[]);
void IMD_Startup(void);

void IMD_Parse_Message(int DLC, uint8_t Data[]) {
    switch (Data[0]) {
        case isolation_state:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Isolation_State(Data);
            break;
        case isolation_resistances:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Isolation_Resistances(Data);
            break;
        case isolation_capacitances:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Isolation_Capacitances(Data);
            break;
        case voltages_Vp_and_Vn:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Voltages_Vp_and_Vn(Data);
            break;
        case battery_voltage:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Battery_Voltage(Data);
            break;
        case Error_flags:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Error_Flags(Data);
            break;
        case safety_touch_energy:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Safety_Touch_Energy(Data);
            break;
        case safety_touch_current:
            IMD_Check_Status_Bits(Data[1]);
            IMD_Check_Safety_Touch_Current(Data);
            break;
        case Temperature:
            IMD_Check_Temperature(Data);
            break;
        case Max_battery_working_voltage:
            IMD_Check_Max_Battery_Working_Voltage(Data);
            break;
        case Part_name_0:
        case Part_name_1:
        case Part_name_2:
        case Part_name_3:
            IMD_Check_Part_Name(Data);
            break;
        case Version_0:
        case Version_1:
        case Version_2:
            IMD_Check_Version(Data);
            break;
        case Serial_number_0:
        case Serial_number_1:
        case Serial_number_2:
        case Serial_number_3:
            IMD_Check_Serial_Number(Data);
            break;
        case Uptime_counter:
            IMD_Check_Uptime(Data);
            break;
        default:
            Error_Handler();
            break;
    }
}

void IMD_Request_Status(uint8_t RequestMux) {
    TxHeader.ExtId = IMD_CAN_ID_Tx;
    TxHeader.DLC = 1;
    TxData[0] = RequestMux;

    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler();
    }
}

//TODO: complete these error flag handles
void IMD_Check_Status_Bits(uint8_t status_bits) {
    IMD_status_bits = status_bits;
    if (status_bits & Hardware_Error) {
        if (!IMD_error_flags_requested) {
            IMD_Request_Status(Error_flags);
            IMD_error_flags_requested = 1;
        }
    }
    if (status_bits & Low_Battery_Voltage) {
        // Handle low battery voltage
    }
    if (status_bits & High_Battery_Voltage) {
        // Handle high battery voltage
    }
}

//TODO: complete these error flag handles
void IMD_Check_Error_Flags(uint8_t Data[]) {
    uint16_t error_flags = (Data[1] << 8) | Data[2];
    if (error_flags & Err_Vx1) {
        // Handle VX1 error
    }
    if (error_flags & Err_Vx2) {
        // Handle VX2 error
    }
    if (error_flags & Err_CH) {
        // Handle CH connection error
    }
    if (error_flags & Err_Vexi) {
        // Handle excitation voltage error
    }
    if (error_flags & Err_temp) {
        // Handle over-temperature error
    }
}

void IMD_Check_Isolation_State(uint8_t Data[]) {
    uint16_t isolation = (Data[2] << 8) | Data[3];
    uint8_t uncertainty = Data[4];
    if (isolation < 500 && uncertainty <= 5) {
        // Isolation fault, trigger shutdown
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Isolation_Resistances(uint8_t Data[]) {
    uint16_t Rp_resistance = (Data[2] << 8) | Data[3];
    uint16_t Rn_resistance = (Data[5] << 8) | Data[6];
    if (Rp_resistance < 250 || Rn_resistance < 250) {
        // Isolation resistance fault
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Isolation_Capacitances(uint8_t Data[]) {
    uint16_t Cp = (Data[2] << 8) | Data[3];
    uint16_t Cn = (Data[5] << 8) | Data[6];
    if (Cp > 100 || Cn > 100) {
        // Capacitance exceeds threshold
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Voltages_Vp_and_Vn(uint8_t Data[]) {
    uint16_t Vp = (Data[2] << 8) | Data[3];
    uint16_t Vn = (Data[5] << 8) | Data[6];
    if (Vp - Vn > 1000) {
        // Voltage imbalance detected
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Battery_Voltage(uint8_t Data[]) {
    uint16_t Vb = (Data[2] << 8) | Data[3];
    if (Vb < 100 || Vb > 800) {
        // Battery voltage out of range
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Temperature(uint8_t Data[]) {
    IMD_Temperature = (Data[1] << 24) | (Data[2] << 16) | (Data[3] << 8) | Data[4];
    if (IMD_Temperature > 105000) {
        // Handle over-temperature condition
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Safety_Touch_Energy(uint8_t Data[]) {
    uint16_t energy = (Data[2] << 8) | Data[3];
    if (energy > 200) {
        // Touch energy exceeds safety threshold
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Safety_Touch_Current(uint8_t Data[]) {
    uint16_t current = (Data[2] << 8) | Data[3];
    if (current > 5) {
        // Touch current exceeds safety threshold
        Trigger_Shutdown_Circuit();
    }
}


//TODO: check the actual max working voltage
void IMD_Check_Max_Battery_Working_Voltage(uint8_t Data[]) {
    uint16_t max_voltage = (Data[1] << 8) | Data[2];
    if (max_voltage != 571) {
        // Max voltage mismatch
        Trigger_Shutdown_Circuit();
    }
}

void IMD_Check_Part_Name(uint8_t Data[]) {
    uint8_t index = Data[0] - Part_name_0;
    IMD_Read_Part_Name[index] = (Data[4] << 24) | (Data[3] << 16) | (Data[2] << 8) | Data[1];
    if (index == 3) {
        for (int i = 0; i < 4; ++i) {
            if (IMD_Read_Part_Name[i] != IMD_Expected_Part_Name[i]) {
                // Part name mismatch
                Trigger_Shutdown_Circuit();
            }
        }
    }
}

void IMD_Check_Version(uint8_t Data[]) {
    uint8_t index = Data[0] - Version_0;
    IMD_Read_Version[index] = (Data[3] << 16) | (Data[2] << 8) | Data[1];
    if (index == 2) {
        for (int i = 0; i < 3; ++i) {
            if (IMD_Read_Version[i] != IMD_Expected_Version[i]) {
                // Version mismatch
                Trigger_Shutdown_Circuit();
            }
        }
    }
}

void IMD_Check_Serial_Number(uint8_t Data[]) {
    uint8_t index = Data[0] - Serial_number_0;
    IMD_Read_Serial_Number[index] = (Data[1] << 24) | (Data[2] << 16) | (Data[3] << 8) | Data[4];
    if (index == 3) {
        for (int i = 0; i < 4; ++i) {
            if (IMD_Read_Serial_Number[i] != IMD_Expected_Serial_Number[i]) {
                // Serial number mismatch
                Trigger_Shutdown_Circuit();
            }
        }
    }
}

void IMD_Check_Uptime(uint8_t Data[]) {
    IMD_Uptime = (Data[1] << 24) | (Data[2] << 16) | (Data[3] << 8) | Data[4];
    // Use uptime for diagnostics
}

void IMD_Startup(void) {
    IMD_Request_Status(Serial_number_0);
    IMD_Request_Status(Serial_number_1);
    IMD_Request_Status(Serial_number_2);
    IMD_Request_Status(Serial_number_3);
    IMD_Request_Status(Version_0);
    IMD_Request_Status(Version_1);
    IMD_Request_Status(Version_2);
    IMD_Request_Status(Part_name_0);
    IMD_Request_Status(Part_name_1);
    IMD_Request_Status(Part_name_2);
    IMD_Request_Status(Part_name_3);
    IMD_Request_Status(Max_battery_working_voltage);
    IMD_Request_Status(Uptime_counter);
}
