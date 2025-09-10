#ifndef CONIFER_H
#define CONIFER_H

//Enum that is the human readable names of the abstract conifer outputs
typedef enum CONIFER_OUTPUT{
	VCU_SAFETY,
	BAMO_RFE,
	BAMO_RUN,
	BAMO_PWR,
	SDC_START,
	COOLANT_PUMP1,
	COOLANT_PUMP2,
	RAD_FANS1,
	RAD_FANS2,
	RAD_FANS3,
	RAD_FANS4,
	ACCU_FANS1,
	ACCU_FANS2,
	BRAKE_LIGHT,
	HORN,
	DRS_FL1,
	DRS_FL2,
	DRS_FR1,
	DRS_FR2,
	DRS_R1,
	DRS_R2,
	DRS_R3,
	BMS_ALWAYS_ON,
	BMS_DISCHARGE_PWR,
	IMD_PWR,
	GENERAL_ACCU_PWR1,
	GENERAL_ACCU_PWR2,
	VCU_PWR, //Yes, the VCU can turn itself off technically. You should not however.
	RTML_PWR,
	TSSI_PWR,
	BSPD_PWR,
	DASH_PWR,

	FINAL_CONIFER_OUTPUT
}conifer_output_channel;

#define EXPECT_UV19PDU (0x01U<<0) //if this bit is set, that means that there should be a pdu connected to the CANbus
#define EXPECT_ECUMASTER_PMU16 (0x01U<<1) //if this bit is set, that means there should be a PMU16 somewhere
#define EN_SW_OCP (0x01U<<2) //Enables SW OCP
#define EN_DYNAMIC_LOAD_SHEDDING (0x01U<<3) //Enables dynamic load shedding

#define ROUND_PWM_ON_BINARY_CHANNELS (0x01U<<8) //For case of setting PWM on a bin channel
#define USE_ABS_VAL_MAPPING_HBRIDGE_TO_PWM (0x01U<<9)

//The base config settings for CONIFER
typedef struct conifer_settings{
	uint32_t conif_flags;

	uint16_t OCP_flt_time; //ms
	uint16_t OCP_load_shed_time; //ms

	uint16_t sys_fault_current; //Fault current of entire sys
	uint16_t sys_cont_current; //Continous Current Rating of system
	uint16_t sys_excess_I2T; //in A*ms
	uint16_t sys_ocp_timeout;

	uint16_t sys_max_voltage;
	uint16_t sys_lv_threshold_voltage; //If V_SYS drops below this value, it will trigger emergency load shedding
	uint16_t sys_cutoff_voltage; //If V_SYS drops below this voltage

	//These vars are for from the future, you wouldnt understand them yet
	uint16_t regbusA_target_voltage;
	uint16_t regbusA_fault_current;
	uint16_t regbusA_continous_current;

	uint16_t regbusB_target_voltage;
	uint16_t regbusB_fault_current;
	uint16_t regbusB_continous_current;

	uint8_t n_ch;

}conifer_settings;

//Enum representing the physical location of a conifer channel
typedef enum conifer_ch_location{
	LOCAL_CH = 0b000,
	UV19_PDU_CH = 0b001,
	ECUMASTER_PMU16_CH = 0b010,
	RESERVED_CH_POS1 = 0b011,
	RESERVED_CH_POS2 = 0b100,
	RESERVED_CH_POS3 = 0b101,
	RESERVED_CH_POS4 = 0b110,
	INVALID_MAPPING = 0b111
}conifer_ch_location;


//This is the abstract layer of a conifer channel
typedef struct abstract_conifer_channel{
	uint16_t status_control_reg;
	uint16_t hardware_mapping;
}abstract_conifer_channel;

//Macros for use with the conifer_status_control_reg
#define CONIFER_CH_EN_BIT (0x0001U<<0)
#define CONIFER_CH_IS_CRIT_BIT (0x0001U<<1)
#define CONIFER_CH_PSRC_BIT (0x0001U<<2)
#define CONIFER_CH_HB_DIR_MASK ((0x0001U<<3)|(0x0001U<<4))
#define CONIFER_CH_EN_OCP (0x0001U<<5)

//Macros for usage with the hardware_mapping field of the abstract_conifer_channel
#define CONIFER_CH_IN_USE (0x0001U<<15)
#define CONIFER_CH_LOC_MASK (0x0700U)
#define CONIFER_HW_CH_ID_MASK (0x00FF)

#define IS_CONIFER_CH_USED(ch) (ch->hardware_mapping&CONIFER_CH_IN_USE)
#define GET_CONIFER_CH_LOC(ch) ((ch->hardware_mapping&CONIFER_CH_LOC_MASK)>>8)

typedef enum conifer_hw_ch_type{ //This represents the type of a hardware channel
	BIN_CH = 0x00,
	DUAL_BUS_CH = 0x01,
	PWM_CH = 0x02,
	DUAL_BUS_PWM_CH = 0x03,
	H_BRIDGE = 0x04,
	INDEPENDENT_REG_CH = 0x05,
	EXT_BIN_CH = 0x06,
	SPECIAL = 0x07
}conifer_hw_ch_type;

//This is a lower level hardware level thing
typedef struct conifer_hw_channel{
	uint16_t ch_info_reg;

}conifer_hw_channel;

#define CONIFER_CH_TYPE_MASK 0x0007

//Initializes whatever conifer things need initialized
uv_status coniferInit();
uv_status coniferDeInit();


uv_status coniferEnChannel(conifer_output_channel ch); //
uv_status coniferDisChannel(conifer_output_channel ch);
uv_status coniferToggleChannel(conifer_output_channel ch);
uv_status coniferSetDutyCycle(conifer_output_channel ch);
uv_status coniferSetDirection(conifer_output_channel ch);
uv_status coniferSetDutyCycleAndDirection(conifer_output_channel ch);

uint16_t coniferGetChannelCurrent(conifer_output_channel ch);
uint16_t coniferGetChannelFbck(conifer_output_channel ch);
uint16_t coniferGetChannelFaults(conifer_output_channel ch);



#endif
