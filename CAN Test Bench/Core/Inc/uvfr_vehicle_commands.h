/*
 * uvfr_vehicle_commands.h
 *
 *  Created on: Nov 27, 2024
 *      Author: byo10
 */

/** @defgroup uvfr_veh_commands UVFR Vehicle Commands
 *
 * @brief A fun lil API which is used to get the vehicle to do stuff
 *
 * This is designed to be portable between different versions of the VCU and PMU
 *
 */

#ifndef INC_UVFR_VEHICLE_COMMANDS_H_
#define INC_UVFR_VEHICLE_COMMANDS_H_

#include "uvfr_global_config.h"
#include "uvfr_utils.h"

#if (UV19_PDU + ECUMASTER_PMU) > 1
#error "Invalid PDU configuration"
#endif

/*
 *
 */
typedef struct output_channel{
	uint16_t I_max;
	uint8_t flags;
	uint8_t chID;
}output_channel;

/*
 *
 */
typedef struct output_channel_settings{
	uint16_t var;
}output_channel_settings;

#ifndef uvOpenSDC


	#define uvOpenSDC() coniferDisChannel(HVIL_PWR)
#endif

#ifndef uvCloseSDC

	#define uvCloseSDC() coniferEnChannel(HVIL_PWR)

#endif //Close SDC

#ifndef uvStartFans

#define uvStartFans() do{\
		coniferEnChannel(RAD_FANS1);\
		coniferEnChannel(RAD_FANS2);\
}while(0);

#endif //start fans

#ifndef uvStopFans

#define uvStopFans(x) do{\
		coniferDisChannel(RAD_FANS1);\
		coniferDisChannel(RAD_FANS2);\
}while(0);

#endif //start fans

#ifndef uvStartCoolantPump

#define uvStartCoolantPump() confiferEnChannel(COOLANT_PUMP1)

#endif

//Coolant pump
#ifndef uvStopCoolantPump

	#define uvStopCoolantPump() confiferDisChannel(COOLANT_PUMP1)

#endif

//Turn on the horn
void uvActivateHorn();

//Turn off the horn
void uvSilenceHorn();

//Beep Beep
void BeepBeepMotherFucker();

//Put vehicle into a safe state
void uvSecureVehicle();


#endif /* INC_UVFR_VEHICLE_COMMANDS_H_ */
