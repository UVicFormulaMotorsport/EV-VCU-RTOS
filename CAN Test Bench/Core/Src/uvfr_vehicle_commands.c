#include "uvfr_utils.h"

typedef struct output_channel_settings output_channel_settings;

void BeepBeepMotherFucker(){
	coniferEnChannel(HORN);
	vTaskDelay(300);
	coniferToggleChannel(HORN);
	vTaskDelay(300);
	coniferToggleChannel(HORN);
	vTaskDelay(300);
	coniferDisChannel(HORN);
}


/** @brief Function to put vehicle into safe state.
 *
 * Should perform the following functions in order:
 * - Prevent new MC torque or speed requests
 * - Open shutdown cct
 *
 */
void uvSecureVehicle(){
	//Stop MCU Torque requests
	MC_Shutdown();

	coniferDisChannel(BAMO_RFE);
	vTaskDelay(2);
	//open SDC >:)
	uvOpenSDC();

}
