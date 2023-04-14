/*
 * backup_rtc.h
 *
 *  Created on: Aug 10, 2022
 *      Author: ayeiser
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include "structs.h"

// 0 (1)
#define CONFIG_MODE_FLAG (1u << 1)		// Manual (Auto)
#define CONFIG_UNITS_FLAG (1u << 2)		// Fahrenheit (Celsius)
#define CONFIG_BUZZER_FLAG (1u << 3)	// Off (On)
#define CONFIG_ECO_FLAG (1u << 4)		// Off (On)

// if this bit is not 1, the backup registers were reset
#define DEFAULT_CONFIG_FLAGS 1u
#define DEFAULT_TOP_TEMP -40
#define DEFAULT_BOTTOM_TEMP -40
#define DEFAULT_PRESS_TIME 1000			// milliseconds
#define DEFAULT_BURPS 1

#define BURPS_LOWER_LIM 0
#define BURPS_UPPER_LIM 5

#define PRESS_TIME_LOWER_LIM 500
#define PRESS_TIME_UPPER_LIM 10000

#define TEMP_LOWER_LIM_F 120
#define TEMP_UPPER_LIM_F 270 // 400

#define TEMP_LOWER_LIM_C 50
#define TEMP_UPPER_LIM_C 130 // 205

void backup_settings(Config*);
void restore_settings(Config*);
void reset_defaults(Config*);
void config_to_setpoints(Press*);


#endif /* INC_CONFIG_H_ */
