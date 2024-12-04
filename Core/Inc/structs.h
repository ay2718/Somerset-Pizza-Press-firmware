/**
 * @file structs.h
 * @author Aaron Yeiser
 * @brief 760 Pizza Press structures
 * @date 2022-08-10
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "main.h"

/// State machine states for pressing dough
typedef enum {
	PRESS_READY,
	PRESS_ERROR,
	PRESS_DOWN,
	PRESS_DWELL,
	PRESS_UP,
	PRESS_DONE,
	PRESS_JOG
} PressMode;

/// State machine states for tapping the dough
typedef enum {
	PRESS_FASTDROP,
	PRESS_PERIOD1,
	PRESS_TAPS,
	PRESS_PERIOD2
} PressCycleMode;

/// Enum of menu types
typedef enum {
	MENU,
	MENU_NUM,
	MENU_FLAG,
	MENU_TEMP,
	MENU_RESET,
	MENU_TEMP_UNITS,
	MENU_STATUS,
	MENU_JOG,
	MENU_DEBUG,
	MENU_RESET_COUNT,
	MENU_CYCLE,
	MENU_OTHER,
} MenuType;

/// Debounced button state
typedef struct {
	int ctr;
	int repeat_ctr;
	bool state;
	bool rising_edge_flag;
	bool falling_edge_flag;
} Button;

/// Mechanical press setpoint data
typedef struct {
	int16_t burps;
	int16_t press_ticks1;
	int16_t press_ticks2;
	bool auto_mode;
	bool enable;
} PressSetpoint;

/// Mechanical press state data
typedef struct {
	PressMode mode;
	PressCycleMode cycle;
	bool overload_flag;
	int16_t burp_ctr;
	int16_t ticks_until_next;
	float motor_setpoint;
	float motor_slew_limited_setpoint;
	float current_limit;
	uint32_t error_code;
} PressState;

/// Thermal press setpoint data
typedef struct {
	float top_temp;
	float bottom_temp;
	bool enable;
} ThermalSetpoint;

/// Thermal press state data
typedef struct {
	union {
		float temp_buf[4];
		struct {
			float top1, bottom1, top2, bottom2;
		};
	};
	float top_temp;
	float bottom_temp;
	float top_threshold;
	float bottom_threshold;
	bool top_ready;
	bool bottom_ready;
	uint16_t bad_read_countdown[4];
	uint8_t error;
	uint32_t error_code;
	bool top_ssr_on;
	bool bottom_ssr_on;
} ThermalState;

/// Press configuration data stored in RTC registers
/// Functions in config.h
typedef	union {
	uint32_t regs[5];
	struct {
		uint16_t flags; //reg0
		int16_t top_temp;
		int16_t bottom_temp; //reg1
		int16_t press_time1;
		int16_t press_time2; //reg2
		int16_t burps;
		uint32_t ctr; //reg3
	};
} Config;

/// Container for all press mechanical and thermal setpoint/state
typedef struct {
	PressSetpoint press_setpoint;
	PressState press_state;
	ThermalSetpoint thermal_setpoint;
	ThermalState thermal_state;
	Config config;
} Press;

/// PI controller parameters and state
typedef struct {
	float KP;
	float TI;
	float accum;
	float max_accum;
} MotorPI;

/// Menu item structure--contains all data needed to display a menu item
typedef struct __MenuItem{
	/// 0: menu
	/// 1: numerical entry
	/// 2: yes/no entry
	MenuType type;

	uint16_t length;  	///< menu length
	int16_t upper;		///< numerical upper bound for value

	uint16_t index;		///< menu index
	int16_t lower;		///< numerical lower bound for value

	int16_t step;	///< numerical entry step
	int16_t flag;	///< yes/no entry flag mask


	int16_t value; ///< if applicable, target value modified by menu action
	int16_t* target; ///< pointer to target value to edit

	const char* name; ///< menu item name
	const char* titlename; ///< menu item title (usually shorter)

	struct __MenuItem* parent; ///< Null ptr if top level menu

	struct __MenuItem* items[16]; ///< Child menu entries are NULL if not defined

	HAL_StatusTypeDef (*display)(struct __MenuItem*); ///< Write to the display
} MenuItem;

#endif
