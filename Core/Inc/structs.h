#ifndef STRUCTS_H_
#define STRUCTS_H_

#include "main.h"

typedef enum {
	PRESS_READY,
	PRESS_ERROR,
	PRESS_DOWN,
	PRESS_DWELL,
	PRESS_UP,
	PRESS_DONE,
	PRESS_JOG
} PressMode;

typedef enum {
	PRESS_FASTDROP,
	PRESS_PERIOD1,
	PRESS_TAPS,
	PRESS_PERIOD2
} PressCycleMode;

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

typedef struct {
	int ctr;
	int repeat_ctr;
	bool state;
	bool rising_edge_flag;
	bool falling_edge_flag;
} Button;

typedef struct {
	int16_t burps;
	int16_t press_ticks1;
	int16_t press_ticks2;
	bool auto_mode;
	bool enable;
} PressSetpoint;

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

typedef struct {
	float top_temp;
	float bottom_temp;
	bool enable;
} ThermalSetpoint;

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
} ThermalState;

// this allows us to map config settings to registers
typedef	union {
	uint32_t regs[5];
	struct {
		uint16_t flags;
		int16_t top_temp;
		int16_t bottom_temp;
		int16_t press_time1;
		int16_t press_time2;
		int16_t burps;
	};
} Config;

typedef struct {
	PressSetpoint press_setpoint;
	PressState press_state;
	ThermalSetpoint thermal_setpoint;
	ThermalState thermal_state;
	Config config;
} Press;

// TI is the integral time
// If implemented, TD is the derivative time (1/KD)
typedef struct {
	float KP;
	float TI;
	float accum;
	float max_accum;
} MotorPI;

typedef struct __MenuItem{
	// 0: menu
	// 1: numerical entry
	// 2: yes/no entry
	MenuType type;

	uint16_t length;  	// menu length
	int16_t upper;		// numerical upper bound

	uint16_t index;		// menu index
	int16_t lower;		// numerical lower bound

	int16_t step;	// numerical entry step
	int16_t flag;	// yes/no entry flag mask

	// if applicable, target modified by menu action
	int16_t value;
	int16_t* target;

	// item name
	char name[16];

	// NULL if top level
	struct __MenuItem* parent;

	// entries are NULL if not defined
	struct __MenuItem* items[16];

	HAL_StatusTypeDef (*display)(struct __MenuItem*);
} MenuItem;

#endif
