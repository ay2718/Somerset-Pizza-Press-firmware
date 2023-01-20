/*
 * control.h
 *
 *  Created on: Aug 4, 2022
 *      Author: Aaron Yeisr
 *      Email: ayeiser@mit.edu
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "debounce.h"
#include "config.h"
#include "structs.h"

#define ECO_TIMEOUT 900000ul // 15 minutes = 900 seconds

// duty cycle
#define MAX_SLEW_RATE 0.007f

#define DUTY_CYCLE_FAST 0.99f
#define DUTY_CYCLE_SLOW 0.5f
#define DUTY_CYCLE_JOG 0.3f


#define PRESS_TIME_FASTDROP 1000
#define PRESS_TIME_TAP_UP 700
#define PRESS_TIME_TAP_DOWN 2000


// overcurrent protection
#define MOTOR_CURRENT_MAX 14.0f
#define MOTOR_CURRENT_HIGH 10.0f
#define MOTOR_CURRENT_LOW 5.0f

#define MAX_CURRENT_SQR (MOTOR_CURRENT_MAX*MOTOR_CURRENT_MAX)
#define CURRENT_FILT 0.05f


// current measurement
#define R_SHUNT 0.002f
#define ADC_VOLTAGE 3.3f
#define SHUNT_GAIN 20.0f
#define ADC_CONV_FACTOR (ADC_VOLTAGE / (SHUNT_GAIN * R_SHUNT * 4096.0f))

// thermal control
#define THERM_FILTER_COEFF 0.01f	// update rate of 250 Hz --> 400 ms time constant
#define THERM_DEADBAND 0.5f

// error code flags
#define PRESS_OK 0u

#define ERR_INTERLOCK 1ul
#define ERR_OVERCURRENT (1ul << 1)
#define ERR_BAD_MOTOR (1ul << 2)
#define ERR_BAD_SWITCH (1ul << 3)
#define ERR_OVERSHOOT (1ul << 4)

#define ERR_BAD_TOP_THERMO1 (1ul << 5)
#define ERR_BAD_BOTTOM_THERMO1 (1ul << 6)
#define ERR_BAD_TOP_THERMO2 (1ul << 7)
#define ERR_BAD_BOTTOM_THERMO2 (1ul << 8)

#define ERR_TOP_THERMO_MISMATCH (1ul << 9)
#define ERR_BOTTOM_THERMO_MISMATCH (1ul << 10)

#define ERR_TOP_HEATER (1ul << 11)
#define ERR_BOTTOM_HEATER (1ul << 12)

uint32_t check_interlocks(Press*);
void motor_state_machine(TIM_HandleTypeDef*, Press*);

float get_shunt_current(ADC_HandleTypeDef*);
float motor_pi_update(MotorPI*, float);
void motor_pwm_update(TIM_HandleTypeDef*, Press*, float);

HAL_StatusTypeDef read_thermocouples(SPI_HandleTypeDef*, Press*);
void thermal_control_loop(SPI_HandleTypeDef*, Press*);

int getTopTempDisplay(Press*);
int getBottomTempDisplay(Press*);

#endif /* INC_CONTROL_H_ */
