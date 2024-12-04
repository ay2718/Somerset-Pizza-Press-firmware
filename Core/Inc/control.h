/**
 * @file control.h
 * @author Aaron Yeiser
 * @brief 760 Pizza Press mechanical and thermo controls
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "main.h"
#include "debounce.h"
#include "config.h"
#include "structs.h"

#define ECO_TIMEOUT 900000ul // 15 minutes = 900 seconds

// duty cycle
#define MAX_SLEW_RATE 0.01f

#define DUTY_CYCLE_FAST 0.99f
#define DUTY_CYCLE_SLOW 0.5f
#define DUTY_CYCLE_JOG 0.3f

// time units in milliseconds
#define PRESS_TIME_FASTDROP 1000
#define PRESS_TIME_TAP_UP 700
#define PRESS_TIME_TAP_DOWN 2000


// overcurrent protection (amps)
#define MOTOR_CURRENT_MAX 14.0f
#define MOTOR_CURRENT_HIGH 8.0f
#define MOTOR_CURRENT_LOW 4.0f

#define MAX_CURRENT_SQR (MOTOR_CURRENT_MAX*MOTOR_CURRENT_MAX)
#define CURRENT_FILT 0.05f


// current measurement
#define R_SHUNT 0.002f
#define ADC_VOLTAGE 3.3f
#define SHUNT_GAIN 43.0f
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

/**
 * @brief Check press safety interlocks
 * @param press Press state
 *
 * ERR_INTERLOCK flag is set if the tray is open (interlock = 1) and the press is not homed to the top of travel
 * ERR_INTERLOCK flag is set if the tray is open and the press state is not READY or DONE
 * ERR_BAD_SWITCH flag is set if the top and bottom switches are both tripped
 *
 * @return error state flags
 */
uint32_t check_interlocks(Press* press);

/**
 * @brief Press mechanical state machine
 * @param htim PWM timer for motor control
 * @param press Press state
 *
 * @note press->press_state.ticks_until_next is used to execute an action after some time delay
 *
 * PRESS_READY: The press is sitting at top stroke.  It will start descending when buttons are pressed
 * 		All relevant state variables are reset
 *
 * PRESS_ERROR: Something bad happened. Slowly jog the press up to top of stroke at low current
 * 		PRESS_DONE state is entered once the press is homed
 *
 * PRESS_DOWN: Move the press down
 * 		If cycle mode is PRESS_FASTDROP (exiting READY mode) move down quickly for PRESS_TIME_FASTDROP ticks
 * 		Once fastdrop is done we limit the drop speed to avoid crashing the press
 *
 * 		In manual mode we continue pressing until the bottom limit switch is tripped or the buttons released
 * 		In auto mode we continue pressing until a timeout is tripped or bottom limit switch is tripped
 *
 * PRESS_DWELL: Press is all the way down
 * 		Handle dough tapping in auto mode (count number of taps remaining)
 * 		Start upward motion if buttons released (manual mode) or tapping timeout (auto mode)
 *
 * PRESS_UP: Move the press up (slow if tapping dough, fast otherwise)
 *		Move the press down if in manual mode or buttons pressed
 *		Move the press down if tapping dough and tap counter is nonzero
 *
 * PRESS_DONE: Immediately goes to PRESS_READY after buttons released
 * 		This prevents the press from immediately cycling in manual mode
 *
 * PRESS_JOG: Jog mode, used to move the platen up and down with menu buttons
 *
 */
void motor_state_machine(TIM_HandleTypeDef* htim, Press* press);

/**
 * @brief Read the ADC current in Amps
 * @param hadc the ADC handle to read
 * @return float the current
 */
float get_shunt_current(ADC_HandleTypeDef* hadc);

/**
 * @brief Run PI controller for motor control
 * @param state PI controller state
 * @param err setpoint - measured error
 * @return controller effort
 *
 * @note this is not currently used
 */
float motor_pi_update(MotorPI* state, float err);

/**
 * @brief Set motor duty cycle with slew rate limit and current limit
 * @param htim PWM timer handle
 * @param press Press state
 * @param current Measured press current
 */
void motor_pwm_update(TIM_HandleTypeDef* htim, Press* press, float current);

/**
 * @brief Read SPI thermocouple readers
 * @param hspi SPI handle
 * @param press Press state
 * @return HAL_StatusTypeDef SPI read status
 *
 * @note we cycle through thermocouples, only one of four TCs is read per tick
 */
HAL_StatusTypeDef read_thermocouples(SPI_HandleTypeDef* hspi, Press* press);

/**
 * @brief run thermal bang-bang control loop
 * @param hspi SPI handle for reading thermocouples
 * @param press Press state
 *
 *
 */
void thermal_control_loop(SPI_HandleTypeDef* hspi, Press* press);

/**
 * @brief Top temperature rounded to int and in the correct units
 * @param press Press state
 * @return int Temperature to display
 */
int getTopTempDisplay(Press* press);

/**
 * @brief Bottom temperature rounded to int and in the correct units
 * @param press Press state
 * @return int Temperature to display
 */
int getBottomTempDisplay(Press* press);

#endif /* INC_CONTROL_H_ */
