/**
 * @file control.c
 * @author Aaron Yeiser
 * @brief 760 Pizza Press mechanical and thermo controls
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#include "control.h"

// external variables
uint32_t ticks = 0;
extern uint16_t adc_output;
extern float shunt_current;
extern float max_current;
float shunt_current_sqr_filt = 0.0f;

//internal variables
uint32_t press_count = 0;
bool cycle_mode = 0;
bool cycle_state = 0;
uint32_t cycle_ticks = 5000;
uint8_t active_thermocouple = 0;
uint8_t thermo_buf[4];
volatile bool adc_ready;
volatile uint16_t buzzer_ctr = 0;
volatile uint16_t white_led_ctr = 0;
volatile uint16_t blue_led_ctr = 0;

/// Signal the ADC is ready when new data has occurred
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adc_ready = true;
}

/**
 * @brief Write to LED lights and buzzer.  Called once per tick
 * @param press Press state
 */
static void lights_and_buzzers(Press *press) {
	if (buzzer_ctr > 0) {
		buzzer_ctr--;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	} else {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
	}

	if (press->thermal_state.top_ready && press->thermal_state.bottom_ready) {
		__WRITE_WHITE_LED(1);
	}
	else
	{
		__WRITE_WHITE_LED(0);
	}
//	else if (white_led_ctr++ >= 1000) {
//		__TOGGLE_WHITE_LED();
//		white_led_ctr = 0;
//	}

	if (press->press_state.mode == PRESS_READY) {
		__WRITE_BLUE_LED(1);
	} else if (blue_led_ctr++ >= 200) {
		__TOGGLE_BLUE_LED();
		blue_led_ctr = 0;
	}
}

/// This gets called every millisecond
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM1) {
		// Process interlock state
		// Up/down magnets, pizza door latch

		// Process button presses
		// Set flag to update screen (if data changed)

		// execute motor control loop
		debounce_interlock();
		debounce_activate_buttons();

		motor_state_machine(&htim1, &press);

		debounce_menu_buttons();

		read_thermocouples(&hspi1, &press);
		thermal_control_loop(&hspi1, &press);
		lights_and_buzzers(&press);
		ticks++;

	}
}

uint32_t check_interlocks(Press* press) {
	int err = PRESS_OK;

	// interlock switches are normally open with pullups
	// 0 is on, 1 is off
	if (tray_interlock.state) {  // if tray is out
		if (press_top_limit.state) {  // if press is not at top
			err |= ERR_INTERLOCK;
		}
		if (press->press_state.mode != PRESS_READY && press->press_state.mode != PRESS_DONE) {
			err |= ERR_INTERLOCK;
		}
	}

	if (!press_top_limit.state && !press_bottom_limit.state) {
		err |= ERR_BAD_SWITCH;
	}

	if (err != PRESS_OK) {
		press->press_state.mode = PRESS_ERROR;
		press->press_state.error_code |= err;
	}

	return err;
}

void motor_state_machine(TIM_HandleTypeDef *htim, Press* press) {

	// If press mechanical side is disabled, set press to safe values and return
	if (!press->press_setpoint.enable) {
		press->press_state.motor_setpoint = 0.0f;
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_slew_limited_setpoint = 0.0f;
		motor_pwm_update(&htim1, press, shunt_current);
		return;
	}

	// Retrieve button states
	bool top_lim = press_top_limit.state;
	bool bottom_lim = press_bottom_limit.state;
	bool tray_open = tray_interlock.state;

	bool left_button = activate_left_button.state;
	bool right_button = activate_right_button.state;

	bool press_active = (!left_button) && (!right_button) && (!tray_open);

	// Cycle mode for testing, this allows the press to run indefinitely
	// Not recommended for production
#ifdef CYCLE_MODE
	if (cycle_mode) {
		press_active |= (cycle_state && (!tray_open));
		if (cycle_ticks-- <= 0) {
			if (cycle_state) {
				cycle_ticks = 5000;
				cycle_state = 0;
			} else {
				cycle_ticks = 3000;
				cycle_state = 1;
			}
		}
		if (press->press_state.mode != PRESS_READY && cycle_state == 0) {
			cycle_ticks = 5000;
		}
	} else {
		cycle_ticks = 5000;
	}
#endif


	// Throw an error state if the interlock is tripped
	check_interlocks(press);

	// Measure motor current and throw ERR_OVERCURRENT flag if overcurrent tripped
	shunt_current = get_shunt_current(&hadc);
	shunt_current_sqr_filt = (1.0f - CURRENT_FILT) * shunt_current_sqr_filt + CURRENT_FILT * shunt_current*shunt_current;
	if (shunt_current_sqr_filt > MAX_CURRENT_SQR) {
		press->press_state.mode = PRESS_ERROR;
		press->press_state.error_code |= ERR_OVERCURRENT;
	}

	// max_current is used to track maximum current draw per cycle for debugging
	max_current = max(max_current, shunt_current);
	max_current = max(max_current, -shunt_current);

	switch (press->press_state.mode) {

	case PRESS_READY:

		// Reset all relevant parameters
		max_current = 0.0f;
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_setpoint = 0.0f;
		press->press_state.dwell_timer = 0;
		config_to_setpoints(press);

		// if somehow we're not at the top at the ready state
		if (top_lim) {
			press->press_state.error_code |= ERR_OVERSHOOT;
		}

		// both buttons pressed and tray closed
		// setup everything, start press movement
		if (press_active) {
			press->press_state.ticks_until_next = PRESS_TIME_FASTDROP;
			press->press_state.burp_ctr = press->press_setpoint.burps;
			press->press_state.cycle = PRESS_FASTDROP;
			press->press_state.mode = PRESS_DOWN;
		}
		break;

	case PRESS_ERROR:

		// Low speed and low current
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = -DUTY_CYCLE_JOG;

		// top limit switch triggered
		if (!top_lim) {
			press->press_state.mode = PRESS_DONE;
//			press->press_state.motor_setpoint = 0.0f;
		}
		break;

	case PRESS_DOWN:

		// High current setting, high speed if FASTDROP is set
		// FASTDROP is only set after READY to rapidly move the press down
		// It must be unset after a set time to prevent crashing the press
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		if (press->press_state.cycle == PRESS_FASTDROP) {
			press->press_state.motor_setpoint = DUTY_CYCLE_FAST;
		} else {
			press->press_state.motor_setpoint = DUTY_CYCLE_SLOW;
		}

		// Timer event handling
		if (press->press_state.ticks_until_next-- <= 0) {
			if (press->press_state.cycle == PRESS_FASTDROP) {
				// If we're in fastdrop mode, exit fastdrop and enter burp mode
				press->press_state.cycle = PRESS_PERIOD1;
				press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
			} else if (press->press_setpoint.auto_mode){
				// If not fastdropping and in auto mode, go to dwell mode
				press->press_state.mode = PRESS_DWELL;
			}
		}

		// Press has bottomed out, go to dwell mode
		if (!bottom_lim) {
			press->press_state.mode = PRESS_DWELL;
			if (press->press_state.cycle == PRESS_FASTDROP) {
				press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
				press->press_state.cycle = PRESS_PERIOD1;
			}
		}

		// Bring press up if in manual mode and buttons released
		if (!press->press_setpoint.auto_mode && !press_active) {
			press->press_state.mode = PRESS_UP;
		}

		// auto mode and not down, buttons released
		// Operator must hold the buttons for at least a second or so in auto mode
		if (press->press_setpoint.auto_mode
				&& (press->press_state.cycle == PRESS_FASTDROP)
				&& !press_active) {
			press->press_state.motor_setpoint = 0.0f;
			press->press_state.mode = PRESS_ERROR;
		}

		press->press_state.dwell_timer = 0;
		break;

	case PRESS_DWELL:

		// Set low current limit and command zero motion
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		//press->press_state.motor_setpoint = 0.0f;
		press->press_state.dwell_timer++;

		// if the bottom sensor is not triggered something bad has happened
		if (bottom_lim) {
			//press->press_state.error_code |= ERR_OVERSHOOT;
			if ((press->press_state.dwell_timer > 250) && (fabsf(shunt_current) < 4.5f)) {
				press->press_state.motor_setpoint = 0.15f;
			} else {
				press->press_state.motor_setpoint = 0.0f;
			}
		} else {
			press->press_state.motor_setpoint = 0.0f;
		}

		// Manual mode and press buttons released--press goes up
		if (!press->press_setpoint.auto_mode && !press_active) {
			press->press_state.mode = PRESS_UP;
		}

		// Event occurs--in auto mode this means dough tapping timeout is reached
		if (press->press_setpoint.auto_mode &&
				(press->press_state.ticks_until_next-- <= 0))
		{
			switch (press->press_state.cycle) {

			// If cycle mode is dropping the press, set it to PRESS_TAPS
			case PRESS_FASTDROP:
			case PRESS_PERIOD1:
				press->press_state.cycle = PRESS_TAPS;
				break;

			// Execute once per tap cycle: generate event to trigger moving the press up
			case PRESS_TAPS:
				if (press->press_state.burp_ctr > 0) {
					// if taps remaining, move press up for PRESS_TIME_TAP_UP ticks
					press->press_state.mode = PRESS_UP;
					press->press_state.ticks_until_next = PRESS_TIME_TAP_UP;
				} else {
					// otherwise hold down for press_ticks2 ticks
					press->press_state.cycle = PRESS_PERIOD2;
					press->press_state.ticks_until_next = press->press_setpoint.press_ticks2;
				}
				break;

			// Move the press up
			case PRESS_PERIOD2:
			default:
				press->press_state.mode = PRESS_UP;
				break;
			}
		}
		break;

	case PRESS_UP:
		// Use low current. Move press slowly if tapping dough, fast otherwise
		if (press->press_setpoint.auto_mode && (press->press_state.burp_ctr > 0)) {
			press->press_state.current_limit = MOTOR_CURRENT_LOW;
			press->press_state.motor_setpoint = -DUTY_CYCLE_SLOW;
		} else {
			press->press_state.current_limit = MOTOR_CURRENT_LOW;
			press->press_state.motor_setpoint = -DUTY_CYCLE_FAST;
		}

		// Tap timer expired: decrement tap counter and bring the press down again
		if (press->press_setpoint.auto_mode
				&& (press->press_state.burp_ctr > 0)
				&& (press->press_state.ticks_until_next-- <= 0)){
			press->press_state.burp_ctr--;
			press->press_state.mode = PRESS_DOWN;
			press->press_state.ticks_until_next = PRESS_TIME_TAP_DOWN;
		}

		// Manual mode and buttons pressed--bring the press down
		if (!press->press_setpoint.auto_mode && press_active) {
			press->press_state.cycle = PRESS_PERIOD1;
			press->press_state.mode = PRESS_DOWN;
			press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
		}

		// top limit reached, press id done!
		if (!top_lim) {
			press->press_state.mode = PRESS_DONE;
			press_count++;
			press->config.ctr++;
			backup_settings(&(press->config));
			if (press->config.flags & CONFIG_BUZZER_FLAG) {
				buzzer_ctr = 200;
			}
		}
		break;

	case PRESS_DONE:
		// High current limit and no motion
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_setpoint = 0.0f;


		// both buttons released
		if (left_button && right_button) {
			press->press_state.mode = PRESS_READY;
		}
		break;

	case PRESS_JOG:
		// Low current mode and default of no motion
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = 0.0f;

		// Move press up if top limit switch not tripped and up button pressed
		if (top_lim && menu_up_button.state && !menu_down_button.state) {
			press->press_state.motor_setpoint = -DUTY_CYCLE_JOG;
		}

		// Move press down if bottom limit switch not tripped and down button pressed
		if (bottom_lim && menu_down_button.state && !menu_up_button.state) {
			press->press_state.motor_setpoint = DUTY_CYCLE_JOG;
		}
		break;

	default:
		// Should never occur
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = 0.0f;
		press->press_state.error_code |= ERR_OVERSHOOT;
	}

	// Set motor duty cycle (with slew rate limit)
	motor_pwm_update(&htim1, press, shunt_current);
}

float get_shunt_current(ADC_HandleTypeDef *hadc) {
	while(!adc_ready);
	adc_ready = false;
	return ADC_CONV_FACTOR * ((float) (adc_output - 2048));
}

float motor_pi_update(MotorPI *state, float err) {
	float scaled_err = err * state->KP;
	state->accum += scaled_err * (state->TI / PWM_FREQ);

	// enforce limits on integrator
	state->accum = state->accum < state->max_accum ? state->accum : state->max_accum;
	state->accum = state->accum > -state->max_accum ? state->accum : -state->max_accum;

	return state->accum + scaled_err;
}

void motor_pwm_update(TIM_HandleTypeDef *htim, Press* press, float current) {
	const float duty_to_cmd = ((float) PWM_PERIOD);
	// Channel 1 has the shunt, channel 2 does not
	// Forward:  Channel 1 ground, channel 2 PWM
	// Current is inverted!!!!

	// Ensure press setpoint is between -0.99 and 0.99
	press->press_state.motor_setpoint =
			clip(press->press_state.motor_setpoint,
					DUTY_CYCLE_FAST,
					-DUTY_CYCLE_FAST);
	float step;
	const float deadband = 0.5f;
	float itarget;
	int8_t dir;

	if (press->press_state.motor_setpoint > 0.0f) {
		// if press is moving down, we want positive current
		itarget = press->press_state.current_limit - deadband;
		dir = 1;
	} else if (press->press_state.motor_setpoint < 0.0f) {
		// if press is moving up we want negative current
		itarget = -press->press_state.current_limit + deadband;
		dir = -1;
	} else {
		// zero current if press is not moving
		itarget = 0.0f;
		dir = 0;
	}

	// try to speed up press if current is too low
	// try to slow down press if current is too high
	if (-current > itarget + deadband) {
		step = -MAX_SLEW_RATE;
	} else if (-current < itarget - deadband) {
		step = MAX_SLEW_RATE;
	} else {
		step = 0.0f;
	}

	// Prevent duty cycle from exceeding motor_slew_limited_setpoint
	float step_max = clip(press->press_state.motor_setpoint - press->press_state.motor_slew_limited_setpoint,
			MAX_SLEW_RATE,
			-MAX_SLEW_RATE);

	if (dir == 1) {
		step = min(step, step_max);
	} else if (dir == -1) {
		step = max(step, step_max);
	} else {
		step = step_max;
	}

	press->press_state.motor_slew_limited_setpoint =
			clip(press->press_state.motor_slew_limited_setpoint + step,
					DUTY_CYCLE_FAST,
					-DUTY_CYCLE_FAST);


	// Set PWM duty cycle from slew limited setpoint
	int16_t pwm_cmd = (int16_t) (press->press_state.motor_slew_limited_setpoint * duty_to_cmd);
	if (pwm_cmd > 0) {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_PERIOD);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_PERIOD-pwm_cmd);
	}
	// Reverse:  Channel 1 PWM (must not exceed PWM_PERIOD - SHUNT_TICKS), Channel 2 ground
	else {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, PWM_PERIOD+pwm_cmd);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, PWM_PERIOD);
	}
}

HAL_StatusTypeDef read_thermocouples(SPI_HandleTypeDef *hspi, Press* press) {
	if (hspi->State == HAL_SPI_STATE_READY) {
		// bring correct CS low
		switch (active_thermocouple) {
		case 0: // top 1
			__WRITE_THERMO_TOP1_CS(0);
			break;
		case 1: // bottom 1
			__WRITE_THERMO_BOTTOM1_CS(0);
			break;
		case 2: // top 2
			__WRITE_THERMO_TOP2_CS(0);
			break;
		case 3: // bottom 2
			__WRITE_THERMO_BOTTOM2_CS(0);
			break;
		}
		HAL_StatusTypeDef status = HAL_SPI_Receive(hspi, thermo_buf, 4, 1);
		// set all CS pins high
		__WRITE_THERMO_TOP1_CS(1);
		__WRITE_THERMO_TOP2_CS(1);
		__WRITE_THERMO_BOTTOM1_CS(1);
		__WRITE_THERMO_BOTTOM2_CS(1);

		// Detect if any fault bits are set (see thermocouple reader datasheet)
		bool fault_state = (thermo_buf[1] & 0b1u) || (thermo_buf[3] & 0b111u);
//		bool fault_state = thermo_buf[3] & 0b101u;
		if (fault_state) {
			press->thermal_state.error_code |= (ERR_BAD_TOP_THERMO1 << active_thermocouple);
			press->thermal_state.bad_read_countdown[active_thermocouple] = 1000;
		} else {
			if (press->thermal_state.bad_read_countdown[active_thermocouple] == 0) {
				press->thermal_state.error_code &= (~(ERR_BAD_TOP_THERMO1 << active_thermocouple));
			} else {
				press->thermal_state.bad_read_countdown[active_thermocouple]--;
			}
		}
		// decode (big-endian) temperature
		int16_t temp_raw = (thermo_buf[0] << 8) + thermo_buf[1];
		float measured_temp = ((float) temp_raw) * 0.0625f * THERMO_SCALING_FACTOR;

		// Low pass filtering
		press->thermal_state.temp_buf[active_thermocouple] =
				(1.0f - THERM_FILTER_COEFF) * press->thermal_state.temp_buf[active_thermocouple] +
				THERM_FILTER_COEFF * measured_temp;

		// Read next thermocouple
		active_thermocouple++;
		active_thermocouple &= 0b11;

		return status;
	} else {
		return HAL_BUSY;
	}
}

void thermal_control_loop(SPI_HandleTypeDef* hspi, Press* press) {

	// Read top platen thermocouple and backup thermocouple.
	// Fallback to backup if main thermocouple is bad, throw error if both are bad
	if (!(press->thermal_state.error_code & ERR_BAD_TOP_THERMO1)) {
		press->thermal_state.top_temp = press->thermal_state.top1;
		press->thermal_state.error &= ~1;
	} else if (!(press->thermal_state.error_code & ERR_BAD_TOP_THERMO2)) {
		press->thermal_state.top_temp = press->thermal_state.top2;
		press->thermal_state.error &= ~1;
	} else {
		press->thermal_state.top_temp = 1000.0f; // set unreasonably high to guarantee controller turns off
//		press->thermal_setpoint.enable = false;
		press->thermal_state.error |= 1;
		__WRITE_TOP_PLATTER_HEAT(0);
//		__WRITE_BOTTOM_PLATTER_HEAT(0);
//		return;
	}

	// Read bottom platten thermoucouple and backup thermocouple
	if (!(press->thermal_state.error_code & ERR_BAD_BOTTOM_THERMO1)) {
		press->thermal_state.bottom_temp = press->thermal_state.bottom1;
		press->thermal_state.error &= ~2;
	} else if (!(press->thermal_state.error_code & ERR_BAD_BOTTOM_THERMO2)) {
		press->thermal_state.bottom_temp = press->thermal_state.bottom2;
		press->thermal_state.error &= ~2;
	} else {
		press->thermal_state.bottom_temp = 1000.0f; // set unreasonably high
//		press->thermal_setpoint.enable = false;
		press->thermal_state.error |= 2;
//		__WRITE_TOP_PLATTER_HEAT(0);
		__WRITE_BOTTOM_PLATTER_HEAT(0);
//		return;
	}

	// If press heaters are disabled, turn off elements and return
	if (!press->thermal_setpoint.enable) {
		__WRITE_TOP_PLATTER_HEAT(0);
		__WRITE_BOTTOM_PLATTER_HEAT(0);
		press->thermal_state.top_ready = 0;
		press->thermal_state.bottom_ready = 0;
		return;
	}

	// If temperature are more than 3 degrees low, signal that press is not ready
	if (press->thermal_setpoint.top_temp - press->thermal_state.top_temp > 3.0f) {
		press->thermal_state.top_ready = false;
	}
	if (press->thermal_setpoint.bottom_temp - press->thermal_state.bottom_temp > 3.0f) {
		press->thermal_state.bottom_ready = false;
	}

	// Set heating elements on/off if under/over setpoint temperature
	press->thermal_state.top_ssr_on = (press->thermal_state.top_temp < press->thermal_state.top_threshold);
	press->thermal_state.bottom_ssr_on = (press->thermal_state.bottom_temp < press->thermal_state.bottom_threshold);

	__WRITE_TOP_PLATTER_HEAT(press->thermal_state.top_ssr_on);
	__WRITE_BOTTOM_PLATTER_HEAT(press->thermal_state.bottom_ssr_on);

	// Bang-bang deadband control to set top_threshold and bottom_threshold
	if (press->thermal_state.top_ssr_on) {
		press->thermal_state.top_threshold =
				press->thermal_setpoint.top_temp + THERM_DEADBAND;
	} else {
		press->thermal_state.top_threshold =
				press->thermal_setpoint.top_temp - THERM_DEADBAND;
		press->thermal_state.top_ready = true;
	}

	if (press->thermal_state.bottom_ssr_on) {
		press->thermal_state.bottom_threshold =
				press->thermal_setpoint.bottom_temp + THERM_DEADBAND;
	} else {
		press->thermal_state.bottom_threshold =
				press->thermal_setpoint.bottom_temp - THERM_DEADBAND;
		press->thermal_state.bottom_ready = true;
	}
}


int getTopTempDisplay(Press* press) {
	float temp;
	if (press->config.flags & CONFIG_UNITS_FLAG) {
		temp = press->thermal_state.top_temp;
	} else {
		temp = __C_TO_F_FLOAT(press->thermal_state.top_temp);
	}
	return lroundf(clip(temp, 999.0f, 0.0f));
}

int getBottomTempDisplay(Press* press) {
	float temp;
	if (press->config.flags & CONFIG_UNITS_FLAG) {
		temp = press->thermal_state.bottom_temp;
	} else {
		temp = __C_TO_F_FLOAT(press->thermal_state.bottom_temp);
	}
	return lroundf(clip(temp, 999.0f, 0.0f));
}






