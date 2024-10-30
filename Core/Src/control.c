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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adc_ready = true;
}

void lights_and_buzzers(Press *press) {
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

// Use only if SPI is in interrupt or DMA mode
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
//	if (hspi->Instance == SPI1) {
//		// write all thermocouple CS pins high
//		__WRITE_THERMO_TOP1_CS(1);
//		__WRITE_THERMO_TOP2_CS(1);
//		__WRITE_THERMO_BOTTOM1_CS(1);
//		__WRITE_THERMO_BOTTOM2_CS(1);
//	}
//}

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
	if (!press->press_setpoint.enable) {
		press->press_state.motor_setpoint = 0.0f;
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_slew_limited_setpoint = 0.0f;
		motor_pwm_update(&htim1, press, shunt_current);
		return;
	}

	bool top_lim = press_top_limit.state;
	bool bottom_lim = press_bottom_limit.state;
	bool tray_open = tray_interlock.state;

	bool left_button = activate_left_button.state;
	bool right_button = activate_right_button.state;

	bool press_active = (!left_button) && (!right_button) && (!tray_open);
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

	shunt_current = get_shunt_current(&hadc);
	shunt_current_sqr_filt = (1.0f - CURRENT_FILT) * shunt_current_sqr_filt + CURRENT_FILT * shunt_current*shunt_current;
	if (shunt_current_sqr_filt > MAX_CURRENT_SQR) {
		press->press_state.mode = PRESS_ERROR;
		press->press_state.error_code |= ERR_OVERCURRENT;
	}
	max_current = max(max_current, shunt_current);
	max_current = max(max_current, -shunt_current);

	switch (press->press_state.mode) {

	case PRESS_READY:
		max_current = 0.0f;
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_setpoint = 0.0f;
		config_to_setpoints(press);
		if (top_lim) {
			// if somehow we're not at the top at the ready state
			press->press_state.error_code |= ERR_OVERSHOOT;
		}
		if (press_active) {
			// both buttons pressed and tray closed
			// setup everything
			press->press_state.ticks_until_next = PRESS_TIME_FASTDROP;
			press->press_state.burp_ctr = press->press_setpoint.burps;
			press->press_state.cycle = PRESS_FASTDROP;
			press->press_state.mode = PRESS_DOWN;
		}
		break;

	case PRESS_ERROR:
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = -DUTY_CYCLE_JOG;
		// top limit switch triggered
		if (!top_lim) {
			press->press_state.mode = PRESS_DONE;
//			press->press_state.motor_setpoint = 0.0f;
		}
		break;

	case PRESS_DOWN:
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		if (press->press_state.cycle == PRESS_FASTDROP) {
			press->press_state.motor_setpoint = DUTY_CYCLE_FAST;
		} else {
			press->press_state.motor_setpoint = DUTY_CYCLE_SLOW;
		}

		if (press->press_state.ticks_until_next-- <= 0) {
			if (press->press_state.cycle == PRESS_FASTDROP) {
				// exit fast drop mode
				press->press_state.cycle = PRESS_PERIOD1;
				press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
			} else if (press->press_setpoint.auto_mode){
				// go to dwell mode
				press->press_state.mode = PRESS_DWELL;
			}
		}

		// Press has bottomed out
		if (!bottom_lim) {
			press->press_state.mode = PRESS_DWELL;
			if (press->press_state.cycle == PRESS_FASTDROP) {
				press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
				press->press_state.cycle = PRESS_PERIOD1;
			}
		}

		// manual mode and buttons released
		if (!press->press_setpoint.auto_mode && !press_active) {
			press->press_state.mode = PRESS_UP;
		}

		// auto mode and not down, buttons released
		if (press->press_setpoint.auto_mode
				&& (press->press_state.cycle == PRESS_FASTDROP)
				&& !press_active) {
			press->press_state.motor_setpoint = 0.0f;
			press->press_state.mode = PRESS_ERROR;
		}
		break;

	case PRESS_DWELL:
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = 0.0f;

		// if the bottom sensor is not triggered
		if (bottom_lim) {
			//press->press_state.error_code |= ERR_OVERSHOOT;
			press->press_state.current_limit = MOTOR_CURRENT_LOW;
			press->press_state.motor_setpoint = 0.0f;
		}

		// Manual mode and press buttons released
		if (!press->press_setpoint.auto_mode && !press_active) {
			press->press_state.mode = PRESS_UP;
		}

		// Auto mode and ticks expired
		if (press->press_setpoint.auto_mode &&
				(press->press_state.ticks_until_next-- <= 0))
		{
			switch (press->press_state.cycle) {

			case PRESS_FASTDROP:
			case PRESS_PERIOD1:
				press->press_state.cycle = PRESS_TAPS;
				break;

			case PRESS_TAPS:
				if (press->press_state.burp_ctr > 0) {
					press->press_state.mode = PRESS_UP;
					press->press_state.ticks_until_next = PRESS_TIME_TAP_UP;
				} else {
					press->press_state.cycle = PRESS_PERIOD2;
					press->press_state.ticks_until_next = press->press_setpoint.press_ticks2;
				}
				break;

			case PRESS_PERIOD2:
			default:
				press->press_state.mode = PRESS_UP;
				break;
			}
		}
		break;

	case PRESS_UP:
		// Default speed setting
		if (press->press_setpoint.auto_mode && (press->press_state.burp_ctr > 0)) {
			press->press_state.current_limit = MOTOR_CURRENT_LOW;
			press->press_state.motor_setpoint = -DUTY_CYCLE_SLOW;
		} else {
			press->press_state.current_limit = MOTOR_CURRENT_LOW;
			press->press_state.motor_setpoint = -DUTY_CYCLE_FAST;
		}

		// Tap timer expired
		if (press->press_setpoint.auto_mode
				&& (press->press_state.burp_ctr > 0)
				&& (press->press_state.ticks_until_next-- <= 0)){
			press->press_state.burp_ctr--;
			press->press_state.mode = PRESS_DOWN;
			press->press_state.ticks_until_next = PRESS_TIME_TAP_DOWN;
		}

		// Manual mode and buttons pressed
		if (!press->press_setpoint.auto_mode && press_active) {
			press->press_state.cycle = PRESS_PERIOD1;
			press->press_state.mode = PRESS_DOWN;
			press->press_state.ticks_until_next = press->press_setpoint.press_ticks1;
		}

		// top limit reached!
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
		press->press_state.current_limit = MOTOR_CURRENT_HIGH;
		press->press_state.motor_setpoint = 0.0f;


		// both buttons released
		if (left_button && right_button) {
			press->press_state.mode = PRESS_READY;
		}
		break;

	case PRESS_JOG:
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = 0.0f;

		if (top_lim && menu_up_button.state && !menu_down_button.state) {
			press->press_state.motor_setpoint = -DUTY_CYCLE_JOG;
		}

		if (bottom_lim && menu_down_button.state && !menu_up_button.state) {
			press->press_state.motor_setpoint = DUTY_CYCLE_JOG;
		}
		break;

	default:
		press->press_state.current_limit = MOTOR_CURRENT_LOW;
		press->press_state.motor_setpoint = 0.0f;
		press->press_state.error_code |= ERR_OVERSHOOT;
	}

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

	press->press_state.motor_setpoint =
			clip(press->press_state.motor_setpoint,
					DUTY_CYCLE_FAST,
					-DUTY_CYCLE_FAST);
	float step;
	const float deadband = 0.5f;
	float itarget;
	int8_t dir;

	if (press->press_state.motor_setpoint > 0.0f) {
		itarget = press->press_state.current_limit - deadband;
		dir = 1;
	} else if (press->press_state.motor_setpoint < 0.0f) {
		itarget = -press->press_state.current_limit + deadband;
		dir = -1;
	} else {
		itarget = 0.0f;
		dir = 0;
	}

	if (-current > itarget + deadband) {
		step = -MAX_SLEW_RATE;
	} else if (-current < itarget - deadband) {
		step = MAX_SLEW_RATE;
	} else {
		step = 0.0f;
	}

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

//	// forward current is too high
//	if (-current > press->press_state.current_limit) {
//		step = -MAX_SLEW_RATE;
//	// reverse current too high
//	} else if (current > press->press_state.current_limit) {
//		step = MAX_SLEW_RATE;
//	} else {
//		step = clip(press->press_state.motor_setpoint - press->press_state.motor_slew_limited_setpoint,
//			MAX_SLEW_RATE,
//			-MAX_SLEW_RATE);
//	}

	press->press_state.motor_slew_limited_setpoint =
			clip(press->press_state.motor_slew_limited_setpoint + step,
					DUTY_CYCLE_FAST,
					-DUTY_CYCLE_FAST);


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

		// Detect if any fault bits are set
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

	if (!press->thermal_setpoint.enable) {
		__WRITE_TOP_PLATTER_HEAT(0);
		__WRITE_BOTTOM_PLATTER_HEAT(0);
		press->thermal_state.top_ready = 0;
		press->thermal_state.bottom_ready = 0;
		return;
	}

	if (press->thermal_setpoint.top_temp - press->thermal_state.top_temp > 3.0f) {
		press->thermal_state.top_ready = false;
	}
	if (press->thermal_setpoint.bottom_temp - press->thermal_state.bottom_temp > 3.0f) {
		press->thermal_state.bottom_ready = false;
	}

	press->thermal_state.top_ssr_on = (press->thermal_state.top_temp < press->thermal_state.top_threshold);
	press->thermal_state.bottom_ssr_on = (press->thermal_state.bottom_temp < press->thermal_state.bottom_threshold);

	__WRITE_TOP_PLATTER_HEAT(press->thermal_state.top_ssr_on);
	__WRITE_BOTTOM_PLATTER_HEAT(press->thermal_state.bottom_ssr_on);

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






