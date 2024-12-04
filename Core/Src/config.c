/**
 * @file config.c
 * @author Aaron Yeiser
 * @brief 760 Pizza Press configuration constants, backup, and restore
 * @date 2022-08-10
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#include "config.h"

Press press = {
		.press_setpoint = {
				.burps = 1,
				.press_ticks1 = DEFAULT_PRESS_TIME,
				.press_ticks2 = DEFAULT_PRESS_TIME,
				.auto_mode = false,
				.enable = false
		},

		.press_state = {
				.mode = PRESS_READY,
				.error_code = 0,
				.motor_setpoint = 0.0f,
				.motor_slew_limited_setpoint = 0.0f,
				.current_limit = 0.0f
		},

		.thermal_setpoint = {
				.enable = false
		},

		.thermal_state = {
				.top1 = 999,
				.top2 = 999,
				.bottom1 = 999,
				.bottom2 = 999,
				.top_temp = 999.0f,
				.bottom_temp = 999.0f,
				.error = 0,
				.error_code = 0
		},

		.config = {
				.flags = DEFAULT_CONFIG_FLAGS,
				.top_temp = DEFAULT_TOP_TEMP,
				.bottom_temp = DEFAULT_BOTTOM_TEMP,
				.press_time1 = DEFAULT_PRESS_TIME,
				.press_time2 = DEFAULT_PRESS_TIME,
				.burps = DEFAULT_BURPS
		},
};

void backup_settings(Config* config) {
	config->flags |= DEFAULT_CONFIG_FLAGS;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, config->regs[0]);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, config->regs[1]);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, config->regs[2]);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, config->regs[3]);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, config->regs[4]);
}

void restore_settings(Config* config) {
	config->regs[0] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	config->regs[1] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	config->regs[2] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	config->regs[3] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
	config->regs[4] = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

	if (!(config->flags & DEFAULT_CONFIG_FLAGS)) {
		config->ctr = 0;
		reset_defaults(config);
		backup_settings(config);
	}
}

void reset_defaults(Config* config) {
	config->flags = DEFAULT_CONFIG_FLAGS;
	config->top_temp = DEFAULT_TOP_TEMP;
	config->bottom_temp = DEFAULT_BOTTOM_TEMP;
	config->press_time1 = DEFAULT_PRESS_TIME;
	config->press_time2 = DEFAULT_PRESS_TIME;
	config->burps = DEFAULT_BURPS;
}

void config_to_setpoints(Press* press) {
	if (press->config.flags & CONFIG_UNITS_FLAG) { // if using celsius
		press->thermal_setpoint.top_temp = (float) press->config.top_temp;
		press->thermal_setpoint.bottom_temp = ( float)press->config.bottom_temp;
	} else {
		press->thermal_setpoint.top_temp = __F_TO_C_FLOAT((float) press->config.top_temp);
		press->thermal_setpoint.bottom_temp = __F_TO_C_FLOAT((float) press->config.bottom_temp);
	}
	press->press_setpoint.press_ticks1 = press->config.press_time1;
	press->press_setpoint.press_ticks2 = press->config.press_time2;
	press->press_setpoint.burps = press->config.burps;
	press->press_setpoint.auto_mode = press->config.flags & CONFIG_MODE_FLAG;
}
