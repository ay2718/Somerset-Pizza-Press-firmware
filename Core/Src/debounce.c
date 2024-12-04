/**
 * @file debounce.c
 * @author Aaron Yeiser
 * @brief 760 Pizza Press debounced buttons
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#include "debounce.h"

Button menu_up_button;
Button menu_down_button;
Button menu_enter_button;

Button activate_left_button;
Button activate_right_button;

Button press_top_limit;
Button press_bottom_limit;
Button tray_interlock;

bool debounce(Button* button, bool state) {
	if (button->state == state) {
		// If no state change has occurred since last rising or falling edge detected...
		button->ctr = 0;

		// Handle repeat counter
		if (++button->repeat_ctr > REPEAT_TIME) {

			// Generate rising or falling edge if button is held down
			button->repeat_ctr = REPEAT_TIME - REPEAT_INTERVAL;
			if (button->state) {
				button->rising_edge_flag = true;
			} else {
				button->falling_edge_flag = true;
			}
		}
	} else {
		// If state change has occurred, we need at least SETTLING_TIME consecutive button reads
		// with the newt state before we detect a rising or falling edge
		button->repeat_ctr = 0;
		if (++button->ctr > SETTLING_TIME) {
			button->state = state;
			if (state) {
				button->rising_edge_flag = true;
			} else {
				button->falling_edge_flag = true;
			}
		}
	}
	return button->state;
}

/*
 * TODO: de-invert these for the new board
 */
void debounce_menu_buttons(void) {
	debounce(&menu_up_button, !__READ_UP_SW());
	debounce(&menu_down_button, !__READ_DOWN_SW());
	debounce(&menu_enter_button, !__READ_ENTER_SW());
}

void debounce_activate_buttons(void) {
	debounce(&activate_left_button, __READ_LEFT_ACTIVATE_SW());
	debounce(&activate_right_button, __READ_RIGHT_ACTIVATE_SW());
}
void debounce_interlock(void) {
	debounce(&press_top_limit, __READ_TOP_TRAVEL_SW());
	debounce(&press_bottom_limit, __READ_BOTTOM_TRAVEL_SW());
	debounce(&tray_interlock, __READ_PLATTER_SW());
}
