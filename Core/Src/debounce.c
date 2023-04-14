/*
 * debouncea.c
 *
 *  Created on: Aug 5, 2022
 *      Author: ayeiser
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
		button->ctr = 0;
		if (++button->repeat_ctr > REPEAT_TIME) {
			button->repeat_ctr = REPEAT_TIME - REPEAT_INTERVAL;
			if (button->state) {
				button->rising_edge_flag = true;
			} else {
				button->falling_edge_flag = true;
			}
		}
	} else {
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
