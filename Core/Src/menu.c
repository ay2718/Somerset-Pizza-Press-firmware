/*
 * menu.c
 *
 *  Created on: Aug 5, 2022
 *      Author: ayeiser
 */


#include "menu.h"
#include "control.h"

MenuItem status_menu = {
		.type=MENU_STATUS,
		.parent=NULL,
		.display=&status_display
};


MenuItem main_menu = {
		.type=MENU,
		.name="Main Menu",
		.display=&generic_display
};

// define target here
MenuItem top_temp_menu = {
		.type=MENU_TEMP,
		.name="Top Temp",
		.lower=TEMP_LOWER_LIM_F,
		.upper=TEMP_UPPER_LIM_F,
		.step=5,
		.display=&temperature_display,
		.target=&(press.config.top_temp)
};

MenuItem bottom_temp_menu = {
		.type=MENU_TEMP,
		.name="Bottom Temp",
		.lower=TEMP_LOWER_LIM_F,
		.upper=TEMP_UPPER_LIM_F,
		.step=5,
		.display=&temperature_display,
		.target=&(press.config.bottom_temp)
};

MenuItem press_reset_count = {
		.type=MENU_RESET_COUNT,
		.name="Reset Count?",
		.display=&reset_display,
		.value = 0
};

MenuItem mode_menu = {
		.type=MENU_FLAG, // not implemented
		.name="Auto Mode",
		.display=&manual_mode_display,
		.flag=CONFIG_MODE_FLAG,
		.target=(int16_t*) &(press.config.flags)
};

MenuItem press_time1_menu = {
		.type=MENU_NUM,
		.name="1st Press Time",
		.lower=PRESS_TIME_LOWER_LIM,
		.upper=PRESS_TIME_UPPER_LIM,
		.step=500,
		.display=&press_time_display,
		.target=&(press.config.press_time1)
};

MenuItem press_time2_menu = {
		.type=MENU_NUM,
		.name="2nd Press Time",
		.lower=0,
		.upper=PRESS_TIME_UPPER_LIM,
		.step=500,
		.display=&press_time_display,
		.target=&(press.config.press_time2)
};

MenuItem burps_menu = {
		.type=MENU_NUM,
		.name="Taps",
		.lower=BURPS_LOWER_LIM,
		.upper=BURPS_UPPER_LIM,
		.step=1,
		.display=&generic_display,
		.target=&(press.config.burps)
};

MenuItem eco_mode_menu = {
		.type=MENU_FLAG,
		.name="Eco Mode",
		.display=&generic_display,
		.flag=CONFIG_ECO_FLAG,
		.target=(int16_t*) &(press.config.flags)
};

MenuItem buzzer_menu = {
		.type=MENU_FLAG,
		.name="Buzzer",
		.display=&generic_display,
		.flag=CONFIG_BUZZER_FLAG,
		.target=(int16_t*) &(press.config.flags)
};

MenuItem service_menu = {
		.type=MENU,
		.name="Service",
		.display=&generic_display
};

MenuItem reset_menu = {
		.type=MENU_RESET,
		.name="Reset All?",
		.display=&reset_display,
		.value = 0
};

MenuItem units_menu = {
		.type = MENU_TEMP_UNITS,
		.name = "Temp Units",
		.display=&units_display,
		.target=(int16_t*) &(press.config.flags),
		.flag = CONFIG_UNITS_FLAG
};

MenuItem jog_menu = {
		.type = MENU_JOG,
		.name = "Jog",
		.display = &jog_display
};

MenuItem lifetime_menu = {
		.type = MENU_DEBUG,
		.name = "Lifetime Cycles",
		.display = &lifetime_display
};

MenuItem debug_menu = {
		.type = MENU_DEBUG,
		.name = "Diagnostics",
		.display = &debug_display
};

MenuItem cycle_menu = {
		.type = MENU_CYCLE,
		.name = "Cycle",
		.display = &reset_display
};

MenuItem* current_menu = &status_menu;

uint8_t display_row = 0;
char screen_buf[256];
uint8_t screen_fonts[8];
extern uint32_t press_count;
extern bool cycle_mode;


void init_menus(void) {
	link_menus(&status_menu, &main_menu);

	link_menus(&main_menu, &press_reset_count);
	link_menus(&main_menu, &top_temp_menu);
	link_menus(&main_menu, &bottom_temp_menu);
	link_menus(&main_menu, &mode_menu);
	link_menus(&main_menu, &press_time1_menu);
	link_menus(&main_menu, &press_time2_menu);
	link_menus(&main_menu, &burps_menu);
	link_menus(&main_menu, &eco_mode_menu);
	link_menus(&main_menu, &buzzer_menu);
	link_menus(&main_menu, &service_menu);

	link_menus(&service_menu, &jog_menu);
	link_menus(&service_menu, &units_menu);
	link_menus(&service_menu, &lifetime_menu);
	link_menus(&service_menu, &debug_menu);
#ifdef CYCLE_MODE
	link_menus(&service_menu, &cycle_menu);
#endif
	link_menus(&service_menu, &reset_menu);
}

void link_menus(MenuItem* parent, MenuItem* child) {
	if (parent->length < 16) {
		parent->items[(parent->length)++] = child;
		child->parent = parent;
	}
}

MenuItem* menu_up(MenuItem* item) {
	switch(item->type) {
	case MENU_JOG:
		press.press_state.mode = PRESS_JOG;
		break;
	case MENU_STATUS:
		item = item->items[0];
		break;
	case MENU_DEBUG:
		item = item->parent;
		break;
	case MENU: // item is another menu
		item->index = max(item->index - 1, 0);
		break;
	case MENU_TEMP:
		if (press.config.flags & CONFIG_UNITS_FLAG) {
			item->upper = TEMP_UPPER_LIM_C;
			item->lower = TEMP_LOWER_LIM_C;
		} else {
			item->upper = TEMP_UPPER_LIM_F;
			item->lower = TEMP_LOWER_LIM_F;
		}
	case MENU_NUM: // numerical entry
		item->value = clip(item->value + item->step, item->upper, item->lower);
		break;
	case MENU_TEMP_UNITS:
	case MENU_FLAG: // yes/no
		item->value |= (item->flag);
		break;
	case MENU_CYCLE:
	case MENU_RESET_COUNT:
	case MENU_RESET:
		item->value = 1;
		break;
	case MENU_OTHER:
	default:
		break;
	}
	item->display(item);
	return item;

}

MenuItem* menu_down(MenuItem* item) {
	switch(item->type) {
	case MENU_JOG:
		press.press_state.mode = PRESS_JOG;
		break;
	case MENU_STATUS:
		item = item->items[0];
		break;
	case MENU_DEBUG:
		item = item->parent;
		break;
	case MENU: // item is another menu
		item->index = min(item->index + 1, item->length);
		break;
	case MENU_TEMP:
		if (press.config.flags & CONFIG_UNITS_FLAG) {
			item->upper = TEMP_UPPER_LIM_C;
			item->lower = TEMP_LOWER_LIM_C;
		} else {
			item->upper = TEMP_UPPER_LIM_F;
			item->lower = TEMP_LOWER_LIM_F;
		}
		item->value = clip(item->value, item->upper, item->lower);
		item->value -= item->step;
		if (item->value < item->lower) {
			item->value = -40; // lol same in C and F
		}
		break;
	case MENU_NUM: // numerical entry
		item->value = clip(item->value - item->step, item->upper, item->lower);
		break;
	case MENU_TEMP_UNITS:
	case MENU_FLAG: // yes/no
		item->value &= ~(item->flag);
		break;
	case MENU_CYCLE:
	case MENU_RESET_COUNT:
	case MENU_RESET:
		item->value = 0;
		break;
	case MENU_OTHER:
	default:
		break;
	}
	item->display(item);
	return item;
}

MenuItem* menu_enter(MenuItem* item) {
	switch(item->type) {
	case MENU_STATUS:
		item = item->items[0];
		break;
	case MENU_DEBUG:
		item = item->parent;
		break;
	case MENU: // item is another menu
		if (item->index > 0) {
			item = item->items[item->index - 1]; // enter child menu if possible
			item->index = 0;
			if (item->target != NULL) {
				item->value = *(item->target);   // copy current target into value
			}
		} else { // back button
			if (item->parent != NULL) {
				item = item->parent; // return to parent if not top level
			} else {
				item->index = 0; // return to top of menu otherwise
			}
		}
		break;
	case MENU_RESET:
		if (item->value) {
			item->value = 0;
			reset_defaults(&(press.config));
		}
		goto menu_enter_default;
	case MENU_JOG:
		press.press_state.mode = PRESS_DONE;
		goto menu_enter_default;
	case MENU_RESET_COUNT:
		if (item->value) {
			item->value = 0;
			press_count = 0;
		}
		goto menu_enter_default;
	case MENU_CYCLE:
		cycle_mode = item->value;
		if (item->value) {
			item->value = 0;
		}
		goto menu_enter_default;
	case MENU_TEMP_UNITS:
	{
		bool current_temp_units = press.config.flags & CONFIG_UNITS_FLAG;
		bool new_temp_units = item->value & item->flag;
		if (current_temp_units && !new_temp_units) { // convert from C to F
			press.config.top_temp = __ROUND5(__C_TO_F(press.config.top_temp)); // round to the nearest 5
			press.config.bottom_temp = __ROUND5(__C_TO_F(press.config.bottom_temp));
		}

		if (!current_temp_units && new_temp_units) { // convert from F to C
			press.config.top_temp = __ROUND5(__F_TO_C(press.config.top_temp)); // round to the nearest 5
			press.config.bottom_temp = __ROUND5(__F_TO_C(press.config.bottom_temp));
		}

	}
	case MENU_TEMP:
	case MENU_NUM: // numerical and yes/no do the same thing here
	case MENU_FLAG:
		if (item->target != NULL) {
			*(item->target) = item->value; // enter target
		}
menu_enter_default:
	default:
		if (item->parent != NULL) {
			item = item->parent; // return to parent menu
		}
		backup_settings(&(press.config));
		break;
	}
	item->display(item);
	return item;
}

HAL_StatusTypeDef menu_return_home(void) {
	current_menu = &status_menu;
	if (press.press_state.mode == PRESS_JOG){
		press.press_state.mode = PRESS_ERROR;
	}
	return current_menu->display(current_menu);
}

HAL_StatusTypeDef thermocouple_readout(void) {
	char str[32] = {0};
	sprintf(str, "%d C  %d C", (int)(press.thermal_state.top1), (int)(press.thermal_state.top2));
	set_row(str, 1, 0);
	memset(str, 0, 32);
	sprintf(str, "%d C  %d C", (int)(press.thermal_state.bottom1), (int)(press.thermal_state.bottom2));
	set_row(str, 3, 0);
	return write_row(0);
}
extern float shunt_current;
HAL_StatusTypeDef debug_display(MenuItem* item) {
	char msg[32]; int len;

	len = sprintf(msg, "Top: %dC %dC",
			(int) press.thermal_state.top1 % 1000,
			(int) press.thermal_state.top2 % 1000);
	set_row(msg, 0, 0); memset(msg, 0, 32);

	len = sprintf(msg, "Bot: %dC %dC",
			(int) press.thermal_state.bottom1 % 1000,
			(int) press.thermal_state.bottom2 % 1000);
	set_row(msg, 1, 0); memset(msg, 0, 32);

	len = sprintf(msg, "SSR Top:%d Bot:%d",
			__READ_TOP_SSR(),
			__READ_BOTTOM_SSR());
	set_row(msg, 2, 0); memset(msg, 0, 32);

	len = sprintf(msg, "Tray open:%d", tray_interlock.state);
	set_row(msg, 3, 0); memset(msg, 0, 32);

	len = sprintf(msg, "TRVL Top:%d Bot:%d",
			!press_top_limit.state,
			!press_bottom_limit.state);
	set_row(msg, 4, 0); memset(msg, 0, 32);

	len = sprintf(msg, "Activ L:%d R:%d",
			!activate_left_button.state,
			!activate_right_button.state);
	set_row(msg, 5, 0); memset(msg, 0, 32);

	len = sprintf(msg, "Motor:%d%% I:%dA",
			(int) (100.0f*press.press_state.motor_setpoint),
			(int) shunt_current);
	set_row(msg, 6, 0); memset(msg, 0, 32);

	len = sprintf(msg, "State:%d", press.press_state.mode);
	set_row(msg, 7, 0); memset(msg, 0, 32);

	return write_row(0);
}

HAL_StatusTypeDef lifetime_display(MenuItem* item) {
	set_row("LIFETIME", 0, 1);
	char str[32];
	int len;
	len = sprintf(str, "Cycles: %d", (int) press.config.ctr);
	set_row(str, 3, 0);
	return write_row(0);
}

extern bool cycle_state;
uint8_t eco_flash_ticks = 0;
HAL_StatusTypeDef status_display(MenuItem* item) {
	if (press.press_state.mode == PRESS_ERROR) { // TODO interlock error
		if (tray_interlock.state) {
			set_row("CLOSE TRAY!", 0, 1);
		} else {
			set_row("WAIT...", 0, 1);
		}
//	} else if (press.thermal_state.error) {
//		set_row("BAD THERMO!", 0, 1);
	} else if (!press.thermal_setpoint.enable) {
		if (((--eco_flash_ticks) & 0xf) < 8) {
			set_row("", 0, 1);
			set_row("", 2, 1);
			set_row("", 4, 1);
			set_row("", 6, 1);
			return write_row(0);
		} else {
			set_row("ECO MODE", 0, 1);
		}
	} else if (!(press.thermal_state.top_ready && press.thermal_state.bottom_ready)) {
		set_row("PREHEAT...", 0, 1);
	} else if (press.press_state.mode == PRESS_READY){
		if (press.press_state.overload_flag) {
			set_row("OVERLOAD", 0, 1);
		} else {
			set_row("READY!", 0, 1);
		}
	} else {
		set_row("PRESSING...", 0, 1);
	}

#ifdef CYCLE_MODE
	{
		char str[32] = {0};
		if (cycle_mode) {
			sprintf(str, "Cycle Mode On");
		} else {
			sprintf(str, "Cycle Mode Off");
		}
		set_row(str, 2, 0);
	}
#endif

	char unit;
	if (press.config.flags & CONFIG_UNITS_FLAG) {
		unit = 'C';
	} else {
		unit = 'F';
	}

	if (press.thermal_setpoint.top_temp > 0.0f){
		char str[32] = {0};
		sprintf(str, "%3d: %3d %c", press.config.top_temp, getTopTempDisplay(&press), unit);
		set_row(str, 3, 1);
	} else {
		char str[32] = {0};
		sprintf(str, "OFF: %3d %c", getTopTempDisplay(&press), unit);
		set_row(str, 3, 1);
	}

	if (press.thermal_setpoint.bottom_temp > 0){
		char str[32] = {0};
		sprintf(str, "%3d: %3d %c", press.config.bottom_temp, getBottomTempDisplay(&press), unit);
		set_row(str, 5, 1);
	} else {
		char str[32] = {0};
		sprintf(str, "OFF: %3d %c", getBottomTempDisplay(&press), unit);
		set_row(str, 5, 1);
	}

	{
		char str[32] = {0};
		sprintf(str, "Count: %d", (int) press_count);
		set_row(str, 7, 0);
	}
	return write_row(0);
}

HAL_StatusTypeDef generic_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	switch(item->type) {
	case MENU: // item is a menu
	{
		int menu_index = min(item->length-4, item->index-2);
		menu_index = max(0, menu_index);
		for (int row_index = 3; row_index < 8; row_index++, menu_index++) {
			char str[32] = {0};
			char cursor = (menu_index == item->index) ? '>' : ' ';
			if (menu_index <= item->length && menu_index > 0) {
				sprintf(str, "%c%s", cursor, (item->items[menu_index-1])->name);
			} else if (menu_index == 0) {
				sprintf(str, "%cBack", cursor);
			}
			set_row(str, row_index, 0);
		}
		break;
	}
	case MENU_NUM:
	{
		char str[32] = {0};
		sprintf(str, "Number: %d", item->value);
		set_row(str, 3, 0);
		break;
	}
	case MENU_FLAG:
	{
		if (item->value & item->flag) {
			set_row("Enabled: Yes", 3, 0);
		} else {
			set_row("Enabled: No", 3, 0);
		}
		break;
	}
	case MENU_OTHER:
	default:
	{
		set_row(">Back", 3, 0);
	}
		break;
	}
	return write_row(0);
}

HAL_StatusTypeDef temperature_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	if (item->value > 0) {
		char str[32] = {0};
		char unit;
		if (press.config.flags & CONFIG_UNITS_FLAG) {
			unit = 'C';
		} else {
			unit = 'F';
		}
		sprintf(str, "Temperature: %d %c", item->value, unit);
		set_row(str, 3, 0);
	} else {
		set_row("Temperature: OFF", 3, 0);
	}
	return write_row(0);
}

HAL_StatusTypeDef press_time_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	char str[32] = {0};
	sprintf(str, "Time: %d.%d s", item->value / 1000, (item->value / 100) % 10);
	set_row(str, 3, 0);
	return write_row(0);
}

HAL_StatusTypeDef manual_mode_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	if (item->value & item->flag) {
		set_row("Mode: Auto", 3, 0);
	} else {
		set_row("Mode: Manual", 3, 0);

	}
	return write_row(0);
}

HAL_StatusTypeDef reset_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	if (item->value) {
		set_row("Yes", 3, 0);
		set_row("Are you sure?", 5, 0);
	} else {
		set_row("No", 3, 0);
	}
	return write_row(0);
}

HAL_StatusTypeDef units_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	if (item->value & item->flag) {
		set_row("Units: Celsius", 3, 0);
	} else {
		set_row("Units: Fahrenheit", 3, 0);
	}
	return write_row(0);
}

HAL_StatusTypeDef jog_display(MenuItem* item) {
	set_row(item->name, 0, 1);
	set_row("Jog press using", 3, 0);
	set_row("menu buttons", 4, 0);
	return write_row(0);
}

bool busy_flag = false;
HAL_StatusTypeDef write_row_innerfunc(void);
HAL_StatusTypeDef write_row(uint8_t rownum) {
	if (!busy_flag) {
		busy_flag = true;
		display_row = rownum;
		return write_row_innerfunc();
	} else {
		return HAL_BUSY;
	}
}

HAL_StatusTypeDef write_row_innerfunc(void) {
	char msg[32];
	sprintf(msg, "Writing to row %d\n\r", display_row);
//	HAL_UART_Transmit(&huart2, msg, len, 100);
	if (display_row >= 8) {
		display_row = 0;
		busy_flag = false;
		return HAL_OK;
	} else {
		uint8_t font = screen_fonts[display_row];
		char* row = screen_buf + (display_row << 5);
		SSD1306_ClearBuf();
		SSD1306_setFont(font);
		SSD1306_writeString(0, row);
		memset(row, 0, 32);
		SSD1306_WriteRow(display_row++);
		return write_row_innerfunc();
	}
}

void set_row(char* str, uint8_t rownum, uint8_t font) {
	char* row = screen_buf+(rownum << 5);
	strncpy(row, str, 32);
	screen_fonts[rownum] = font;
	if (font && rownum<7) {
		screen_fonts[rownum+1] = font+1;
		strncpy(row + (1<<5), str, 32);
	}
}

// Use this only I2C is in interrupt or DMA mode
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == I2C2) {
//		write_row_innerfunc();
//	}
//}

//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == I2C1) {
//		__WRITE_WHITE_LED(1);
//	}
//}
//
//void HAL_I2C_AbortCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == I2C1) {
//		__WRITE_BLUE_LED(1);
//
//	}
//}
