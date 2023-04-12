/*
 * menu.h
 *
 *  Created on: Aug 5, 2022
 *      Author: ayeiser
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "main.h"
#include "config.h"
#include "structs.h"
#include "SSD1306.h"

#define MENU_TIMEOUT 7000ul // 7 seconds

void init_menus(void);
void link_menus(MenuItem*, MenuItem*);

void set_row(char*, uint8_t, uint8_t);
HAL_StatusTypeDef write_row(uint8_t);

MenuItem* menu_up(MenuItem*);
MenuItem* menu_down(MenuItem*);
MenuItem* menu_enter(MenuItem*);

HAL_StatusTypeDef menu_return_home(void);

HAL_StatusTypeDef debug_display(MenuItem*);
HAL_StatusTypeDef lifetime_display(MenuItem*);
HAL_StatusTypeDef status_display(MenuItem*);
HAL_StatusTypeDef thermocouple_readout(void);

HAL_StatusTypeDef generic_display(MenuItem*);
HAL_StatusTypeDef temperature_display(MenuItem*);
HAL_StatusTypeDef press_time_display(MenuItem*);
HAL_StatusTypeDef manual_mode_display(MenuItem*);
HAL_StatusTypeDef reset_display(MenuItem*);
HAL_StatusTypeDef units_display(MenuItem*);
HAL_StatusTypeDef jog_display(MenuItem*);

#endif /* INC_MENU_H_ */
