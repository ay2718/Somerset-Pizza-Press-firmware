/**
 * @file menu.h
 * @author Aaron Yeiser
 * @brief 760 Pizza Press screen menus
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include "main.h"
#include "config.h"
#include "structs.h"
#include "SSD1306.h"

#define MENU_TIMEOUT 7000ul // 7 seconds

/// Initialize menu linkage
void init_menus(void);

/// operation to link parent and child menu items
void link_menus(MenuItem* parent, MenuItem* child);

/**
 * @brief set RAM text buffer for screen
 * @param str Text to write to the row
 * @param rownum Row (0-7) to write to
 * @param font 0 is small, 1 is big
 */
void set_row(const char* str, uint8_t rownum, uint8_t font);

/**
 * @brief write RAM text buffer row to SSD1306
 * @param rownum the row to write to (0-7)
 * @return HAL_StatusTypeDef the status of writing to the row
 *
 * @note this is a blocking function
 */
HAL_StatusTypeDef write_row(uint8_t rownum);

/**
 * @brief Called when menu up button is pressed
 * @param the current menu item
 * @return the next menu item to go to
 */
MenuItem* menu_up(MenuItem*);

/**
 * @brief Called when menu down button is pressed
 * @param the current menu item
 * @return the next menu item to go to
 */
MenuItem* menu_down(MenuItem*);

/**
 * @brief Called when menu enter button is pressed
 * @param the current menu item
 * @return the next menu item to go to
 */
MenuItem* menu_enter(MenuItem*);

HAL_StatusTypeDef menu_return_home(void);

HAL_StatusTypeDef debug_display(MenuItem*);
HAL_StatusTypeDef lifetime_display(MenuItem*);
HAL_StatusTypeDef status_display(MenuItem*);
HAL_StatusTypeDef thermocouple_readout(void);

// function to write data to the screen
HAL_StatusTypeDef generic_display(MenuItem*);
HAL_StatusTypeDef temperature_display(MenuItem*);
HAL_StatusTypeDef press_time_display(MenuItem*);
HAL_StatusTypeDef manual_mode_display(MenuItem*);
HAL_StatusTypeDef reset_display(MenuItem*);
HAL_StatusTypeDef units_display(MenuItem*);
HAL_StatusTypeDef jog_display(MenuItem*);

#endif /* INC_MENU_H_ */
