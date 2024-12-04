/**
 * @file debounce.h
 * @author Aaron Yeiser
 * @brief 760 Pizza Press debounced buttons
 * @date 2022-08-05
 *
 * @copyright Copyright 2024 Boston Precision Motion LLC.
 * This project is released under the MIT License
 */

#ifndef INC_DEBOUNCE_H_
#define INC_DEBOUNCE_H_

#include "main.h"
#include "structs.h"

// Debounce parameters
#define SETTLING_TIME 10
#define REPEAT_TIME 500
#define REPEAT_INTERVAL 50

/**
 * @brief Debounce a button.  This function must be called frequently (at least 1kHz)
 *
 * If the button is held down this will also generate a rising edge flag
 * every REPEAT_INTERVAL after a delay of REPEAT_TIME
 *
 * @param button The button debounced state
 * @param state Measured state of the physical button
 * @return bool Debounced button state
 */
bool debounce(Button* button, bool state);

/// Debounce all of the menu up/down/enter buttons
/// Note that menu buttons are active low but the
/// debounced states are active high
void debounce_menu_buttons(void);

/// Debounce the activate buttons (on press sides)
void debounce_activate_buttons(void);

/// Debounce the press interlock
void debounce_interlock(void);

#endif /* INC_DEBOUNCE_H_ */
