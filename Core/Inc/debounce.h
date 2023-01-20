/*
 * buttons.h
 *
 *  Created on: Aug 5, 2022
 *      Author: ayeiser
 */

#ifndef INC_DEBOUNCE_H_
#define INC_DEBOUNCE_H_

#include "main.h"
#include "structs.h"

#define SETTLING_TIME 10
#define REPEAT_TIME 500
#define REPEAT_INTERVAL 50

bool debounce(Button*, bool);
void debounce_menu_buttons(void);
void debounce_activate_buttons(void);
void debounce_interlock(void);

#endif /* INC_DEBOUNCE_H_ */
