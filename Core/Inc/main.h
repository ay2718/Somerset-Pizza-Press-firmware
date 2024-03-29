/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>

#include "structs.h"

//#define CYCLE_MODE

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern ADC_HandleTypeDef hadc;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;

extern MenuItem* current_menu;
extern MenuItem status_menu;
extern MenuItem debug_menu;
extern MenuItem lifetime_menu;

extern MenuItem jog_menu;

extern Press press;
extern uint32_t ticks;

extern Button menu_up_button;
extern Button menu_down_button;
extern Button menu_enter_button;

extern Button activate_left_button;
extern Button activate_right_button;

extern Button press_bottom_limit;
extern Button press_top_limit;
extern Button tray_interlock;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define max(a,b) ((a)>(b) ? (a) : (b))
#define min(a,b) ((a)<(b) ? (a) : (b))
#define clip(a,b,c) (max(min((a), (b)), (c))) // value, max, min

#define __F_TO_C(f) ((5*((f)-32))/9)
#define __C_TO_F(c) (32+(((c)*9)/5))
#define __ROUND5(x) ((((x)+2)/5)*5)

#define __F_TO_C_FLOAT(f) (((f)-32.0f)/1.8f)
#define __C_TO_F_FLOAT(c) (32.0f+((c)*1.8f))

#define __READ_UP_SW() (HAL_GPIO_ReadPin(UP_BUTTON_GPIO_Port, UP_BUTTON_Pin))
#define __READ_DOWN_SW() (HAL_GPIO_ReadPin(DOWN_BUTTON_GPIO_Port, DOWN_BUTTON_Pin))
#define __READ_ENTER_SW() (HAL_GPIO_ReadPin(ENTER_BUTTON_GPIO_Port, ENTER_BUTTON_Pin))

#define __READ_LEFT_ACTIVATE_SW() (HAL_GPIO_ReadPin(LEFT_ACTIVATE_BUTTON_GPIO_Port, LEFT_ACTIVATE_BUTTON_Pin))
#define __READ_RIGHT_ACTIVATE_SW() (HAL_GPIO_ReadPin(RIGHT_ACTIVATE_BUTTON_GPIO_Port, RIGHT_ACTIVATE_BUTTON_Pin))

#define __READ_BOTTOM_TRAVEL_SW() (HAL_GPIO_ReadPin(BOTTOM_TRAVEL_SWITCH_GPIO_Port, BOTTOM_TRAVEL_SWITCH_Pin))
#define __READ_TOP_TRAVEL_SW() (HAL_GPIO_ReadPin(TOP_TRAVEL_SWITCH_GPIO_Port, TOP_TRAVEL_SWITCH_Pin))
#define __READ_PLATTER_SW() (HAL_GPIO_ReadPin(PLATTER_SWITCH_GPIO_Port, PLATTER_SWITCH_Pin))

#define __WRITE_THERMO_TOP1_CS(x) (HAL_GPIO_WritePin(CS_THERMOCOUPLE_TOP1_GPIO_Port, CS_THERMOCOUPLE_TOP1_Pin, (x)))
#define __WRITE_THERMO_TOP2_CS(x) (HAL_GPIO_WritePin(CS_THERMOCOUPLE_TOP2_GPIO_Port, CS_THERMOCOUPLE_TOP2_Pin, (x)))
#define __WRITE_THERMO_BOTTOM1_CS(x) (HAL_GPIO_WritePin(CS_THERMOCOUPLE_BOTTOM1_GPIO_Port, CS_THERMOCOUPLE_BOTTOM1_Pin, (x)))
#define __WRITE_THERMO_BOTTOM2_CS(x) (HAL_GPIO_WritePin(CS_THERMOCOUPLE_BOTTOM2_GPIO_Port, CS_THERMOCOUPLE_BOTTOM2_Pin, (x)))

#define __WRITE_SCREEN_CS(x) (HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, (x)))
#define __WRITE_SCREEN_DC(x) (HAL_GPIO_WritePin(SCREEN_DATASEL_GPIO_Port, SCREEN_DATASEL_Pin, (x)))
#define __WRITE_SCREEN_RESET(x) (HAL_GPIO_WritePin(SCREEN_RESET_GPIO_Port, SCREEN_RESET_Pin, (x)))

#define __WRITE_BLUE_LED(x) (HAL_GPIO_WritePin(BLUE_LED_PIN_GPIO_Port, BLUE_LED_PIN_Pin, (x)))
#define __WRITE_WHITE_LED(x) (HAL_GPIO_WritePin(WHITE_LED_PIN_GPIO_Port, WHITE_LED_PIN_Pin, (x)))
#define __TOGGLE_BLUE_LED() (HAL_GPIO_TogglePin(BLUE_LED_PIN_GPIO_Port, BLUE_LED_PIN_Pin))
#define __TOGGLE_WHITE_LED() (HAL_GPIO_TogglePin(WHITE_LED_PIN_GPIO_Port, WHITE_LED_PIN_Pin))


#define __WRITE_TOP_PLATTER_HEAT(x) (HAL_GPIO_WritePin(TOP_PLATTER_HEAT_GPIO_Port, TOP_PLATTER_HEAT_Pin, (x)))
#define __WRITE_BOTTOM_PLATTER_HEAT(x) (HAL_GPIO_WritePin(BOTTOM_PLATTER_HEAT_GPIO_Port, BOTTOM_PLATTER_HEAT_Pin, (x)))

#define __READ_TOP_SSR() (HAL_GPIO_ReadPin(TOP_PLATTER_HEAT_GPIO_Port, TOP_PLATTER_HEAT_Pin))
#define __READ_BOTTOM_SSR() (HAL_GPIO_ReadPin(BOTTOM_PLATTER_HEAT_GPIO_Port, BOTTOM_PLATTER_HEAT_Pin))

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_PERIOD (CLOCK_FREQ / BUZZER_FREQ)
#define CLOCK_FREQ 48000000
#define DEAD_TIME 25
#define PWM_FREQ 1000
#define PWM_PERIOD (CLOCK_FREQ / (2*PWM_FREQ))
#define BUZZER_FREQ 2400
#define DOWN_BUTTON_Pin GPIO_PIN_13
#define DOWN_BUTTON_GPIO_Port GPIOC
#define ENTER_BUTTON_Pin GPIO_PIN_14
#define ENTER_BUTTON_GPIO_Port GPIOC
#define UP_BUTTON_Pin GPIO_PIN_15
#define UP_BUTTON_GPIO_Port GPIOC
#define VBUS_SENSE_Pin GPIO_PIN_0
#define VBUS_SENSE_GPIO_Port GPIOA
#define OPAMP_VOUT_Pin GPIO_PIN_1
#define OPAMP_VOUT_GPIO_Port GPIOA
#define SCREEN_RESET_Pin GPIO_PIN_5
#define SCREEN_RESET_GPIO_Port GPIOA
#define BLUE_LED_PIN_Pin GPIO_PIN_6
#define BLUE_LED_PIN_GPIO_Port GPIOA
#define RIGHT_ACTIVATE_BUTTON_Pin GPIO_PIN_1
#define RIGHT_ACTIVATE_BUTTON_GPIO_Port GPIOB
#define WHITE_LED_PIN_Pin GPIO_PIN_2
#define WHITE_LED_PIN_GPIO_Port GPIOB
#define LEFT_ACTIVATE_BUTTON_Pin GPIO_PIN_10
#define LEFT_ACTIVATE_BUTTON_GPIO_Port GPIOB
#define SCREEN_CS_Pin GPIO_PIN_12
#define SCREEN_CS_GPIO_Port GPIOB
#define SCREEN_DATASEL_Pin GPIO_PIN_14
#define SCREEN_DATASEL_GPIO_Port GPIOB
#define PLATTER_SWITCH_Pin GPIO_PIN_10
#define PLATTER_SWITCH_GPIO_Port GPIOA
#define TOP_PLATTER_HEAT_Pin GPIO_PIN_11
#define TOP_PLATTER_HEAT_GPIO_Port GPIOA
#define BOTTOM_PLATTER_HEAT_Pin GPIO_PIN_12
#define BOTTOM_PLATTER_HEAT_GPIO_Port GPIOA
#define TOP_TRAVEL_SWITCH_Pin GPIO_PIN_6
#define TOP_TRAVEL_SWITCH_GPIO_Port GPIOF
#define BOTTOM_TRAVEL_SWITCH_Pin GPIO_PIN_7
#define BOTTOM_TRAVEL_SWITCH_GPIO_Port GPIOF
#define CS_THERMOCOUPLE_BOTTOM2_Pin GPIO_PIN_6
#define CS_THERMOCOUPLE_BOTTOM2_GPIO_Port GPIOB
#define CS_THERMOCOUPLE_BOTTOM1_Pin GPIO_PIN_7
#define CS_THERMOCOUPLE_BOTTOM1_GPIO_Port GPIOB
#define CS_THERMOCOUPLE_TOP2_Pin GPIO_PIN_8
#define CS_THERMOCOUPLE_TOP2_GPIO_Port GPIOB
#define CS_THERMOCOUPLE_TOP1_Pin GPIO_PIN_9
#define CS_THERMOCOUPLE_TOP1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
