/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum ADCChannels
{  // order is important! - this MCU has no ranking and the order of channels is defined by channel number
	  Temp1,  // temperature sensor 1 - PA2
	  Temp2,  // temperature sensor 1 - PA3
	  Temp3,	// temperature sensor 1 - PA4
	  Tempint,  // internal temperature sensor; value in tenths of degrees Celsius
	  Vdda  // internal reference voltage
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/*
 * fan PWM current consumption
 * duty cycle - RPM (measured) - current/mA - tacho signal period (measured approx.)/ms
 * 		100% - 2271 RPM - 64 mA - 13.4
 * 		90% - 2082 RPM - 54 mA - 14
 * 		80% - 1875 RPM - 44 mA - 16
 * 		70% - 1665 RPM - 37 mA - 18
 * 		60% - 1428 RPM - 30 mA - 21
 * 		50% - 1200 RPM - 24 mA - 26
 * 		40% - 936 RPM - 19 mA - 32
 * 		30% - 681 RPM - 16 mA - 44
 * 		20% - 417 RPM - 13 mA - 72
 * 		10% - 179 RPM - 11 mA - 170
 * 		0% - 0 RPM - 9 mA
 */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TS_CAL2 *((uint16_t*) ((uint32_t) 0x1FFFF7C2))
#define FAN_DUTY_CYCLE 0	// fan duty cycle, 0 to 100%
#define TS_CAL1 *((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define TEMPERATURE_OFFSET -35	// correction value (offset) for temperature calculation
#define VREFINT_CAL *((uint16_t*) ((uint32_t) 0x1FFFF7BA))
#define ADC_CHANNELS 5 // 3 external (IN2,3,4) and one internal temperature channels, Vrefint
#define TIMER1_PERIOD 124 // ADC trigger timer
#define ADC_MEASURE_ITERATIONS 80 // measure timer1 period times this number == 1s
#define TIMER2_PERIOD 0xFFFFFFFF // PA5 - fan tacho
#define TIMER3_PERIOD 99 // PA6 - fan PWM
#define TIMER_PRESCALER 799
#define TACHO_BUFFER_LEN 10 // size of ADC readout buffer
#define USE_DEBUG_PIN 0 // make use of PA15
#define USE_SLEEP 1 // enter sleep mode once an ISR is finished
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOA
#define TEMP1_SIGNAL_Pin GPIO_PIN_2
#define TEMP1_SIGNAL_GPIO_Port GPIOA
#define TEMP2_SIGNAL_Pin GPIO_PIN_3
#define TEMP2_SIGNAL_GPIO_Port GPIOA
#define TEMP3_SIGNAL_Pin GPIO_PIN_4
#define TEMP3_SIGNAL_GPIO_Port GPIOA
#define FAN_TACHO_Pin GPIO_PIN_5
#define FAN_TACHO_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_6
#define FAN_PWM_GPIO_Port GPIOA
#define debug_out_Pin GPIO_PIN_15
#define debug_out_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
