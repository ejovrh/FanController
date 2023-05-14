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
	  Tempint,  // internal temperature sensor
	  Vrefint  // internal reference voltage
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIMER_PRESCALER 799
#define TIMER1_PERIOD 1249
#define TIMER3_PERIOD 99
#define FAN_OFF 0
#define FAN_DEBUG 70
#define TIMER2_PERIOD 1250
#define ADC_CHANNELS 5
#define TACHO_BUFFER_LEN 2
#define ADC_MEASURE_ITERATIONS 8
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
