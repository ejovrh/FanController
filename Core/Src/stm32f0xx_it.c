/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f0xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t max(const uint32_t a, const uint32_t b)
{
	return ((a) > (b) ? a : b);
}
static inline uint32_t min(const uint32_t a, const uint32_t b)
{
	return ((a) < (b) ? a : b);
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern DMA_HandleTypeDef hdma_tim2_ch1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern volatile uint32_t adc_buffer[ADC_CHANNELS];  // DMA buffer
extern volatile int32_t adc[ADC_CHANNELS];  // final value
extern uint32_t tacho[TACHO_BUFFER_LEN];  // store for raw tacho readout

static volatile uint32_t adc_avg_buffer[ADC_CHANNELS];  // values are summed up for average calculation
static volatile uint8_t adc_iterator = 0;  // iterators
static volatile uint8_t FlagIsMeasured = 0;
static volatile uint8_t FlagRiseCaptured = 0;
static volatile float riseavg = 0;

extern uint32_t RPM;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while(1)
		{
		}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while(1)
		{
			/* USER CODE BEGIN W1_HardFault_IRQn 0 */
			/* USER CODE END W1_HardFault_IRQn 0 */
		}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVC_IRQn 0 */

	/* USER CODE END SVC_IRQn 0 */
	/* USER CODE BEGIN SVC_IRQn 1 */

	/* USER CODE END SVC_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel 1 interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

	/* USER CODE END DMA1_Channel1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc);
	/* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	/* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel 4 and 5 interrupts.
 */
void DMA1_Channel4_5_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel4_5_IRQn 0 */

	/* USER CODE END DMA1_Channel4_5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_tim2_ch1);
	/* USER CODE BEGIN DMA1_Channel4_5_IRQn 1 */

	/* USER CODE END DMA1_Channel4_5_IRQn 1 */
}

/**
 * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
 */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

	/* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

	/* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
// timer elapsed ISRs
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// timer1 - debug pin high; will be set to low once the ADC conversion is complete
	if(htim == &htim1)	// ADC measurement - every 12.5ms
		{
#ifdef USE_DEBUG_PIN
			HAL_GPIO_WritePin(debug_out_GPIO_Port, debug_out_Pin, GPIO_PIN_SET);	// set debug pin high
#endif
		}
}

// timer2 DMA transfer complete callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
		{
			// If the Interrupt is triggered by 1st Channel
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
				FlagRiseCaptured = 1;

			/* Rest of the calculations will be done,
			 * once both the DMAs have finished capturing enough data */
			if(FlagRiseCaptured)
				{
					uint8_t indxr = 0;
					uint8_t countr = 0;

					/* In case of high Frequencies, the DMA sometimes captures 0's in the beginning.
					 * increment the index until some useful data shows up
					 */
					while(tacho[indxr] == 0)
						indxr++;

					/* Again at very high frequencies, sometimes the values don't change
					 * So we will wait for the update among the values
					 */
					while((min((tacho[indxr + 1] - tacho[indxr]), (tacho[indxr + 2] - tacho[indxr + 1]))) == 0)
						indxr++;

					/* riseavg is the difference in the 2 consecutive rise Time */

					/* Assign a start value to riseavg */
					riseavg += min((tacho[indxr + 1] - tacho[indxr]), (tacho[indxr + 2] - tacho[indxr + 1]));
					indxr++;
					countr++;

					/* start adding the values to the riseavg */
					while(indxr < (TACHO_BUFFER_LEN))
						{
							riseavg += min((tacho[indxr + 1] - tacho[indxr]), riseavg / countr);
							countr++;
							indxr++;
						}

					/* Find the average riseavg, the average time between 2 RISE */
					riseavg = riseavg / countr;

					indxr = 0;
					FlagRiseCaptured = 0;
					FlagIsMeasured = 1;
				}

			if(FlagIsMeasured)
				{
					HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, tacho, TACHO_BUFFER_LEN);
					TIM2->CNT = 0;
					FlagIsMeasured = 0;
				}
		}
}

// ADC measurement - every 125ms
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint32_t maxtemp = 0;  // variable for overall maximal temperature calculation

#ifdef USE_DEBUG_PIN
	// timer1 - ADC conversion complete - debug pin low
	HAL_GPIO_WritePin(debug_out_GPIO_Port, debug_out_Pin, GPIO_PIN_RESET);	// set debug pin low
#endif

	for(uint8_t i = 0; i < ADC_CHANNELS; ++i)  // go over all the channels
		adc_avg_buffer[i] += adc_buffer[i];  // and store raw values in a temporary buffer for averages calculation

	++adc_iterator;  // advance the iterator

	if(adc_iterator == ADC_MEASURE_ITERATIONS)	// 1 Hz average calculation: ADC_MEASURE_ITERATIONS * TIMER1_PERIOD == 1s (1000ms)
		{
			// averages calculation for all raw data
			for(uint8_t i = 0; i < ADC_CHANNELS; ++i)
				adc[i] = (int32_t) ((adc_avg_buffer[i] / adc_iterator) + 0.5);  // calculate and store average via rounding conversion
			// adc[] now contains thousands of units

			// Vrefint to VDDA calculation
			adc[Vdda] = 3300 * VREFINT_CAL / adc[Vdda];  // compute and store actual Vdda value in mV

			for(uint8_t i = 0; i < 3; ++i)	// iterate over 3 temperature sensors (see enum ADCChannels)
				{
					int32_t tmp = (int32_t) (((adc[Vdda] * adc[i]) / 4096) + 0.5);  // compute and store actual temp. sensor output voltage in mV
					// LMT86 datasheet, p. 11, equation 6 (is probably close enough)
					adc[i] = (int32_t) (((tmp - 2103) / -0.109) + 0.5) + TEMPERATURE_OFFSET;	// compute and store degrees centigrade
				}

			adc[Tempint] = (((((double) (adc[Tempint] * VREFINT_CAL) / adc[Vdda]) - TS_CAL1) * 800) / (int16_t) (TS_CAL2 - TS_CAL1)) + 300;  // degrees centigrade

			// zero out stuff for the next iteration
			for(uint8_t i = 0; i < ADC_CHANNELS; ++i)
				adc_avg_buffer[i] = 0;

			RPM = (uint32_t) (((HAL_RCC_GetHCLKFreq() / riseavg) * 30) + 0.5);	// find out the max. temperature from all of the sensors

			maxtemp = max(max(adc[Temp1], adc[Temp2]), adc[Temp3]);

			// kind of a linear response
			if(maxtemp >= 6000)  // if hotter than 60 degrees centigrade
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);	// 100%

			if(maxtemp < 6000)  // if cooler than 60 deg. C
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 75);  // 75%

			if(maxtemp < 5000)  // if cooler than 50 deg. C
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);  // 50%

			if(maxtemp < 4000)  // if cooler than 40 deg. C
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 25);  // 25%

			if(maxtemp < 3000)  // if cooler than 30 deg. C
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	// fan off

			adc_iterator = 0;  // mark end of iteration
		}
}

/* USER CODE END 1 */
