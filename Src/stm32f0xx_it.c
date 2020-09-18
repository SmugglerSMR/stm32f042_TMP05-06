/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
int timer0_count = 0, timer1_count = 0;
int tempsegment = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;
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

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
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
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	if (tempsegment == 0) {
		__HAL_TIM_SET_COUNTER(&htim1,0);
	} else if (tempsegment == 1) {
		temp_high0 = (__HAL_TIM_GET_COUNTER(&htim1)); //Convert to integer
		__HAL_TIM_SET_COUNTER(&htim2,0);
	} else if (tempsegment == 2) {
		temp_low0 = (__HAL_TIM_GET_COUNTER(&htim2)); //Convert to integer
		__HAL_TIM_SET_COUNTER(&htim1,0);
	} else if (tempsegment == 3) {
		temp_high1 = (__HAL_TIM_GET_COUNTER(&htim1));
		__HAL_TIM_SET_COUNTER(&htim2,0);
	} else if (tempsegment == 4) {
		temp_low1 = (__HAL_TIM_GET_COUNTER(&htim2));
		__HAL_TIM_SET_COUNTER(&htim1,0);
	} else if (tempsegment == 5) {
		temp_high2 = (__HAL_TIM_GET_COUNTER(&htim1));
		__HAL_TIM_SET_COUNTER(&htim2,0);
	} else if (tempsegment == 6) {
		temp_low2 = (__HAL_TIM_GET_COUNTER(&htim2));
		__HAL_TIM_SET_COUNTER(&htim2,0);
	} else if (tempsegment == 7) {
		temp_high3 = (__HAL_TIM_GET_COUNTER(&htim1));
		__HAL_TIM_SET_COUNTER(&htim2,0);
	}
	tempsegment++;

//	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)) {
//		__HAL_TIM_SET_COUNTER(&htim2,0);
//	} else {
//		if (tempsegment == 1) {
//			temp_high0 = (__HAL_TIM_GET_COUNTER(&htim2));
//
//			__HAL_TIM_SET_COUNTER(&htim1,0);
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//
//		} else if (tempsegment == 2) {
//			temp_high1 = (__HAL_TIM_GET_COUNTER(&htim2)); //Convert to integer
//			temp_low0 = (__HAL_TIM_GET_COUNTER(&htim1)); //Convert to integer
//
//			__HAL_TIM_SET_COUNTER(&htim1,0); //Reset count
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//		} else if (tempsegment == 3) {
//			temp_low1 = (__HAL_TIM_GET_COUNTER(&htim1)); //Convert to integer
//			temp_high2 = (__HAL_TIM_GET_COUNTER(&htim2));
//			__HAL_TIM_SET_COUNTER(&htim1,0); //Reset count
//			__HAL_TIM_SET_COUNTER(&htim2,0);
//		}
//		tempsegment++;
//	}
  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
