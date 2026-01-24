/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "uvfr_utils.h"

#define NUM_WHEELS 4
#define WHEEL_CIRCUMFERENCE_M 2.0f // just placeholder values here
#define TIMER_TICK_US 1.0f
#define PULSES_PER_REV 20

volatile uint32_t last_timestamp[] = {0,0,0,0};
volatile uint32_t period[] = {0,0,0,0};
volatile float wheel_speed[] = {0,0,0,0};
volatile float wheel_rpm[] = {0,0,0,0};
volatile float frequency[] = {0,0,0,0};

extern SemaphoreHandle_t xWheelSpeedSem;

/* USER CODE END 0 */

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  HAL_TIM_Base_Start(&htim3); //starts free running timer
  /* USER CODE END TIM3_Init 2 */

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  HAL_TIM_Base_Start(&htim5); //starts free running timer
  /* USER CODE END TIM5_Init 2 */

}
/* TIM11 init function */
void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspInit 0 */

  /* USER CODE END TIM11_MspInit 0 */
    /* TIM11 clock enable */
    __HAL_RCC_TIM11_CLK_ENABLE();
  /* USER CODE BEGIN TIM11_MspInit 1 */

  /* USER CODE END TIM11_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspDeInit 0 */

  /* USER CODE END TIM11_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM11_CLK_DISABLE();
  /* USER CODE BEGIN TIM11_MspDeInit 1 */

  /* USER CODE END TIM11_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


// -------- INTERRUPT -----------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_12: handle_wheel_interrupt(0); break; // Wheel 1
    case GPIO_PIN_13: handle_wheel_interrupt(1); break; // Wheel 2
    case GPIO_PIN_14: handle_wheel_interrupt(2); break; // Wheel 3
    case GPIO_PIN_15: handle_wheel_interrupt(3); break; // Wheel 4
    default: break;
  }
}

void handle_wheel_interrupt(uint32_t wheel_index)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim5);
  uint32_t last = last_timestamp[wheel_index];

  if (now >= last) {
      period[wheel_index] = now - last;
  } else {
      period[wheel_index] = (0xFFFFFFFF - last) + now; // Handle timer overflow
  }
  last_timestamp[wheel_index] = now;
}

void WheelSpeed_UpdateAll(void)
{
    // TODO: Loop through each wheel
    // TODO: Convert period → frequency
    // TODO: Convert frequency → speed
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
	  if(__HAL_TIM_GET_COUNTER(&htim5) - last_timestamp[i]  > 500000){
		  frequency[i] = 0.0f;
		  wheel_speed[i] = 0.0f;
		  wheel_rpm[i] = 0.0f;
	  }

    if (period[i] > 0) {
      // Convert period (µs) → frequency (Hz)
      float period_s = (period[i] * TIMER_TICK_US) / 1e6f; // Convert to seconds
      frequency[i] = 1.0f / period_s;

      // Convert frequency → speed
      float rev_per_sec = frequency[i] / PULSES_PER_REV;
      wheel_speed[i] = rev_per_sec * WHEEL_CIRCUMFERENCE_M;
      wheel_rpm[i] = rev_per_sec * 60.0f;
    } else {
      frequency[i] = 0.0f;
      wheel_speed[i] = 0.0f;
      wheel_rpm[i] = 0.0f;
    }
  }
  if (xWheelSpeedSem != NULL){
	  xSemaphoreGive(xWheelSpeedSem);
  }
}

void dispWheelSpeeds(){
	WheelSpeed_UpdateAll();
	printf("WHEEL SPEED DATA \n");
	printf("Periods: %d %d %d %d \n",period[0],period[1],period[2],period[3]);
	uint16_t freq1 = ((uint16_t)frequency[0])*10;
	uint16_t freq2 = ((uint16_t)frequency[1])*10;
	uint16_t freq3 = ((uint16_t)frequency[2])*10;
	uint16_t freq4 = ((uint16_t)frequency[3])*10;
	printf("Frequencies: %d %d %d %d \n",freq1,freq2,freq3,freq4);
}
/* USER CODE END 1 */
