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
#include "gpio.h"
#include <math.h>

/* USER CODE BEGIN 0 */
#define NUM_WHEELS 4
#define WHEEL_CIRCUMFERENCE_M 2.0f

/* USER CODE END 0 */

volatile uint32_t last_timestamp[NUM_WHEELS] = {0};
volatile uint32_t period[NUM_WHEELS] = {0};
volatile float frequency[NUM_WHEELS] = {0.0f};
volatile float wheel_speed[NUM_WHEELS] = {0.0f};
volatile float wheel_rpm[NUM_WHEELS] = {0.0f};

TIM_HandleTypeDef htim3;

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
  htim3.Init.Prescaler = 41999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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

static void handle_wheel_interrupt(uint32_t wheel_index)
{
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim3);
  uint32_t last = last_timestamp[wheel_index];

  if (now >= last) {
      period[wheel_index] = now - last;
  } else {
      period[wheel_index] = (0xFFFFFFFF - last) + now; // Handle timer overflow
  }
  last_timestamp[wheel_index] = now;
}
/* USER CODE END 1 */






/*
IN MAIN.C FILE WE JUST HAVE TO CALL THIS

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();  // initializes and starts timer

    while (1)
    {
        TIM_Process_WheelSpeeds();  // update frequency & speed
        HAL_Delay(100);
    }
}


*/