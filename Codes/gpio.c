/**
  ******************************************************************************
  * @file           : gpio.c
  * @brief          : This file provides code for the configuration of all used GPIO pins.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/**
  * @brief Configure pins as Analog, Input, Output, EVENT_OUT, EXTI.
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin Output Level for LED and IR outputs */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_UD_GPIO_Port, IR_UD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_LR_GPIO_Port, IR_LR_Pin, GPIO_PIN_RESET);

  /* Configure GPIO pin : TOUCH_SENSOR_Pin (PA0) */
  GPIO_InitStruct.Pin = TOUCH_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /* (Optional) Configure GPIO pin : B1_Pin if used as an additional button */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin : LD2_Pin (User LED) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pins for IR sensor outputs: IR_UD_Pin and IR_LR_Pin */
  GPIO_InitStruct.Pin = IR_UD_Pin | IR_LR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_UD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init for TOUCH_SENSOR_Pin (PA0) */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* (Optional) EXTI interrupt init for B1_Pin if used */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
