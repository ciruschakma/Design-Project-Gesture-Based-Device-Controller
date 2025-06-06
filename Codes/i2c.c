/**
  ******************************************************************************
  * @file           : i2c.c
  * @brief          : This file provides code for the configuration of the I2C instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C MSP Initialization
  * @param i2cHandle: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /** I2C1 GPIO Configuration
      PB8     ------> I2C1_SCL
      PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_I2C1_ENABLE();

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

/**
  * @brief I2C MSP De-Initialization
  * @param i2cHandle: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  if(i2cHandle->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /** I2C1 GPIO Configuration
      PB8     ------> I2C1_SCL
      PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
  }
}
