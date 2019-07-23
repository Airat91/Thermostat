/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"
#include "stm32f1xx_ll_gpio.h"

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
}
void i2c_deinit(void)
{
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }
}
static void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c){
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1) {
      /* Peripheral clock enable */
      __HAL_RCC_I2C1_CLK_ENABLE();
      static uint8_t resetTried = 0;
      resetTried = 0;
      if (resetTried == 1) {
          return ;
      }
      uint16_t SDA_PIN = I2C1_SDA;
      uint16_t SCL_PIN = I2C1_SCL;
      GPIO_InitTypeDef GPIO_InitStruct;
      // 1
      __HAL_I2C_DISABLE(hi2c);
      // 2
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin = SDA_PIN;
      HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = SCL_PIN;
      HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

      HAL_GPIO_WRITE_ODR(I2C1_SDA_PORT, SDA_PIN);
      HAL_GPIO_WRITE_ODR(I2C1_SCL_PORT, SCL_PIN);
      // 3
      if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, SDA_PIN) == GPIO_PIN_RESET) {
          for(;;){}
      }
      if (HAL_GPIO_ReadPin(I2C1_SCL_PORT, SCL_PIN) == GPIO_PIN_RESET) {
          for(;;){}
      }
      // 4
      GPIO_InitStruct.Pin = SDA_PIN;
      HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
      HAL_GPIO_TogglePin(I2C1_SDA_PORT, SDA_PIN);
      // 5
      if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, SDA_PIN) == GPIO_PIN_SET) {
          for(;;){}
      }
      // 6
      GPIO_InitStruct.Pin = SCL_PIN;
      HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
      HAL_GPIO_TogglePin(I2C1_SCL_PORT, SCL_PIN);
      // 7
      if (HAL_GPIO_ReadPin(I2C1_SCL_PORT, SCL_PIN) == GPIO_PIN_SET) {
          for(;;){}
      }
      // 8
      GPIO_InitStruct.Pin = SDA_PIN;
      HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
      HAL_GPIO_WRITE_ODR(I2C1_SDA_PORT, SDA_PIN);
      // 9
      if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, SDA_PIN) == GPIO_PIN_RESET) {
          for(;;){}
      }
      // 10
      GPIO_InitStruct.Pin = SCL_PIN;
      HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
      HAL_GPIO_WRITE_ODR(I2C1_SCL_PORT, SCL_PIN);
      // 11
      if (HAL_GPIO_ReadPin(I2C1_SCL_PORT, SCL_PIN) == GPIO_PIN_RESET) {
          for(;;){}
      }
      // 12

      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Pin = I2C1_SDA;
      HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = I2C1_SCL;
      HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);
     // 13
      hi2c->Instance->CR1 |= I2C_CR1_SWRST;
     // 14
      hi2c->Instance->CR1 ^= I2C_CR1_SWRST;
     // 15
      __HAL_I2C_ENABLE(hi2c);
      resetTried = 1;
  }
}
void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR |= GPIO_Pin;
}
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
      __HAL_RCC_I2C1_CLK_DISABLE();

      /**I2C1 GPIO Configuration
      PB8     ------> I2C1_SCL
      PB9     ------> I2C1_SDA
      */
      HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
      LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
      LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
      for (uint8_t i =0;i<8;i++){
          LL_GPIO_SetOutputPin(GPIOB,GPIO_PIN_8);
          LL_GPIO_SetOutputPin(GPIOB,GPIO_PIN_9);
          LL_GPIO_ResetOutputPin(GPIOB,GPIO_PIN_8);
          LL_GPIO_ResetOutputPin(GPIOB,GPIO_PIN_9);
      }
      HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
      __HAL_AFIO_REMAP_I2C1_ENABLE();
      /* Peripheral clock enable */
      __HAL_RCC_I2C1_CLK_ENABLE();

  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
