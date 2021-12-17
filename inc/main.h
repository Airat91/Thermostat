/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

#include "stdint.h"
#include "cmsis_os.h"

#define AIR_PORT GPIOA
#define AIR_PIN  LL_GPIO_PIN_7
#define FLOW_PORT GPIOA
#define FLOW_PIN  LL_GPIO_PIN_5
#define LIGTH_PORT GPIOA
#define LIGTH_PIN  LL_GPIO_PIN_4
#define LIGTH2_PORT GPIOA
#define LIGTH2_PIN  LL_GPIO_PIN_3
#define	_DS18B20_GPIO  GPIOB
#define	_DS18B20_PIN   GPIO_PIN_15
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL  GPIO_PIN_6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA  GPIO_PIN_7
#define ADC0_PIN  GPIO_PIN_0
#define ADC1_PIN  GPIO_PIN_1
#define ADC_PORT GPIOA
#define STEP_OUT1_1 LL_GPIO_PIN_13
#define STEP_OUT1_2 LL_GPIO_PIN_14
#define STEP_OUT2_1 LL_GPIO_PIN_11
#define STEP_OUT2_2 LL_GPIO_PIN_12
#define STEP_PORT GPIOB
/**USART1 GPIO Configuration
PA9     ------> USART1_TX
PA10     ------> USART1_RX
*/
//#define LED_PORT GPIOC
//#define LED_PIN  LL_GPIO_PIN_13


#define TIME_YIELD_THRESHOLD 100
#define MEAS_NUM 13
#define ACT_NUM 3
#define RELE_NUM 2

#define SAVED_PARAMS_SIZE 23


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

#ifdef __cplusplus
 extern "C" {
#endif

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

 typedef enum {
     TMPR_FLOOR_GRAD = 0,
     TMPR_FLOOR_RES,
     TMPR_FLOOR_ADC,
     TMPR_FLOOR_VLT,
     TMPR_REG_GRAD,
     TMPR_REG_ADC,
     TMPR_REG_VLT,
     VREF_VLT,
     VBAT_VLT,
     CONSUMPTION,
     SYNC_0,
     SYNC_1,
     SYNC_FREQ,
 }dcts_meas_t;

 typedef enum {
     HEATING = 0,
     SEMISTOR,
     PWR_PHASE,
 }dcts_act_t;

 typedef enum {
     HEATER = 0,
     LED,
 }dcts_rele_t;

 typedef enum{
     MENU_NAVIGATION,
     DIGIT_EDIT,
     BLOCKED,
 }navigation_t;

 typedef enum{
    SKIN_FULL = 0,
    SKIN_TIME,
    SKIN_TEMP,
 }skin_t;

 typedef enum{
     RULE_RELE,
     RULE_PHASE,
 }ctrl_rule_t;

 typedef enum{
     SENSOR_NTC_10K,
     SENSOR_NTC_100K,
     SENSOR_DB18B20,
 }sensor_type_t;

 typedef enum{
     PHASE_CTRL_PID,
     PHASE_CTRL_MANUAL,
 }phase_pwr_ctrl_t;

 typedef union{
     struct{
         uint16_t act_enable[ACT_NUM];  // 1*3
         float    act_set[ACT_NUM];     // 2*3
         float    act_hyst[ACT_NUM];    // 2*3
         uint16_t rele[RELE_NUM];       // 1*2
         uint8_t  skin;                 // 0.5
         uint8_t  sensor_type;          // 0.5
         float    load_res;             // 2
         uint8_t  backlight_lvl;        // 0.5
         uint8_t  auto_off;             // 0.5
         uint8_t  mdb_address;          // 0.5
         uint8_t  ctrl_rule;            // 0.5
         uint8_t  phase_pwr_ctrl;       // 0.5
     }params;                           //
     uint16_t word[SAVED_PARAMS_SIZE];
 }saved_to_flash_t;

 typedef enum{
     VAL_UNKNOWN = 0,
     VAL_UINT8,
     VAL_INT8,
     VAL_UINT16,
     VAL_INT16,
     VAL_UINT32,
     VAL_INT32,
     VAL_FLOAT,
 }edit_val_type;

 typedef union{
     uint8_t * p_uint8;
     int8_t * p_int8;
     uint16_t * p_uint16;
     int16_t * p_int16;
     uint32_t * p_uint32;
     int32_t * p_int32;
     float * p_float;
 }edit_val_p_type_t;

 typedef union{
     uint8_t uint8;
     int8_t int8;
     uint16_t uint16;
     int16_t int16;
     uint32_t uint32;
     int32_t int32;
     float vfloat;
 }edit_val_type_t;

 typedef struct{
     edit_val_p_type_t p_val;
     edit_val_type_t val_min;
     edit_val_type_t val_max;
     int8_t digit;
     int8_t digit_min;
     int8_t digit_max;
     edit_val_type type;
     uint8_t select_width;
     uint8_t select_shift;
 }edit_val_t;

 extern  osThreadId defaultTaskHandle;
 extern  osThreadId buttonsTaskHandle;
 extern  osThreadId displayTaskHandle;
 extern  osThreadId menuTaskHandle;
 extern  osThreadId controlTaskHandle;
 extern  osThreadId adcTaskHandle;

 extern uint32_t us_cnt_H;
 extern navigation_t navigation_style;
 extern edit_val_t edit_val;
 extern saved_to_flash_t config;

 uint32_t us_tim_get_value(void);
 void us_tim_delay(uint32_t us);
 uint32_t uint32_pow(uint16_t x, uint8_t pow);
 uint16_t uint16_pow(uint16_t x, uint16_t pow);
 float float_pow(float x, int pow);
 void refresh_watchdog(void);
 void rtc_task(void const * argument);

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
