
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
//#include "ds18.h"
#include "control.h"
#include "display.h"
#include "stm32f1xx_ll_gpio.h"
//#include "step.h"
#include "dcts.h"
#include "pin_map.h"
#include "buttons.h"

#define FEEDER 0
#define DEFAULT_TASK_PERIOD 100

typedef enum{
    READ_FLOAT_SIGNED = 0,
    READ_FLOAT_UNSIGNED,
}read_float_bkp_sign_t;


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
IWDG_HandleTypeDef hiwdg;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
osThreadId defaultTaskHandle;
osThreadId buttonsTaskHandle;
osThreadId displayTaskHandle;
osThreadId menuTaskHandle;
osThreadId controlTaskHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void default_task(void const * argument);
static void save_to_bkp(u8 bkp_num, u8 var);
static void save_float_to_bkp(u8 bkp_num, float var);
static u8 read_bkp(u8 bkp_num);
static float read_float_bkp(u8 bkp_num, u8 sign);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_IWDG_Init();
    dcts_init();
    MX_RTC_Init();
    MX_ADC1_Init();
    //MX_USART1_UART_Init();
    //MX_TIM3_Init();
    //MX_TIM2_Init();
    HAL_ADC_Start(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc1);
    osThreadDef(own_task, default_task, osPriorityNormal, 0, 364);
    defaultTaskHandle = osThreadCreate(osThread(own_task), NULL);
#if FEEDER
    osThreadDef(step_task, step_task, osPriorityNormal, 0, 364);
    defaultTaskHandle = osThreadCreate(osThread(step_task), NULL);
#else
    /*osThreadDef(ds18_task, ds18_task, osPriorityHigh, 0, 364);
    defaultTaskHandle = osThreadCreate(osThread(ds18_task), NULL);
*/

    osThreadDef(control_task, control_task, osPriorityNormal, 0, 364);
    controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

    osThreadDef(display_task, display_task, osPriorityNormal, 0, 512);
    displayTaskHandle = osThreadCreate(osThread(display_task), NULL);

    osThreadDef(buttons_task, buttons_task, osPriorityNormal, 0, 128);
    buttonsTaskHandle = osThreadCreate(osThread(buttons_task), NULL);

    osThreadDef(menu_task, menu_task, osPriorityNormal, 0, 364);
    menuTaskHandle = osThreadCreate(osThread(menu_task), NULL);

#endif

    /* Start scheduler */
    osKernelStart();

    while (1)  {

    }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
            |RCC_PERIPHCLK_USB;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    /* Common config */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }

    /* Configure Injected Channels */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedNbrOfConversion = 3;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = ENABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
      Error_Handler();
    }

    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
      Error_Handler();
    }

    sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
      Error_Handler();
    }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* RTC init function */
static void MX_RTC_Init(void){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    u32 data;
    const  u32 data_c = 0x1234;
    data = BKP->DR1;
    if(data!=data_c){   // set default values
        HAL_PWR_EnableBkUpAccess();
        BKP->DR1 = data_c;
        HAL_PWR_DisableBkUpAccess();

        sTime.Hours = dcts.dcts_rtc.hour;
        sTime.Minutes = dcts.dcts_rtc.minute;
        sTime.Seconds = dcts.dcts_rtc.second;

        sDate.Date = dcts.dcts_rtc.day;
        sDate.Month = dcts.dcts_rtc.month;
        sDate.Year = (uint8_t)(dcts.dcts_rtc.year - 2000);

        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        save_float_to_bkp(2, dcts_act[0].set_value);
        save_to_bkp(3, dcts_act[0].state.control);

        save_float_to_bkp(4, sensor_state.dispersion*10);
        save_float_to_bkp(5, sensor_state.hysteresis*10);
        save_float_to_bkp(6, sensor_state.correction*10);
        save_to_bkp(7, sensor_state.buff_size);

        save_float_to_bkp(8, semistor_state.max_tmpr);
    }else{  // read data from bkpram
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = sTime.Hours;
        dcts.dcts_rtc.minute = sTime.Minutes;
        dcts.dcts_rtc.second = sTime.Seconds;

        dcts.dcts_rtc.day = sDate.Date;
        dcts.dcts_rtc.month = sDate.Month;
        dcts.dcts_rtc.year = sDate.Year + 2000;
        dcts.dcts_rtc.weekday = sDate.WeekDay;

        dcts_act[0].set_value = read_float_bkp(2, READ_FLOAT_UNSIGNED);
        dcts_act[0].state.control = read_bkp(3);

        sensor_state.dispersion = read_float_bkp(4, READ_FLOAT_UNSIGNED)/10;
        sensor_state.hysteresis = read_float_bkp(5, READ_FLOAT_UNSIGNED)/10;
        sensor_state.correction = read_float_bkp(6, READ_FLOAT_SIGNED)/10;
        sensor_state.buff_size = read_bkp(7);

        semistor_state.max_tmpr = read_float_bkp(8, READ_FLOAT_UNSIGNED);
    }
}

/* TIM3 init function */
static void MX_TIM3_Init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef pwm_handle;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = MAX_PWM_VALUE;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    pwm_handle.OCMode = TIM_OCMODE_PWM1;
    pwm_handle.Pulse = 16000;
    pwm_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm_handle.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &pwm_handle, TIM_CHANNEL_1) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_TIM_MspPostInit(&htim3);
}


/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/** Configure pins as 
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void){

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Buttons */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = UP_PIN;
    HAL_GPIO_Init(UP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DOWN_PIN;
    HAL_GPIO_Init(DOWN_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LEFT_PIN;
    HAL_GPIO_Init(LEFT_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = RIGHT_PIN;
    HAL_GPIO_Init(RIGHT_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = OK_PIN;
    HAL_GPIO_Init(OK_PORT, &GPIO_InitStruct);

    /* ADC inputs
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = LOAD_TEMP_PIN;
    HAL_GPIO_Init(LOAD_TEMP_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = REG_TEMP_PIN;
    HAL_GPIO_Init(REG_TEMP_PORT, &GPIO_InitStruct);
    */

    /* DEBUG pins */

    /* I2C display pins */

    /* 50 Hz SYNC pin */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = SYNC_PIN;
    HAL_GPIO_Init(SYNC_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* REG_ON pin */
    HAL_GPIO_WritePin(REG_ON_PORT, REG_ON_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = REG_ON_PIN;
    HAL_GPIO_Init(REG_ON_PORT, &GPIO_InitStruct);
}

/* StartDefaultTask function */
void default_task(void const * argument){

    (void)argument;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t last_wake_time = osKernelSysTick();

    HAL_IWDG_Refresh(&hiwdg);
    while(1){
        HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = time.Hours;
        dcts.dcts_rtc.minute = time.Minutes;
        dcts.dcts_rtc.second = time.Seconds;

        dcts.dcts_rtc.day = date.Date;
        dcts.dcts_rtc.month = date.Month;
        dcts.dcts_rtc.year = date.Year + 2000;
        dcts.dcts_rtc.weekday = date.WeekDay;

        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DEFAULT_TASK_PERIOD);
    }
    /* USER CODE END 5 */
}
/* TIM2 init function */
static void MX_TIM2_Init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65000;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }

}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

static void save_to_bkp(u8 bkp_num, u8 var){
    uint32_t data = var;
    if(bkp_num%2 == 1){
        data = data << 8;
    }
    HAL_PWR_EnableBkUpAccess();
    switch (bkp_num / 2){
    case 0:
        BKP->DR1 |= data;
        break;
    case 1:
        BKP->DR2 |= data;
        break;
    case 2:
        BKP->DR3 |= data;
        break;
    case 3:
        BKP->DR4 |= data;
        break;
    case 4:
        BKP->DR5 |= data;
        break;
    case 5:
        BKP->DR6 |= data;
        break;
    case 6:
        BKP->DR7 |= data;
        break;
    case 7:
        BKP->DR8 |= data;
        break;
    case 8:
        BKP->DR9 |= data;
        break;
    case 9:
        BKP->DR10 |= data;
        break;
    }
    HAL_PWR_DisableBkUpAccess();
}

static void save_float_to_bkp(u8 bkp_num, float var){
    char buf[5] = {0};
    sprintf(buf, "%4.0f", (double)var);
    u8 data = (u8)atoi(buf);
    save_to_bkp(bkp_num, data);
}
static u8 read_bkp(u8 bkp_num){
    uint32_t data = 0;
    switch (bkp_num/2){
    case 0:
        data = BKP->DR1;
        break;
    case 1:
        data = BKP->DR2;
        break;
    case 2:
        data = BKP->DR3;
        break;
    case 3:
        data = BKP->DR4;
        break;
    case 4:
        data = BKP->DR5;
        break;
    case 5:
        data = BKP->DR6;
        break;
    case 6:
        data = BKP->DR7;
        break;
    case 7:
        data = BKP->DR8;
        break;
    case 8:
        data = BKP->DR9;
        break;
    case 9:
        data = BKP->DR10;
        break;
    }
    if(bkp_num%2 == 1){
        data = data >> 8;
    }
    return (u8)(data & 0xFF);
}
static float read_float_bkp(u8 bkp_num, u8 sign){
    u8 data = read_bkp(bkp_num);
    char buf[5] = {0};
    if(sign == READ_FLOAT_SIGNED){
        sprintf(buf, "%d", (s8)data);
    }else{
        sprintf(buf, "%d", data);
    }
    return atoff(buf);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

