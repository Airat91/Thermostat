
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
//#include "ds18.h"
#include "control.h"
#include "display.h"
#include "stm32f1xx_ll_gpio.h"
//#include "step.h"
#include "dcts.h"
#include "pin_map.h"
#include "buttons.h"
#include "string.h"
#include "flash.h"
#include "adc.h"
#include "time.h"

#define FEEDER 0
#define RELEASE 0
#define RTC_KEY 0xABCD

typedef enum{
    READ_FLOAT_SIGNED = 0,
    READ_FLOAT_UNSIGNED,
}read_float_bkp_sign_t;


uint32_t us_cnt_H = 0;
navigation_t navigation_style = MENU_NAVIGATION;
edit_val_t edit_val = {0};
saved_to_flash_t config;


/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
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
osThreadId adcTaskHandle;


/* Private function prototypes -----------------------------------------------*/

void dcts_init (void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void RTC_Init(void);
static int RTC_write_cnt(time_t cnt_value);
static void MX_USART1_UART_Init(void);
static void tim2_init(void);
static void MX_TIM3_Init(void);
static void save_to_bkp(u8 bkp_num, uint16_t var);
static void save_float_to_bkp(u8 bkp_num, float var);
static uint16_t read_bkp(u8 bkp_num);
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
#if RELEASE
    MX_IWDG_Init();
#endif // RELEASE
    dcts_init();
    restore_params();
    //MX_USART1_UART_Init();
    //MX_TIM3_Init();
    tim2_init();
    osThreadDef(rtc_task, rtc_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    defaultTaskHandle = osThreadCreate(osThread(rtc_task), NULL);

    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    adcTaskHandle = osThreadCreate(osThread(adc_task), NULL);

    osThreadDef(control_task, control_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

    osThreadDef(display_task, display_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
    displayTaskHandle = osThreadCreate(osThread(display_task), NULL);

    osThreadDef(buttons_task, buttons_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    buttonsTaskHandle = osThreadCreate(osThread(buttons_task), NULL);

    osThreadDef(navigation_task, navigation_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    menuTaskHandle = osThreadCreate(osThread(navigation_task), NULL);

    /* Start scheduler */
    osKernelStart();

    while (1)  {

    }

}

void dcts_init (void) {

    dcts.dcts_id = DCTS_ID_COMBINED;
    strcpy (dcts.dcts_ver, "0.0.1");
    strcpy (dcts.dcts_name, "Thermostat");
    strcpy (dcts.dcts_name_cyr, "Термостат");
    dcts.dcts_address = 0x0B;
    dcts.dcts_rtc.day = 1;
    dcts.dcts_rtc.month = 1;
    dcts.dcts_rtc.year = 2000;
    dcts.dcts_rtc.weekday = 6;
    dcts.dcts_rtc.hour = 12;
    dcts.dcts_rtc.minute = 0;
    dcts.dcts_rtc.second = 0;
    dcts.dcts_pwr = 0.0f;
    dcts.dcts_meas_num = MEAS_NUM;
    dcts.dcts_rele_num = RELE_NUM;
    dcts.dcts_act_num  = ACT_NUM;
    dcts.dcts_alrm_num = ALRM_NUM;

    //meas_channels

    dcts_meas_channel_init(TMPR_FLOOR_GRAD, "Tmpr floor", "Температура пола", "°C", "°C");
    dcts_meas_channel_init(TMPR_FLOOR_RES, "Tmpr floor RES", "Температура пола сопр", "Ohm", "Ом");
    dcts_meas_channel_init(TMPR_FLOOR_ADC, "Tmpr floor ADC", "Температура пола АЦП", "ADC", "АЦП");
    dcts_meas_channel_init(TMPR_FLOOR_VLT, "Tmpr floor Vlt", "Температура пола напр.", "V", "В");
    dcts_meas_channel_init(TMPR_REG_GRAD, "Tmpr sem", "Температура семистора", "°C", "°C");
    dcts_meas_channel_init(TMPR_REG_ADC, "Tmpr sem ADC", "Температура семистора АЦП", "ADC", "АЦП");
    dcts_meas_channel_init(TMPR_REG_VLT, "Tmpr sem Vlt", "Температура семистора напр.", "V", "В");
    dcts_meas_channel_init(VREF_VLT, "Vref V", "Опорное напр. В", "V", "В");
    dcts_meas_channel_init(VBAT_VLT, "RTC battery V", "Батарейка В", "V", "В");
    dcts_meas_channel_init(VBAT_VLT, "Consumption", "Потребление", "W", "Вт");
    dcts_meas_channel_init(VBAT_VLT, "Synchro 0", "Синхроимп 0", "us", "мкс");
    dcts_meas_channel_init(VBAT_VLT, "Synchro 1", "Синхроимп 1", "us", "мкс");
    dcts_meas_channel_init(VBAT_VLT, "AC frequency", "Частота сети", "Hz", "Гц");

    //act_channels

    dcts_act_channel_init(HEATING, "Heating", "Температура пола", "°C", "°C");
    dcts_act_channel_init(SEMISTOR, "Semistor control", "Температура ключа", "°C", "°C");

    //rele_channels

    dcts_rele_channel_init(HEATER, "Heater", "Нагреватель");
    dcts_rele_channel_init(LED, "Indicator", "Индикатор");
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
/*static void MX_ADC1_Init(void)
{
    ADC_InjectionConfTypeDef sConfigInjected = {0};*/

    /* Common config */
    /*hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }*/

    /* Configure Injected Channels */
    /*sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
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

}*/

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
static void RTC_Init(void){
    time_t unix_time = 0;
    struct tm system_time = {0};
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    uint16_t read = read_bkp(0);
    if(read == RTC_KEY){
        dcts.dcts_rtc.state = RTC_STATE_READY;
    }else{   // set default values
        save_to_bkp(0, RTC_KEY);
        dcts.dcts_rtc.state = RTC_STATE_SET;
    }
    if(dcts.dcts_rtc.state == RTC_STATE_SET){

        system_time.tm_hour = dcts.dcts_rtc.hour;
        system_time.tm_min = dcts.dcts_rtc.minute;
        system_time.tm_sec = dcts.dcts_rtc.second;

        system_time.tm_mday = dcts.dcts_rtc.day;
        system_time.tm_mon = dcts.dcts_rtc.month;
        system_time.tm_year = dcts.dcts_rtc.year - 1900;

        unix_time = mktime(&system_time);

        RTC_write_cnt(unix_time);
        dcts.dcts_rtc.state = RTC_STATE_READY;
    }
}
/**
 * @brief RTC_write_cnt
 * @param cnt_value - time in unix format
 * @return  0 - OK,\n
 *          -1 - timeout error,\n
 *          -2 - timeout error
 */
static int RTC_write_cnt(time_t cnt_value){
    int result = 0;
    u32 start = HAL_GetTick();
    u32 timeout = 0;
    PWR->CR |= PWR_CR_DBP;                                          //разрешить доступ к Backup области
    while ((!(RTC->CRL & RTC_CRL_RTOFF))&&(timeout <= start + 500)){//проверить закончены ли изменения регистров RTC
        osDelay(1);
        timeout++;
    }
    if(timeout > start + 500){
        result = -1;
    }
    RTC->CRL |= RTC_CRL_CNF;                                        //Разрешить Запись в регистры RTC
    RTC->CNTH = (u32)cnt_value>>16;                                 //записать новое значение счетного регистра
    RTC->CNTL = (u32)cnt_value;
    RTC->CRL &= ~RTC_CRL_CNF;                                       //Запретить запись в регистры RTC
    start = HAL_GetTick();
    while ((!(RTC->CRL & RTC_CRL_RTOFF))&&(timeout <= start + 500)){//Дождаться окончания записи
        osDelay(1);
        timeout++;
    }
    if(timeout > start + 500){
        result = -2;
    }
    PWR->CR &= ~PWR_CR_DBP;                                         //запретить доступ к Backup области
    return result;
}

/** TIM3 init function
 * check implementation
 */

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

    /* LED pin */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/**
  *
  */
#define RTC_TASK_PERIOD 500
void rtc_task(void const * argument){
    (void)argument;
    time_t unix_time = 0;
    struct tm system_time = {0};
    RTC_Init();
    uint32_t last_wake_time = osKernelSysTick();

    while(1){
        switch (dcts.dcts_rtc.state) {
        case RTC_STATE_READY:   //update dcts_rtc from rtc
            unix_time = (time_t)(RTC->CNTL);
            unix_time |= (time_t)(RTC->CNTH<<16);
            system_time = *localtime(&unix_time);

            taskENTER_CRITICAL();
            dcts.dcts_rtc.hour      = (u8)system_time.tm_hour;
            dcts.dcts_rtc.minute    = (u8)system_time.tm_min;
            dcts.dcts_rtc.second    = (u8)system_time.tm_sec;

            dcts.dcts_rtc.day       = (u8)system_time.tm_mday;
            dcts.dcts_rtc.month     = (u8)system_time.tm_mon;
            dcts.dcts_rtc.year      = (u8)system_time.tm_year + 1900;
            dcts.dcts_rtc.weekday   = (u8)system_time.tm_wday;
            taskEXIT_CRITICAL();
            break;
        case RTC_STATE_SET:     //set new values from dcts_rtc
            system_time.tm_hour = dcts.dcts_rtc.hour;
            system_time.tm_min  = dcts.dcts_rtc.minute;
            system_time.tm_sec  = dcts.dcts_rtc.second;

            system_time.tm_mday = dcts.dcts_rtc.day;
            system_time.tm_mon  = dcts.dcts_rtc.month;
            system_time.tm_year = dcts.dcts_rtc.year - 1900;

            unix_time = mktime(&system_time);

            RTC_write_cnt(unix_time);

            dcts.dcts_rtc.state = RTC_STATE_READY;
            break;
        default:
            break;
        }
        refresh_watchdog();
        osDelayUntil(&last_wake_time, RTC_TASK_PERIOD);
    }
}

/**
 * @brief Init us timer
 * @ingroup MAIN
 */
static void tim2_init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
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
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
}
/**
 * @brief Get value from global us timer
 * @return global us timer value
 * @ingroup MAIN
 */
uint32_t us_tim_get_value(void){
    uint32_t value = us_cnt_H + TIM2->CNT;
    return value;
}
/**
 * @brief Us delayy
 * @param us - delau value
 * @ingroup MAIN
 */
void us_tim_delay(uint32_t us){
    uint32_t current;
    uint8_t with_yield;
    current = TIM2->CNT;
    with_yield = 0;
    if(us > TIME_YIELD_THRESHOLD){
        with_yield =1;
    }
    while ((TIM2->CNT - current)<us){
        if(with_yield){
            osThreadYield();
        }
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

static void save_to_bkp(u8 bkp_num, uint16_t var){
    uint32_t data = var;
    /*if(bkp_num%2 == 1){
        data = data << 8;
    }*/
    HAL_PWR_EnableBkUpAccess();
    switch (bkp_num){
    case 0:
        BKP->DR1 = data;
        break;
    case 1:
        BKP->DR2 = data;
        break;
    case 2:
        BKP->DR3 = data;
        break;
    case 3:
        BKP->DR4 = data;
        break;
    case 4:
        BKP->DR5 = data;
        break;
    case 5:
        BKP->DR6 = data;
        break;
    case 6:
        BKP->DR7 = data;
        break;
    case 7:
        BKP->DR8 = data;
        break;
    case 8:
        BKP->DR9 = data;
        break;
    case 9:
        BKP->DR10 = data;
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
static uint16_t read_bkp(u8 bkp_num){
    uint32_t data = 0;
    switch (bkp_num){
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
    return (u16)(data & 0xFFFF);
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

void refresh_watchdog(void){
#if(RELEASE == 1)
    HAL_IWDG_Refresh(&hiwdg);
#endif//RELEASE
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

