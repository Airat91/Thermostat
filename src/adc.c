#include "adc.h"
#include "pin_map.h"
#include "dcts.h"
#include "dcts_config.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include <math.h>
#include "main.h"
#include "control.h"
/**
  * @defgroup ADC
  * @brief work with ADC channels
  */

ADC_HandleTypeDef hadc1;

#define ADC_BUF_SIZE 20
#define ADC_PERIOD 100
#define ADC_MAX 4095
#define INPUT_RES 10000.0f

#define PWR_K   (float)10.1
#define VREF_INT (float)1.2

/*========== FUNCTIONS ==========*/

static float lm35_get_val(float vlt);
static float ntc10k_get_res(float vlt);
static float ntc10k_get_tmpr(float res);

/**
 * @brief Init and start ADC
 * @return  0 - ADC init successfull,\n
 *          -1 - ADC config error,\n
 *          -2 - PWR channel config error,\n
 *          -3 - WTR_LEV channel config error,\n
 *          -4 - WTR_TMP channel config error,\n
 *          -5 - TMP channel config error,\n
 *          -6 - ADC start error,
 * @ingroup ADC
 */
int adc_init (void){
    int result = 0;
    __HAL_RCC_ADC1_CLK_ENABLE();
    adc_gpio_init();
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    //Common config
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        result = -1;
    }

    if(config.params.sensor_type == SENSOR_NTC_10K){
        sConfigInjected.InjectedNbrOfConversion = 3;
    }else{
        sConfigInjected.InjectedNbrOfConversion = 2;
    }
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = ENABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    //Configure TMPR_REG_GRAD Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -3;
    }
    //Configure VREF_VLT Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -4;
    }
    /*Configure VBAT_VLT Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VBAT;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -5;
    }*/
    //Configure TMPR_FLOOR_GRAD Channel
    if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
        sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
        sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
        if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
        {
            result = -2;
        }
    }
    //Start ADC
    if (HAL_ADC_Start(&hadc1) != HAL_OK){
        result = -6;
    }

    return result;
}
/**
 * @brief Deinit ADC
 * @ingroup ADC
 */
void adc_deinit (void){
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_DeInit(&hadc1);
    __HAL_RCC_ADC1_CLK_DISABLE();
    adc_gpio_deinit();
}
/**
 * @brief Init ADC gpio
 * @ingroup ADC
 */
void adc_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = REG_TEMP_PIN;
    HAL_GPIO_Init(REG_TEMP_PORT, &GPIO_InitStruct);
    if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
        GPIO_InitStruct.Pin = FLOOR_TEMP_PIN;
        HAL_GPIO_Init(FLOOR_TEMP_PORT, &GPIO_InitStruct);
    }
}
/**
 * @brief Deinit ADC gpio
 * @ingroup ADC
 */
void adc_gpio_deinit (void){
    HAL_GPIO_DeInit(REG_TEMP_PORT,REG_TEMP_PIN);
    if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
        HAL_GPIO_DeInit(FLOOR_TEMP_PORT,FLOOR_TEMP_PIN);
    }
}
/**
 * @brief Measure ADC channels and write values to DCTS
 * @param argument - none
 * @ingroup ADC
 */
void adc_task(void const * argument){
    (void)argument;
    uint16_t tmpr_reg = 0;//[ADC_BUF_SIZE];
    uint16_t tmpr_floor = 0;//[ADC_BUF_SIZE];
    //uint16_t vref[ADC_BUF_SIZE];
    uint8_t tick = 1;
    float v_3_3 = 0.0f;
    adc_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        //uint32_t tmpr_reg_sum = 0;
        //uint32_t tmpr_floor_sum = 0;
        //uint32_t vref_sum = 0;


        tmpr_reg = (uint16_t)hadc1.Instance->JDR1;
        //vref[tick] = (uint16_t)hadc1.Instance->JDR2;
        if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
            tmpr_floor = (uint16_t)hadc1.Instance->JDR3;
        }
        v_3_3 = VREF_INT/(uint16_t)hadc1.Instance->JDR2*ADC_MAX;

        /*for(uint8_t i = 0; i < ADC_BUF_SIZE; i++){
            tmpr_reg_sum += tmpr_reg[i];
            //vref_sum += vref[i];
            if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
                tmpr_floor_sum += tmpr_floor[i];
            }else{
                tmpr_floor_sum = 0;
            }
        }*/

        taskENTER_CRITICAL();
        dcts_meas[TMPR_REG_ADC].value = (dcts_meas[TMPR_REG_ADC].value * (tick - 1) + (float)tmpr_reg)/tick;
        dcts_meas[TMPR_REG_VLT].value = dcts_meas[TMPR_REG_ADC].value*v_3_3/ADC_MAX;
        dcts_meas[TMPR_REG_GRAD].value = lm35_get_val(dcts_meas[TMPR_REG_VLT].value);

        dcts_meas[VREF_VLT].value = v_3_3;

        if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
            dcts_meas[TMPR_FLOOR_ADC].value = (dcts_meas[TMPR_FLOOR_ADC].value * (tick - 1) + (float)tmpr_floor)/tick;
            dcts_meas[TMPR_FLOOR_VLT].value = dcts_meas[TMPR_FLOOR_ADC].value*v_3_3/ADC_MAX;
            switch (config.params.sensor_type) {
            case SENSOR_NTC_10K:
                dcts_meas[TMPR_FLOOR_RES].value = ntc10k_get_res(dcts_meas[TMPR_FLOOR_VLT].value);
                dcts_meas[TMPR_FLOOR_GRAD].value = ntc10k_get_tmpr(dcts_meas[TMPR_FLOOR_RES].value);
                break;
            case SENSOR_LM35:
                dcts_meas[TMPR_FLOOR_GRAD].value = lm35_get_val(dcts_meas[TMPR_FLOOR_VLT].value);
                break;
            }
        }

        dcts_meas[TMPR_REG_ADC].valid = TRUE;
        dcts_meas[TMPR_REG_VLT].valid = TRUE;
        if((0.1f < dcts_meas[TMPR_REG_VLT].value)&&(dcts_meas[TMPR_REG_VLT].value < 3.2f)){
            dcts_meas[TMPR_REG_GRAD].valid = TRUE;
        }else{
            dcts_meas[TMPR_REG_GRAD].valid = FALSE;
        }

        dcts_meas[VREF_VLT].valid = TRUE;

        if((config.params.sensor_type == SENSOR_NTC_10K)||(config.params.sensor_type == SENSOR_LM35)){
            dcts_meas[TMPR_FLOOR_ADC].valid = TRUE;
            dcts_meas[TMPR_FLOOR_VLT].valid = TRUE;
            if((0.1f < dcts_meas[TMPR_FLOOR_VLT].value)&&(dcts_meas[TMPR_FLOOR_VLT].value < 3.2f)){
                dcts_meas[TMPR_FLOOR_GRAD].valid = TRUE;
            }else{
                dcts_meas[TMPR_FLOOR_GRAD].valid = FALSE;
            }
        }
        taskEXIT_CRITICAL();

        //tick++;
        if(tick < ADC_BUF_SIZE){
            tick++;
        }
        osDelayUntil(&last_wake_time, ADC_PERIOD);
    }
}

static float lm35_get_val(float vlt){
    float tmpr = vlt*100.0f;

    return tmpr;
}
static float ntc10k_get_res(float vlt){
    float res = vlt/(dcts_meas[VREF_VLT].value - vlt)*INPUT_RES;
    return res;
}
static float ntc10k_get_tmpr(float res){
    float tmpr = 0.0f;
    return tmpr;
}

