/**
 * @file control.c
 * @author Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 * @defgroup src
 * @ingroup src
 * @version 0.1
 * @brief  TODO!!! write brief in
 */
/*
 * Copyright (c) 2018 Snema Service
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the sofi PLC.
 *
 * Author: Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 */
#ifndef CONTROL_C
#define CONTROL_C 1
#include "main.h"
#include "control.h"
#include "cmsis_os.h"
//#include "usbd_cdc_if.h"
#include "ds18.h"
#include "ssd1306.h"
#include "time_table.h"
#include "stm32f1xx_ll_gpio.h"
#include "dcts.h"
#include "pin_map.h"
#include "buttons.h"
#include "math.h"
extern IWDG_HandleTypeDef hiwdg;
uint8_t PWM_duty = 0;
/* fb pid */
typedef union DataTypes_union{
    u8 bit:1;
    u8 uint8;
    u16 uint16;
    u32 uint32;
    float float32;
    u8 array[4];
} data_types;

typedef struct __attribute__((packed)){
    u8 type;
    data_types data;
} register_type;

typedef struct {
    register_type enable;           // bit 0 - Ручное, 1 - Автоматическое
    register_type reverse_control;  // bit 1- реверсивное управление
    register_type rezet;			// bit 1- сброс накопленных параметров
    register_type require_value;    // float Уставка регулирования
    register_type current_value;    // float Регулируемый параметр
    register_type kp;		 		// float Коэффициент пропорциональности
    register_type ki;		  		// float Коэффициент времени интегрирования
    register_type kd;				// float Коэффициент времени интегрирования
    register_type position;	    	// float - необходимое положение регулятора в процентах
    register_type gist_tube;        // float Зона нечувствительности в единицах измеряемого параметра
} pid_in_t;

typedef struct {
    register_type error_integral;		// float - накопленная ошибка интегратора
    register_type prev_error_integral;  // float - предыдущее значение ошибки регулирования
    register_type prev_control_integral;// float - накопленное воздействия на регулирующий орган
    register_type enable_old;			// bit - для отслеживания первого такта включения
    register_type number_tick;			// uint32 - количество тактов после включения,для интервала работы
} pid_var_t;

typedef struct {
    register_type error;	     	// bit Индикация ошибки входных параметров
    register_type output;	    	// float - необходимое положение регулятора в процентах
    register_type test;				// float
} pid_out_t;

sensor_tt sensor_state = {
    SENSOR_OK,
    10,
    HYSTERESIS,
    DISPERSION,
    0.0f
};
semistor_t semistor_state = {
    FALSE,
    0,
    MAX_REG_TEMP
};
phase_tim_t phase_tim = {
    .timeout = 0,
    .state = PHASE_DISABLE,
    .on_delay = 20,
};


void pid(pid_in_t * inputs,pid_var_t * vars,\
                  pid_out_t * outputs);
static void reg_on_control(void);
static float ntc_tmpr_calc(float adc_val);

#define DEFAULT_OUT 0.0f
#define REQUIRE_VALUE 27.0f

extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc1;
void control_task( const void *parameters){
    (void) parameters;
    /*GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;*/
    static tmpr_proc_t sem_temp = TMPR_HEATING;
    static tmpr_proc_t floor_temp = TMPR_HEATING;
    u32 tick=0;
    /*pid_in_t in;
    pid_var_t var;
    pid_out_t out;
    float val;
    float temp_buf[TEMP_MAX_BUFF_SIZE] = {0};
    var.prev_error_integral.data.float32 = 0.0;
    var.error_integral.data.float32  = 0.0;
    var.number_tick.data.uint32=0.0;
    in.enable.data.bit = 1;
    in.reverse_control.data.bit = 0;
    in.rezet.data.bit = 0;
    in.require_value.data.float32 = REQUIRE_VALUE;
    in.current_value.data.float32 = REQUIRE_VALUE;
    in.kp.data.float32 = 75.0f;
    in.ki.data.float32 = 9.0f;
    in.kd.data.float32 = -100.0f;
    in.position.data.float32 = DEFAULT_OUT;
    in.gist_tube.data.float32 = 0.5f;*/
    uint32_t last_wake_time = osKernelSysTick();
    while(1){

        // dcts_act[SEMISTOR]
        if(dcts_act[SEMISTOR].state.control){
            if(dcts_meas[TMPR_REG_GRAD].valid){
                dcts_act[SEMISTOR].meas_value = dcts_meas[TMPR_REG_GRAD].value;
                switch(sem_temp){
                case TMPR_HEATING:
                    dcts_act[SEMISTOR].state.pin_state = 1; // work permit enable
                    if(dcts_act[SEMISTOR].meas_value > dcts_act[SEMISTOR].set_value + 0.5f*dcts_act[SEMISTOR].hysteresis){
                        dcts_act[SEMISTOR].state.pin_state = 0; // work permit disable
                        sem_temp = TMPR_COOLING;
                    }
                    break;
                case TMPR_COOLING:
                    if(dcts_act[SEMISTOR].meas_value < dcts_act[SEMISTOR].set_value - 0.5f*dcts_act[SEMISTOR].hysteresis){
                        dcts_act[SEMISTOR].state.pin_state = 1; // work permit enable
                        sem_temp = TMPR_HEATING;
                    }
                    break;
                }
            }else{
                dcts_act[SEMISTOR].state.pin_state = 0; // work permit disable
            }
        }

        // dcts_act[HEATING]
        if(dcts_act[HEATING].state.control){
            if(dcts_act[SEMISTOR].state.pin_state == 1){ // work permit enable
                switch(config.params.ctrl_rule){
                case RULE_RELE:
                    break;
                case RULE_PHASE:
                    break;
                }
            }else{
                dcts_act[HEATER].state.pin_state = 0;
            }
            // set dcts_rele[HEATER] if control_by_act enable
            if(dcts_rele[HEATER].state.control_by_act == 1){
                if(dcts_rele[HEATER].state.control != dcts_act[HEATING].state.pin_state){
                    dcts_rele[HEATER].state.control = dcts_act[HEATING].state.pin_state;
                }
            }
            // set dcts_rele[LED] if control_by_act enable
            if(dcts_rele[LED].state.control_by_act == 1){
                if(dcts_rele[LED].state.control != dcts_act[HEATING].state.pin_state){
                    dcts_rele[LED].state.control = dcts_act[HEATING].state.pin_state;
                }
            }
        }


        // dcts_rele[HEATER]
        if(dcts_rele[HEATER].state.control == 1){
            //heater on
            if(dcts_rele[HEATER].state.status == 0){
                /*GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Pin = REG_ON_PIN;
                HAL_GPIO_Init(REG_ON_PORT, &GPIO_InitStruct);

                HAL_GPIO_WritePin(REG_ON_PORT, REG_ON_PIN, GPIO_PIN_RESET);*/
                od_pin_ctrl(REG_ON_PORT, REG_ON_PIN, ON);
                dcts_rele[HEATER].state.status = 1;
            }
        }else{
            //heater off
            if(dcts_rele[HEATER].state.status == 1){
                /*GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Pin = REG_ON_PIN;
                HAL_GPIO_Init (REG_ON_PORT, &GPIO_InitStruct);*/
                od_pin_ctrl(REG_ON_PORT, REG_ON_PIN, OFF);
                dcts_rele[HEATER].state.status = 0;
            }
        }

        // dcts_rele[LED]
        if(dcts_rele[LED].state.control == 1){
            //heater on
            if(dcts_rele[LED].state.status == 0){
                //HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                od_pin_ctrl(LED_PORT, LED_PIN, ON);
                dcts_rele[LED].state.status = 1;
            }
        }else{
            //heater off
            if(dcts_rele[LED].state.status == 1){
                //HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
                od_pin_ctrl(LED_PORT, LED_PIN, OFF);
                dcts_rele[LED].state.status = 0;
            }
        }

        /*u8 sensor_data_valid;
        sensor_data_valid = 0;

        u32 value[3];
        value[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        value[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        value[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

        // ADC0
        val = ((float)value[0]/value[2])*1.2f;
        dcts_write_meas_value (3, val);
        if(val < SENSOR_MIN_VOLTAGE){
            sensor_state.error = SENSOR_SHORT;
        }else if(val > SENSOR_MAX_VOLTAGE){
            sensor_state.error = SENSOR_BREAK;
        }else{
            sensor_state.error = SENSOR_OK;
        }

        // ADC1
        val = ((float)value[1]/value[2])*1.2f;
        dcts_write_meas_value (4, val);

        // Floor T
        val = ntc_tmpr_calc(dcts_meas[3].value);
        if(tick < sensor_state.buff_size){
            temp_buf[tick] = val;
            tick++;
            val = 0.0f;
            for(u8 i = 0; i < sensor_state.buff_size; i++){
                val += temp_buf[i];
            }
            val = val/sensor_state.buff_size;
            dcts_write_act_meas_value (0, val);
        }else if((val >= dcts_act[0].meas_value - sensor_state.dispersion) && (val <= dcts_act[0].meas_value + sensor_state.dispersion)){
            temp_buf[tick%sensor_state.buff_size] = val;
            tick++;
            val = 0.0f;
            for(u8 i = 0; i < sensor_state.buff_size; i++){
                val += temp_buf[i];
            }
            val = val/sensor_state.buff_size;
            dcts_write_act_meas_value (0, val);
        }

        // Reg T
        val = dcts_meas[4].value/0.01f;
        dcts_write_meas_value (1, val);*/

        /*in.require_value.data.float32 = act[0].set_value;
        in.current_value.data.float32 = act[0].meas_value;
        pid(&in,&var,&out);*/
        /*if(tick > sensor_state.buff_size){
            reg_on_control();
        }*/

        refresh_watchdog();
        osDelayUntil(&last_wake_time,CONTROL_TASK_PERIOD);
    }
}

#define IntegralAccum -25
void pid(pid_in_t * FBInputs,pid_var_t * FBVars,\
                  pid_out_t * FBOutputs) {
    pid_in_t *IN =  FBInputs;
    pid_var_t *VAR =  FBVars;
    pid_out_t *OUT = FBOutputs;
    float error_diff; //"невязка" и её изменение
    float du_kp;
    float du_ki;
    float du_kd;
    float du_out;
    if (IN->rezet.data.bit){
        VAR->prev_error_integral.data.float32 = 0.0;
        VAR->error_integral.data.float32  = 0.0;
        VAR->number_tick.data.uint32=0.0;
        VAR->prev_control_integral.data.float32 = IN->position.data.float32;
        IN->rezet.data.bit = 0;
    }
    if (IN->enable.data.bit){
        error_diff = IN->require_value.data.float32 - IN->current_value.data.float32;    //гистерезис не реагирования
        if ((error_diff > -IN->gist_tube.data.float32)&&(error_diff < IN->gist_tube.data.float32)) {
//            error_diff = 0.0;
        }
        if (VAR->number_tick.data.uint32 != 0){
            du_kp = IN->kp.data.float32 * ((error_diff - VAR->error_integral.data.float32));
            du_ki = IN->ki.data.float32 * error_diff;
            if (VAR->number_tick.data.uint32 >= 2){  //добавим дифф составляющию только после 3 такта
                du_kd = IN->kd.data.float32 * ((error_diff - 2*VAR->error_integral.data.float32 + VAR->prev_error_integral.data.float32));
            }else{
                du_kd = 0.0;
            }
        }else{  //при первом расчете расчитываем только Коэфф пропорциональности
            du_kp = IN->kp.data.float32 * (error_diff); //при включении PID не делим на dT
            if (IN->position.data.float32 > 100.0f){VAR->prev_control_integral.data.float32  = 100.0f;}
            if (IN->position.data.float32 < -100.0f){VAR->prev_control_integral.data.float32  = -100.0f;}
            else{ VAR->prev_control_integral.data.float32  = IN->position.data.float32; }
            du_ki = 0.0f;
            du_kd = 0.0f;
        }
        du_out = du_kp + du_ki + du_kd;
        if (du_out > 100.0f) du_out = 100.0f;
        else if (du_out < -100.0f) du_out = -100.0f;
        if (IN->reverse_control.data.bit) du_out = -du_out;
        VAR->prev_control_integral.data.float32 += du_out ;
        VAR->prev_error_integral.data.float32 = VAR->error_integral.data.float32;
        VAR->error_integral.data.float32 = error_diff;
        if (VAR->prev_control_integral.data.float32 > (100.0f + IntegralAccum)) VAR->prev_control_integral.data.float32 = 100.0f + IntegralAccum;
        else if (VAR->prev_control_integral.data.float32 < (-100.0f - IntegralAccum)) VAR->prev_control_integral.data.float32 = -100.0f - IntegralAccum;
        OUT->test.data.float32 = du_out;
        VAR->number_tick.data.uint32++;
    }else{
        VAR->prev_error_integral.data.float32 =  0.0f;
        VAR->error_integral.data.float32  =  0.0f;
        VAR->number_tick.data.uint32= 0.0f;
        if (IN->position.data.float32 > 100.0f){VAR->prev_control_integral.data.float32  = 100.0f;}
        if (IN->position.data.float32 < -100.0f){VAR->prev_control_integral.data.float32  = -100.0f;}
        else{ VAR->prev_control_integral.data.float32  = IN->position.data.float32; }
    }
    //выдаем значение на выход
    OUT->output.data.float32 = VAR->prev_control_integral.data.float32 ;
}
static void reg_on_control(void){
    static u8 last_overheat = FALSE;
    if (dcts_meas[1].value > semistor_state.max_tmpr){  // overheating
        dcts_act[0].state.short_cir = TRUE;
        semistor_state.overheat = TRUE;
        if(last_overheat == FALSE){
            semistor_state.overheat_cnt++;
        }
    }else{
        dcts_act[0].state.short_cir = FALSE;
        semistor_state.overheat = FALSE;
        if (dcts_act[0].meas_value >= dcts_act[0].set_value){
            PWM_duty = 0;
        }else if ((dcts_act[0].meas_value >= dcts_act[0].set_value - sensor_state.hysteresis) && (dcts_act[0].meas_value < dcts_act[0].set_value)){
            PWM_duty = 30;
        }else{
            PWM_duty = 100;
        }
    }
    last_overheat = semistor_state.overheat;
}

static float ntc_tmpr_calc(float volt){
    float result = 0.0f;
    /* T = A*x^3 + B*x^2 + C*x + D */
#define A   -4.6277f
#define B   25.09f
#define C   -70.672f
#define D   83.718f
    result = A*volt*volt*volt + B*volt*volt + C*volt + D;
    return result;
}

void od_pin_ctrl(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, u8 ctrl){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    switch(ctrl){
    case 0:
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
        break;
    case 1:
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOx,GPIO_Pin, GPIO_PIN_RESET);
        break;
    }
}

const float phase_pwr_coef[101] = {
    1.00000f,
    0.93623f,
    0.90967f,
    0.88918f,
    0.87181f,
    0.85643f,
    0.84246f,
    0.82954f,
    0.81745f,
    0.80603f,
    0.79517f,
    0.78478f,
    0.77480f,
    0.76517f,
    0.75586f,
    0.74682f,
    0.73802f,
    0.72944f,
    0.72107f,
    0.71287f,
    0.70483f,
    0.69695f,
    0.68920f,
    0.68158f,
    0.67407f,
    0.66667f,
    0.65936f,
    0.65215f,
    0.64502f,
    0.63797f,
    0.63099f,
    0.62408f,
    0.61722f,
    0.61043f,
    0.60368f,
    0.59699f,
    0.59033f,
    0.58372f,
    0.57715f,
    0.57061f,
    0.56409f,
    0.55761f,
    0.55115f,
    0.54471f,
    0.53829f,
    0.53188f,
    0.52549f,
    0.51911f,
    0.51274f,
    0.50637f,
    0.50000f,
    0.49363f,
    0.48726f,
    0.48089f,
    0.47451f,
    0.46812f,
    0.46171f,
    0.45529f,
    0.44885f,
    0.44239f,
    0.43591f,
    0.42939f,
    0.42285f,
    0.41628f,
    0.40967f,
    0.40301f,
    0.39632f,
    0.38957f,
    0.38278f,
    0.37592f,
    0.36901f,
    0.36203f,
    0.35498f,
    0.34785f,
    0.34064f,
    0.33333f,
    0.32593f,
    0.31842f,
    0.31080f,
    0.30305f,
    0.29517f,
    0.28713f,
    0.27893f,
    0.27056f,
    0.26198f,
    0.25318f,
    0.24414f,
    0.23483f,
    0.22520f,
    0.21522f,
    0.20483f,
    0.19397f,
    0.18255f,
    0.17046f,
    0.15754f,
    0.14357f,
    0.12819f,
    0.11082f,
    0.09033f,
    0.06377f,
    0.00000f
};

u16 calc_phase_delay(float act_time, float zero_time, u16 percentage){
    u16 result = 0;
    /*float temp = (act_time + zero_time)*(100.0f - percentage)/100.0f;
    if(temp > act_time + zero_time/4.0f){
        temp = act_time + zero_time/4.0f;
    }
    temp -= zero_time/4.0f;
    result = (u16)temp;*/

    float temp = phase_pwr_coef[percentage]*(act_time + zero_time);
    temp -= zero_time/2.0f;
    if(temp > act_time + zero_time/8.0f){
        temp = act_time + zero_time/8.0f;
    }
    result = (u16)temp;

    return result;
}


#endif //CONTROL_C
