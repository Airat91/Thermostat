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

sensor_t sensor_state = {
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
    u32 tick=0;
    pid_in_t in;
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
    in.gist_tube.data.float32 = 0.5f;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        u8 sensor_data_valid;
        sensor_data_valid = 0;

        u32 value[3];
        value[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        value[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        value[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

        /* ADC0 */
        val = ((float)value[0]/value[2])*1.2f;
        dcts_write_meas_value (3, val);
        if(val < SENSOR_MIN_VOLTAGE){
            sensor_state.error = SENSOR_SHORT;
        }else if(val > SENSOR_MAX_VOLTAGE){
            sensor_state.error = SENSOR_BREAK;
        }else{
            sensor_state.error = SENSOR_OK;
        }

        /* ADC1 */
        val = ((float)value[1]/value[2])*1.2f;
        dcts_write_meas_value (4, val);

        /* Floor T */
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

        /* Reg T */
        val = dcts_meas[4].value/0.01f;
        dcts_write_meas_value (1, val);

        /*in.require_value.data.float32 = act[0].set_value;
        in.current_value.data.float32 = act[0].meas_value;
        pid(&in,&var,&out);*/
        if(tick > sensor_state.buff_size){
            reg_on_control();
        }

        HAL_IWDG_Refresh(&hiwdg);
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


#endif //CONTROL_C
