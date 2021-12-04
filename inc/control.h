/**
 * @file control.h
 * @author Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 * @defgroup inc
 * @ingroup inc
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
#ifndef CONTROL_H
#define CONTROL_H 1
 
/*add includes below */
#include "type_def.h"
#include "stm32f1xx_hal.h"
/*add includes before */
#ifdef __cplusplus 
   extern "C" {
#endif
/*add functions and variable declarations below */
extern TIM_HandleTypeDef htim3;
extern uint8_t PWM_duty;
void control_task( const void *parameters);
#define MAX_PWM_VALUE 32768
#define CONTROL_TASK_PERIOD 100
#define MAX_REG_TEMP 100.0f
#define MAX_SET_TEMP 40.0f
#define MIN_SET_TEMP 5.0f
#define HYSTERESIS 1.0f
#define TEMP_MAX_BUFF_SIZE  32
#define DISPERSION  1.0f
#define SENSOR_MIN_VOLTAGE  0.01f
#define SENSOR_MAX_VOLTAGE  3.0f
#define MAX_CORRECTION  10.0f
#define MIN_CORRECTION  -10.0f

typedef enum{
    SENSOR_OK = 0,
    SENSOR_BREAK,
    SENSOR_SHORT,
}sensor_err;
typedef struct{
    u8 error;
    u8 buff_size;
    float hysteresis;
    float dispersion;
    float correction;
}sensor_tt;
typedef enum {
    SENSOR_NTC_10K,
    SENSOR_DS18B20,
    SENSOR_LM35,
}sensor_t;
typedef enum {
    CTRL_RULE_PWM,
    CTRL_RULE_TMPR,
}ctrl_rule_t;
typedef enum {
    TMPR_HEATING,
    TMPR_COOLING,
}tmpr_proc_t;
extern const char sensor_descr[][20];
extern const char ctrl_rule_descr[][20];
extern sensor_tt sensor_state;
typedef struct{
    u8 overheat;
    u16 overheat_cnt;
    float max_tmpr;
}semistor_t;
extern semistor_t semistor_state;
/*add functions and variable declarations before */
#ifdef __cplusplus
}
#endif
#endif //CONTROL_H
