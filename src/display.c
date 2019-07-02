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
#ifndef DISPLAY_C
#define DISPLAY_C 1
#include "main.h"
#include "display.h"
#include "cmsis_os.h"
#include "dcts.h"
//#include "usbd_cdc_if.h"
#include "ssd1306.h"
#include "stm32f1xx_ll_gpio.h"
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
static u8 display_time(void);
void display_task( const void *parameters){
    u32 tick=0;
    taskENTER_CRITICAL();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_Init();
    HAL_IWDG_Refresh(&hiwdg);
    //SSD1306_DrawCircle(10, 33, 7, SSD1306_COLOR_WHITE,0.5);
    SSD1306_UpdateScreen();
    //u8 measerment_number = sizeof(meas)/sizeof(meas_t);
    taskEXIT_CRITICAL();
    while(1){
        char buff[32];
        sprintf(buff,"temperature off");
        SSD1306_GotoXY(0, 50); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
        SSD1306_UpdateScreen();
        if(SSD1306.error_num){
            SSD1306.Initialized = 0;
        }
        if((tick % 5) == 0){
            if(!SSD1306.Initialized ){
                SSD1306_Init();
            }
        }
        for (u8 i = 0; i < MEAS_NUM; i++){

            if (!memcmp(meas[i].name,"Floor",sizeof("Floor"))){
                sprintf(buff,"Floor %3.3f ", meas[i].value);
                strcat(buff, meas[i].unit);
                SSD1306_GotoXY(0, 10); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
                SSD1306_UpdateScreen();

            }else if (!memcmp(meas[i].name,"Reg",sizeof("Reg"))){
                sprintf(buff,"Reg   %3.3f ",meas[i].value);
                strcat(buff, meas[i].unit);
                SSD1306_GotoXY(0, 20); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
                SSD1306_UpdateScreen();

            }else if (!memcmp(meas[i].name,"ADC0",sizeof("ADC0"))){
                sprintf(buff,"ADC0  %3.3f ",meas[i].value);
                strcat(buff, meas[i].unit);
                SSD1306_GotoXY(0, 30); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
                SSD1306_UpdateScreen();

            }else if(!memcmp(meas[i].name,"ADC1",sizeof("ADC0"))){
                sprintf(buff,"ADC1  %3.3f ",meas[i].value);
                strcat(buff, meas[i].unit);
                SSD1306_GotoXY(0, 40); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
                SSD1306_UpdateScreen();

            }
        }

        display_time();
        osDelay(500);
        HAL_IWDG_Refresh(&hiwdg);
        //osDelayUntil(&ds18_time,_DS18B20_UPDATE_INTERVAL_MS);
    }
}
u8 display_time(void){
    RTC_TimeTypeDef time;
    LL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
    char buff[32];
    sprintf(buff,"time %2u:%2u:%2u",time.Hours,time.Minutes,time.Seconds);
    SSD1306_GotoXY(0, 0); //Устанавливаем курсор в позицию 0;44. Сначала по горизонтали, потом вертикали.
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE); //пишем надпись в выставленной позиции шрифтом "Font_7x10" белым цветом.
    SSD1306_UpdateScreen();

    return 0x00;
}

#endif //DISPLAY_C
