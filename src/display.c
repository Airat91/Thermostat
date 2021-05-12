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
#include "task.h"
#include "FreeRTOS.h"
#include "dcts.h"
//#include "usbd_cdc_if.h"
#include "ssd1306.h"
#include "buttons.h"
#include "stm32f1xx_ll_gpio.h"
#include "control.h"

extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern osThreadId defaultTaskHandle;
extern osThreadId displayTaskHandle;
extern osThreadId menuTaskHandle;
extern osThreadId buttonsTaskHandle;
static u8 display_time(u8 y);
static void clock_set(void);
static void max_reg_temp_set(void);
enum skin_t {
    SKIN_FULL = 0,
    SKIN_1,
    SKIN_2,
    SKIN_EMPTY,
    SKIN_END_OF_LIST,
};
enum menu_page_t {
    PAGE_CLOCK,
    PAGE_HYSTERESIS,
    PAGE_PROGRAMM,
    PAGE_STATISTIC,
    PAGE_MAX_TEMP_REG,
    PAGE_END_OF_LIST,
};
#define MENU_LEVEL_NUM 2
typedef struct {
    u8 level;
    u8 page[MENU_LEVEL_NUM];
}navigation_t;
static navigation_t menu;
static const u8 menu_max_page[] = {4,1};
void display_task( const void *parameters){
    (void) parameters;
    u8 skin = SKIN_FULL;
    u32 tick=0;
    uint32_t last_wake_time = osKernelSysTick();
    taskENTER_CRITICAL();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_Init();
    HAL_IWDG_Refresh(&hiwdg);
    SSD1306_UpdateScreen();
    taskEXIT_CRITICAL();
    while(1){
        /* Reinit SSD1306 if error */
        char buff[32];
        if(SSD1306.error_num){
            SSD1306.Initialized = 0;
        }
        if((tick % 5) == 0){
            if(!SSD1306.Initialized ){
                SSD1306_Init();
            }
        }

        /* Buttons read */
        if (pressed_time.up){
            if(dcts_act[0].set_value < MAX_SET_TEMP){
                dcts_act[0].set_value += 1.0f;
                HAL_PWR_EnableBkUpAccess();
                BKP->DR2 = (uint32_t)dcts_act[0].set_value;
                HAL_PWR_DisableBkUpAccess();
            }
        }
        if (pressed_time.down){
            if(dcts_act[0].set_value > MIN_SET_TEMP){
                dcts_act[0].set_value -= 1.0f;
                HAL_PWR_EnableBkUpAccess();
                BKP->DR2 = (uint32_t)dcts_act[0].set_value;
                HAL_PWR_DisableBkUpAccess();
            }
        }
        if (pressed_time.left){
            if(skin == 0){
                skin = SKIN_END_OF_LIST - 1;
            }else{
                skin--;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.right){
            if(skin == SKIN_END_OF_LIST - 1){
                skin = SKIN_FULL;
            }else{
                skin++;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.ok){
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            vTaskSuspend(buttonsTaskHandle);
            pressed_time.ok = 0;
            vTaskResume(menuTaskHandle);
            vTaskSuspend(NULL);
        }
        if (pressed_time.up && pressed_time.down){
            if(dcts_act[0].state.control == TRUE){
                dcts_act[0].state.control = FALSE;
            }else if(dcts_act[0].state.control == FALSE){
                dcts_act[0].state.control = TRUE;
            }

            HAL_PWR_EnableBkUpAccess();
            BKP->DR3 = (uint32_t)dcts_act[0].state.control;
            HAL_PWR_DisableBkUpAccess();
        }

        /* Print main screen */
        if(skin == SKIN_FULL){  // Full information

            if(sensor_state.error == SENSOR_OK){
                sprintf(buff,"%2.1f", (double)dcts_act[0].meas_value);
                SSD1306_GotoXY(0, 14);
                SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_BREAK){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÎÁÐÛÂ");
                SSD1306_GotoXY(15, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_SHORT){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÇÀÌÛÊÀÍÈÅ");
                SSD1306_GotoXY(0, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }

            if(dcts_act[0].state.control == TRUE){
                sprintf(buff,"Óñò %2.0f%s", (double)dcts_act[0].set_value, dcts_act[0].unit);
            }else{
                sprintf(buff,"Âûêëþ÷åí");
            }
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            sprintf(buff,"Ðåã%3.0f%s", (double)dcts_meas[1].value, dcts_meas[1].unit);
            SSD1306_GotoXY(70, 29);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            display_time(0);

        }else if(skin == SKIN_1){    // Only current and required temperature

            if(sensor_state.error == SENSOR_OK){
                sprintf(buff,"%2.1f", (double)dcts_act[0].meas_value);
                SSD1306_GotoXY(0, 14);
                SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_BREAK){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÎÁÐÛÂ");
                SSD1306_GotoXY(15, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_SHORT){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÇÀÌÛÊÀÍÈÅ");
                SSD1306_GotoXY(0, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }

            if(dcts_act[0].state.control == TRUE){
                sprintf(buff,"Óñò %2.0f%s", (double)dcts_act[0].set_value, dcts_act[0].unit);
            }else {
                sprintf(buff,"Âûêëþ÷åí");
            }
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

        }else if(skin == SKIN_2){    // Current and required temperature and clock

            if(sensor_state.error == SENSOR_OK){
                sprintf(buff,"%2.1f", (double)dcts_act[0].meas_value);
                SSD1306_GotoXY(0, 14);
                SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_BREAK){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÎÁÐÛÂ");
                SSD1306_GotoXY(15, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }else if(sensor_state.error == SENSOR_SHORT){
                SSD1306_DrawFilledRectangle(0,14,64,26,SSD1306_COLOR_BLACK);    // clear area
                sprintf(buff,"ÇÀÌÛÊÀÍÈÅ");
                SSD1306_GotoXY(0, 15);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
                sprintf(buff,"ÄÀÒ×ÈÊÀ");
                SSD1306_GotoXY(7, 29);
                SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);
            }

            if(dcts_act[0].state.control == TRUE){
                sprintf(buff,"Óñò %2.0f%s", (double)dcts_act[0].set_value, dcts_act[0].unit);
            }else {
                sprintf(buff,"Âûêëþ÷åí");
            }
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            display_time(0);
        }else if(skin == SKIN_EMPTY){   // Empty screen

        }

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
        if(eTaskGetState(buttonsTaskHandle) == eSuspended){
            vTaskDelay(DISPLAY_TASK_PERIOD);
            vTaskResume(buttonsTaskHandle);
        }
    }
}
u8 display_time(u8 y){
    char buff[20] = {0};
    char weekday[3] = {0};
    switch (dcts.dcts_rtc.weekday) {
    case 1:
        strcpy(weekday, "Ïí");
        break;
    case 2:
        strcpy(weekday, "Âò");
        break;
    case 3:
        strcpy(weekday, "Ñð");
        break;
    case 4:
        strcpy(weekday, "×ò");
        break;
    case 5:
        strcpy(weekday, "Ïò");
        break;
    case 6:
        strcpy(weekday, "Ñá");
        break;
    case 7:
        strcpy(weekday, "Âñ");
        break;
    }
    sprintf(buff,"%2d.%2d.%4d ", dcts.dcts_rtc.day, dcts.dcts_rtc.month, dcts.dcts_rtc.year);
    if(dcts.dcts_rtc.day < 10){
        buff[0] = '0';
    }
    if(dcts.dcts_rtc.month < 10){
        buff[3] = '0';
    }
    SSD1306_GotoXY(0, y);
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(74, y);
    SSD1306_Puts(weekday, &Font_7x10, SSD1306_COLOR_WHITE);

    sprintf(buff,"%2d:%2d", dcts.dcts_rtc.hour, dcts.dcts_rtc.minute);
    if(dcts.dcts_rtc.hour < 10){
        buff[0] = '0';
    }
    if(dcts.dcts_rtc.minute < 10){
        buff[3] = '0';
    }
    SSD1306_GotoXY(92, y);
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

    return 0x00;
}

void menu_task( const void *parameters){
    (void) parameters;
    vTaskSuspend(NULL); // Suspend menu_task after create
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        /*if(eTaskGetState(displayTaskHandle) != eSuspended){
            vTaskSuspend(displayTaskHandle);
        }*/

        /* Read buttons */
        if (pressed_time.left){
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            vTaskSuspend(buttonsTaskHandle);
            pressed_time.left = 0;
            vTaskResume(displayTaskHandle);
            vTaskSuspend(NULL);
        }
        if (pressed_time.right || pressed_time.ok){
            if(menu.level < MENU_LEVEL_NUM){
                menu.level++;
                SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
                SSD1306_UpdateScreen();
            }
        }
        if (pressed_time.down){
            if(menu.page[menu.level] == menu_max_page[menu.level]){
                menu.page[menu.level] = 0;
            }else{
                menu.page[menu.level]++;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.up){
            if(menu.page[menu.level] == 0){
                menu.page[menu.level] = menu_max_page[menu.level] - 1;
            }else{
                menu.page[menu.level]--;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }

        /* Print menu */
        if(menu.level == 0){
            SSD1306_GotoXY(50, 0);
            SSD1306_Puts("ÌÅÍÞ", &Font_7x10, SSD1306_COLOR_WHITE);
            if(menu.page[menu.level] <= PAGE_STATISTIC){
                SSD1306_GotoXY(8, 16);
                SSD1306_Puts("Äàòà è âðåìÿ", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(8, 27);
                SSD1306_Puts("Ãèñòåðåçèñ", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(8, 38);
                SSD1306_Puts("Ðåæèì ðàáîòû", &Font_7x10, SSD1306_COLOR_WHITE);
                SSD1306_GotoXY(8, 49);
                SSD1306_Puts("Ñòàòèñòèêà", &Font_7x10, SSD1306_COLOR_WHITE);
            }else if(menu.page[menu.level] > PAGE_STATISTIC && menu.page[menu.level] <= PAGE_END_OF_LIST){
                SSD1306_GotoXY(8, 16);
                SSD1306_Puts("Ìàêñ. Ò ðåã-ðà", &Font_7x10, SSD1306_COLOR_WHITE);
            }

            /* Print cursor */
            SSD1306_GotoXY(0, 16 + 11 * (menu.page[menu.level] % 4));
            SSD1306_Puts(">", &Font_7x10, SSD1306_COLOR_WHITE);

        }else if(menu.level == 1){
            if(menu.page[menu.level - 1] == PAGE_CLOCK){
                vTaskSuspend(buttonsTaskHandle);
                pressed_time.ok = 0;
                pressed_time.right = 0;
                clock_set();
            }else if(menu.page[menu.level - 1] == PAGE_HYSTERESIS){
                // hysteresis_set();
                SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
                SSD1306_UpdateScreen();
                vTaskResume(displayTaskHandle);
                vTaskSuspend(NULL);
            }else if(menu.page[menu.level - 1] == PAGE_PROGRAMM){
                // programm_set();
                SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
                SSD1306_UpdateScreen();
                vTaskResume(displayTaskHandle);
                vTaskSuspend(NULL);
            }else if(menu.page[menu.level - 1] == PAGE_STATISTIC){
                // statistic_info();
                SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
                SSD1306_UpdateScreen();
                vTaskResume(displayTaskHandle);
                vTaskSuspend(NULL);
            }else if(menu.page[menu.level - 1] == PAGE_MAX_TEMP_REG){
                vTaskSuspend(buttonsTaskHandle);
                pressed_time.ok = 0;
                pressed_time.right = 0;
                max_reg_temp_set();
            }
        }

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
        if(eTaskGetState(buttonsTaskHandle) == eSuspended){
            vTaskDelay(DISPLAY_TASK_PERIOD);
            vTaskResume(buttonsTaskHandle);
        }
    }

}

#define RTC_MAX_DAY 31
#define RTC_MAX_MONTH 12
#define RTC_MAX_YEAR 3000
#define RTC_MIN_YEAR 2000
#define RTC_MAX_WEEKDAY 7
#define RTC_MAX_HOUR 24
#define RTC_MAX_MINUTE 59

static void clock_set(void){
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t last_wake_time = osKernelSysTick();
    vTaskSuspend(defaultTaskHandle);
    SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
    u8 position = 0;
    u8 x = 0;
    while(1){
        /* Read buttons */
        if (pressed_time.left){
            if(position == 0){
                position = 12;
            }else{
                position--;
            }
            SSD1306_DrawFilledRectangle(0,10,128,54,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.right){
            if(position == 13){
                position = 0;
            }else{
                position++;
            }
            SSD1306_DrawFilledRectangle(0,10,128,54,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.down){
            switch (position) {
            case 0:
                if(dcts.dcts_rtc.day < 10){
                    dcts.dcts_rtc.day = 0;
                }else{
                    dcts.dcts_rtc.day -= 10;
                }
                break;
            case 1:
                if(dcts.dcts_rtc.day < 1){
                    dcts.dcts_rtc.day = 0;
                }else{
                    dcts.dcts_rtc.day -= 1;
                }
                break;
            case 2:
                if(dcts.dcts_rtc.month < 10){
                    dcts.dcts_rtc.month = 0;
                }else{
                    dcts.dcts_rtc.month -= 10;
                }
                break;
            case 3:
                if(dcts.dcts_rtc.month < 1){
                    dcts.dcts_rtc.month = 0;
                }else{
                    dcts.dcts_rtc.month -= 1;
                }
                break;
            case 4:
                if(dcts.dcts_rtc.year < RTC_MIN_YEAR){
                    dcts.dcts_rtc.year = RTC_MIN_YEAR;
                }else{
                    dcts.dcts_rtc.year -= 1000;
                }
                break;
            case 5:
                if(dcts.dcts_rtc.year < RTC_MIN_YEAR){
                    dcts.dcts_rtc.year = RTC_MIN_YEAR;
                }else{
                    dcts.dcts_rtc.year -= 100;
                }
                break;
            case 6:
                if(dcts.dcts_rtc.year < RTC_MIN_YEAR){
                    dcts.dcts_rtc.year = RTC_MIN_YEAR;
                }else{
                    dcts.dcts_rtc.year -= 10;
                }
                break;
            case 7:
                if(dcts.dcts_rtc.year < RTC_MIN_YEAR){
                    dcts.dcts_rtc.year = RTC_MIN_YEAR;
                }else{
                    dcts.dcts_rtc.year -= 1;
                }
                break;
            case 8:
                if(dcts.dcts_rtc.weekday < 2){
                    dcts.dcts_rtc.weekday = 1;
                }else{
                    dcts.dcts_rtc.weekday -= 1;
                }
                break;
            case 9:
                if(dcts.dcts_rtc.hour < 10){
                    dcts.dcts_rtc.hour = 0;
                }else{
                    dcts.dcts_rtc.hour -= 10;
                }
                break;
            case 10:
                if(dcts.dcts_rtc.hour < 1){
                    dcts.dcts_rtc.hour = 0;
                }else{
                    dcts.dcts_rtc.hour -= 1;
                }
                break;
            case 11:
                if(dcts.dcts_rtc.minute < 10){
                    dcts.dcts_rtc.minute = 0;
                }else{
                    dcts.dcts_rtc.minute -= 10;
                }
                break;
            case 12:
                if(dcts.dcts_rtc.minute < 1){
                    dcts.dcts_rtc.minute = 0;
                }else{
                    dcts.dcts_rtc.minute -= 1;
                }
                break;
            }
            SSD1306_DrawFilledRectangle(0,10,128,54,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.up){
            switch (position) {
            case 0:
                dcts.dcts_rtc.day += 10;
                if(dcts.dcts_rtc.day > RTC_MAX_DAY){
                    dcts.dcts_rtc.day = RTC_MAX_DAY;
                }
                break;
            case 1:
                dcts.dcts_rtc.day += 1;
                if(dcts.dcts_rtc.day > RTC_MAX_DAY){
                    dcts.dcts_rtc.day = RTC_MAX_DAY;
                }
                break;
            case 2:
                dcts.dcts_rtc.month += 10;
                if(dcts.dcts_rtc.month > RTC_MAX_MONTH){
                    dcts.dcts_rtc.month = RTC_MAX_MONTH;
                }
                break;
            case 3:
                dcts.dcts_rtc.month += 1;
                if(dcts.dcts_rtc.month > RTC_MAX_MONTH){
                    dcts.dcts_rtc.month = RTC_MAX_MONTH;
                }
                break;
            case 4:
                dcts.dcts_rtc.year += 1000;
                if(dcts.dcts_rtc.year > RTC_MAX_YEAR){
                    dcts.dcts_rtc.year = RTC_MAX_YEAR;
                }
                break;
            case 5:
                dcts.dcts_rtc.year += 100;
                if(dcts.dcts_rtc.year > RTC_MAX_YEAR){
                    dcts.dcts_rtc.year = RTC_MAX_YEAR;
                }
                break;
            case 6:
                dcts.dcts_rtc.year += 10;
                if(dcts.dcts_rtc.year > RTC_MAX_YEAR){
                    dcts.dcts_rtc.year = RTC_MAX_YEAR;
                }
                break;
            case 7:
                dcts.dcts_rtc.year += 1;
                if(dcts.dcts_rtc.year > RTC_MAX_YEAR){
                    dcts.dcts_rtc.year = RTC_MAX_YEAR;
                }
                break;
            case 8:
                dcts.dcts_rtc.weekday += 1;
                if(dcts.dcts_rtc.weekday > RTC_MAX_WEEKDAY){
                    dcts.dcts_rtc.weekday = RTC_MAX_WEEKDAY;
                }
                break;
            case 9:
                dcts.dcts_rtc.hour += 10;
                if(dcts.dcts_rtc.hour > RTC_MAX_HOUR){
                    dcts.dcts_rtc.hour = RTC_MAX_HOUR;
                }
                break;
            case 10:
                dcts.dcts_rtc.hour += 1;
                if(dcts.dcts_rtc.hour > RTC_MAX_HOUR){
                    dcts.dcts_rtc.hour = RTC_MAX_HOUR;
                }
                break;
            case 11:
                dcts.dcts_rtc.minute += 10;
                if(dcts.dcts_rtc.minute > RTC_MAX_MINUTE){
                    dcts.dcts_rtc.minute = RTC_MAX_MINUTE;
                }
                break;
            case 12:
                dcts.dcts_rtc.minute += 1;
                if(dcts.dcts_rtc.minute > RTC_MAX_MINUTE){
                    dcts.dcts_rtc.minute = RTC_MAX_MINUTE;
                }
                break;
            }
            SSD1306_DrawFilledRectangle(0,10,128,54,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.ok){
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            break;
        }

        /* print values on screen */

        SSD1306_GotoXY(22, 0);
        SSD1306_Puts("Äàòà è âðåìÿ", &Font_7x10,SSD1306_COLOR_WHITE);
        display_time(20);
        switch (position){
        case 0:
            x = 0;
            break;
        case 1:
            x = 7;
            break;
        case 2:
            x = 21;
            break;
        case 3:
            x = 28;
            break;
        case 4:
            x = 42;
            break;
        case 5:
            x = 49;
            break;
        case 6:
            x = 56;
            break;
        case 7:
            x = 63;
            break;
        case 8:
            x = 76;
            break;
        case 9:
            x = 92;
            break;
        case 10:
            x = 99;
            break;
        case 11:
            x = 113;
            break;
        case 12:
            x = 120;
            break;
        }
        SSD1306_GotoXY(x, 32);
        SSD1306_Putc('^', &Font_7x10,SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
        if(eTaskGetState(buttonsTaskHandle) == eSuspended){
            vTaskDelay(DISPLAY_TASK_PERIOD);
            vTaskResume(buttonsTaskHandle);
        }
    }
    /* save new time and data */
    time.Hours = dcts.dcts_rtc.hour;
    time.Minutes = dcts.dcts_rtc.minute;
    time.Seconds = 0;

    date.Date = dcts.dcts_rtc.day;
    date.Month = dcts.dcts_rtc.month;
    date.Year = (uint8_t)(dcts.dcts_rtc.year - 2000);
    date.WeekDay = dcts.dcts_rtc.weekday;

    HAL_RTC_SetDate(&hrtc,&date,RTC_FORMAT_BIN);
    HAL_RTC_SetTime(&hrtc,&time,RTC_FORMAT_BIN);
    menu.level--;

    vTaskResume(defaultTaskHandle);
}

#define REG_MAX_TMPR    150
#define REG_MIN_TMPR    10
static void max_reg_temp_set(void){
    char buff[7] = {0};
    uint32_t last_wake_time = osKernelSysTick();
    SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
    SSD1306_GotoXY(22,0);
    SSD1306_Puts("Ìàêñèìàëüíàÿ", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(25,12);
    SSD1306_Puts("òåìïåðàòóðà", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(29, 24);
    SSD1306_Puts("ñèììèñòîðà", &Font_7x10, SSD1306_COLOR_WHITE);
    while(1){
        /* Read buttons */
        if (pressed_time.ok){
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            break;
        }
        if(pressed_time.up){
            if(semistor_state.max_tmpr > REG_MAX_TMPR){
                semistor_state.max_tmpr = REG_MAX_TMPR;
            }else{
                semistor_state.max_tmpr++;
            }
        }
        if(pressed_time.down){
            if(semistor_state.max_tmpr < REG_MIN_TMPR){
                semistor_state.max_tmpr = REG_MIN_TMPR;
            }else{
                semistor_state.max_tmpr--;
            }
        }

        /* Print screen */
        SSD1306_GotoXY(46, 46);
        sprintf(buff, "%3.0f°C", (double)semistor_state.max_tmpr);
        SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
        if(eTaskGetState(buttonsTaskHandle) == eSuspended){
            vTaskDelay(DISPLAY_TASK_PERIOD);
            vTaskResume(buttonsTaskHandle);
        }
    }

    HAL_PWR_EnableBkUpAccess();
    BKP->DR7 = (uint32_t)semistor_state.max_tmpr;
    HAL_PWR_DisableBkUpAccess();
    menu.level--;
}

#endif //DISPLAY_C
