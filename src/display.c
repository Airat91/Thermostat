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
#include "buttons.h"
#include "stm32f1xx_ll_gpio.h"
extern IWDG_HandleTypeDef hiwdg;
extern RTC_HandleTypeDef hrtc;
extern osThreadId displayTaskHandle;
extern osThreadId menuTaskHandle;
static u8 display_time(void);
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
    PAGE_END_OF_LIST,
};

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
            while(pressed_time.up){
                if(pressed_time.up > DISPLAY_TASK_PERIOD){
                    pressed_time.up = 0;
                    break;
                }
                osDelay(1);
            }
            act[0].set_value += 1.0f;
        }
        if (pressed_time.down){
            while(pressed_time.down){
                if(pressed_time.down > DISPLAY_TASK_PERIOD){
                    pressed_time.down = 0;
                    break;
                }
                osDelay(1);
            }
            act[0].set_value -= 1.0f;
        }
        if (pressed_time.left){
            while(pressed_time.left){
                if(pressed_time.left > DISPLAY_TASK_PERIOD*2){
                    pressed_time.left = 0;
                    break;
                }
                osDelay(1);
            }
            skin--;
            if(skin == 0xFF){
                skin = SKIN_END_OF_LIST - 1;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.right){
            while(pressed_time.right){
                if(pressed_time.right > DISPLAY_TASK_PERIOD*2){
                    pressed_time.right = 0;
                    break;
                }
                osDelay(1);
            }
            skin++;
            if(skin == SKIN_END_OF_LIST){
                skin = SKIN_FULL;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.ok > 3000){
            pressed_time.ok = 0;
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            vTaskResume(menuTaskHandle);
            vTaskSuspend(NULL);
        }

        /* Print main screen */
        if(skin == SKIN_FULL){  // Full information

            sprintf(buff,"%2.1f", (double)act[0].meas_value);
            SSD1306_GotoXY(0, 14);
            SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);

            sprintf(buff,"Уст %2.0f%s", (double)act[0].set_value, act[0].unit);
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            sprintf(buff,"Рег%3.0f%s", (double)meas[1].value, meas[1].unit);
            SSD1306_GotoXY(70, 29);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            display_time();

        }else if(skin == SKIN_1){    // Only current and required temperature

            sprintf(buff,"%2.1f", (double)act[0].meas_value);
            SSD1306_GotoXY(0, 14);
            SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);

            sprintf(buff,"Уст %2.0f%s", (double)act[0].set_value, act[0].unit);
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

        }else if(skin == SKIN_2){    // Current and required temperature and clock

            sprintf(buff,"%2.1f", (double)act[0].meas_value);
            SSD1306_GotoXY(0, 14);
            SSD1306_Puts(buff, &Font_16x26, SSD1306_COLOR_WHITE);

            sprintf(buff,"Уст %2.0f%s", (double)act[0].set_value, act[0].unit);
            SSD1306_GotoXY(70, 16);
            SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

            display_time();
        }else if(skin == SKIN_EMPTY){   // Empty screen

        }

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
    }
}
u8 display_time(void){
    char buff[20] = {0};
    char weekday[3] = {0};
    switch (rtc.weekday) {
    case 1:
        strcpy(weekday, "Пн");
        break;
    case 2:
        strcpy(weekday, "Вт");
        break;
    case 3:
        strcpy(weekday, "Ср");
        break;
    case 4:
        strcpy(weekday, "Чт");
        break;
    case 5:
        strcpy(weekday, "Пт");
        break;
    case 6:
        strcpy(weekday, "Сб");
        break;
    case 7:
        strcpy(weekday, "Вс");
        break;
    }
    sprintf(buff,"%2d.%2d.%4d ", rtc.day, rtc.month, rtc.year);
    if(rtc.day < 10){
        buff[0] = '0';
    }
    if(rtc.month < 10){
        buff[3] = '0';
    }
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(74, 0);
    SSD1306_Puts(weekday, &Font_7x10, SSD1306_COLOR_WHITE);

    sprintf(buff,"%2d:%2d", rtc.hour, rtc.minute);
    if(rtc.hour < 10){
        buff[0] = '0';
    }
    if(rtc.minute < 10){
        buff[3] = '0';
    }
    SSD1306_GotoXY(92, 0);
    SSD1306_Puts(buff, &Font_7x10, SSD1306_COLOR_WHITE);

    return 0x00;
}

void menu_task( const void *parameters){
    (void) parameters;
    vTaskSuspend(NULL); // Suspend menu_task after create
    uint32_t last_wake_time = osKernelSysTick();
    u8 page = PAGE_CLOCK;
    while(1){
        /*if(eTaskGetState(displayTaskHandle) != eSuspended){
            vTaskSuspend(displayTaskHandle);
        }*/

        /* Read buttons */
        if (pressed_time.left){
            while(pressed_time.left){
                if(pressed_time.left > DISPLAY_TASK_PERIOD*2){
                    pressed_time.left = 0;
                    break;
                }
                osDelay(1);
            }
            page--;
            if(page == 0xFF){
                page = PAGE_END_OF_LIST - 1;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.right){
            while(pressed_time.right){
                if(pressed_time.right > DISPLAY_TASK_PERIOD*2){
                    pressed_time.right = 0;
                    break;
                }
                osDelay(1);
            }
            page++;
            if(page == PAGE_END_OF_LIST){
                page = PAGE_CLOCK;
            }
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
        }
        if (pressed_time.ok > 3000){
            pressed_time.ok = 0;
            SSD1306_DrawFilledRectangle(0,0,128,64,SSD1306_COLOR_BLACK);    // clear display
            SSD1306_UpdateScreen();
            vTaskResume(displayTaskHandle);
            vTaskSuspend(NULL);
        }

        if(page == PAGE_CLOCK){
            SSD1306_GotoXY(5, 0);
            SSD1306_Puts("Установка времени", &Font_7x10, SSD1306_COLOR_WHITE);
        }else if(page == PAGE_HYSTERESIS){
            SSD1306_GotoXY(29, 0);
            SSD1306_Puts("Гистерезис", &Font_7x10, SSD1306_COLOR_WHITE);
        }else if(page == PAGE_PROGRAMM){
            SSD1306_GotoXY(22, 0);
            SSD1306_Puts("Выбор режима", &Font_7x10, SSD1306_COLOR_WHITE);
        }else if(page == PAGE_STATISTIC){
            SSD1306_GotoXY(29, 0);
            SSD1306_Puts("Статистика", &Font_7x10, SSD1306_COLOR_WHITE);
        }

        SSD1306_UpdateScreen();
        HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DISPLAY_TASK_PERIOD);
    }

}

#endif //DISPLAY_C
