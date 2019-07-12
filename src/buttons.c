#include "pin_map.h"
#include "buttons.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"

#define BUTTONS_TASK_PERIOD 1

button_t pressed_time = {0};

void buttons_task (void const * argument){
    (void) argument;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        if(HAL_GPIO_ReadPin(UP_PORT, UP_PIN)){
            pressed_time.up = 0;
        }else{
            pressed_time.up += BUTTONS_TASK_PERIOD;
        }
        if(HAL_GPIO_ReadPin(DOWN_PORT, DOWN_PIN)){
            pressed_time.down = 0;
        }else{
            pressed_time.down += BUTTONS_TASK_PERIOD;
        }
        if(HAL_GPIO_ReadPin(LEFT_PORT, LEFT_PIN)){
            pressed_time.left = 0;
        }else{
            pressed_time.left += BUTTONS_TASK_PERIOD;
        }
        if(HAL_GPIO_ReadPin(RIGHT_PORT, RIGHT_PIN)){
            pressed_time.right = 0;
        }else{
            pressed_time.right += BUTTONS_TASK_PERIOD;
        }
        if(HAL_GPIO_ReadPin(OK_PORT, OK_PIN)){
            pressed_time.ok = 0;
        }else{
            pressed_time.ok += BUTTONS_TASK_PERIOD;
        }


        osDelayUntil(&last_wake_time, BUTTONS_TASK_PERIOD);
    }
}
