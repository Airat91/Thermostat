#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define LOAD_TEMP_PORT  GPIOA
#define LOAD_TEMP_PIN   GPIO_PIN_0
#define REG_TEMP_PORT   GPIOA
#define REG_TEMP_PIN    GPIO_PIN_1
#define UP_PORT         GPIOA
#define UP_PIN          GPIO_PIN_2
#define DOWN_PORT       GPIOA
#define DOWN_PIN        GPIO_PIN_3
#define LEFT_PORT       GPIOA
#define LEFT_PIN        GPIO_PIN_4
#define RIGHT_PORT      GPIOA
#define RIGHT_PIN       GPIO_PIN_5
#define OK_PORT         GPIOA
#define OK_PIN          GPIO_PIN_6

#define DEBUG_TMS_PORT  GPIOA
#define DEBUG_TMS_PIN   GPIO_PIN_13
#define DEBUG_TCK_PORT  GPIOA
#define DEBUG_TCK_PIN   GPIO_PIN_14

#define DISP_SCL_PORT   GPIOB
#define DISP_SCL_PIN    GPIO_PIN_6
#define DISP_SDA_PORT   GPIOB
#define DISP_SDA_PIN    GPIO_PIN_7

#define SYNC_PORT       GPIOB
#define SYNC_PIN        GPIO_PIN_9

#define REG_ON_PORT     GPIOC
#define REG_ON_PIN      GPIO_PIN_13

#endif // PIN_MAP_H
