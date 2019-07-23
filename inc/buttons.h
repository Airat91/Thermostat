#ifndef BUTTONS_H
#define BUTTONS_H

#include "type_def.h"
#include "stm32f1xx_hal.h"

typedef struct {
    uint16_t up;
    uint16_t down;
    uint16_t left;
    uint16_t right;
    uint16_t ok;
}button_t;

extern button_t pressed_time;

void buttons_task(void const * argument);

#endif // BUTTONS_H
