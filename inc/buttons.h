#ifndef BUTTONS_H
#define BUTTONS_H

#include "type_def.h"
#include "stm32f1xx_hal.h"

/**
  * @brief list of buttons
  * @ingroup button
  */

/*========== TYPEDEFS ==========*/

typedef enum {
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_OK,
}button_list_t;
/**
  * @brief button states
  * @ingroup button
  */
typedef enum {
    BUTTON_PRESSED,
    BUTTON_RELEASE,
}button_state_t;
/**
  * @brief struct for read button state
  * @ingroup button
  */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint16_t pressed;
    button_state_t last_state;
}button_t;

/*========== GLOBAL VARS ==========*/

extern button_t pressed_time[];

/*========== FUNCTION PROTOTYPES ==========*/

void buttons_task(void const * argument);
uint8_t button_click(button_list_t button, uint16_t time);
uint8_t button_clamp(button_list_t button, uint16_t time);

#endif // BUTTONS_H
