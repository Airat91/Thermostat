#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"

#ifndef ADC_H
#define ADC_H 1

/**
  * @addtogroup ADC
  * @{
  */
extern ADC_HandleTypeDef hadc1;
/**
  * @}
  */

/*========== TYPEDEFS ==========*/



/*========== FUNCTION PROTOTYPES ==========*/

int adc_init (void);
void adc_deinit (void);
void adc_gpio_init (void);
void adc_gpio_deinit (void);
void adc_task(void const * argument);
float adc_tmpr_calc(float adc);
float adc_lvl_calc(float adc);

#endif // ADC_H
