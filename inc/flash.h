#ifndef FLASH_H
#define FLASH_H 1

#include "stdint.h"
#include "stm32f1xx_hal.h"

/**
  * @addtogroup LCD
  * @{
  */
#define SAVE_AREA_SIZE 128   // One record size in bytes
#define PAGE_SIZE 1024
#define FLASH_SAVE_PAGE 127
#define FLASH_START_ADDRESS 0x08000000
#define FLASH_SAVE_PAGE_ADDRESS FLASH_START_ADDRESS+FLASH_SAVE_PAGE*PAGE_SIZE
#define SAVE_AREA_NMB PAGE_SIZE/SAVE_AREA_SIZE
/**
  * @}
  */


/*========== TYPEDEFS ==========*/



/*========= GLOBAL VARIABLES ==========*/



/*========== FUNCTION PROTOTYPES ==========*/

int save_to_flash(int area_cnt, uint8_t start_position, uint16_t *data);
int find_free_area(void);

#endif // FLASH_H
