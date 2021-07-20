/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>
   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#ifndef FONTS_H
#define FONTS_H 1

/**
 *
 * Default fonts library. It is used in all LCD based libraries.
 *
 * \par Supported fonts
 * 
 * Currently, these fonts are supported:
 *  - 5 x 7 pixels
 *  - 7 x 10 pixels
 *  - 11 x 18 pixels
 *  - 16 x 26 pixels
 *  - 16 x 16 pixels icons
 */
#include "stm32f1xx_hal.h"
#include "string.h"

/**
 * @defgroup FONTS
 * @ingroup LCD
 * @brief Fonts for print on LCD
 */

/*========== TYPEDEFS ==========*/

/**
 * @brief  Font structure used in my LCD libraries
 * @ingroup FONTS
 */
typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
    uint8_t FontHeight;   /*!< Font height in pixels */
    uint8_t data_size_in_bytes; /*!< font data size */
    uint16_t shift; /*!< Shift char number for first symbol*/
    void *data; /*!< Pointer to data font data array */
} FontDef_t;

/**
 * @brief  5 x 7 pixels font size structure
 * @ingroup FONTS
 */
extern FontDef_t Icon_16x16;

/**
 * @brief Data of Font_5x7
 * @ingroup FONTS
 */
extern const uint16_t Icon16x16[];

/**
 * @brief  5 x 7 pixels font size structure
 * @ingroup FONTS
 */
extern FontDef_t Font_5x7;

/**
 * @brief Data of Font_5x7
 * @ingroup FONTS
 */
extern const uint8_t Font5x7[];

/**
 * @brief  7 x 10 pixels font size structure 
 * @ingroup FONTS
 */
extern FontDef_t Font_7x10;

/**
 * @brief Data of Font_7x10
 * @ingroup FONTS
 */
extern const uint8_t Font7x10[];

/**
 * @brief  11 x 18 pixels font size structure 
 * @ingroup FONTS
 */
extern FontDef_t Font_11x18;

/**
 * @brief Data of Font_11x18
 * @ingroup FONTS
 */
extern const uint16_t Font11x18[];

/**
 * @brief  16 x 26 pixels font size structure 
 * @ingroup FONTS
 */
extern FontDef_t Font_16x26;

/**
 * @brief Data of Font_16x26
 * @ingroup FONTS
 */
extern const uint16_t Font16x26[];

#endif
