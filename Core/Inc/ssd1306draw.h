/**
 * @file ssd1306draw.h
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief Header file for SSD1306 tracking data plotting library for STM32 written using STM32 HAL
 * @version 0.1
 * @date 2023-06-17
 * 
 * MIT License. Copyright (c) 2023.
 * 
 */
//#ifdef HAL_I2C_MODULE_ENABLED

#ifndef MP_SSD1306GRAPH_h
#define MP_SSD1306GRAPH_h


/*****************************************************************
 * Includes
******************************************************************/

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#if defined(STM32G030xx) || defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#endif
#ifdef STM32F411xE
#include "stm32f4xx_hal.h"
#endif

/*****************************************************************/


/*****************************************************************
 * Types
******************************************************************/

typedef struct {
    int x;
    int y;
} coordinate_t;

/*****************************************************************/


/*****************************************************************
 * Variables and constants
******************************************************************/

#ifndef  M_PI
#define  M_PI  																	3.1415926535897932384626433
#endif

#define FONT_SIZE                               5
//#define SSD1306_USE_DMA                         1

//#define IIC_FREQ                                100000UL
//#define IIC_STATUS_OK                           0xFF

#define SSD1306_ADDRESS                         0x3C
#define SSD1306_COMMAND                         0x00
#define SSD1306_SET_MUX_RATIO                   0xA8
#define SSD1306_SET_DISPLAY_OFFSET              0xD3
#define SSD1306_SET_DISPLAY_START_LINE          0x40
#define SSD1306_SET_SEGMENT_REMAP               0xA0
#define SSD1306_COM_OUTPUT_SCAN_UP              0xC0
#define SSD1306_COM_OUTPUT_SCAN_DOWN            0xC8
#define SSD1306_SET_COM_PINS                    0xDA
#define SSD1306_SET_CONTRAST_CONTROL            0x81
#define SSD1306_SET_MEMORY_ADDRESSING_MODE      0x20
#define SSD1306_DISABLE_ENTIRE_DISPLAY_ON       0xA4
#define SSD1306_SET_NORMAL_DISPLAY              0xA6
#define SSD1306_SET_OSC_FREQUENCY               0xD5
#define SSD1306_ENABLE_CHARGE_PUMP              0x8D
#define SSD1306_DISPLAY_ON                      0xAF
#define SSD1306_DISPLAY_OFF                     0xAE
#define SSD1306_RIGHT_HORIZONTAL_SCROLL         0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL          0x27
#define SSD1306_SET_VCOMH_DESELECT_LEVEL        0xDB
#define SSD1306_HORIZONTAL_ADDRESSING_MODE	    0x00
#define SSD1306_VERTICAL_ADDRESSING_MODE	  	0x01
#define SSD1306_PAGE_ADDRESSING_MODE	        0x02
#define SSD1306_SET_COLUMN_ADDRESS              0x21
#define SSD1306_SET_PAGE_ADDRESS                0x22

#ifndef SSD1306_NUM_PAGES
#define SSD1306_NUM_PAGES						8
#endif // SSD1306_NUM_PAGES
#define SSD1306_BUFFER_SIZE                     SSD1306_NUM_PAGES * 128

#define SSD1306_MENU_SIZE						2
#define SSD1306_MENU_ITEM_MAX_LENGTH			18

/*****************************************************************/


/*****************************************************************
 * Function prototypes
******************************************************************/

void ssd1306_init(I2C_HandleTypeDef *hi2c);
void ssd1306_clear(void);
void ssd1306_clearrow(uint8_t row);
void ssd1306_line(coordinate_t p0, coordinate_t p1);
void ssd1306_commit(I2C_HandleTypeDef *hi2c);
void ssd1306_printchar(uint8_t character, uint8_t row, uint8_t column);
void ssd1306_printstring(char *str, uint8_t row, uint8_t column);
void ssd1306_printint(int16_t number, uint8_t row, uint8_t column);
void ssd1306_printhex(int16_t number, uint8_t row, uint8_t column);
coordinate_t ssd1306_endpoint(uint16_t angle, coordinate_t p0, int length);
void ssd1306_drawmenu(uint8_t active_item);

void _swap_points(coordinate_t *a, coordinate_t *b);
void _swap_xy(coordinate_t *a, coordinate_t *b);
int8_t _mirror_y(uint8_t a, uint8_t b);
void _ssd1306_set_addressing_mode(I2C_HandleTypeDef *hi2c, uint8_t mode);
void _ssd1306_tx_command(I2C_HandleTypeDef *hi2c, uint8_t command);


/*****************************************************************/

#endif // MP_SSD1306GRAPH_h
//#endif // HAL_I2C_MODULE_ENABLED
