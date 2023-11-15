/**
 * @file ssd_lora_config_menu.h
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief Transmitter library with devices related functions
 * @version 0.1
 * @date 2023-08-24
 *
 * MIT License. Copyright (c) 2023.
*/
#ifndef __MP_SSD_LORA_CONFIG_MENU_H__
#define __MP_SSD_LORA_CONFIG_MENU_H__



/*****************************************************************
 * Includes
******************************************************************/

#include <stdint.h>
#include <string.h>
#include "sx1278.h"

/*****************************************************************
 * Variables and constants
******************************************************************/

#define MENU_SIZE                   5
#define MENU_ITEMNAME_MAXSIZE       18
#define MENU_ITEM_SIZE              10

/*****************************************************************
 * Exported Types
 *****************************************************************/

/**
 * @brief Holds states of the buttons
 * 
 */
typedef enum
{
  BUTTON_OFF = 0x00U,
  BUTTON_ON = 0x01U
} Button_State;

/**
 * @brief All data needed for menu management
 * 
 */
typedef struct {
  char menu_position[MENU_SIZE][MENU_ITEMNAME_MAXSIZE];
  uint8_t menu_items[MENU_SIZE][MENU_ITEM_SIZE];
  uint8_t menu_items_size[MENU_SIZE];
  uint32_t btn1_time_clicked;
  uint32_t btn2_time_clicked;
  Button_State btn1_state;
  Button_State btn2_state;
  uint8_t active_position;
  uint8_t active_item;
} MENU_TypeDef;

/*****************************************************************
 * Function prototypes
******************************************************************/

MENU_TypeDef new_menu(void);
void menu_get_active_item(MENU_TypeDef *menu, SX1278_TypeDef *lora);
void menu_increase_item(MENU_TypeDef *menu, SX1278_TypeDef *lora);
// void increase_position(Config_Position *ptr);
// void increase_sf(SX1278_TypeDef *ptr);
// void increase_bw(SX1278_TypeDef *ptr);
// void increase_cr(SX1278_TypeDef *ptr);
// void increase_crc(SX1278_TypeDef *ptr);

/*****************************************************************/
#endif /* __MP_SSD_LORA_CONFIG_MENU_H__ */