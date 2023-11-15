/**
 * @file ssd_lora_config_menu.c
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief 
 * @version 0.1
 * @date 2023-09-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "ssd_lora_config_menu.h"


/**
 * @brief Ensure rotation across values for a position and set Lora structure values accordingly
 * 
 * @param menu 
 * @param lora 
 */
void menu_increase_item(MENU_TypeDef *menu, SX1278_TypeDef *lora)
{
    if (menu->active_item < menu->menu_items_size[menu->active_position] - 1) {
        menu->active_item++;
    } else {
        menu->active_item = 0;
    }
    switch(menu->active_position) {
    case 1:
        lora->spreading_factor = menu->menu_items[menu->active_position][menu->active_item];
        break;
    case 2:
        lora->bandwidth = menu->menu_items[menu->active_position][menu->active_item];
        break;
    case 3:
        lora->coding_rate = menu->menu_items[menu->active_position][menu->active_item];
        break;
    case 4:
        lora->crc = menu->menu_items[menu->active_position][menu->active_item];
        break;
    default:
        break;
    }

}

/**
 * @brief Gets active item for active position and sets it in menu structure
 * 
 * @param menu 
 * @param lora 
 */
void menu_get_active_item(MENU_TypeDef *menu, SX1278_TypeDef *lora)
{
    uint8_t value_to_find;

    switch(menu->active_position) {
        case 1:
            value_to_find = lora->spreading_factor;
            break;
        case 2:
            value_to_find = lora->bandwidth;
            break;
        case 3:
            value_to_find = lora->coding_rate;
            break;
        case 4:
            value_to_find = lora->crc;
            break;
        default:
            value_to_find = 0;
            break;
    }
    for (uint8_t i=0; i < menu->menu_items_size[menu->active_position]; i++) {
        if (menu->menu_items[menu->active_position][i] == value_to_find) {
            menu->active_item = i;
        }
    }
}


/**
 * @brief Initialize menu structure
 * 
 * @return MENU_TypeDef 
 */
MENU_TypeDef new_menu(void)
{   
    MENU_TypeDef menu;
    sprintf(menu.menu_position[0], "NULL");
    sprintf(menu.menu_position[1], "Spreading Factor");
    sprintf(menu.menu_position[2], "Bandwidth");
    sprintf(menu.menu_position[3], "Coding Rate");
    sprintf(menu.menu_position[4], "CRC");
    memset(menu.menu_items, 0x00, MENU_SIZE * MENU_ITEM_SIZE * sizeof(uint8_t));
    /****** Spreading factors ******/
    menu.menu_items[1][0] = SF_7;
    menu.menu_items[1][1] = SF_8;
    menu.menu_items[1][2] = SF_9;
    menu.menu_items[1][3] = SF_10;
    menu.menu_items[1][4] = SF_11;
    menu.menu_items[1][5] = SF_12;
    /****** Bandwidth ******/
    menu.menu_items[2][0] = BW_7_8KHz;
    menu.menu_items[2][1] = BW_10_4KHz;
    menu.menu_items[2][2] = BW_15_6KHz;
    menu.menu_items[2][3] = BW_20_8KHz;
    menu.menu_items[2][4] = BW_31_25KHz;
    menu.menu_items[2][5] = BW_41_7KHz;
    menu.menu_items[2][6] = BW_62_5KHz;
    menu.menu_items[2][7] = BW_125KHz;
    menu.menu_items[2][8] = BW_250KHz;
    menu.menu_items[2][9] = BW_500KHz;
    /****** Coding rate ******/
    menu.menu_items[3][0] = CODING_RATE_4_5;
    menu.menu_items[3][1] = CODING_RATE_4_6;
    menu.menu_items[3][2] = CODING_RATE_4_7;
    menu.menu_items[3][3] = CODING_RATE_4_8;
    /****** CRC ******/
    menu.menu_items[4][0] = 0x00;
    menu.menu_items[4][1] = RX_PAYLOAD_CRC_ON;

    menu.menu_items_size[0] = 0;
    menu.menu_items_size[1] = 6;
    menu.menu_items_size[2] = 10;
    menu.menu_items_size[3] = 4;
    menu.menu_items_size[4] = 2;
    
    menu.btn1_time_clicked = 0;
    menu.btn2_time_clicked = 0;
    menu.btn1_state = BUTTON_OFF;
    menu.btn2_state = BUTTON_OFF;
    menu.active_position = 0;
    return menu;
}

// void increase_position(Config_Position *ptr)
// {
//     if (*ptr < CRC_SETTING) { *ptr += 1; }
//     else if (*ptr == CRC_SETTING) { *ptr = OUT; }
// }
