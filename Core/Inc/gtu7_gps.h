/**
 * @file gtu7_gps.h
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief Header file for SSD1306Graph library
 * @version 0.1
 * @date 2023-06-18
 *
 * MIT License. Copyright (c) 2023.
 *
 */

#ifndef MP_GTU7_GPS_H_
#define MP_GTU7_GPS_H_

/*****************************************************************
 * Includes
******************************************************************/

#include <stdint.h>
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
 * Variables and constants
******************************************************************/

#define GPS_INIT_MSG_LEN            29
#define GPS_REQUEST_MSG_LEN         13
#define GPS_UART_TIMEOUT			50

#define GPS_CONFIG_MSG1				(uint8_t *)&"$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
#define GPS_CONFIG_MSG2				(uint8_t *)&"$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
#define GPS_CONFIG_MSG3				(uint8_t *)&"$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"
#define GPS_CONFIG_MSG4				(uint8_t *)&"$PUBX,40,RMC,0,0,0,0,0,0*47\r\n"
#define GPS_CONFIG_MSG5				(uint8_t *)&"$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n"
#define GPS_CONFIG_MSG6				(uint8_t *)&"$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"
#define GPS_REQUEST_MSG				(uint8_t *)&"$PUBX,00*33\r\n"

#define GPS_POS_LAT                 19
#define GPS_POS_NS                  30
#define GPS_POS_LON                 32
#define GPS_POS_WE                  44

/*****************************************************************/


/*****************************************************************
 * Function prototypes
******************************************************************/

HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef gps_position_request(UART_HandleTypeDef *huart);

/*****************************************************************/




#endif /* MP_GTU7_GPS_H_ */
