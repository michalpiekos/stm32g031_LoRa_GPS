/**
 * @file gtu7_gps.c
 * @author Michal Piekos
 * @brief GT-U7 GPS access library
 * @version 0.1
 * @date 2023-06-18
 *
 * @copyright MIT License. Copyright (c) 2023.
 *
 */

#include "gtu7_gps.h"
#include <string.h>


/**
 * @brief Initialize GT-U7 GPS using 6 prepared messages. After init GPS will be waiting for request for position message.
 * 
 * @param huart HAL handle to UART
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart)
{
	HAL_StatusTypeDef uart_status;
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG1, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	if (!uart_status == HAL_OK)
		return uart_status;
	HAL_Delay(5);
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG2, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	if (!uart_status == HAL_OK)
		return uart_status;
	HAL_Delay(5);
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG3, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	if (!uart_status == HAL_OK)
		return uart_status;
	HAL_Delay(5);
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG4, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	if (!uart_status == HAL_OK)
		return uart_status;
	HAL_Delay(5);
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG5, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	if (!uart_status == HAL_OK)
		return uart_status;
	HAL_Delay(5);
	uart_status = HAL_UART_Transmit(huart, GPS_CONFIG_MSG6, GPS_INIT_MSG_LEN, GPS_UART_TIMEOUT);
	HAL_Delay(5);
	return uart_status;
}


/**
 * @brief Send request for NMEA message with position
 * 
 * @param huart HAL handle to UART
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef gps_position_request(UART_HandleTypeDef *huart)
{
  return HAL_UART_Transmit(huart, GPS_REQUEST_MSG, GPS_REQUEST_MSG_LEN, GPS_UART_TIMEOUT);
}
