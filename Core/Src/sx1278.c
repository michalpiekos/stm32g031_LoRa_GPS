/**
 * @file peripherals.c
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief
 * @version 0.1
 * @date 2023-08-22
 *
 * MIT License. Copyright (c) 2023.
*/
#include <sx1278.h>


/**
 * @brief Initialize LoRa to SLEEP mode and configure it
 * For 433MHz frequency the value to be written to 3 RegFrXXX registers is 7094272.
 *
 * @param lora pointer to lora initialization structure
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 *
 */
SX1278_Status_TypeDef lora_init(SX1278_TypeDef *lora)
{
	lora_reset(lora);
	if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | SLEEP_MODE) != LORA_OK) {
		return WRITE_ERROR;
	}
  if (lora_set_frequency(lora, lora->freqMHz) != LORA_OK) { return WRITE_ERROR; }
  if (lora_set_power(lora, lora->power) != LORA_OK) { return WRITE_ERROR; }
  if (lora_safe_write(lora, RegModemConfig2, lora->spreading_factor | lora->crc | RX_TIMEOUT_MSB) != LORA_OK) { return WRITE_ERROR; }
  if (lora_safe_write(lora, RegSymbTimeoutL, RX_TIMEOUT_LSB) != LORA_OK) { return WRITE_ERROR; }
  if (lora_safe_write(lora, RegModemConfig1, lora->bandwidth | lora->coding_rate) != LORA_OK) {	return WRITE_ERROR; }
  if (lora_safe_write(lora, RegFiFoTxBaseAddr, 0) != LORA_OK) {	return WRITE_ERROR;	}
  if (lora_safe_write(lora, RegFiFoRxBaseAddr, 0) != LORA_OK) {	return WRITE_ERROR;	}
  if (lora_safe_write(lora, RegLna, LNA_GAIN_G1 | LNA_BOOST_HF) != LORA_OK) { return WRITE_ERROR; }
  if (lora_safe_write(lora, RegModemConfig3, AGC_AUTO_ON) != LORA_OK) {	return WRITE_ERROR; }
  if (lora_safe_write(lora, RegSyncWord, lora->syncword) != LORA_OK) { return WRITE_ERROR; }
  
  if (lora->device == TRANSMITTER) {
	if (lora_safe_write(lora, RegDioMapping1, DIO_MAP_TXDONE) != LORA_OK) { return WRITE_ERROR; }
	if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | SLEEP_MODE) != LORA_OK) { return WRITE_ERROR; }
  } else if (lora->device == RECEIVER) {
	if (lora_safe_write(lora, RegDioMapping1, DIO_MAP_RXDONE) != LORA_OK) { return WRITE_ERROR; }
	if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | RXCONTIN_MODE) != LORA_OK) { return WRITE_ERROR; }
  }
  return LORA_OK;
}


/**
 * @brief Receive data from LoRa using interrupt RxDone. This function should be inside interrupt function.
 * 
 * @param lora pointer to initialization structure
 * @param buf buffer for received data
 * @param len length of buffer
 * 
 * @return length of received data
 */
uint8_t lora_receive_IT(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len)
{
	uint8_t buffer_address;
	uint8_t read_bytes = 0;
	uint8_t irq;

	read_bytes = lora_read_register(lora, RegRxNbBytes);
	buffer_address = lora_read_register(lora, RegFiFoRxCurrentAddr);

	lora_write_register(lora, RegFiFoAddrPtr, buffer_address);
	for (uint8_t i = 0; i < read_bytes; i++) {
		buf[i] = lora_read_register(lora, RegFiFo);
	}

	/******TESTING*******/
	// lora_write_register(lora, RegFiFoAddrPtr, 0);
	// for (uint8_t i = 0; i <= 0xFF; i++) {
	// 	buf[i] = lora_read_register(lora, RegFiFo);
	// }

	debug_print("INFO: Received msg: %s\r\n", buf);
	irq = lora_read_register(lora, RegIrqFlags);
	debug_print("INFO: Flags with message: %x\r\n", irq);
	lora_write_register(lora, RegIrqFlags, IRQ_CLEAR_MASK);
	debug_print("INFO: Read: %d  %x\r\n", read_bytes, buffer_address);
	return read_bytes;
}


/**
 * @brief Receive data from LoRa
 * 
 * @param lora pointer to initialization structure
 * @param buf buffer for received data
 * @param len length of buffer
 * 
 * @return length of received data
 */
uint8_t lora_receive(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len)
{
	uint8_t buffer_address;
	uint8_t read_bytes = 0;
	uint8_t irq;

	// for (uint8_t i = 0; i < len; i++) {
	// 	buf[i] = '\0';
	// }
//	if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | STNBY_MODE) != LORA_OK) {
//		return 0;
//	}
	if ((irq = lora_read_register(lora, RegIrqFlags) & IRQ_RX_DONE_MASK) == 0) {
		lora_write_register(lora, RegIrqFlags, IRQ_TX_DONE_MASK);
		read_bytes = lora_read_register(lora, RegRxNbBytes);
		buffer_address = lora_read_register(lora, RegFiFoRxCurrentAddr);
		if (lora_safe_write(lora, RegFiFoAddrPtr, buffer_address) != LORA_OK) {
		return 0;
	}
		if (read_bytes > len) {
			debug_print("ERROR: Provided buffer with size %d is not big enough for received message of size %d\r\n", len, read_bytes);
		}
		for (uint8_t i = 0; i < read_bytes; i++) {
			buf[i] = lora_read_register(lora, RegFiFo);
		}
		debug_print("INFO: Received msg: %s\r\n", buf);
	} else if (irq == IRQ_PAYLOAD_CRC_ERROR_MASK) {
		debug_print("ERROR: %s\r\n", "CRC Error");
	}
	// if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | RXCONTIN_MODE) != LORA_OK) {
	// 	return 0; // TODO this must be changed. Logic failure possible.
	// }
	lora_write_register(lora, RegOpMode, LONG_RANGE_MODE | RXCONTIN_MODE);
	return read_bytes;
}


/**
 * @brief LoRa send function. State machine from SX1278 datasheet page 38.
 *
 * @param lora pointer to lora initialization structure
 * @param buf pointer to a buffer
 * @param len length of the buffer
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 */
SX1278_Status_TypeDef lora_send(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len)
{
//	uint8_t irq;

	// if (lora_safe_write(lora, RegOpMode, LONG_RANGE_MODE | STNBY_MODE) != LORA_OK) {
	// 	return WRITE_ERROR;
	// }
	lora_write_register(lora, RegOpMode, LONG_RANGE_MODE | STNBY_MODE);
	HAL_Delay(100);
	if (lora_safe_write(lora, RegFiFoAddrPtr, 0) != LORA_OK) {
		return WRITE_ERROR;
	}
	if (lora_safe_write(lora, RegPayloadLength, 0) != LORA_OK) {
		return WRITE_ERROR;
	}
	for (uint8_t i = 0; i < len; i++ ) {
    lora_write_register(lora, RegFiFo, buf[i]);
  }
	if (lora_safe_write(lora, RegPayloadLength, len) != LORA_OK) {
		return WRITE_ERROR;
	}
	lora_write_register(lora, RegOpMode, LONG_RANGE_MODE | TX_MODE);
	return LORA_OK;
}


/**
 * @brief Create LoRa initialization structure
 * 
 * @param device TRANSMITTER or RECEIVER
 * @param hspi HAL handle to SPI
 * @param GPIOxNSS 
 * @param NSS_pin 
 * @param GPIOxRST 
 * @param RST_pin 
 * @return SX1278_TypeDef 
 */
SX1278_TypeDef new_lora(TXRX_Device device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOxNSS, uint16_t NSS_pin, GPIO_TypeDef *GPIOxRST, uint16_t RST_pin)
{
	SX1278_TypeDef mp_lora;
	mp_lora.device = device;
	mp_lora.hspi = hspi;
	mp_lora.GPIOx_NSS = GPIOxNSS;
	mp_lora.NSS_pin = NSS_pin;
	mp_lora.GPIOx_RST = GPIOxRST;
	mp_lora.RST_pin = RST_pin;
	mp_lora.freqMHz = 434;
	mp_lora.bandwidth = BW_125KHz;
	mp_lora.spreading_factor = SF_11;
	mp_lora.coding_rate = CODING_RATE_4_5;
	mp_lora.syncword = 0x12;
	mp_lora.power = 17;
	mp_lora.crc = 0x00;
	return mp_lora;
}


/**
 * @brief Write single byte data to register under provided address and ensure it is written correctly.
 *
 * @param lora pointer to lora initialization structure
 * @param address 7-bit address, MSB is reserved for R/W bit
 * @param data 8-bit data to write
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 */
SX1278_Status_TypeDef lora_safe_write(SX1278_TypeDef *lora, uint8_t address, uint8_t data)
{
	uint8_t tx_buffer[2];
	uint8_t result;

	/* Write */
	address = address | 0x80;
	tx_buffer[0] = address;
	tx_buffer[1] = data;
	HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lora->hspi, tx_buffer, 2, 10);
	HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_SET);
	if (address == RegOpMode) {
		HAL_Delay(100);
	}

	/* Verify */
	address = address & 0x7F;
	HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lora->hspi, &address, 1, 10);
	HAL_SPI_Receive(lora->hspi, &result, 1, 10);
	HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_SET);

	if (result == data) {
		// debug_print("INFO: Register 0x%x expected 0x%x == 0x%x written \r\n", address, data, result);
		return LORA_OK;
	} else {
		debug_print("WARN: Register 0x%x expected 0x%x != 0x%x written. Retrying...\r\n", address, data, result);
		result = lora_read_register(lora, address);
		if (data == result) {
			// debug_print("INFO: Register 0x%x expected 0x%x == 0x%x written \r\n", address, data, result);
			return LORA_OK;
		} else {
			debug_print("ERROR: Retrying didn't help. Still read value is: %x\r\n", result);
			return WRITE_ERROR;
		}
		return WRITE_ERROR;
	}
}


/**
 * @brief Set lora frequency.
 *
 * @param lora pointer to lora initialization structure
 * @param freqMHz frequency in MHz
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 */
SX1278_Status_TypeDef lora_set_frequency(SX1278_TypeDef *lora, uint32_t freqMHz)
{
	uint32_t regvalue = (freqMHz * 524288) >> 5;
	uint8_t value;

	value = (regvalue >> 16);
	if (lora_safe_write(lora, RegFrMsb, value) != LORA_OK) {
		return WRITE_ERROR;
	}
	value = (regvalue >> 8);
	if (lora_safe_write(lora, RegFrMid, value) != LORA_OK) {
		return WRITE_ERROR;
	}
	value = (regvalue >> 0);
	if (lora_safe_write(lora, RegFrLsb, value) != LORA_OK) {
		return WRITE_ERROR;
	}
	return LORA_OK;
}


/**
 * @brief Set lora power.
 *
 * @param lora pointer to lora initialization structure
 * @param power between 2 and 20
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 */
SX1278_Status_TypeDef lora_set_power(SX1278_TypeDef *lora, uint8_t power)
{
	/* Parameter sanity check */
	if (power > 20) {
		power = 20;
	} else if (power < 2) {
		power = 2;
	}

	/* Power check */
	if (power > 17) {
		power -= 3;
		if (lora_safe_write(lora, RegPaDac, 0x87) != LORA_OK) {
			return WRITE_ERROR;
		}
		if (lora_set_ocp(lora, 140) != LORA_OK) {
			return WRITE_ERROR;
		}
	} else {
		if (lora_safe_write(lora, RegPaDac, 0x84) != LORA_OK) {
			return WRITE_ERROR;
		}
		if (lora_set_ocp(lora, 100) != LORA_OK) {
			return WRITE_ERROR;
		}
	}
	if (lora_safe_write(lora, RegPaConfig, 0x8F) != LORA_OK) { // PA_SELECT | (power - 2)
		return WRITE_ERROR;
	}
	return LORA_OK;
}


/**
 * @brief Set lora over current protection.
 *
 * @param lora pointer to lora initialization structure
 * @param power up to 240mA
 *
 * @return SX1278_Status_TypeDef - WRITE_ERROR in case the write fails, LORA_OK if all is good
 */
SX1278_Status_TypeDef lora_set_ocp(SX1278_TypeDef *lora, uint8_t mA)
{
	uint8_t ocpTrim = 27;  // max value

	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <=240) {
		ocpTrim = (mA + 30) / 10;
	}
	if (lora_safe_write(lora, RegOcp, 0x20 | (0x1F & ocpTrim)) != LORA_OK) {
		return WRITE_ERROR;
	}
	return LORA_OK;
}


/**
 * @brief Write single byte data to register under provided address
 *
 * @param lora pointer to lora initialization structure
 * @param address 7-bit address, MSB is reserved for R/W bit
 * @param data 8-bit data to write
 *
 */
void lora_write_register(SX1278_TypeDef *lora, uint8_t address, uint8_t data)
{

  uint8_t tx_buffer[2];

  address = address | 0x80;
  tx_buffer[0] = address;
  tx_buffer[1] = data;
  HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(lora->hspi, tx_buffer, 2, 10);
  HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_SET);
}


/**
 * @brief Read single byte register from the provided address
 *
 * @param lora pointer to lora initialization structure
 * @param address 7-bit address, MSB is reserved for R/W bit
 * @return uint8_t - register content
 */
uint8_t lora_read_register(SX1278_TypeDef *lora, uint8_t address)
{
  uint8_t result;
  address = address & 0x7F;
  HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(lora->hspi, &address, 1, 10);
  HAL_SPI_Receive(lora->hspi, &result, 1, 10);
//  HAL_SPI_TransmitReceive(hspi, &address, &result, 1, 10);
//  while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) ;
  HAL_GPIO_WritePin(lora->GPIOx_NSS, lora->NSS_pin, GPIO_PIN_SET);
  return result;
}


/**
 * @brief Print value of all registers
 *
 * @param lora pointer to lora initialization structure
 */
void lora_dump_registers(SX1278_TypeDef *lora)
{
	for (uint8_t i=0; i<128; i++) {
		printf("0x%x: 0x%x\r\n", i, lora_read_register(lora, i));
	}
}

/**
 * @brief Resets lora
 * 
 * @param lora pointer to lora initialization structure
 */
void lora_reset(SX1278_TypeDef *lora)
{
	HAL_GPIO_WritePin(lora->GPIOx_RST, lora->RST_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(lora->GPIOx_RST, lora->RST_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}


int16_t get_pktRSSI(SX1278_TypeDef *lora)
{
	return -164 + lora_read_register(lora, RegPktRssiValue);
}

int16_t get_RSSI(SX1278_TypeDef *lora)
{
	return -164 + lora_read_register(lora, RegRssiValue);
}

int16_t get_SNR(SX1278_TypeDef *lora)
{
	return (int16_t)lora_read_register(lora, RegRssiValue) / 4;
}

