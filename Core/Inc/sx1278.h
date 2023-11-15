/**
 * @file sx1278.h
 * @author Michal Piekos (michal.public@wp.pl)
 * @brief Transmitter library with devices related functions
 * @version 0.1
 * @date 2023-08-24
 *
 * MIT License. Copyright (c) 2023.
*/
#ifndef __MP_SX1278_H__
#define __MP_SX1278_H__


/*****************************************************************
 * Includes
******************************************************************/
#if defined(STM32G030xx) || defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif
#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#endif
#ifdef STM32F411xE
#include "stm32f4xx_hal.h"
#endif

#include "stdio.h"
#include "common.h"

/*****************************************************************
 * Variables and constants
******************************************************************/

//------ LoRa registers ------//

#define RegFiFo								0x00
#define RegOpMode							0x01
#define RegFrMsb							0x06
#define RegFrMid							0x07
#define RegFrLsb							0x08
#define RegPaConfig							0x09
#define RegOcp								0x0B
#define RegLna								0x0C
#define RegFiFoAddrPtr						0x0D
#define RegFiFoTxBaseAddr					0x0E
#define RegFiFoRxBaseAddr					0x0F
#define RegFiFoRxCurrentAddr				0x10
#define RegIrqFlags							0x12
#define RegRxNbBytes						0x13
#define RegModemStat						0x18
#define RegPktSnrValue						0x19
#define RegPktRssiValue						0x1A
#define RegRssiValue						0x1B
#define	RegModemConfig1						0x1D
#define RegModemConfig2						0x1E
#define RegSymbTimeoutL						0x1F
#define RegPreambleMsb						0x20
#define RegPreambleLsb						0x21
#define RegPayloadLength					0x22
#define RegModemConfig3						0x26
#define RegSyncWord							0x39
#define RegDioMapping1						0x40
#define RegDioMapping2						0x41
#define RegVersion							0x42
#define RegPaDac							0x4D

//--- Hardcoded values ----//

#define FREQ_MSB                            0x6C		// Values for 434MHz
#define FREQ_MID                            0x80
#define FREQ_LSB                            0x00
#define OVERCURRENT_PROTECTION              0x3B    // 0x2F for 120mA, 0x3B for 240mA (max)

//--------- Modes ---------//

#define SLEEP_MODE                          0x00
#define STNBY_MODE                          0x01
#define FSTX_MODE                           0x02
#define TX_MODE                             0x03
#define FSRX_MODE                           0x04
#define RXCONTIN_MODE                       0x05
#define RXSINGLE_MODE                       0x06
#define CAD_MODE                            0x07
#define LONG_RANGE_MODE     		    	0x80
#define LOW_FREQUENCY_MODE_ON               0x08

//------- Config -------//

#define BW_7_8KHz				      		0x00
#define BW_10_4KHz					  		0x10
#define BW_15_6KHz				       		0x20
#define BW_20_8KHz				       		0x30
#define BW_31_25KHz				       		0x40
#define BW_41_7KHz				       		0x50
#define BW_62_5KHz				       		0x60
#define BW_125KHz   					    0x70
#define BW_250KHz	    				    0x80
#define BW_500KHz		    			    0x90

#define CODING_RATE_4_5						0x02
#define CODING_RATE_4_6						0x04
#define CODING_RATE_4_7						0x06
#define CODING_RATE_4_8						0x08

#define IMPLICIT_HEADER_MODE_ON				0x01

#define SF_6								0x60
#define SF_7								0x70
#define SF_8								0x80
#define SF_9								0x90
#define SF_10								0xA0
#define SF_11								0xB0
#define SF_12								0xC0

#define TX_CONTINUOUS_MODE					0x08
#define RX_PAYLOAD_CRC_ON					0x04	// In explicit mode enable only on RX side

#define RX_TIMEOUT_MSB						0x03
#define RX_TIMEOUT_LSB						0xFF

#define LOW_DATA_RATE_OPTIMIZE				0x08
#define AGC_AUTO_ON							0x04

#define LNA_GAIN_G1							0x20
#define LNA_GAIN_G2							0x40
#define LNA_GAIN_G3							0x60
#define LNA_GAIN_G4							0x80
#define LNA_GAIN_G5							0xA0
#define LNA_GAIN_G6							0xC0
#define LNA_BOOST_HF						0x03

#define PA_SELECT							0x80

#define SYNC_WORD							0xBE

#define DIO_MAP_TXDONE						0x40
#define DIO_MAP_RXDONE						0x00

//------ IRQ masks ------//

#define IRQ_TX_DONE_MASK			        0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK	        0x20
#define IRQ_RX_DONE_MASK			        0x40
#define IRQ_CLEAR_MASK						0xFF

/*****************************************************************
 * Exported Types
 *****************************************************************/

typedef enum
{
  LORA_OK = 0x00U,
  WRITE_ERROR = 0x01U
} SX1278_Status_TypeDef;

typedef enum
{
  TRANSMITTER = 0x00U,
  RECEIVER = 0x01U
} TXRX_Device;

typedef struct {
	TXRX_Device device;
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *GPIOx_NSS;
	uint16_t NSS_pin;
	GPIO_TypeDef *GPIOx_RST;
	uint16_t RST_pin;
	uint32_t freqMHz;
	uint8_t spreading_factor;
	uint8_t bandwidth;
	uint8_t syncword;
	uint8_t coding_rate;
	uint8_t power;
	uint8_t crc;
} SX1278_TypeDef;

/*****************************************************************
 * Function prototypes
******************************************************************/

SX1278_TypeDef new_lora(TXRX_Device device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOxNSS, uint16_t NSS_pin, GPIO_TypeDef *GPIOxRST, uint16_t RST_pin);
void lora_reset(SX1278_TypeDef *lora);
uint8_t lora_read_register(SX1278_TypeDef *lora, uint8_t address);
void lora_write_register(SX1278_TypeDef *lora, uint8_t address, uint8_t data);
void lora_dump_registers(SX1278_TypeDef *lora);
SX1278_Status_TypeDef lora_safe_write(SX1278_TypeDef *lora, uint8_t address, uint8_t data);
SX1278_Status_TypeDef lora_set_frequency(SX1278_TypeDef *lora, uint32_t freqMHz);
SX1278_Status_TypeDef lora_set_power(SX1278_TypeDef *lora, uint8_t power);
SX1278_Status_TypeDef lora_set_ocp(SX1278_TypeDef *lora, uint8_t mA);
SX1278_Status_TypeDef lora_init(SX1278_TypeDef *lora);
SX1278_Status_TypeDef lora_send(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len);
uint8_t lora_receive(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len);
uint8_t lora_receive_IT(SX1278_TypeDef *lora, uint8_t *buf, uint8_t len);
int16_t get_SNR(SX1278_TypeDef *lora);
int16_t get_RSSI(SX1278_TypeDef *lora);
int16_t get_pktRSSI(SX1278_TypeDef *lora);


#endif /* __MP_SX1278_H__ */
