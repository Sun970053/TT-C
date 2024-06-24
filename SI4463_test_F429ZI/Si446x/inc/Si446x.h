/*
 * Si446x.h
 *
 *  Created on: Nov 24, 2023
 *      Author: spacelab-cute-PC
 */

#ifndef INC_SI446X_H_
#define INC_SI446X_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "radio_config_Si4463.h"

/* Boot Commands */
#define POWER_UP					0x02

/* Common Commands */
#define NOP							0x00
#define PART_INFO					0x01
#define FUNC_INFO					0x10
#define SET_PROPERTY				0x11
#define GET_PROPERTY				0x12
#define GPIO_PIN_CFG				0x13
#define FIFO_INFO					0x15
#define GET_INT_STATUS				0x20
#define REQUEST_DEVICE_STATE		0x33
#define CHANGE_STATE				0x34
#define OFFLINE_RECAL				0x38
#define READ_CMD_BUFFER				0x44
#define CMD_FAST_RESPONSE_REG_A		0x50
#define CMD_FAST_RESPONSE_REG_B 	0x51
#define CMD_FAST_RESPONSE_REG_C		0x53
#define CMD_FAST_RESPONSE_REG_D		0x57

/* IR_CAL_COMMANDS */
#define IRCAL						0x17
#define IRCAL_MANUAL				0x19

/* Tx Commands */
#define START_TX					0x31
#define TX_HOP						0x37
#define WRITE_TX_FIFO				0x66

/* Rx Commands */
#define GET_PACKET_INFO 			0x16
#define GET_MODEM_STATUS 			0x22
#define START_RX					0x32
#define RX_HOP						0x36
#define READ_RX_FIFO				0x77

/* Advanced Commands */
#define GET_ADC_READING 			0x14
#define GET_PH_STATUS				0x21
#define GET_MODEM_STATUS			0x22
#define GET_CHIP_STATUS 			0x23

/* Other Commands */
#define PROTOCOL_CFG				0x18
#define PATCH_IMAGE					0x04

#define MAX_CTS_RETRY				1000
// microsecond
#define Si446x_TRANSMIT_TIMEOUT		2000
#define Si446x_RECEIVE_TIMEOUT		2000

// Property number
#define PROPERTY_GLOBAL_XO_TUNE     0x0000
#define PROPERTY_GLOBAL_CONFIG      0x0003
#define PROPERTY_INT_CTL_ENABLE     0x0100
#define PROPERTY_INT_CTL_PH_ENABLE  0x0101
#define PROPERTY_INT_CTL_MODEM_ENABLE 0x0102
#define PROPERTY_INT_CTL_CHIP_ENABLE 0x0103
#define PROPERTY_PREAMBLE_TX_LENGTH 0x1000
#define PROPERTY_PREAMBLE_CONFIG    0x1004
#define PROPERTY_SYNC_CONFIG        0x1100
#define PROPERTY_PKT_CRC_CONFIG     0x1200
#define PROPERTY_PKT_CONFIG_1       0x1206
#define PROPERTY_PKT_LEN            0x1208
#define PROPERTY_PKT_LEN_FIELD_SOURCE 0x1209
#define PKT_FIELD_1_LENGTH_12_8     0x120D
#define PKT_FIELD_1_LENGTH_7_0      0x120E
#define PKT_FIELD_1_CONFIG          0x120F
#define PKT_FIELD_1_CRC_CONFIG      0x1210
#define PKT_FIELD_2_LENGTH_12_8     0x1211
#define PKT_FIELD_2_LENGTH_7_0      0x1212
#define PKT_FIELD_2_CONFIG          0x1213
#define PKT_FIELD_2_CRC_CONFIG      0x1214
#define PKT_FIELD_3_LENGTH_12_8     0x1215
#define PKT_FIELD_3_LENGTH_7_0      0x1216
#define PKT_FIELD_3_CONFIG          0x1217
#define PKT_FIELD_3_CRC_CONFIG      0x1218
#define PKT_FIELD_4_LENGTH_12_8     0x1219
#define PKT_FIELD_4_LENGTH_7_0      0x121A
#define PKT_FIELD_4_CONFIG          0x121B
#define PKT_FIELD_4_CRC_CONFIG      0x121C
#define PKT_FIELD_5_LENGTH_12_8     0x121D
#define PKT_FIELD_5_LENGTH_7_0      0x121E
#define PKT_FIELD_5_CONFIG          0x121F
#define PKT_FIELD_5_CRC_CONFIG      0x1220
#define PROPERTY_PA_MODE            0x2200
#define PROPERTY_PA_PWR_LVL         0x2201
#define PROPERTY_PA_BIAS_CLKDUTY    0x2202
#define PROPERTY_PA_TC              0x2203

//#define GPIO_PIN_CFG                0x13
typedef enum
{
	GPIO_NO_CHANGE,
	GPIO_DISABLE,
	GPIO_OUPPUT_LOW,
	GPIO_OUTPUT_HIGH,
	GPIO_INPUT,
	GPIO_32_KHZ_CLOCK,
	GPIO_DATA_OUT = 11,
	GPIO_TX_STATE = 32,
	GPIO_RX_STATE = 33,
	GPIO_INT_SIGNAL = 39
}gpio_mode;

//#define PROPERTY_INT_CTL_ENABLE     0x0100
#define CHIP_INT_STATUS_EN          0x04
#define MODEM_INT_STATUS_EN         0x02
#define PH_INT_STATUS_EN            0x01

//#define PROPERTY_INT_CTL_PH_ENABLE  0x0101
#define FILTER_MATCH_EN             0x80
#define FILTER_MISS_EN              0x40
#define PACKET_SENT_EN              0x20
#define PACKET_RX_EN                0x10
#define CRC_ERROR_EN                0x08
#define TX_FIFO_ALMOST_EMPTY_EN     0x02
#define RX_FIFO_ALMOST_FULL_EN      0x01

//#define PROPERTY_INT_CTL_MODEM_ENABLE 0x0102
#define INVALID_SYNC_EN             0x20
#define RSSI_JUMP_EN                0x10
#define RSSI_EN                     0x08
#define INVALID_PREAMBLE_EN         0x04
#define PREAMBLE_DETECT_EN          0x02
#define SYNC_DETECT_EN              0x01

//#define PROPERTY_INT_CTL_CHIP_ENABLE 0x0103
#define FIFO_UNDERFLOW_OVERFLOW_ERROR_EN 0x20
#define STATE_CHANGE_EN             0x10
#define CMD_ERROR_EN                0x08
#define CHIP_READY_EN               0x04
#define LOW_BATT_EN                 0x02
#define WUT_EN                      0x01

//#define PROPERTY_PREAMBLE_CONFIG    0x1004
#define PREAM_FIRST_1               0x20
#define PREAM_FIRST_0               0x00
#define LENGTH_CONFIG_NIBBLES       0x00
#define LENGTH_CONFIG_BYTES         0x10
#define MAN_CONST                   0x08
#define MAN_ENABLE                  0x04
#define STANDARD_PREAM_NON          0x00
#define STANDARD_PREAM_1010         0x01
#define STANDARD_PREAM_0101         0x02

//#define PROPERTY_SYNC_CONFIG        0x1100
#define SYNC_SKIP_TX                0x80
#define SYNC_RX_ERRORS_MASK         0x70
#define SYNC_4FSK                   0x08
#define SYNC_MANCH                  0x04
#define SYNC_LENGTH_MASK            0x03

//#define PROPERTY_PKT_CRC_CONFIG     0x1200
#define CRC_SEED_ALL_0S             0x00
#define CRC_SEED_ALL_1S             0x80
#define CRC_POLYNOMIAL_MASK         0x0F

typedef enum
{
	NO_CRC =            0x00,
	ITU_T_CRC8 =        0x01,
	IEC_16 =            0x02,
	BAICHEVA_16 =       0x03,
	CRC_16 =            0x04,
	CCIT_16 =           0x05,
	KOOPMAN =           0x06,
	IEEE_802_3 =        0x07,
	CASTAGNOLI =        0x08
}property_crc_poly;

//#define PROPERTY_PKT_CONFIG_1       0x1206
#define PH_FIELD_SPLIT              0x80
#define PH_RX_DISABLE               0x40
#define MODEM_4FSK_EN               0x20
#define RX_MULTI_PKT                0x10
#define MANCH_POL                   0x08
#define CRC_INVERT                  0x04
#define CRC_BIG_ENDIAN              0x02
#define BIT_ORDER                   0x01

//#define PROPERTY_PKT_LEN            0x1208
#define PKT_LITTLE_ENDIAN           0x00
#define PKT_BIG_ENDIAN              0x20
#define PKT_SIZE_1_BYTE             0x00
#define PKT_SIZE_2_BYTES            0x10
#define PKT_IN_FIFO                 0x08
#define PKT_DST_FIEL_MASK           0x07

typedef enum
{
	DST_FIELD_ENUM_0,
	DST_FIELD_ENUM_1,
	DST_FIELD_ENUM_2,
	DST_FIELD_ENUM_3,
	DST_FIELD_ENUM_4,
	DST_FIELD_ENUM_5,
	DST_FIELD_ENUM_6,
	DST_FIELD_ENUM_7
}pkt_len_dst;

//#define PROPERTY_PKT_LEN_FIELD_SOURCE 0x1209
typedef enum
{
	SRC_FIELD_ENUM_0,
	SRC_FIELD_ENUM_1,
	SRC_FIELD_ENUM_2,
	SRC_FIELD_ENUM_3,
	SRC_FIELD_ENUM_4,
	SRC_FIELD_ENUM_5,
}pkt_len_fleid_src;

//#define PKT_FIELD_1_CONFIG          0x120F
//#define PKT_FIELD_2_CONFIG          0x1213
//#define PKT_FIELD_3_CONFIG          0x1217
//#define PKT_FIELD_4_CONFIG          0x121B
//#define PKT_FIELD_5_CONFIG          0x121F
#define FIELD_CONFIG_4FSK           0x10
#define FIELD_CONFIG_PN_START       0x04
#define FIELD_CONFIG_WHITEN         0x02
#define FIELD_CONFIG_MANCH          0x01

//#define PKT_FIELD_1_CRC_CONFIG      0x1210
//#define PKT_FIELD_2_CRC_CONFIG      0x1214
//#define PKT_FIELD_3_CRC_CONFIG      0x1218
//#define PKT_FIELD_4_CRC_CONFIG      0x121C
//#define PKT_FIELD_5_CRC_CONFIG      0x1220
#define FIELD_CRC_CONFIG_CRC32_START  0x80
#define FIELD_CRC_CONFIG_SEND_CRC32   0x20
#define FIELD_CRC_CONFIG_CHECK_CRC32  0x08
#define FIELD_CRC_CONFIG_CRC32_ENABLE 0x02

typedef enum
{
	NO_ERROR = 			0x00,
	BAD_CMD_ISSUE = 	0x10,
	ARGS_INVALID = 		0x11,
	PREV_CMD_UNDONE = 	0x12,
	BAD_PROP_ID = 		0x40
}cmd_err_status;

//#define START_TX					0x31
typedef enum
{
	NO_CHANGE = 		0x0,
	SLEEP_STATE = 		0x1,
	SPI_ACTIVE_STATE = 	0x2,
	READY_STATE = 		0x3,
	READY_STATE_2 = 	0x4,
	TUNE_STATE_FOR_TX = 0x5,
	TUNE_STATE_FOR_RX = 0x6,
	TX_STATE = 			0x7,
	RX_STATE =			0x8
}txcomplete_state;

/* Initialize Si446x */
void Si446x_set_pin();
uint8_t Si446x_init(SPI_HandleTypeDef* hspi);
uint8_t Si446x_part_info(void);
uint8_t Si446x_power_on_reset(void);
uint8_t Si446x_enter_standby_mode(void);
uint8_t Si446x_set_SyncWords(uint8_t synclen, uint8_t* syncdata);
uint8_t Si446x_set_CRC(uint8_t crcSeed, uint8_t crcPoly);
uint8_t Si446x_set_tx_power(uint8_t power);
uint8_t Si446x_transmit(uint8_t txlen, uint8_t* txdata);
uint8_t Si446x_receive_init(void);
uint8_t Si446x_receive(uint8_t rxlen, uint8_t* rxdata);
uint8_t Si446x_get_chip_status(void);
uint8_t Si446x_get_func_status(void);
uint8_t Si446x_get_pckt_handler_status(void);
uint8_t Si446x_get_modem_status(void);
uint8_t Si446x_get_fifo_info(void);
uint8_t Si446x_request_device_state(void);
uint8_t Si446x_change_state(void);
#endif /* INC_SI446X_H_ */
