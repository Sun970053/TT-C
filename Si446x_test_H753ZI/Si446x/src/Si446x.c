/*
 * Si443x.c
 *
 *  Created on: Nov 24, 2023
 *      Author: spacelab-cute-PC
 */

#include "Si446x.h"
#include <string.h>
#include <stdio.h>

SPI_HandleTypeDef* Si446x_hspi;
extern TIM_HandleTypeDef htim1;

// Configuration parameters from "config_Si4463.h"
uint8_t RF4463_CONFIGURATION_DATA[] = RADIO_CONFIGURATION_DATA_ARRAY;

uint8_t Si446x_SendCommand(uint8_t cmdLength, uint8_t* cmdData);
uint8_t Si446x_WaitforCTS();
uint8_t Si446x_GetResponse(uint8_t RespLength, uint8_t* RespData);
uint8_t Si446x_WriteTxDataBuffer(uint8_t TxFifoLength, uint8_t* TxFifoData);
uint8_t Si446x_ReadRxDataBuffer(uint8_t RxFifoLength, uint8_t* RxFifoData);
uint8_t Si446x_GetFastResponseRegister(uint8_t StartRegs, uint8_t RegsNum, uint8_t* RegValues);
uint8_t Si446x_Configuration(uint8_t* configArray);
uint8_t Si446x_SetProperties(uint16_t propertyNum, uint8_t cmdLength, uint8_t* cmdData);
uint8_t Si446x_GetProperties(uint16_t propertyNum, uint8_t cmdLength, uint8_t* cmdData);
uint8_t Si446x_FifoReset();
uint8_t Si446x_TxInterrupt();
uint8_t Si446x_ClearInterrupt();
uint8_t Si446x_Start_Tx();
uint8_t Si446x_Start_Rx();
uint8_t Si446x_RxInterrupt();

void Delay_us(uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1)) < time);
}

uint8_t Si446x_init(SPI_HandleTypeDef* hspi)
{
	uint8_t buff[12] = {0};
		Si446x_hspi = hspi;
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_SET);
		Delay_us(10);
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);
		// Wait for POR (Power on reset)
		while(HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin) == GPIO_PIN_RESET);
		//Delay_us(10000);
		// Start the radio
		Si446x_Configuration(RF4463_CONFIGURATION_DATA);
		// setting of GPIO
		buff[0] = GPIO_PIN_CFG;
		buff[1] = GPIO_NO_CHANGE;
		buff[2] = GPIO_NO_CHANGE;
		buff[3] = GPIO_RX_STATE;
		buff[4] = GPIO_TX_STATE;
		buff[5] = GPIO_INT_SIGNAL;
		buff[6] = GPIO_DATA_OUT;
		if(!Si446x_SendCommand(7, buff)) return 0;
		//if(!Si446x_GetResponse(7, buff)) return 0;
		//frequency adjust
//		buff[0] = 82;
//		Si446x_SetProperties(PROPERTY_GLOBAL_XO_TUNE, 1, buff);
//		//
//		buff[0] = 0x40;
//		Si446x_SetProperties(PROPERTY_GLOBAL_CONFIG, 1, buff);

		// set preamble
//		buff[0] = 0x08;
//		buff[1] = 0x14;
//		buff[2] = 0x00;
//		buff[3] = 0x0f;
//		buff[4] = PREAM_FIRST_1|LENGTH_CONFIG_BYTES|STANDARD_PREAM_1010;
//		buff[5] = 0x00;
//		buff[6] = 0x00;
//		buff[7] = 0x00;
//		buff[8] = 0x00;
//		Si446x_SetProperties(PROPERTY_PREAMBLE_TX_LENGTH, 9, buff);

		// get preamble
		Si446x_GetProperties(PROPERTY_PREAMBLE_TX_LENGTH, 9, buff);
		// Set SyncWords
//		buff[0] = 0x7E;
//		buff[1] = 0x7E;
//		Si446x_set_SyncWords(2, buff);
		// Set CRC
//		buff[0] = CRC_SEED_ALL_1S | CCIT_16;
//		Si446x_SetProperties(PROPERTY_PKT_CRC_CONFIG, 1, buff);
//		buff[0] = CRC_BIG_ENDIAN;
//		Si446x_SetProperties(PROPERTY_PKT_CONFIG_1, 1, buff);

		// Set Pckt
//		buff[0] = PKT_IN_FIFO | DST_FIELD_ENUM_2;
//		buff[1] = SRC_FIELD_ENUM_1;
//		buff[2] = 0x00;
//		Si446x_SetProperties(PROPERTY_PKT_LEN, 3, buff);

		//set length of field, set field 2 as data field.
//		buff[0] = 0x00;
//		buff[1] = 0x01;
//		buff[2] = FIELD_CONFIG_PN_START;
//		buff[3] = FIELD_CRC_CONFIG_CRC32_START | FIELD_CRC_CONFIG_SEND_CRC32 |
//				FIELD_CRC_CONFIG_CHECK_CRC32 | FIELD_CRC_CONFIG_CRC32_ENABLE;
//		buff[2] = 0x00;
//		buff[3] = 0x00;
//		buff[4] = 0x00;
//		buff[5] = 50;
//		buff[6] = FIELD_CONFIG_PN_START;
//		buff[7] = FIELD_CRC_CONFIG_CRC32_START | FIELD_CRC_CONFIG_SEND_CRC32 |
//				FIELD_CRC_CONFIG_CHECK_CRC32 | FIELD_CRC_CONFIG_CRC32_ENABLE;
//		buff[6] = 0x00;
//		buff[7] = 0x00;
//		buff[8] = 0x00;
//		buff[9] = 0x00;
//		buff[10] = 0x00;
//		buff[11] = 0x00;
//		Si446x_SetProperties(PKT_FIELD_1_LENGTH_12_8, 12, buff);
//
//		buff[0] = 0x00;
//		buff[1] = 0x00;
//		buff[2] = 0x00;
//		buff[3] = 0x00;
//		buff[4] = 0x00;
//		buff[5] = 0x00;
//		buff[6] = 0x00;
//		buff[7] = 0x00;
//		Si446x_SetProperties(PKT_FIELD_4_LENGTH_12_8, 8, buff);

		//set tx power
		Si446x_set_tx_power(20);

		return 1;
}

uint8_t Si446x_power_on_reset(void)
{
	HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_SET);
	Delay_us(10);
	HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);
	// Wait for POR (Power on reset)
	while(HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin) == GPIO_PIN_RESET);
	// Power on
	uint8_t buff[] = {RF_POWER_UP};
	Si446x_SendCommand(sizeof(buff), buff);
	Si446x_WaitforCTS();
	return 1;
}

uint8_t Si446x_enter_standby_mode(void)
{
	uint8_t buff[2];
	buff[0] = CHANGE_STATE;
	buff[1] = SLEEP_STATE;

	if(!Si446x_SendCommand(sizeof(buff), buff)) return 0;
	if(!Si446x_WaitforCTS()) return 0;
	return 1;
}

uint8_t Si446x_set_SyncWords(uint8_t synclen, uint8_t* syncdata)
{
	uint8_t buff[5] = {0};
	buff[0] = SYNC_LENGTH_MASK & (synclen - 1);
	memcpy(&buff[1], syncdata, synclen);
	return Si446x_SetProperties(PROPERTY_SYNC_CONFIG, synclen + 1, buff);
}

uint8_t Si446x_set_CRC(uint8_t crcSeed, uint8_t crcPoly)
{
	if((CRC_POLYNOMIAL_MASK & crcPoly) > 8) return 0;
	uint8_t buff = crcSeed | (CRC_POLYNOMIAL_MASK & crcPoly);
	return Si446x_SetProperties(PROPERTY_PKT_CRC_CONFIG, 1, &buff);
}

uint8_t Si446x_set_tx_power(uint8_t power)
{
	// Range: 0-127
	if(power > 127) return 0;
	uint8_t buff[4];
	buff[0] = 0x08;
	buff[1] = power;
	buff[2] = 0x00;
	buff[3] = 0x3D;

	return Si446x_SetProperties(PROPERTY_PA_MODE, 4, buff);
}

uint8_t Si446x_transmit(uint8_t txlen, uint8_t* txdata)
{
	if(!Si446x_FifoReset()) return 0;
	if(!Si446x_WriteTxDataBuffer(txlen, txdata)) return 0;
	if(!Si446x_TxInterrupt()) return 0;
	if(!Si446x_ClearInterrupt()) return 0;
	if(!Si446x_Start_Tx()) return 0;

	int counter = 0;
	while(counter < Si446x_TRANSMIT_TIMEOUT)
	{
		if(HAL_GPIO_ReadPin(IRQ_GPIO_Port ,IRQ_Pin) == GPIO_PIN_RESET)
			return 1;
		Delay_us(1000);
		counter++;
	}
	Si446x_init(Si446x_hspi);

	return 0;
}

uint8_t Si446x_receive_init()
{
	uint8_t length = 50;
	if(!Si446x_SetProperties(PKT_FIELD_2_LENGTH_7_0, sizeof(length), &length)) return 0;
	if(!Si446x_FifoReset()) return 0;
	if(!Si446x_RxInterrupt()) return 0;
	if(!Si446x_ClearInterrupt()) return 0;
	if(!Si446x_Start_Rx()) return 0;
	return 1;
}

uint8_t Si446x_receive(uint8_t rxlen, uint8_t* rxdata)
{
	if(!Si446x_ReadRxDataBuffer(rxlen, rxdata)) return 0;
	if(!Si446x_FifoReset()) return 0;
	return 1;
}

uint8_t Si446x_get_chip_status()
{
	uint8_t Cmd = GET_CHIP_STATUS;
	uint8_t rxbuff[3];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(3, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x %02x\r\n", rxbuff[0], rxbuff[1], rxbuff[2]);
	return 1;
}

uint8_t Si446x_get_modem_status()
{
	uint8_t Cmd = GET_MODEM_STATUS;
	uint8_t rxbuff[6];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(6, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5]);
	return 1;
}

uint8_t Si446x_part_info()
{
	uint8_t Cmd = PART_INFO;
	uint8_t rxbuff[8];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(8, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5], rxbuff[6], rxbuff[7]);
	if((uint16_t)((rxbuff[1] << 8) | rxbuff[2]) != 0x4463) return 0;
	else return 1;
}

uint8_t Si446x_get_fifo_info()
{
	uint8_t Cmd[2];
	uint8_t rxbuff[2];

	Cmd[0] = FIFO_INFO;
	Cmd[1] = 0x03;

	if(Si446x_SendCommand(2, &Cmd[0]) == 0) return 0;
	if(Si446x_GetResponse(2, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x\r\n",
			rxbuff[0], rxbuff[1]);
	return 1;
}

uint8_t Si446x_get_func_status()
{
	uint8_t Cmd = FUNC_INFO;
	uint8_t rxbuff[6];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(6, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5]);
	return 1;
}

uint8_t Si446x_get_pckt_handler_status()
{
	uint8_t Cmd = GET_PH_STATUS;
	uint8_t rxbuff[2];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(2, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x\r\n",
			rxbuff[0], rxbuff[1]);
	return 1;
}

uint8_t Si446x_request_device_state()
{
	uint8_t Cmd = REQUEST_DEVICE_STATE;
	uint8_t rxbuff[4];

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	if(Si446x_GetResponse(4, rxbuff) == 0) return 0;
	printf("rxbuff: %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3]);
	return 1;
}

uint8_t Si446x_change_state()
{
	uint8_t Cmd = CHANGE_STATE;

	if(Si446x_SendCommand(1, &Cmd) == 0) return 0;
	return 1;
}

uint8_t Si446x_SendCommand(uint8_t cmdLength, uint8_t* cmdData)
{
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Si446x_hspi,  cmdData, cmdLength, Si446x_TRANSMIT_TIMEOUT);
	//while (HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t Si446x_WaitforCTS(){
	uint8_t output_address[2] = {0};
	uint8_t CtsValue[2] = {0};
	uint16_t ErrCnt = 0;

	while(CtsValue[1] != 0xFF)
	{
		if(++ErrCnt > MAX_CTS_RETRY)
		{
			return 0;
		}
		output_address[0] = READ_CMD_BUFFER;
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
		//HAL_SPI_Transmit(Si446x_hspi, &output_address, 1, Si446x_TRANSMIT_TIMEOUT);
		//while (HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY);
		//HAL_SPI_Receive(Si446x_hspi, &CtsValue, 1, Si446x_RECEIVE_TIMEOUT);
		//while (HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY);
		HAL_SPI_TransmitReceive(Si446x_hspi, &output_address[0], &CtsValue[0], 2, 1000);
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);

	}
	return 1;
}

uint8_t Si446x_GetResponse(uint8_t RespLength, uint8_t* RespData){
	uint8_t output_address[2] = {0};
	uint8_t CtsValue[2] = {0};
	output_address[0] = READ_CMD_BUFFER;
	if (!Si446x_WaitforCTS()) return 0;
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(Si446x_hspi, &output_address[0], &CtsValue[0], 2, 1000);
	HAL_SPI_Receive(Si446x_hspi, RespData, RespLength, Si446x_RECEIVE_TIMEOUT);
	//while (HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t Si446x_WriteTxDataBuffer(uint8_t TxFifoLength, uint8_t* TxFifoData)
{
	if(!Si446x_SetProperties(PKT_FIELD_2_LENGTH_7_0, sizeof(TxFifoLength), &TxFifoLength)) return 0;
	uint8_t buff[TxFifoLength + 2];
	buff[0] = WRITE_TX_FIFO;
	buff[1] = TxFifoLength;
	memcpy(&buff[2], TxFifoData, TxFifoLength);
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Si446x_hspi, buff, sizeof(buff), Si446x_TRANSMIT_TIMEOUT);
	if(HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY) return 0;
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t Si446x_ReadRxDataBuffer(uint8_t RxFifoLength, uint8_t* RxFifoData)
{
	uint8_t output_address = READ_RX_FIFO;

	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Si446x_hspi, &output_address, 1, Si446x_TRANSMIT_TIMEOUT);
	if(HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY) return 0;
	HAL_SPI_Receive(Si446x_hspi, RxFifoData, RxFifoLength, Si446x_RECEIVE_TIMEOUT);
	if(HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY) return 0;
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t Si446x_GetFastResponseRegister(uint8_t StartRegs, uint8_t RegsNum, uint8_t* RegValues)
{
	if((RegsNum == 0) || (RegsNum > 4)) return 0;
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(Si446x_hspi, &StartRegs, 1, Si446x_TRANSMIT_TIMEOUT);
	if(HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY) return 0;
	HAL_SPI_Receive(Si446x_hspi, RegValues, RegsNum, Si446x_RECEIVE_TIMEOUT);
	if(HAL_SPI_GetState(Si446x_hspi) != HAL_SPI_STATE_READY) return 0;
	HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	return 1;
}

uint8_t Si446x_Configuration(uint8_t* configArray)
{
	uint16_t index = 0, currentNum = 0;
	uint16_t propertyNum = 0;
	while(configArray[index])
	{
		currentNum = configArray[index];

		if(Si446x_SendCommand(currentNum, &configArray[index + 1]) == 0) return 0;
		if(Si446x_WaitforCTS() == 0) return 0;
		if(configArray[index + 1] == 0x11)
		{
			propertyNum = (configArray[index + 2] << 8) | configArray[index + 4];
			printf("Property number: 0x%04x\r\n", propertyNum);
		}
		else
		{
			propertyNum = configArray[index + 1];
			printf("Command number: 0x%02x\r\n", propertyNum);
		}

		index = index + currentNum + 1;
	}
	return 1;
}

uint8_t Si446x_SetProperties(uint16_t propertyNum, uint8_t cmdLength, uint8_t* cmdData)
{
	uint8_t buffer[16] = {0};
	// SET_PROPERTY
	buffer[0] = SET_PROPERTY;
	// GROUP[7:0]
	buffer[1] = propertyNum>>8;
	//  NUM_PROPS[7:0]
	buffer[2] = cmdLength;
	//  START_PROP[7:0]
	buffer[3] = (uint8_t)(propertyNum & 0xff);
	// copy the remain DATA
	memcpy(&buffer[4], cmdData, cmdLength);
	if(!Si446x_SendCommand(cmdLength + 4, buffer)) return 0;
	if(!Si446x_WaitforCTS()) return 0;
	else
	{
//		printf("--------------------------\r\n");
//		printf("Property number: 0x%04x\r\n", propertyNum);
	}
	return 1;
}

uint8_t Si446x_GetProperties(uint16_t propertyNum, uint8_t cmdLength, uint8_t* cmdData)
{
	uint8_t buff[4];
	buff[0] = GET_PROPERTY;
	buff[1] = (uint8_t)(propertyNum >> 8);
	buff[2] = cmdLength;
	buff[3] = (uint8_t)(propertyNum & 0xff);

	if(!Si446x_SendCommand(4, buff)) return 0;
	if(!Si446x_GetResponse(cmdLength, cmdData)) return 0;
	else{
		printf("--------------------------\r\n");
		printf("Get property data: ");
		for(int i = 0; i < cmdLength;i++)
		{
			printf("0x%02x ", cmdData[i]);
		}
		printf("\r\n");
	}
	return 1;
}

uint8_t Si446x_FifoReset()
{
	uint8_t buff[] = {FIFO_INFO, 0x03};
	if(Si446x_SendCommand(sizeof(buff), buff) == 0) return 0;
	if(Si446x_WaitforCTS() == 0) return 0;
	return 1;
}

uint8_t Si446x_TxInterrupt()
{
	uint8_t buff[4] = {0};
	buff[0] = PH_INT_STATUS_EN;
	buff[1] = PACKET_SENT_EN;
	buff[2] = 0x00;
	buff[3] = CHIP_READY_EN;
	return Si446x_SetProperties(PROPERTY_INT_CTL_ENABLE, sizeof(buff), buff);
}

uint8_t Si446x_ClearInterrupt()
{
	uint8_t buff[4];
	buff[0] = GET_INT_STATUS;
	buff[1] = 0;
	buff[2] = 0;
	buff[3] = 0;
	if(Si446x_SendCommand(sizeof(buff), buff) == 0) return 0;
	if(Si446x_WaitforCTS() == 0) return 0;
	return 1;
}

uint8_t Si446x_Start_Tx()
{
	uint8_t buff[5] = {0};
	buff[0] = START_TX;
	// channel
	buff[1] = 0x00;
	// tx complete state
	buff[2] = (uint8_t)(READY_STATE << 4);
	if(Si446x_SendCommand(sizeof(buff), buff)  == 0) return 0;
	if(Si446x_WaitforCTS() == 0) return 0;
	return 1;
}

uint8_t Si446x_RxInterrupt()
{
	uint8_t buff[4] = {0};
	buff[0] = MODEM_INT_STATUS_EN;
	buff[1] = PACKET_RX_EN | CRC_ERROR_EN;
	buff[2] = 0x00;
	buff[3] = CHIP_READY_EN;
	return Si446x_SetProperties(PROPERTY_INT_CTL_ENABLE, sizeof(buff), buff);
}

uint8_t Si446x_Start_Rx()
{
	uint8_t buff[8] = {0};
	buff[0] = START_RX;
	buff[1] = 0;
	buff[2] = 0;
	buff[3] = 0;
	buff[4] = 0;
	buff[5] = 0;
	buff[6] = 0x08;
	buff[7] = 0x08;

	if(Si446x_SendCommand(sizeof(buff), buff)  == 0) return 0;
	if(Si446x_WaitforCTS() == 0) return 0;
	return 1;
}

uint8_t* Si446x_nrzi_encode(uint8_t* input, uint8_t len)
{
	uint8_t* output = (uint8_t*)malloc(len*sizeof(uint8_t));
	uint8_t  nrz, nrzi, prevnrzi = 0;
	for(int i = 0; i < len; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			nrz = (input[i] >> j) & 0x01;
			if(nrz)
				nrzi = prevnrzi;
			else
				nrzi = ~prevnrzi;

			if(nrzi)
				output[i] |= (1 << j);
			else
				output[i] &= ~(1 << j);
			prevnrzi = nrzi;
		}
	}
	return output;
}

uint8_t* Si446x_nrzi_decode(uint8_t* input, uint8_t len)
{
	uint8_t* output = (uint8_t*)malloc(len*sizeof(uint8_t));
	uint8_t nrz, nrzi, prevnrzi = 0;
	for(int i = 0; i < len; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			nrzi = (input[i] >> j) & 0x01;
			nrz = (~(nrzi ^ prevnrzi)) & 0x01;
			if(nrz)
				output[i] |= (1 << j);
			else
				output[i] &= ~(1 << j);
			prevnrzi = nrzi;
		}
	}
	return output;
}

uint8_t* Si446x_g3ruh_scrambler(uint8_t* input, uint8_t len)
{
	uint8_t* output = (uint8_t*)malloc(len*sizeof(uint8_t));
	uint8_t unscrambled_bit, scrambled_bit;
	uint32_t LFSR = 0;
	for(int i = 0; i < len; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			unscrambled_bit = (input[i] >> j) & 0x01;
			scrambled_bit = ((LFSR >> 11) & 0x01) ^ ((LFSR >> 16) & 0x01) ^ unscrambled_bit;
			if(scrambled_bit)
				output[i] |= (1 << j);
			else
				output[i] &= ~(1 << j);
			LFSR = (LFSR << 1) | scrambled_bit;
		}
	}
	return output;
}

uint8_t* Si446x_g3ruh_descrambler(uint8_t* input, uint8_t len)
{
	uint8_t* output = (uint8_t*)malloc(len*sizeof(uint8_t));
	uint8_t unscrambled_bit, scrambled_bit;
	uint32_t LFSR = 0;
	for(int i = 0; i < len; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			scrambled_bit = (input[i] >> j) & 0x01;
			unscrambled_bit = ((LFSR >> 11) & 0x01) ^ ((LFSR >> 16) & 0x01) ^ scrambled_bit;
			if(unscrambled_bit)
				output[i] |= (1 << j);
			else
				output[i] &= ~(1 << j);
			LFSR = (LFSR << 1) | scrambled_bit;
		}
	}
	return output;
}
