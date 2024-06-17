/*
 * si4463_huang.c
 *
 *  Created on: June 13, 2024
 *      Author: Ting-Shan, Huang
 */

#include "si4463_huang.h"
#include <string.h>
#include <stdio.h>

int8_t si4463_sendCommand(si4463_t* si4463, uint8_t* cmdData, uint8_t cmdLen);
int8_t si4463_waitforCTS(si4463_t* si4463);
int8_t si4463_getResponse(si4463_t* si4463, uint8_t* respData, uint8_t respLen);
int8_t si4463_writeTxFiFo(si4463_t* si4463, uint8_t* txFifoData, uint8_t txFifoLen);
int8_t si4463_startTx(si4463_t* si4463, uint16_t dataLen, si4463_state nextState);
int8_t si4463_readRxDataBuff(uint8_t* rxFifoData, uint8_t rxFifoLength);
// int8_t si4463_getFastResponseReg(uint8_t* regVal, uint8_t startRegs, uint8_t regsNum);
int8_t si4463_configArray(si4463_t* si4463, uint8_t* configArray);
int8_t si4463_setProperties(si4463_t* si4463, uint8_t* setData, uint8_t setDataLen, uint16_t propNum);
int8_t si4463_getProperties(si4463_t* si4463, uint8_t* getData, uint8_t getDataLen, uint16_t propNum);
int8_t si4463_txInterrupt(si4463_t* si4463);
int8_t si4463_rxInterrupt(si4463_t* si4463);

uint8_t SI4463_CONFIGURATION_DATA[] = RADIO_CONFIGURATION_DATA_ARRAY;

int8_t si4463_powerOnReset(si4463_t* si4463)
{
    si4463->SDN(true);
    si4463->DelayUs(10);
    si4463->SDN(false);

    // Wait for POR (Power on reset) on GPIO1. The delay threshold is 10 ms.
    int count = 0;
    while(si4463->gpios.GPIO1() == si4463->gpios.gpio_low)
    {
        if(count < 10000)
        {
            count++;
            si4463->DelayUs(1);
        }
        else
            return SI4463_INIT_TIMEOUT;
    }
    // This first SPI transaction has to take less than 4ms (NSEL LOW time).
    // If it cannot be guaranteed, send a shorter command (e.g. NOP) first,
    // check CTS, then send POWER_UP or patch.
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;
    return si4463_checkNop(si4463);
}

int8_t si4463_init(si4463_t* si4463)
{
    int res = si4463_powerOnReset(si4463);
    if(res != SI4463_OK) return res;
    // Start to configurate the radio
    return si4463_configArray(si4463, SI4463_CONFIGURATION_DATA);
}

int8_t si4463_checkNop(si4463_t* si4463)
{
    uint8_t cmd = NOP;

    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
	if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

	return SI4463_OK;
}

int8_t si4463_getPartInfo(si4463_t* si4463)
{
    uint8_t cmd = PART_INFO;
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;
    DEBUG_PRINTF("rxbuff: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5], rxbuff[6], rxbuff[7]);
    if((uint16_t)((rxbuff[1] << 8) | rxbuff[2]) != 0x4463) return SI4463_ERR_CHIP_VERSION;
    return SI4463_OK;
}

int8_t si4463_getFuncInfo(si4463_t* si4463)
{
    uint8_t cmd = FUNC_INFO;
    uint8_t rxbuff[6] = {0};
    if(!si4463_sendCommand(si4463, &cmd, 1)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;
    DEBUG_PRINTF("rxbuff: %02x %02x %02x %02x %02x %02x\r\n",
			rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3], rxbuff[4], rxbuff[5]);
    return SI4463_OK;
}

int8_t si4463_getTxFifoInfo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x00};
    uint8_t rxbuff[2] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    return rxbuff[1];
}

int8_t si4463_getRxFifoInfo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x00};
    uint8_t rxbuff[2] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 2)) return SI4463_ERR_READ_REG;

    return rxbuff[0];
}

int8_t si4463_clearTxFifo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x01};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_clearRxFifo(si4463_t* si4463)
{
    uint8_t cmd[2] = {FIFO_INFO, 0x02};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_clearInterrupts(si4463_t* si4463)
{
    uint8_t cmd[4] = {GET_INT_STATUS, 0x00, 0x00, 0x00};
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

    return SI4463_OK;
}
int8_t si4463_getInterrupts(si4463_t* si4463)
{
    uint8_t cmd[4] = {GET_INT_STATUS, 0xFF, 0xFF, 0xFF};
    uint8_t rxbuff[8] = {0};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 8)) return SI4463_ERR_READ_REG;

    /* Get pend bytes */
    uint8_t phPend = rxbuff[2];
    uint8_t modemPend = rxbuff[4];
    uint8_t chipPend = rxbuff[6];

    si4463->interrupts.filterMatch = (phPend >> 7) & 0x1;
    si4463->interrupts.filterMiss = (phPend >> 6) & 0x1;
    si4463->interrupts.packetSent = (phPend >> 5) & 0x1;
    si4463->interrupts.packetRx = (phPend >> 4) & 0x1;
    si4463->interrupts.crcError = (phPend >> 3) & 0x1;
    si4463->interrupts.txFifoAlmostEmpty = (phPend >> 1) & 0x1;
    si4463->interrupts.rxFifoAlmostFull = phPend & 0x1;

    si4463->interrupts.postambleDetect = (modemPend >> 6) & 0x1;
    si4463->interrupts.invalidSync = (modemPend >> 5) & 0x1;
    si4463->interrupts.rssiJump = (modemPend >> 4) & 0x1;
    si4463->interrupts.rssi = (modemPend >> 3) & 0x1;
    si4463->interrupts.invalidPreamble = (modemPend >> 2) & 0x1;
    si4463->interrupts.preambleDetect = (modemPend >> 1) & 0x1;
    si4463->interrupts.packetSent = modemPend & 0x1;

    si4463->interrupts.cal = (chipPend >> 6) & 0x1;
    si4463->interrupts.fifoUnderflowOverflowError = (chipPend >> 5) & 0x1;
    si4463->interrupts.stateChange = (chipPend >> 4) & 0x1;
    si4463->interrupts.cmdError = (chipPend >> 3) & 0x1;
    si4463->interrupts.chipReady = (chipPend >> 2) & 0x1;
    si4463->interrupts.lowBatt = (chipPend >> 1) & 0x1;
    si4463->interrupts.wut = chipPend & 0x1;

    return SI4463_OK;
}

int8_t si4463_clearChipStatus(si4463_t* si4463)
{
    uint8_t cmd[2] = {GET_CHIP_STATUS, 0x00};
    uint8_t rxbuff[4] = {0};
    if(!si4463_sendCommand(si4463, cmd, 2)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, rxbuff, 4)) return SI4463_ERR_READ_REG;

    return SI4463_OK;
}

int8_t si4463_transmit(si4463_t* si4463, uint8_t* txData, uint8_t txDataLen, si4463_state nextState)
{
	int8_t result = si4463_txInterrupt(si4463);
	if(result != SI4463_OK) return result;
    result = si4463_clearInterrupts(si4463);
    if(result != SI4463_OK) return result;
    result = si4463_getTxFifoInfo(si4463);
    if(result >= txDataLen)
    {
        result = si4463_writeTxFiFo(si4463, txData, txDataLen);
        if(result != SI4463_OK) return result;
        result = si4463_startTx(si4463, txDataLen, nextState);
        if(result != SI4463_OK) return result;
        int counter = 0;
        // Check if IRQ pin is pulled down
        while(counter < SI4463_TRANSMIT_TIMEOUT)
        {
            if(si4463->gpios.IRQ() == si4463->gpios.gpio_low) return SI4463_OK;
            si4463->DelayUs(1000);
            counter++;
        }
    }
    else if(result >= 0)
        return SI4463_ERR_OVER_TX_FIFO;

    return result;
}

int8_t si4463_startRx(si4463_t* si4463, uint16_t dataLen, si4463_state nextStateAfterTimeOut, si4463_state nextStateAfterValid, si4463_state nextStateAfterInvalid)
{
    uint8_t cmd[8] = {START_RX, RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 0x00, ((dataLen >> 8) & 0x1F),  
            (dataLen & 0x00FF), nextStateAfterTimeOut, nextStateAfterValid, nextStateAfterInvalid};
    if(!si4463_sendCommand(si4463, cmd, 8)) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_sendCommand(si4463_t* si4463, uint8_t* cmdData, uint8_t cmdLen)
{
    si4463->NSEL(false);
    si4463->SPI_Write(cmdData, cmdLen, SI4463_SPI_TIMEOUT);
    si4463->NSEL(true);
    if(si4463->SPI_CheckState() != si4463->spi_state_ready)
        return 0;
    else
        return 1;
}

int8_t si4463_waitforCTS(si4463_t* si4463)
{
    uint8_t output_address[2] = {0};
    uint8_t ctsValue[2] = {0};
    uint16_t errCnt = 0;

    while(ctsValue[1] != 0xFF)
    {
        if(++errCnt > MAX_CTS_RETRY)
            return 0;
        output_address[0] = READ_CMD_BUFFER;
        si4463->NSEL(false);
        si4463->SPI_WriteRead(&output_address[0], &ctsValue[0], 2, SI4463_SPI_TIMEOUT);
        si4463->NSEL(true);
    }
    return 1;
}

int8_t si4463_getResponse(si4463_t* si4463, uint8_t* respData, uint8_t respLen)
{
	uint8_t output_address[2] = {0};
	uint8_t ctsValue[2] = {0};
	uint16_t errCnt = 0;

	while(ctsValue[1] != 0xFF)
	{
		if(++errCnt > MAX_CTS_RETRY)
			return 0;
		output_address[0] = READ_CMD_BUFFER;
		si4463->NSEL(false);
		si4463->SPI_WriteRead(&output_address[0], &ctsValue[0], 2, SI4463_SPI_TIMEOUT);
		if(ctsValue[1] == 0xFF)
			si4463->SPI_Read(respData, respLen, SI4463_SPI_TIMEOUT);
		si4463->NSEL(true);
	}
    if(si4463->SPI_CheckState() != si4463->spi_state_ready)
        return 0;
    else
        return 1;
}

int8_t si4463_writeTxFiFo(si4463_t* si4463, uint8_t* txFifoData, uint8_t txFifoLen)
{
    uint8_t cmd[txFifoLen + 1];
    memset(cmd, '\0', txFifoLen);
    cmd[0] = WRITE_TX_FIFO;
    memcpy(&cmd[1], txFifoData, txFifoLen);
    if(!si4463_sendCommand(si4463, cmd, sizeof(cmd))) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_startTx(si4463_t* si4463, uint16_t dataLen, si4463_state nextState)
{
    uint8_t cmd[5] = {START_TX, RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER, 
            (nextState << 4), ((dataLen >> 8) & 0x001F), (dataLen & 0x00FF)};
    if(!si4463_sendCommand(si4463, cmd, sizeof(cmd))) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_configArray(si4463_t* si4463, uint8_t* configArray)
{
    uint16_t index = 0, currentNum = 0;
    uint16_t propertyNum = 0;
    while(configArray[index])
    {
        currentNum = configArray[index];
        if(!si4463_sendCommand(si4463, &configArray[index + 1], currentNum))
            return SI4463_ERR_WRITE_REG;
        if(!si4463_waitforCTS(si4463))
            return SI4463_CTS_TIMEOUT;
        if(configArray[index + 1] == 0x11)
        {
            propertyNum = (configArray[index + 2] << 8) | configArray[index + 4];
            DEBUG_PRINTF("Property number: 0x%04x\r\n", propertyNum);
        }
		else
		{
            propertyNum = configArray[index + 1];
            DEBUG_PRINTF("Command number: 0x%02x\r\n", propertyNum);
        }
		index = index + currentNum + 1;
    }
    return SI4463_OK;
}

int8_t si4463_setProperties(si4463_t* si4463, uint8_t* setData, uint8_t setDataLen, uint16_t propNum)
{
    uint8_t cmd[setDataLen + 4];
    cmd[0] = SET_PROPERTY;
    cmd[1] = (uint8_t)(propNum >> 8);
    cmd[2] = setDataLen;
    cmd[3] = (uint8_t)(propNum & 0x00FF);
    memcpy(&cmd[4], setData, setDataLen);
    if(!si4463_sendCommand(si4463, cmd, sizeof(cmd))) return SI4463_ERR_WRITE_REG;
    if(!si4463_waitforCTS(si4463)) return SI4463_CTS_TIMEOUT;

    return SI4463_OK;
}

int8_t si4463_getProperties(si4463_t* si4463, uint8_t* getData, uint8_t getDataLen, uint16_t propNum)
{
    uint8_t cmd[4] = {GET_PROPERTY, (uint8_t)(propNum >> 8), getDataLen, (uint8_t)(propNum & 0x00FF)};
    if(!si4463_sendCommand(si4463, cmd, 4)) return SI4463_ERR_WRITE_REG;
    if(!si4463_getResponse(si4463, getData, getDataLen)) return SI4463_ERR_READ_REG;

    DEBUG_PRINTF("--------------------------\r\n");
		DEBUG_PRINTF("Get property data: ");
		for(int i = 0; i < getDataLen;i++)
		{
			DEBUG_PRINTF("0x%02x ", getData[i]);
		}
    DEBUG_PRINTF("\r\n");

    return SI4463_OK;
}

int8_t si4463_txInterrupt(si4463_t* si4463)
{
	uint8_t buff[4] = {0};
	buff[0] = PH_INT_STATUS_EN;
	buff[1] = PACKET_SENT_EN;
	buff[2] = 0x00;
	buff[3] = CHIP_READY_EN;
	return si4463_setProperties(si4463, buff, 4, PROP_INT_CTL_ENABLE);
}

