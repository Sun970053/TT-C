/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "si4463_huang.h"
#include "ax25_huang.h"
#include "morse_huang.h"
#include "HashTable.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define CMD_HELP                (0)
#define CMD_RESET               (1)
#define CMD_INIT                (2)
#define CMD_NOP                 (3)
#define CMD_TX                  (11)
#define CMD_CONTINUE_TX         (15)
#define CMD_RX                  (12)
#define CMD_GET                 (50)
#define CMD_SET                 (51)
#define PARAM_PART              (4)
#define PARAM_FUNC              (5)
#define PARAM_CURRSSI           (6)
#define PARAM_LATRSSI           (7)
#define PARAM_INTS              (10)
#define PARAM_TXPWR             (13)
#define PARAM_PREAMBLE          (14)
#define PARAM_SYNCWORD          (20)
#define PARAM_CRC               (30)
#define PARAM_FREQUENCY         (31)
#define PARAM_MODULATION        (32)
#define PARAM_DATARATE          (33)
#define PARAM_CRC_POLY          (34)
#define PARAM_CRC_SEED          (35)
#define PARAM_TX_MODULATION     (36)
#define PARAM_RX_MODULATION     (37)
#define PARAM_TX_DATARATE       (38)
#define PARAM_RX_DATARATE       (39)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* -----si4463----- */
si4463_t si4463;
bool irqFlag = false;

/* -----Command Line----- */
Hash_Table* ht;
typedef struct
{
	uint8_t temp[30];
	uint8_t cmd[10];
	uint8_t param[50];
	uint8_t param2[30];
	uint8_t cmdLen;
	uint8_t paramLen;
	uint8_t paramLen2;
}rxCmd;
rxCmd myRxCmd;
uint8_t pos = 0;
uint8_t rxData = 0;
volatile uint8_t uartFlag = 0;
volatile uint8_t strFlag = 0;
volatile uint8_t cmdFlag = 0;
volatile uint8_t rxFlag = 0;

/* -----AX.25 Frame ----- */
ax25frame_t ax25frame;
morse_t morse;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void si4463MenuDisplay(void);
void initHashTable(void);
uint8_t si4463_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout);
uint8_t si4463_SPI_CheckState(void);
void si4463_DelayUs(uint32_t delay);
void si4463_setNSEL(bool val);
void si4463_setSDN(bool val);
void si4463_setOOK(bool val);
bool si4463_getIRQ(void);
bool si4463_getGPIO1(void);

void morse_resetToNow(timerange_t*);
uint32_t morse_millisecondsElapsed(timerange_t*);
uint32_t morse_secondsElapsed(timerange_t*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		/* Enabling interrupt receive again */
		HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxData,1);
		switch(rxData)
		{
		case ' ':
			// first space
			if(uartFlag == 0 && strFlag == 0)
			{
				// clear array
				if(myRxCmd.cmdLen)
					memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
				memcpy(myRxCmd.cmd, myRxCmd.temp, pos + 1);
				memset(myRxCmd.temp, '\0', pos + 1);
				myRxCmd.cmdLen = pos + 1;
				pos = 0;
				uartFlag++;
				printf(" ");
			}
			// second space
			else if(uartFlag == 1 && strFlag == 0)
			{
				// clear array
				if(myRxCmd.paramLen)
					memset(myRxCmd.param, '\0', myRxCmd.paramLen);
				memcpy(myRxCmd.param, myRxCmd.temp, pos + 1);
				memset(myRxCmd.temp, '\0', pos + 1);
				myRxCmd.paramLen = pos + 1;
				pos = 0;
				uartFlag++;
				printf(" ");
			}
			if(strFlag == 1)
			{
				myRxCmd.temp[pos] = rxData;
				pos++;
				printf(" ");
			}

			break;
		case '\"':
			if(uartFlag == 1)
			{
				if(strFlag) strFlag = 0;
				else strFlag = 1;
				printf("\"");
			}
			break;
		case '\r':
			// two parameter
			if(uartFlag == 2)
			{
				// clear array
				if(myRxCmd.paramLen2)
					memset(myRxCmd.param2, '\0', myRxCmd.paramLen2);
				memcpy(myRxCmd.param2, myRxCmd.temp, pos + 1);
				myRxCmd.paramLen2 = pos + 1;
			}
			// one parameter
			else if(uartFlag == 1)
			{
				// clear array
				if(myRxCmd.paramLen)
					memset(myRxCmd.param, '\0', myRxCmd.paramLen);
				memcpy(myRxCmd.param, myRxCmd.temp, pos + 1);
				myRxCmd.paramLen = pos + 1;
			}
			else
			{
				// clear array
				if(myRxCmd.cmdLen)
					memset(myRxCmd.cmd, '\0', myRxCmd.cmdLen);
				memcpy(myRxCmd.cmd, myRxCmd.temp, pos + 1);
				myRxCmd.cmdLen = pos + 1;
			}
			memset(myRxCmd.temp, '\0', pos + 1);
			pos = 0;
			uartFlag = 0;
			/* Start execute command */
			cmdFlag = 1;
			printf("\r\n");
			break;
		default:
			myRxCmd.temp[pos] = rxData;
			pos++;
			printf("%c", rxData);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B1_Pin)
	{
		printf("Button interrupt !\r\n");
		si4463_getInterrupts(&si4463);
		printf("Current RSSI: %d\r\n", si4463_getCurrentRSSI(&si4463));
		irqFlag = true;
	}
	if(GPIO_Pin == IRQ_Pin)
	{
		irqFlag = true;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1); // us delay timer
  HAL_TIM_Base_Start(&htim2); // us delay timer

  HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxData,1); // Enabling interrupt receive again
    //HAL_I2C_Slave_Receive_IT(&hi2c1, isr_rxData, SLAVE_RX_BUFFER_SIZE);
    initHashTable();

	si4463.SPI_Write = si4463_SPI_Write;
	si4463.SPI_Read = si4463_SPI_Read;
	si4463.SPI_WriteRead = si4463_SPI_WriteRead;
	si4463.SPI_CheckState = si4463_SPI_CheckState;
	si4463.spi_state_ready = HAL_SPI_STATE_READY;
	si4463.DelayUs = si4463_DelayUs;
	si4463.NSEL = si4463_setNSEL;
	si4463.SDN = si4463_setSDN;
	si4463.OOK = si4463_setOOK;
	si4463.gpios.IRQ = si4463_getIRQ;
	si4463.gpios.GPIO1 = si4463_getGPIO1;
	si4463.gpios.gpio_low = GPIO_PIN_RESET;
	si4463.gpios.gpio_high = GPIO_PIN_SET;

	morse.time.resetToNow = morse_resetToNow;
	morse.time.millisecondsElapsed = morse_millisecondsElapsed;
	morse.time.secondsElapsed = morse_secondsElapsed;
	//HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	/* Initialize si4463 */
	int res = si4463_init(&si4463);
	if(res == SI4463_OK)
		printf("Si4463 init .. ok !\r\n");
	else
		printf("Si4463 init .. fail ! error code: %d\r\n", res);

	si4463_getPartInfo(&si4463);
	si4463_getFuncInfo(&si4463);

	/* Functional test: Packet generator send data to packet parser.
	 * AX.25 -> HDLC -> NRZI -> G3RUH
	 * G3RUH -> NRZI -> HDLC -> AX.25*/
	char testInfo[] = "What is 'X' SATORO ??";
	uint8_t control = RADIOLIB_AX25_CONTROL_U_UNNUMBERED_INFORMATION | RADIOLIB_AX25_CONTROL_POLL_FINAL_DISABLED | RADIOLIB_AX25_CONTROL_UNNUMBERED_FRAME;
	ax25sendframe_t* testsendframe = createAX25SendFrame("STARL", 0, "NCKU", 0, control, RADIOLIB_AX25_PID_NO_LAYER_3, (uint8_t*)testInfo, strlen(testInfo), 8);
	ax25frame.ax25SendFrame = testsendframe;
	uint16_t testhdlcLen = 0;
	uint8_t* testhdlcbuff;
	uint8_t pres = AX25Frame_HDLC_Generator(&ax25frame, &testhdlcbuff, &testhdlcLen);
	if(!pres)
	{
		printf("Generate AX.25 frame .. success ! \r\n");
		printf("AX.25/HDLC frame\r\n");
		printf("packet: ");
		for(int i = 0; i <  testhdlcLen; i++)
			printf("0x%02x ", testhdlcbuff[i]);
		printf("\r\n");
	}
	else
		printf("Generate AX.25 frame .. fail ! \r\n");
	ax25_nrzi_encode(testhdlcbuff, testhdlcbuff, testhdlcLen);
	printf("NRZI encode\r\n");
	printf("packet: ");
	for(int i = 0; i <  testhdlcLen; i++)
		printf("0x%02x ", testhdlcbuff[i]);
	printf("\r\n");
	ax25_g3ruh_scrambler_init(0x21000UL);
	ax25_g3ruh_scrambler(testhdlcbuff, testhdlcbuff, testhdlcLen);
	printf("G3RUH scramble\r\n");
	printf("packet: ");
	for(int i = 0; i <  testhdlcLen; i++)
		printf("0x%02x ", testhdlcbuff[i]);
	printf("\r\n");

	ax25_g3ruh_scrambler_init(0x21000UL);
	ax25_g3ruh_descrambler(testhdlcbuff, testhdlcbuff, testhdlcLen);
	printf("G3RUH descramble\r\n");
	printf("packet: ");
	for(int i = 0; i <  testhdlcLen; i++)
		printf("0x%02x ", testhdlcbuff[i]);
	printf("\r\n");

	ax25_nrzi_decode(testhdlcbuff, testhdlcbuff, testhdlcLen);
	printf("NRZI decode\r\n");
	printf("packet: ");
	for(int i = 0; i <  testhdlcLen; i++)
		printf("0x%02x ", testhdlcbuff[i]);
	printf("\r\n");

	ax25receiveframe_t* testrcvframe = createAX25ReceiveFrame("STARL", 0, "NCKU", 0, control, RADIOLIB_AX25_PID_NO_LAYER_3, 8);
	ax25frame.ax25RcvFrame = testrcvframe;

	pres = AX25Frame_HDLC_Parser(&ax25frame, testhdlcbuff, testhdlcLen);
	if(!pres)
		printf("Parse AX.25 frame .. success ! \r\n");
	else
		printf("Parse AX.25 frame .. fail !  CRC Error !\r\n");

	printf("AX.25/HDLC frame information:\r\n");
	printf("From %s to %s ", ax25frame.ax25RcvFrame->srcCallsign, ax25frame.ax25RcvFrame->destCallsign);
	printf("<Control: %d, PID: %02X, Length: %d> \r\n", ax25frame.ax25RcvFrame->control, ax25frame.ax25RcvFrame->protocolID, ax25frame.ax25RcvFrame->infoLen);
	printf("Message: %s \r\n", ax25frame.ax25RcvFrame->info);
	printf("\r\n");
	//HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	uint8_t rxDataLen = 0;
	uint8_t rxData[100] = {0};
	int cmdCode = -1;
	int paramCode = -1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if(rxFlag)
	{
		if(si4463.gpios.IRQ() == si4463.gpios.gpio_low)
		{
			printf("RSSI: %d\r\n", si4463_getLatchRSSI(&si4463));
			si4463_getInterrupts(&si4463);
			res = si4463_getRxFifoInfo(&si4463);
			if(res > 0)
			{
				rxDataLen = res;
				res = si4463_receive(&si4463, rxData, rxDataLen);

				if(res == SI4463_OK)
				{
					printf("Receive .. ok !\r\n");
					printf("Message: ");
					for(int i = 0;i < rxDataLen; i++)
					{
						printf("0x%02x ", rxData[i]);
					}
					printf("\r\n");
					memset(rxData, '\0', rxDataLen);

//					ax25_g3ruh_descrambler(rxData, rxData, rxDataLen);
//					ax25_nrzi_decode(rxData, rxData, rxDataLen);
//					ax25receiveframe_t* rcvframe = createAX25ReceiveFrame("STARL", 0, "NCKU", 0, control, RADIOLIB_AX25_PID_NO_LAYER_3, 8);
//					ax25frame.ax25RcvFrame = rcvframe;
//
//					pres = AX25Frame_HDLC_Parser(&ax25frame, rxData, rxDataLen);
//					if(!pres)
//						printf("Parse AX.25 frame .. success ! \r\n");
//					else
//						printf("Parse AX.25 frame .. fail !  CRC Error !\r\n");
//
//					printf("AX.25/HDLC frame information:\r\n");
//					printf("From %s to %s ", ax25frame.ax25RcvFrame->srcCallsign, ax25frame.ax25RcvFrame->destCallsign);
//					printf("<Control: %d, PID: %02X, Length: %d> \r\n", ax25frame.ax25RcvFrame->control, ax25frame.ax25RcvFrame->protocolID, ax25frame.ax25RcvFrame->infoLen);
//					printf("Message: %s \r\n", ax25frame.ax25RcvFrame->info);
//					printf("\r\n");

				}
				else
					printf("Receive .. fail ! error code: %d\r\n", res);
			}
			else
			{
				printf("Get Rx FiFO info .. fail ! error code: %d\r\n", res);
			}
		}
	}

	if(cmdFlag)
	{
		cmdCode = HT_searchKey(ht, (char*)&myRxCmd.cmd[0]);
		paramCode = HT_searchKey(ht, (char*)&myRxCmd.param[0]);
//		param2Code = HT_searchKey(ht, (char*)&myRxCmd.param2[0]);
		switch(cmdCode)
		{
		case CMD_HELP:
			si4463MenuDisplay();
			break;
		case CMD_RESET:
			res = si4463_powerOnReset(&si4463);
			if(res == SI4463_OK)
				printf("Si4463 power on reset .. ok !\r\n");
			else
				printf("Si4463 power on reset .. fail ! error code: %d\r\n", res);
			break;
		case CMD_INIT:
			res = si4463_init(&si4463);
			if(res == SI4463_OK)
				printf("Si4463 init .. ok !\r\n");
			else
				printf("Si4463 init .. fail ! error code: %d\r\n", res);
			break;
		case CMD_TX:
			if(myRxCmd.paramLen > 0)
			{
				if(si4463.settings.txMod != MOD_OOK)
				{
					uint8_t control = RADIOLIB_AX25_CONTROL_U_UNNUMBERED_INFORMATION | RADIOLIB_AX25_CONTROL_POLL_FINAL_DISABLED | RADIOLIB_AX25_CONTROL_UNNUMBERED_FRAME;
					ax25sendframe_t* ax25sendframe = createAX25SendFrame("STARL", 0, "NCKU", 0, control, RADIOLIB_AX25_PID_NO_LAYER_3, (uint8_t*)myRxCmd.param, myRxCmd.paramLen - 1, 8);
					ax25frame.ax25SendFrame = ax25sendframe;
					uint16_t hdlcLen = 0;
					uint8_t* hdlcbuff;
					uint8_t pres = AX25Frame_HDLC_Generator(&ax25frame, &hdlcbuff, &hdlcLen);
					if(!pres)
					{
						ax25_nrzi_encode(hdlcbuff, hdlcbuff, hdlcLen);
						ax25_g3ruh_scrambler_init(0x21000UL);
						ax25_g3ruh_scrambler(hdlcbuff, hdlcbuff, hdlcLen);

						res = si4463_transmit(&si4463, hdlcbuff, hdlcLen, STATE_NO_CHANGE);
						if(res == SI4463_OK)
						{
							printf("Si4463 Transmit .. ok !\r\n");
							printf("packet: ");
							for(int i = 0; i <  hdlcLen; i++)
								printf("0x%02x ", hdlcbuff[i]);
							printf("\r\n");
						}
						else
							printf("Si4463 Transmit .. fail ! error code: %d\r\n", res);
					}
					else
					{
						printf("Tx error ! error code: %d", pres);
					}
				}
				// OOK transmission
				else
				{
					morse_setText(&morse, (char*)myRxCmd.param);
					morse_start(&morse);
					si4463_startTx(&si4463, 0, STATE_NO_CHANGE);
					while(morse.currentState != msEndOfText)
					{
						// We give CPU time to morse subsystem.
						morse_handleTimeout(&morse);
						bool toneOn = morse_isToneActive(&morse);
						si4463_controlOOK(&si4463, toneOn);
					}
					printf("Sending Morse code.. Complete !\r\n");
				}
			}
			else
			{
				printf("Invalid packets ! \r\n");
			}
			break;
		case CMD_CONTINUE_TX:
			printf("TBD \r\n");
			break;
		case CMD_RX:
			res = si4463_initRx(&si4463, 0, STATE_RX, STATE_RX, STATE_RX);
			if(res == SI4463_OK)
			{
				printf("Start Rx .. ok !\r\n");
				rxFlag = 1;
			}
			else
				printf("Start Rx .. fail ! error code: %d\r\n", res);
			break;
		case CMD_NOP:
			res = si4463_checkNop(&si4463);
			if(res == SI4463_OK)
				printf("Si4463 has responded !\r\n");
			else
				printf("Si4463 NOP .. fail ! error code: %d\r\n", res);
			break;
		case CMD_SET:
			if(paramCode == PARAM_TXPWR)
			{
				int txpwr = atoi((char*)myRxCmd.param2);
				if(txpwr > 0)
				{
					res = si4463_setTxPower(&si4463, txpwr);
					if(res == SI4463_OK)
					{
						printf("Set Tx power .. ok !\r\n");
						printf("Get Tx power %d \r\n", si4463_getTxPower(&si4463));
					}
					else
						printf("Set Tx power .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_PREAMBLE)
			{
				int preamble = atoi((char*)myRxCmd.param2);
				if(preamble > 0)
				{
					res = si4463_setPreamble(&si4463, preamble);
					if(res == SI4463_OK)
					{
						printf("Set Tx preamble .. ok !\r\n");
						printf("Get Tx preamble %d \r\n", si4463_getPreamble(&si4463));
					}
					else
						printf("Set Tx power .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_SYNCWORD)
			{
				printf("TBD \r\n");
			}
			else if(paramCode == PARAM_CRC_POLY)
			{
				int poly = atoi((char*)myRxCmd.param2);
				if(poly > 0)
				{
					res = si4463_setCRC(&si4463, si4463.settings.crcSeed, poly);
					if(res == SI4463_OK)
					{
						si4463_getCRC(&si4463);
						printf("Set CRC polynomial .. ok !\r\n");
						printf("Get CRC polynomial %d \r\n", si4463.settings.crcPoly);
					}
					else
						printf("Set Tx power .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_CRC_SEED)
			{
				int seed = atoi((char*)myRxCmd.param2);
				if(seed == 0 || seed == 1)
				{
					res = si4463_setCRC(&si4463, seed, si4463.settings.crcPoly);
					if(res == SI4463_OK)
					{
						si4463_getCRC(&si4463);
						printf("Set CRC polynomial .. ok !\r\n");
						printf("Get CRC polynomial %d \r\n", si4463.settings.crcSeed);
					}
					else
						printf("Set Tx power .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_FREQUENCY)
			{
				int freq = atoi((char*)myRxCmd.param2);
				if(freq >= 400000000 && freq < 436000000)
				{
					res = si4463_setFrequency(&si4463, freq);
					if(res == SI4463_OK)
					{
						printf("Set frequency .. ok !\r\n");
						printf("Get frequency %ld \r\n", si4463_getFrequency(&si4463));
					}
					else
						printf("Set Tx power .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_TX_MODULATION)
			{
				int txmod = atoi((char*)myRxCmd.param2);
				if(txmod >= 0)
				{
					res = si4463_setTxModulation(&si4463, txmod);
					if(res == SI4463_OK)
					{
						printf("Set Tx modulation .. ok !\r\n");
						printf("Get Tx modulation %d \r\n", si4463.settings.txMod);
					}
					else
						printf("Set Tx modulation .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_RX_MODULATION)
			{
				int rxmod = atoi((char*)myRxCmd.param2);
				if(rxmod >= 0)
				{
					res = si4463_setRxModulation(&si4463, rxmod);
					if(res == SI4463_OK)
					{
						printf("Set Rx modulation .. ok !\r\n");
						printf("Get Rx modulation %d \r\n", si4463.settings.rxMod);
					}
					else
						printf("Set Rx modulation .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_TX_DATARATE)
			{
				int txdr = atoi((char*)myRxCmd.param2);
				if(txdr >= 0)
				{
					res = si4463_setTxDataRate(&si4463, txdr);
					if(res == SI4463_OK)
					{
						printf("Set Tx data rate .. ok !\r\n");
						printf("Get Tx data rate %d \r\n", si4463.settings.txDataRate);
					}
					else
						printf("Set Tx data rate .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else if(paramCode == PARAM_RX_DATARATE)
			{
				int rxdr = atoi((char*)myRxCmd.param2);
				if(rxdr >= 0)
				{
					res = si4463_setRxDataRate(&si4463, rxdr);
					if(res == SI4463_OK)
					{
						printf("Set Rx data rate .. ok !\r\n");
						printf("Get Rx data rate %d \r\n", si4463.settings.rxDataRate);
					}
					else
						printf("Set Rx data rate .. fail ! error code: %d\r\n", res);
				}
				else
				{
					printf("Invalid parameter ! \r\n");
				}
			}
			else
			{
				printf("Invalid parameter ! \r\n");
			}
			break;
		case CMD_GET:
			if(paramCode == PARAM_PART)
			{
				res = si4463_getPartInfo(&si4463);
				if(res == SI4463_OK)
					printf("Si4463 get part info .. ok !\r\n");
				else
					printf("Si4463 get part info .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_FUNC)
			{
				res = si4463_getFuncInfo(&si4463);
				if(res == SI4463_OK)
					printf("Si4463 get function info .. ok !\r\n");
				else
					printf("Si4463 get function info .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_CURRSSI)
			{
				res = si4463_getCurrentRSSI(&si4463);
				if(res < -12)
					printf("Si4463 get current RSSI: %d\r\n", res);
				else
					printf("Si4463 get current RSSI .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_LATRSSI)
			{
				res = si4463_getLatchRSSI(&si4463);
				if(res < -12)
					printf("Si4463 get latch RSSI: %d\r\n", res);
				else
					printf("Si4463 get latch RSSI .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_INTS)
			{
				res = si4463_getInterrupts(&si4463);
				if(res == SI4463_OK)
					printf("Si4463 get interrupts' info .. ok !\r\n");
				else
					printf("Si4463 get interrupts' info .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_TXPWR)
			{
				res = si4463_getTxPower(&si4463);
				if(res >= 0)
					printf("Si4463 get Tx power: %d\r\n", res);
				else
					printf("Si4463 get Tx power .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_PREAMBLE)
			{
				res = si4463_getPreamble(&si4463);
				if(res >= 0)
					printf("Si4463 get preamble: %d\r\n", res);
				else
					printf("Si4463 get preamble .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_SYNCWORD)
			{
				res = si4463_getSyncWords(&si4463);
				if(res >= 0)
				{
					printf("Si4463 get sync words: ");
					for(int i = 0; i < res; i++)
					{
						printf("0x%02x ", si4463.settings.syncWords[i]);
					}
					printf("\r\n");
				}
				else
					printf("Si4463 get sync words .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_CRC)
			{
				res = si4463_getCRC(&si4463);
				if(res == SI4463_OK)
					printf("Si4463 get CRC Poly: %d, Seed: %d\r\n", si4463.settings.crcPoly, si4463.settings.crcSeed);
				else
					printf("Si4463 get CRC Poly .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_FREQUENCY)
			{
				res = si4463_getFrequency(&si4463);
				if(res >= 0)
					printf("Si4463 get Tx/Rx frequency: %d\r\n", res);
				else
					printf("Si4463 get Tx/Rx frequency .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_MODULATION)
			{
				res = si4463_getModulation(&si4463);
				if(res >= 0)
					printf("Si4463 get Tx modulation: %d\r\n", res);
				else
					printf("Si4463 get Tx modulation .. fail ! error code: %d\r\n", res);
			}
			else if(paramCode == PARAM_DATARATE)
			{
				res = si4463_getDataRate(&si4463);
				if(res >= 0)
					printf("Si4463 get Tx data rate: %d\r\n", res);
				else
					printf("Si4463 get Tx data rate .. fail ! error code: %d\r\n", res);
			}
			else
			{
				printf("Invalid parameter ! \r\n");
			}
			break;
		default:
			printf("Display the command menu -----------> help \r\n\r\n");
		}
		cmdFlag = 0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSEL_Pin */
  GPIO_InitStruct.Pin = NSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDN_Pin */
  GPIO_InitStruct.Pin = SDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO0_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t si4463_SPI_Write(uint8_t* pTxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_Transmit(&hspi1, pTxData, dataLen, timeout);
}

uint8_t si4463_SPI_Read(uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_Receive(&hspi1, pRxData, dataLen, timeout);
}

uint8_t si4463_SPI_WriteRead(uint8_t* pTxData, uint8_t* pRxData, uint8_t dataLen, uint32_t timeout)
{
	return HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, dataLen, timeout);
}

uint8_t si4463_SPI_CheckState(void)
{
	return HAL_SPI_GetState(&hspi1);
}

void si4463_DelayUs(uint32_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while((__HAL_TIM_GET_COUNTER(&htim1)) < delay);
}

void si4463_setNSEL(bool val)
{
	if(val == true)
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(NSEL_GPIO_Port, NSEL_Pin, GPIO_PIN_RESET);
}

void si4463_setSDN(bool val)
{
	if(val == true)
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(SDN_GPIO_Port, SDN_Pin, GPIO_PIN_RESET);
}

void si4463_setOOK(bool val)
{
	if(val == true)
		HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);
}

bool si4463_getIRQ(void)
{
	return HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin);
}

bool si4463_getGPIO1(void)
{
	return HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin);
}

void si4463MenuDisplay(void)
{
	printf("============== Command Menu ============== \r\n");
	printf("Display the command menu -----------> help \r\n");
	printf("Power on reset si4463 chip ---------> reset \r\n");
	printf("Initialize si4463 configuration ----> init \r\n");
	printf("Communicate to si4463 with NOP -----> nop \r\n");
	printf("Transmit packets -------------------> tx  [string] \r\n");
	printf("Send the most recently sent data ---> contx \r\n");
	printf("Start to receive packets -----------> rx \r\n");
	printf("Basic information about si4463 -----> get part \r\n");
	printf("Function revision information ------> get func \r\n");
	printf("Obtain the current RSSI ------------> get curRSSI \r\n");
	printf("Get RSSI when receiving signal -----> get latRSSI \r\n");
	printf("Get the status of all interrupts ---> get ints \r\n");
	printf("Set the transmission power ---------> set txpwr    [0-127] \r\n");
	printf("Get the transmission power ---------> get txpwr \r\n");
	printf("Set the preamble number ------------> set preamble [5-255] \r\n");
	printf("Get the preamble number ------------> get preamble \r\n");
	printf("Set the syncwords ------------------> set syncword [TBD] \r\n");
	printf("Get the syncwords ------------------> get syncword \r\n");
	printf("Set the CRC polynomial -------------> set crc      [0-9] \r\n");
	printf("Get the CRC polynomial/seed --------> get crc \r\n");
	printf("Set TX/RX frequency ----------------> set freq     [Hz] \r\n");
	printf("Get TX/RX frequency ----------------> get freq \r\n");
	printf("Set the TX modulation --------------> set txmod    [0-2] \r\n");
	printf("Set the RX modulation --------------> set rxmod    [0-2] \r\n");
	printf("Get the TX modulation --------------> get mod \r\n");
	printf("Set the TX data rate ---------------> set txdr     [0-3] \r\n");
	printf("Set the RX data rate ---------------> set rxdr     [0-3] \r\n");
	printf("Get the TX data rate ---------------> get dr \r\n");
}

void initHashTable(void)
{
	// define the function pointer
	HT_malloc = &malloc;
	HT_free = &free;
	// select commands
	ht = HT_createTable(50);
	HT_insertItem(ht, "help",    0);
	HT_insertItem(ht, "reset",   1);
	HT_insertItem(ht, "init",    2);
	HT_insertItem(ht, "nop",     3);
	HT_insertItem(ht, "part",    4);
	HT_insertItem(ht, "func",    5);
	HT_insertItem(ht, "curRSSI", 6);
	HT_insertItem(ht, "latRSSI", 7);
	HT_insertItem(ht, "ints",    10);
	HT_insertItem(ht, "tx",      11);
	HT_insertItem(ht, "rx",      12);
	HT_insertItem(ht, "txpwr",   13);
	HT_insertItem(ht, "preamble",14);
	HT_insertItem(ht, "contx",   15);
	HT_insertItem(ht, "syncword",20);
	HT_insertItem(ht, "crc",     30);
	HT_insertItem(ht, "freq",    31);
	HT_insertItem(ht, "mod",     32);
	HT_insertItem(ht, "dr",      33);
	HT_insertItem(ht, "crcpoly", 34);
	HT_insertItem(ht, "crcseed", 35);
	HT_insertItem(ht, "txmod",   36);
	HT_insertItem(ht, "rxmod",   37);
	HT_insertItem(ht, "txdr",    38);
	HT_insertItem(ht, "rxdr",    39);
	HT_insertItem(ht, "get",     50);
	HT_insertItem(ht, "set",     51);
	HT_print(ht);

	printf("Search Key %s: Value %d \r\n", "check", HT_searchKey(ht, "check"));
}

void morse_resetToNow(timerange_t* time)
{
	time->stamp = __HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t morse_millisecondsElapsed(timerange_t* time)
{
	uint32_t current = __HAL_TIM_GET_COUNTER(&htim2);
	if(time->stamp < current)
	{
		return current - time->stamp;
	}
	// unsigned integer overflow
	else
	{
		return UINT32_MAX - (time->stamp - current);
	}
}

uint32_t morse_secondsElapsed(timerange_t* time)
{
	return morse_millisecondsElapsed(time)/1000;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
