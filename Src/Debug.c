/**
  ******************************************************************************
  * File Name          : Debug.c
  * Description        : Utilities for debugging the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "Debug.h"
#include "main.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
static const int DEBUG_TASK_PERIOD = 100;
static const uint16_t memAddress = 0x07;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

void debugTask(void const* arg) {
    uint32_t prevWakeTime = osKernelSysTick();
    uint8_t buffer = 0x00;
    LogEntry debugData;

//    bool initStatus = W25qxx_Init();

    //erase entire chip before logging
    W25qxx_EraseChip();


	uint8_t checkLib1[8] = {1,2,3,4,5,6,7,8};
	uint8_t checkLib2[8];

	while (1) {
		osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD*5);
//		HAL_UART_Receive(&huart5, &buffer, 1, 100);
//========= TESTING NEW SPI DRIVER

		W25qxx_WriteSector(checkLib1, 1, 0, 8);

		W25qxx_ReadSector(checkLib2, 1, 0, 8);




//========= TESTING RADIO
/*
 		HAL_UART_Receive(&huart2, &buffer, sizeof(buffer), 100);
		HAL_UART_Transmit(&huart2, "a", 1, 10);
		HAL_UART_Transmit(&huart2, "b", 1, 10);
		HAL_UART_Receive(&huart5, &buffer, 1, 100);
 */

//========= TESTING UMBILICAL
/*
		HAL_UART_Receive(&huart1, &buffer, sizeof(buffer), 100);
		HAL_UART_Transmit(&huart1, "foo", 3, 10);
		buffer = 't';

		// LOGIC
		HAL_UART_Transmit(&huart5, (uint8_t*)(&buffer),sizeof(buffer),10);
		HAL_UART_Transmit(&huart5, " ",1,10);

		HAL_UART_Transmit(&huart1, (uint8_t*)(&buffer),sizeof(buffer),10);
		HAL_UART_Transmit(&huart1, " ",1,10);

		uint8_t res = 5;
		uint8_t checkArray[5] = {1, 1, 1, 1, 1};
	    HAL_UART_Transmit(&huart5, checkArray, sizeof(checkArray),100);
*/


//========= TESTING WRITE AND READ SEPARATELY
/*
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	    HAL_StatusTypeDef writeStatus = HAL_SPI_Transmit(&hspi2, checkArray, 5, 1000);
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	    HAL_StatusTypeDef readStatus = HAL_SPI_Receive(&hspi2, checkArray, 5, 1000);
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	    HAL_UART_Transmit(&huart5, checkArray, sizeof(checkArray),100);

	    HAL_UART_Transmit(&huart5, "354", 3,100);
*/

//========= TESTING DEBUG CASES
/*
		switch(buffer){
			case 't':
	            HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
				initializeLogEntry(&debugData);
				writeLogEntryToEEPROM(memAddress,&debugData);
				HAL_UART_Transmit(&huart5, (uint8_t*)(&debugData), sizeof(debugData),100);
				break;
			case 'd':
				readLogEntryFromEEPROM(memAddress, &debugData);
				HAL_UART_Transmit(&huart5, (uint8_t*)(&debugData),sizeof(debugData),100);
				buffer = 't';
				break;
			default:
				break;
		}
*/


//========= USING HAL Functions
/*
		// WRITE PROTECT
		// HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,RESET);
		// HAL_GPIO_WritePin(MEM_WP_GPIO_Port,MEM_WP_Pin,SET);
		// HAL_GPIO_WritePin(LED_1_GPIO_Port,LED_1_Pin,SET);

	    uint8_t Write_Enable = 0x06;
	    uint8_t Page_Program = 0x02;
	    uint32_t Address = 0x00000000;
	    uint8_t txBuffer[10] = {0x02, 0x04, 0x06, 0x08, 0xa2, 0xa4, 0xa6, 0xa8, 0x00, 0x00};
	    uint8_t rxBuffer[10];

	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,RESET);
	    volatile HAL_StatusTypeDef weStatus = HAL_SPI_Transmit(&hspi2,&Write_Enable,1,1000);
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,SET);

	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,RESET);
	    volatile HAL_StatusTypeDef ppStatus = HAL_SPI_Transmit(&hspi2,&Page_Program,1,1000);
	    volatile HAL_StatusTypeDef addStatus = HAL_SPI_Transmit(&hspi2,&Address,4,1000);
	    volatile HAL_StatusTypeDef txStatus = HAL_SPI_Transmit(&hspi2,txBuffer,10,1000);
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,SET);

	    uint8_t Read_Data = 0x03;

	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,RESET);
	    volatile HAL_StatusTypeDef readCommandStatus = HAL_SPI_Transmit(&hspi2,&Read_Data,1,1000);
	    volatile HAL_StatusTypeDef addressCommandStatus = HAL_SPI_Transmit(&hspi2,&Address,4,1000);
	    volatile HAL_StatusTypeDef rxStatus = HAL_SPI_Receive(&hspi2,rxBuffer,10,1000);
	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,SET);
*/

	}
}
