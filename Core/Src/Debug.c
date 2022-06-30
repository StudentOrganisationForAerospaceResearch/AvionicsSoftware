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
#include "LogData.h"
#include "main.h"

#include <string.h>
#include <stdlib.h>

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
static const int DEBUG_TASK_PERIOD = 100;

/* Variables -----------------------------------------------------------------*/
uint8_t debugMsg[DEBUG_RX_BUFFER_SZ_B + 1]; // Ensure last byte always 0
uint8_t debugMsgIdx = 0;
uint8_t isDebugMsgReady = 0;

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

uint32_t str2uint32(uint8_t* s, uint32_t max_size) {
  uint32_t ret = 0;
  for (uint8_t i = 0; i < max_size; i++) {
    if (s[i] >= '0' && s[i] <= '9') {
      ret = (ret * 10) + (s[i] - '0');
    } else {
      break;
    }
  }
  return ret;
}

void debugTask(void const* arg) {
  uint32_t prevWakeTime = osKernelSysTick();

  while (1) {
    osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

    if (!isDebugMsgReady) {
      continue;
    }

    if (strcmp(debugMsg, "erase") == 0) {
//      isErasing = 1;
    } else if (strcmp(debugMsg, "startlog") == 0) {
//      isOkayToLog = 1;
    } else if (strcmp(debugMsg, "stoplog") == 0) {
      isOkayToLog = 0;
    } else if (strcmp(debugMsg, "resetlog") == 0) {
      isOkayToLog = 0;
//      currentSectorAddr = 0;
//      currentSectorOffset_B = 0;
    } else {
      switch (debugMsg[0]) {
        case 'p':
          asm volatile ("nop"); // Labels can't point to declarations?
          uint32_t page_num = str2uint32(&debugMsg[1], DEBUG_RX_BUFFER_SZ_B - 1);
          if (page_num >= w25qxx.PageCount) {
            HAL_UART_Transmit(&huart5, "BAD INDEX\n", 10, 1000);
            break;
          }
          uint8_t* page_data = malloc(w25qxx.PageSize);
          memset(page_data, 0, w25qxx.PageSize);
          W25qxx_ReadPage(page_data, page_num, 0, w25qxx.PageSize);
          HAL_UART_Transmit(&huart5, "\n======== BEGIN PAGE ========\n", 30, 1000);
          HAL_UART_Transmit(&huart5, page_data, w25qxx.PageSize, 1000);
          HAL_UART_Transmit(&huart5, "\n======== END PAGE ========\n", 28, 1000);
          free(page_data);
          break;

        case 's':
          asm volatile ("nop"); // Labels can't point to declarations?
          uint32_t sector_num = str2uint32(&debugMsg[1], DEBUG_RX_BUFFER_SZ_B - 1);
          if (sector_num >= w25qxx.SectorCount) {
            HAL_UART_Transmit(&huart5, "BAD INDEX\n", 10, 1000);
            break;
          }
          uint8_t* sector_data = malloc(w25qxx.SectorSize);
          memset(sector_data, 0, w25qxx.SectorSize);
          W25qxx_ReadSector(sector_data, sector_num, 0, w25qxx.SectorSize);
          HAL_UART_Transmit(&huart5, "\n======== BEGIN SECTOR ========\n", 32, 1000);
          HAL_UART_Transmit(&huart5, sector_data, w25qxx.SectorSize, 10000);
          HAL_UART_Transmit(&huart5, "\n======== END SECTOR ========\n", 30, 1000);
          free(sector_data);
          break;

        case 'b':
          asm volatile ("nop"); // Labels can't point to declarations?
          uint32_t block_num = str2uint32(&debugMsg[1], DEBUG_RX_BUFFER_SZ_B - 1);
          if (block_num >= w25qxx.BlockCount) {
            HAL_UART_Transmit(&huart5, "BAD INDEX\n", 10, 1000);
            break;
          }
          uint8_t* block_data = malloc(w25qxx.BlockSize);
          memset(block_data, 0, w25qxx.BlockSize);
          W25qxx_ReadBlock(block_data, block_num, 0, w25qxx.BlockSize);
          HAL_UART_Transmit(&huart5, "\n======== BEGIN BLOCK ========\n", 31, 1000);
          HAL_UART_Transmit(&huart5, block_data, w25qxx.BlockSize, 25000);
          HAL_UART_Transmit(&huart5, "\n======== END BLOCK ========\n", 29, 1000);
          free(block_data);
          break;
      }
    }

    isDebugMsgReady = 0;
    debugMsgIdx = 0;
    memset(debugMsg, 0, DEBUG_RX_BUFFER_SZ_B + 1);
    HAL_UART_Receive_IT(&huart5, &debugRxChar, 1);
  }
}
