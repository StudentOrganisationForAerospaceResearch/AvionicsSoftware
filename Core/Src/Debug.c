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
#define UART5_PAGE_TX_TIMEOUT ((w25qxx.PageSize * 8 / huart5.Init.BaudRate) * 1000 * 10)
#define UART5_SECTOR_TX_TIMEOUT ((w25qxx.SectorSize * 8 / huart5.Init.BaudRate) * 1000 * 10)

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
static const int DEBUG_TASK_PERIOD = 100;
static const int RX_BUFFER_SZ_B = 8;

/* Variables -----------------------------------------------------------------*/

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
  uint8_t rx_buffer[RX_BUFFER_SZ_B + 1]; // Ensure last byte always 0

  uint8_t initStatus = W25qxx_Init();

  while (1) {
    osDelayUntil(&prevWakeTime, DEBUG_TASK_PERIOD);

    memset(rx_buffer, 0, RX_BUFFER_SZ_B);
    for (uint8_t i = 0; i < RX_BUFFER_SZ_B; i++) {
      HAL_UART_Receive(&huart5, &rx_buffer[i], 1, 0);
      if (rx_buffer[i] < '0' || rx_buffer[i] > '9') { // If not an ASCII number character...
        rx_buffer[i] |= 0x20; // Set bit 5, so capital ASCII letters are now lowercase
        if (rx_buffer[i] < 'a' || rx_buffer[i] > 'z') { // If not an ASCII lowercase letter...
          rx_buffer[i] = 0; // Terminate the string
          break;
        }
      }
    }

    switch (rx_buffer[0]) {
      case 'e':
        if (strcmp(rx_buffer, "erase") == 0) {
          HAL_UART_Transmit(&huart5, "ERASING\n", 8, 1000);
          W25qxx_EraseChip();
          HAL_UART_Transmit(&huart5, "ERASED\n", 7, 1000);
        }
        break;

      case 'p':
        asm volatile ("nop"); // Labels can't point to declarations?
        uint32_t page_num = str2uint32(&rx_buffer[1], RX_BUFFER_SZ_B - 1);
        if (page_num >= w25qxx.PageCount) {
          HAL_UART_Transmit(&huart5, "BAD INDEX\n", 10, 1000);
        }
        uint8_t* page_data = malloc(w25qxx.PageSize);
        memset(page_data, 0, w25qxx.PageSize);
        W25qxx_ReadPage(page_data, page_num, 0, w25qxx.PageSize);
        HAL_UART_Transmit(&huart5, "\n======== BEGIN PAGE ========\n", 30, 1000);
        HAL_UART_Transmit(&huart5, page_data, w25qxx.PageSize, UART5_PAGE_TX_TIMEOUT);
        HAL_UART_Transmit(&huart5, "\n======== END PAGE ========\n", 28, 1000);
        free(page_data);
        break;

      case 's':
        asm volatile ("nop"); // Labels can't point to declarations?
        uint32_t sector_num = str2uint32(&rx_buffer[1], RX_BUFFER_SZ_B - 1);
        if (sector_num >= w25qxx.SectorCount) {
          HAL_UART_Transmit(&huart5, "BAD INDEX\n", 10, 1000);
        }
        uint8_t* sector_data = malloc(w25qxx.SectorSize);
        memset(sector_data, 0, w25qxx.SectorSize);
        W25qxx_ReadSector(sector_data, sector_num, 0, w25qxx.SectorSize);
        HAL_UART_Transmit(&huart5, "\n======== BEGIN SECTOR ========\n", 32, 1000);
        HAL_UART_Transmit(&huart5, sector_data, w25qxx.SectorSize, UART5_SECTOR_TX_TIMEOUT);
        HAL_UART_Transmit(&huart5, "\n======== END SECTOR ========\n", 30, 1000);
        free(sector_data);
        break;
    }
  }
}
