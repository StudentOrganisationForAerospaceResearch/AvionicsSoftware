
#include "FakeHalTypes.hpp"

/* Variables ---------------------------------------------------------------- */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
CRC_HandleTypeDef hcrc;

/* Functions ---------------------------------------------------------------- */

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout) {
  return HAL_OK;
}

void HAL_NVIC_SystemReset() {}
