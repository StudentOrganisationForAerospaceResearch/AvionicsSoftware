
#ifndef FAKEHALTYPES_HPP
#define FAKEHALTYPES_HPP

#include <stdint.h>

/* Constants ---------------------------------------------------------------- */

/* Types -------------------------------------------------------------------- */

typedef enum {
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
  uint8_t fakeVal;
} UART_HandleTypeDef;

typedef struct {
  uint8_t fakeVal;
} SPI_HandleTypeDef;

typedef struct {
  uint8_t fakeVal;
} ADC_HandleTypeDef;

typedef struct {
  uint8_t fakeVal;
} DMA_HandleTypeDef;

typedef struct {
  uint8_t fakeVal;
} CRC_HandleTypeDef;

/* Variables ---------------------------------------------------------------- */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern CRC_HandleTypeDef hcrc;

/* Declarations ------------------------------------------------------------- */

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size,
                                    uint32_t Timeout);

void HAL_NVIC_SystemReset();

#endif