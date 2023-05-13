
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_crc.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"

UART_HandleTypeDef huart1;   
UART_HandleTypeDef huart2;   
UART_HandleTypeDef huart4;   
UART_HandleTypeDef huart5;   

ADC_HandleTypeDef hadc1;      
ADC_HandleTypeDef hadc2;      

SPI_HandleTypeDef hspi1;      
SPI_HandleTypeDef hspi3;      

CRC_HandleTypeDef hcrc;       

DMA_HandleTypeDef hdma_uart4_rx; 
DMA_HandleTypeDef hdma_uart5_rx; 
DMA_HandleTypeDef hdma_uart5_tx; 