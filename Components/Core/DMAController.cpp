/**
 ******************************************************************************
 * File Name          : DMAController.cpp
 * Description        : DMA Controler implementation
 ******************************************************************************
*/
#include "DMAController.hpp"
#include "stm32f4xx_hal.h"

/**
 * @brief Default constructor
 */
DMAController::DMAController() {}

/**
 * @brief Sets up HAL callbacks
 */
void DMAController::SetupCallbacks() {
  //HAL_UART_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_CPLT_CB_ID, &DMAController::DMACompleteCallback);
}
