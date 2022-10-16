/**
 ******************************************************************************
 * File Name          : DMAController.cpp
 * Description        : DMA Controler implementation
 ******************************************************************************
*/
#include "DMAController.hpp"
#include "stm32f4xx_hal.h"

/**
 * @brief Constructor with the DMA handle and DMA buffer size
 */
DMAController::DMAController(DMA_HandleTypeDef* dmaHandle, uint16_t bufferSize)
{
	// Set member variables
	this->dmaHandle = dmaHandle;
	this->dmaBufferSize = bufferSize;

	// Each DMA controller has two buffers, one for the current DMA transfer and one for the next, allocate these
	activeBuffer = (uint8_t*)soar_malloc(bufferSize);
	nextBuffer = (uint8_t*)soar_malloc(bufferSize);

	// Ready DMA for the first transfers
}

/**
 * @brief Sets up HAL callbacks
 */
void DMAController::SetupCallbacks()
{
	//HAL_UART_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_CPLT_CB_ID, &DMAController::DMACompleteCallback);
}
