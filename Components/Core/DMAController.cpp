/**
 ******************************************************************************
 * File Name          : DMAController.cpp
 * Description        : DMA Controler implementation
 ******************************************************************************
*/
#include "DMAController.hpp"
#include "SystemDefines.hpp"

#include <cstring>

#include "stm32f4xx_hal.h"

/* File Defines --------------------------------------------------------------------*/
constexpr uint16_t BLOCK_TIME_BUSY_MS = 25;		//Time to wait for read to be not busy from a task
constexpr uint16_t BLOCK_TIME_FLUSH_ISR_MS = 0;		//There is no point blocking in an ISR for the mutex
constexpr uint16_t BLOCK_TIME_FLUSH_TASK_MS = 50;	//Time to wait to flush buffer from a task

/* HAL Callbacks --------------------------------------------------------------------*/
/* Note: We use this instead of HAL_UART_RegisterCallback with a static because CubeIDE */
/* Doesn't have a great way of enabling the macro USER_CALLBACK macro along with other IDEs */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//	// We will assume the DMA has a linked controller (otherwise it should NEVER be armed!)
//	DMAController* dmaController = (DMAController*)huart->hdmarx->Parent;
//
//	// Call the DMAController's handler for a full buffer
//
//
//}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	// We will assume the DMA has a linked controller (otherwise it should NEVER be armed!)
	DMAController* dmaController = (DMAController*)huart->Controller;

	//TODO: Sanity check, controller buffer is the same as the handle buffer ?? is the UART handle buffer supposed to be the same??

	// Call the DMAController's handler
	dmaController->RxEventHandler( size);
}

/* DMAController --------------------------------------------------------------------*/
//TODO: Somewhat difficult to make DMA base class, I'm going to try to make this UART only for now we could have a union Handle later as a param.. or a template class
//TODO: and also just for RX, because it was originally going to support either RX or TX per controller... but yeah that's not going to work if we need the full handle we need to do both or make 2 different classes
/**
 * @brief Constructor with the DMA handle and DMA buffer size
 */
DMAController::DMAController(UART_HandleTypeDef* uartHandle, uint16_t bufferSize)
{
	// This doesn't work... it's parent is the UART Handle itself :p
	SOAR_ASSERT(uartHandle->hdmarx != nullptr, "Cannot create DMAController with a UART handle that does not have a DMA Rx handle!"); //fyi if this fails take in the hdma and set it here...
	uartHandle->Controller = this;

	// Set member variables and objects
	this->uartHandle = uartHandle;
	this->dmaBufferSize = bufferSize;

	// Each DMA controller has two buffers, one for the active DMA buffer and the read from buffer, allocate these
	activeBuffer = (uint8_t*)soar_malloc(bufferSize);
	readBuffer = (uint8_t*)soar_malloc(bufferSize);

	// Disable the HT interrupt since we don't need it
	__HAL_DMA_DISABLE_IT(uartHandle->hdmarx, DMA_IT_HT);

	// Ready DMA for the first transfers
	readSemaphore.Lock(); // mark that we have no data
	this->Arm();
}

/**
 * @brief Blocks until data is read, returns the number of bytes read and available in the buffer.
 * If this data isn't read BEFORE the next set of data, then the data will be gone forever
 * You MUST quickly process this data and call Release() otherwise there will be lost data
 *
 * @param ptr Sets to the read data pointer
 * @return Number of bytes read, can be zero. If zero, ignore data and run Release()
 */
uint16_t DMAController::BlockUntilDataRead(uint8_t*& ptr)
{
	if(readSemaphore.Lock(MAX_DELAY_MS) && readBusy.Lock(BLOCK_TIME_BUSY_MS)) {
		ptr = readBufferBytes ? readBuffer : nullptr;	// assign ptr to readBuffer if data available, otherwise assign to nullptr
		return readBufferBytes;
	}

	SOAR_ASSERT(false, "DMAController mutex acquisition failed"); //TODO: Not sure if we want this

	return 0;
}

/**
 * @brief Releases the read data storage, after the read data is released data should NOT be read external to the DMA Controller 
 */
void DMAController::Release(uint8_t*& ptr)
{
	SOAR_ASSERT(ptr == readBuffer, "Correct pointer needs to be returned to Release");
	ptr = nullptr;

	//Release the buffer
	readBufferBytes = 0;
	readBusy.Unlock(); //buffer is no longer busy

	//Try flushing active buffer, if we don't have any space then we need to keep trying to flush this (act like we have data so the task continues)
	if (FlushActiveToReadBuffer(BLOCK_TIME_FLUSH_TASK_MS) == 0)
		readSemaphore.Unlock();
}

/**
 * @brief Handles a read event interrupt (IDLE line, end of buffer, pattern match)
 *  Not meant to be called by the user, but by the HAL
 */
void DMAController::RxEventHandler(uint16_t bytesRead)
{
	// Try to flush the data to the read buffer within the ISR
	this->activeBufferCount += bytesRead;
	FlushActiveToReadBuffer(BLOCK_TIME_FLUSH_ISR_MS);

	//Re-arm the DMA
	this->Arm(); 
}

/**
 * @brief Arm the DMA based on the current data index in the buffer
 */
void DMAController::Arm()
{
	uint16_t bufLeft = dmaBufferSize - activeBufferCount;

	// If we still have space, restart DMA with remaining space
	if(bufLeft > 0)
		HAL_UARTEx_ReceiveToIdle_DMA(uartHandle, &activeBuffer[activeBufferCount], bufLeft);
	else {
		// We have a full buffer, and the read is busy, we allow the system to resume even with no data to allow the reading task to flush the active buffer
		readSemaphore.Unlock();

		//TODO: This is worse case scenario, to prevent stall we have to discard data
		//if(FlushActiveToReadBuffer() == 0) {
		//	HAL_UARTEx_ReceiveToIdle_DMA(uartHandle, activeBuffer, bufLeft);
		//	SOAR_PRINT("SEVERE WARNING: DMA Controller Discarded Data");
		//}
		//else {
		//	HAL_UARTEx_ReceiveToIdle_DMA(uartHandle, &activeBuffer[activeBufferCount], bufLeft);
		//}
	}
}

/**
 * @brief Flushes data from the active buffer to the read buffer. Best effort, if the readBuffer is busy then we can't do anything.
 * @param Max block time in milliseconds, if this is an ISR it MUST be 0 or very close to 0
 * @return Bytes left available in the DMA active buffer
 */
uint16_t DMAController::FlushActiveToReadBuffer(uint16_t blockTime_ms)
{
	//If the readBuffer is not busy, flush the data to the readBuffer
	if (activeBufferCount > 0 && readBusy.Lock(blockTime_ms)) {
		// Assuming this is called from the interrupt, we need to copy the data to the read buffer, and internally signal data pending
		memcpy(readBuffer, activeBuffer, activeBufferCount);
		readBufferBytes = activeBufferCount;
		activeBufferCount = 0;
		readSemaphore.Unlock(); // mark that we have data
		readBusy.Unlock();		// mark that we are no longer busy
	}

	return dmaBufferSize - activeBufferCount;
}
