/**
 ******************************************************************************
 * File Name          : DMAController.hpp
 * Description        : DMA Base class, handles interrupt setup and DMA buffer configuration
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_DMA_CONTROLLER_BASE_H
#define AVIONICS_INCLUDE_SOAR_CORE_DMA_CONTROLLER_BASE_H
/* Includes ------------------------------------------------------------------*/
#include "SystemDefines.hpp"

/* Macros --------------------------------------------------------------------*/

/* Enums -----------------------------------------------------------------*/
enum RegisteredDMA
{
	DMA_UART5 = 0,
	
	DMA_LAST	// Must always be last
};

// What I'm thining right now is that based on https://www.youtube.com/watch?v=Bo6MC5A8uTE we can refer to a particular huart by checking huart->Instance ...
// so we can have an enum and a static array of pointers to the correct DMAController... and then we can just do DMAController::dmaControllers[DMA_UART5]->Ops() or something like that
// might be easier to just make it almost not dependant on the callback though (callback just re-arms the DMA and prints overflow)
// basically... we constantly poll the DMA just using this class...

/* Class -----------------------------------------------------------------*/

// BTW we might want to separate the DMA controller into a base class and a UART class, because we might want to use DMA for other things (like ADC)
// Also, this is the implementation for POLLING from a DMA buffer, if we are able to use interrupts we can use the HAL callbacks instead (basically if we either have pattern match (special terminators) or a fixed length message we can use interrupts)

/**
 * @brief DMA Controller Base class, handles interrupt setup and DMA buffer configuration, derivable for particular peripherals
*/
class DMAController
{
public:
	// Constructors
	DMAController(UART_HandleTypeDef* uartHandle, uint16_t bufferSize);

	// Functions
	uint16_t BlockUntilDataRead(uint8_t*& ptr);
	void Release(uint8_t*& ptr);

	// Internal Functions (should not be called by the user, must be public due to non-member HAL callbacks)
	void RxEventHandler(uint16_t bytesRead);

	// Getters
	uint8_t* GetReadBuffer() { return readBuffer; }
	//uint8_t* GetActiveBuffer() { return activeBuffer; } // would only be used for sanity check

	// Setters

	// Static/Setup

protected:
	// Functions
	void Arm();
	uint16_t FlushActiveToReadBuffer(uint16_t blockTime_ms);
	
	// Member variables
	UART_HandleTypeDef* uartHandle;	// DMA handle
	uint16_t dmaBufferSize;			// Size of the DMA buffer
	uint8_t* activeBuffer;			// Pointer to the active DMA buffer
	uint8_t* readBuffer;			// Pointer to the inactive DMA buffer
	uint16_t readBufferBytes;		// Number of bytes available in the read buffer
	uint16_t activeBufferCount;		// Number of bytes pending in the active buffer

	Mutex readSemaphore;			// Semaphore for the DMA buffer - we use this to signal when the DMA buffer has data (LOCKED = no data, UNLOCKED = data)
	Mutex readBusy;					// Mutex for DMA Buffer - we use this to check if the DMA buffer is currently being read

private:

};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_DMA_CONTROLLER_BASE_H */