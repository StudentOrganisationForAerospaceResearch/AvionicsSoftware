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

/* Class -----------------------------------------------------------------*/

/**
 * @brief DMA Controller Base class, handles interrupt setup and DMA buffer configuration, derivable for particular peripherals
*/
class DMAController
{
public:
	// Constructors
	DMAController(DMA_HandleTypeDef* dmaHandle, uint16_t bufferSize);

	// Functions

	// Getters

	// Setters

	// Static/Setup
	static void SetupCallbacks();	// Register the DMA callbacks

protected:
	DMA_HandleTypeDef* dmaHandle;	// DMA handle
	uint16_t dmaBufferSize;			// Size of the DMA buffer
	uint8_t* activeBuffer;			// Pointer to the active DMA buffer
	uint8_t* nextBuffer;			// Pointer to the next DMA buffer

private:

};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_DMA_CONTROLLER_BASE_H */