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
	DMAController(void);

	// Functions

	// Getters

	// Setters

	// Static/Setup
	static void SetupCallbacks();	// Register the DMA callbacks

protected:

private:

};

#endif /* AVIONICS_INCLUDE_SOAR_CORE_DMA_CONTROLLER_BASE_H */