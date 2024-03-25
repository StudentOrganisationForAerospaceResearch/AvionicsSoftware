#ifndef SOAR_SPI_DRIVER_HPP_
#define SOAR_SPI_DRIVER_HPP_
/**
 ******************************************************************************
 * File Name          : SPIDriver.hpp
 * Description        : SPI Driver
 * Author             :
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_adc.h"

/* SPI Driver Instances ------------------------------------------------------------------*/
class SPIDriver;

namespace SPIDriver {
//	extern SPIDriver spi1;

}

/* SPI Driver Aliases ------------------------------------------------------------------*/
namespace SPI {
//	constexpr UARTDriver* Umbilical_RCU = &Driver::uart1;
}


/* SPI Driver Class ------------------------------------------------------------------*/

class SPIDriver
{
public:
//	UARTDriver(USART_TypeDef* uartInstance) :
//		kUart_(uartInstance),
//		rxCharBuf_(nullptr),
//		rxReceiver_(nullptr) {}

	// Polling Functions
	bool Transmit(uint8_t* data, uint16_t len);


protected:
	// Helper Functions
	//	bool HandleAndClearRxError();
	//	bool GetRxErrors();


	// Constants
	//	USART_TypeDef* kUart_; // Stores the UART instance

	// Variables
	uint8_t* rxCharBuf_; // Stores a pointer to the buffer to store the received data
	//	UARTReceiverBase* rxReceiver_; // Stores a pointer to the receiver object
};



#endif // SOAR_SPI_DRIVER_HPP_
