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
//#include "stm32f4xx_ll_usart.h"
//#include "stm32f4xx_hal_rcc.h"
//#include "stm32f4xx_ll_dma.h"
//#include "cmsis_os.h"
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
//	constexpr UARTDriver* Radio = &Driver::uart2;
//	constexpr UARTDriver* Conduit_PBB = &Driver::uart3;
//	// UART 4 (GPS) uses HAL
//	constexpr UARTDriver* Debug = &Driver::uart5;
}

/* SPI Receiver Base Class ------------------------------------------------------------------*/
/**
 * @brief Any classes that are expected to receive using a UART driver
 *		  must derive from this base class and provide an implementation
 *		  for InterruptRxData
 */
class SPIReceiverBase
{
public:
	virtual void InterruptRxData(uint8_t errors) = 0;
};


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

	// Interrupt Functions
	bool ReceiveIT(uint8_t* charBuf, UARTReceiverBase* receiver);


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
