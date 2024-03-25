/**
 ******************************************************************************
 * File Name          : SPIDriver.cpp
 * Description        : SPI Driver
 * Author             :
 ******************************************************************************
 *
 * Notes:
 *
 ******************************************************************************
*/
#include "SPIDriver.hpp"
#include "main_avionics.hpp"

// Declare the global SPI driver objects
namespace Driver {
//    UARTDriver uart1(USART1);
//    UARTDriver uart2(USART2);
//    UARTDriver uart3(USART3);
//	UARTDriver uart5(UART5);
}

/**
 * @brief Transmits data via polling
 * @param data The data to transmit
 * @param len The length of the data to transmit
 * @return True if the transmission was successful, false otherwise
 */
bool UARTDriver::Transmit(uint8_t* data, uint16_t len)
{
//	// Loop through and transmit each byte via. polling
//	for (uint16_t i = 0; i < len; i++) {
//		LL_USART_TransmitData8(kUart_, data[i]);
//
//		// Wait until the TX Register Empty Flag is set
//		while (!LL_USART_IsActiveFlag_TXE(kUart_)) {}
//	}
//
//	// Wait until the transfer complete flag is set
//	while (!LL_USART_IsActiveFlag_TC(kUart_)) {}

	return true;
}


/**
* @brief Receives 1 byte of data via interrupt
* @param receiver
* @return TRUE if interrupt was successfully enabled, FALSE otherwise
*/
bool UARTDriver::ReceiveIT(uint8_t* charBuf, UARTReceiverBase* receiver)
{
//	// Check flags
//	HandleAndClearRxError();
//	if (LL_USART_IsActiveFlag_RXNE(kUart_)) {
//		LL_USART_ClearFlag_RXNE(kUart_);
//	}
//
//	// Set the buffer and receiver
//	rxCharBuf_ = charBuf;
//	rxReceiver_ = receiver;
//
//	// Enable the receive interrupt
//	LL_USART_EnableIT_RXNE(kUart_);

	return true;
}
