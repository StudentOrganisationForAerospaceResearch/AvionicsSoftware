/**
 ******************************************************************************
 * File Name          : UARTDriver.cpp
 * Description        : UART Driver
 * References	      : Based in part on MaJerle's stm32-usart-uart-dma-rx-tx
 *						https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx/blob/main/projects/usart_rx_idle_line_irq_rtos_F4/Src/main.c
 * Author             : cjchanx (Chris)
 ******************************************************************************
*/
#include "UARTDriver.hpp"
#include "main_avionics.hpp"

// Declare the global UART driver objects
namespace Driver {
    UARTDriver uart1(USART1);
    UARTDriver uart2(USART2);
}

/**
 * @brief Transmits data via polling
 * @param data The data to transmit
 * @param len The length of the data to transmit
 * @return True if the transmission was successful, false otherwise
 */
bool UARTDriver::Transmit(uint8_t* data, uint16_t len)
{
	// Loop through and transmit each byte via. polling
	for (uint16_t i = 0; i < len; i++) {
		LL_USART_TransmitData8(kUart_, data[i]);

		// Wait until the TX Register Empty Flag is set
		while (!LL_USART_IsActiveFlag_TXE(kUart_)) {}
	}

	// Wait until the transfer complete flag is set
	while (!LL_USART_IsActiveFlag_TC(kUart_)) {}

	return true;
}

/**
* @brief Receives 1 byte of data via interrupt
* @param charBuf The buffer to store the received data
* @param fxCallback The function to call when the data is received
* @return TRUE if interrupt was successfully enabled, FALSE otherwise
*/
bool UARTDriver::ReceiveIT(uint8_t* charBuf, UARTReceiverBase* receiver)
{
//	// Check if the interrupt is enabled, if so disable it
//	if (LL_USART_IsEnabledIT_RXNE(kUart_)) {
//		LL_USART_DisableIT_RXNE(kUart_);
//		//SOAR_PRINT("Warning, ReceiveIT called while interrupt was enabled, disabling interrupt\n");
//	}

	// Check flags
	HandleAndClearRxError();
	if (LL_USART_IsActiveFlag_RXNE(kUart_)) {
		//SOAR_PRINT("Warning, RXNE was not cleared before call to ReceiveIT(), likely loss of data\n");
		LL_USART_ClearFlag_RXNE(kUart_);
	}

	// Set the buffer and receiver
	rxCharBuf_ = charBuf;
	rxReceiver_ = receiver;

	// Enable the receive interrupt
	LL_USART_EnableIT_RXNE(kUart_);

	return true;
}

/**
 * @brief Clears any error flags that may have been set, printing a warning message if necessary
 * @return true if flags had to be cleared, false otherwise
 */
bool UARTDriver::HandleAndClearRxError()
{
	bool shouldClearFlags = false;
	if (LL_USART_IsActiveFlag_ORE(kUart_)) {
		//SOAR_PRINT("Warning, overrun error detected\n");
		shouldClearFlags = true;
	}
	if (LL_USART_IsActiveFlag_NE(kUart_)) {
		//SOAR_PRINT("Warning, noise error detected\n");
		shouldClearFlags = true;
	}
	if(LL_USART_IsActiveFlag_FE(kUart_)) {
		//SOAR_PRINT("Warning, framing error detected\n");
		shouldClearFlags = true;
	}
	if(LL_USART_IsActiveFlag_PE(kUart_)) {
		//SOAR_PRINT("Warning, parity error detected\n");
		shouldClearFlags = true;
	}

	// Clearing the ORE here also clears PE, NE, FE, IDLE
	if(shouldClearFlags)
		LL_USART_ClearFlag_ORE(kUart_);

	return !shouldClearFlags;
}

/**
 * @brief Checks UART Rx error flags, if any are set returns true
 * @return true if any error flags are set, false otherwise
 */
bool UARTDriver::GetRxErrors()
{
	bool hasErrors = false;

	if (LL_USART_IsActiveFlag_ORE(kUart_)) {
		hasErrors = true;
	}
	else if (LL_USART_IsActiveFlag_NE(kUart_)) {
		hasErrors = true;
	}
	else if(LL_USART_IsActiveFlag_FE(kUart_)) {
		hasErrors = true;
	}
	else if(LL_USART_IsActiveFlag_PE(kUart_)) {
		hasErrors = true;
	}

	return hasErrors;
}

/**
 * @brief Handles an interrupt for the UART
 * @attention MUST be called inside USARTx_IRQHandler
 */
void UARTDriver::HandleIRQ_UART()
{
	// Call the callback if RXNE is set
	if (LL_USART_IsActiveFlag_RXNE(kUart_)) {
		// Read the data from the data register
		if (rxCharBuf_ != nullptr) {
			*rxCharBuf_ = LL_USART_ReceiveData8(kUart_);
		}

		// Call the receiver interrupt
		if(rxReceiver_ != nullptr) {
			rxReceiver_->InterruptRxData(GetRxErrors());
		}
	}
}
