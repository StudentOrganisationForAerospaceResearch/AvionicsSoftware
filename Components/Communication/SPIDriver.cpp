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
namespace SPIDriver {
//    UARTDriver uart1(USART1);
//	  SPIDriver spi(SPI);

}

/**
 * @brief Initializes SPI driver for the specified ADC
*/
void SPIDriver::InitTask(SPI_HandleTypeDef *hspi, Command& cm)
{
		// // mcp github start - commands need to be changed
		uint8_t cmd[4] = {0,0,0,0};

		// 8-bit CONFIG registers
		cmd[0]  = MCP3561_CONFIG0_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG0;
		_MCP3561_write(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG1_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG1;
		_MCP3561_write(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG2_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG2;
		cmd[1] += 3; // last two bits must always be '11'
		_MCP3561_write(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG3_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG3;
		_MCP3561_write(hspi, cmd, 2);

		cmd[0]  = MCP3561_IRQ_WRITE;
		cmd[1]  = MCP3561_USERCONF_IRQ_REG;
		_MCP3561_write(hspi, cmd, 2);

		// 24-bit CONFIG registers

		// configure SCAN mode to automatically cycle through channels
		// only available for MCP3562 and MCP3564, and only for certain input combinations
		// @see Datasheet Table 5-14 on p. 54
		#ifdef MCP3561_USERCONF_SCAN_ENABLE
			uint32_t reg_val;
			reg_val = MCP3561_USERCONF_SCAN_REG;
			cmd[0] = MCP3561_SCAN_WRITE;
			cmd[1] = (uint8_t)((reg_val >> 16) & 0xff);
			cmd[2] = (uint8_t)((reg_val >>  8) & 0xff);
			cmd[3] = (uint8_t)((reg_val)       & 0xff);
			_MCP3561_write(hspi, cmd, 4);

			reg_val = MCP3561_USERCONF_TIMER_VAL;
			cmd[0] = MCP3561_TIMER_WRITE;
			cmd[1] = (uint8_t)((reg_val >> 16) & 0xff);
			cmd[2] = (uint8_t)((reg_val >>  8) & 0xff);
			cmd[3] = (uint8_t)((reg_val)       & 0xff);
			_MCP3561_write(hspi, cmd, 4);
		#endif

		// mcp github end

}

/**
 * @brief Transmits data via polling
 * @param data The data to transmit
 * @param len The length of the data to transmit
 * @return True if the transmission was successful, false otherwise
 */
bool SPIDriver::WriteADC(int channel)
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
bool SPIDriver::ReadADC(int channel)
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

// start of github
/**
 * @brief prints the configuration registers content
 */
void MCP3561_PrintRegisters(SPI_HandleTypeDef *hspi){
	uint8_t reg8 = 0;
	uint8_t cmd [5] = {0,0,0,0,0};

	cmd[0] = MCP3561_CONFIG0_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF0: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG1_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF1: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG2_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF2: %02x\n", reg8);

	cmd[0] = MCP3561_CONFIG3_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("CONF3: %02x\n", reg8);

	cmd[0] = MCP3561_IRQ_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("IRQ  : %02x\n", reg8);

	cmd[0] = MCP3561_MUX_SREAD;
	reg8 = _MCP3561_sread(hspi, cmd);
	printf("MUX  : %02x\n", reg8);

	cmd[0] = MCP3561_SCAN_SREAD;
	uint8_t resp [5] = {0,0,0,0,0};

	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, resp, 4, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);

	printf("SCAN : %02x %02x %02x\n", resp[1], resp[2], resp[3]);

	cmd[0] = MCP3561_TIMER_SREAD;

	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, resp, 4, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(MCP3561_CHIP_SELECT_GPIO_Port, MCP3561_CHIP_SELECT_GPIO_Pin, GPIO_PIN_SET);

	printf("TIMER: %02x %02x %02x\n", resp[1], resp[2], resp[3]);

	/* @todo all the remaining registers */


}
// end of github
