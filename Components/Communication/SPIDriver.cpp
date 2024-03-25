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
	SPI_Handle spi_Handle;
	GPIO_Port gpio_port;
	GPIO_pin gpio_pin;

}

/**
 * @brief Initializes SPI driver for the specified ADC (Master)
 * 		  slaves (barometer,...) are using the read and write
 * 		  thus they are setting the handles
*/
void SPIDriver::Init(SPI_HandleTypeDef *hspi, GPIO_Port gpio_Port, GPIO_Pin gpio_Pin)
{
		// set driver's handle to the given handle
		spi_Handle = hspi;
		gpio_port = gpio_Port;
		gpio_pin = gpio_Pin;

		// // mcp github start - commands need to be changed
		uint8_t cmd[4] = {0,0,0,0};

		// 8-bit CONFIG registers
		cmd[0]  = MCP3561_CONFIG0_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG0;
		insideWrite(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG1_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG1;
		insideWrite(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG2_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG2;
		cmd[1] += 3; // last two bits must always be '11'
		insideWrite(hspi, cmd, 2);

		cmd[0]  = MCP3561_CONFIG3_WRITE;
		cmd[1]  = MCP3561_USERCONF_REG3;
		insideWrite(hspi, cmd, 2);

		cmd[0]  = MCP3561_IRQ_WRITE;
		cmd[1]  = MCP3561_USERCONF_IRQ_REG;
		insideWrite(hspi, cmd, 2);

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
			insideWrite(hspi, cmd, 4);

			reg_val = MCP3561_USERCONF_TIMER_VAL;
			cmd[0] = MCP3561_TIMER_WRITE;
			cmd[1] = (uint8_t)((reg_val >> 16) & 0xff);
			cmd[2] = (uint8_t)((reg_val >>  8) & 0xff);
			cmd[3] = (uint8_t)((reg_val)       & 0xff);
			insideWrite(hspi, cmd, 4);
		#endif

		// mcp github end

}

// inspired by github (modified) ::: was _MCP3561_write
// manually operates !CS signal, since STM32 hardware NSS signal doesn't work
void insideWrite(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t size){
	// port and pin are ADC specific
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, pData, size, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);
}

// taken from github (modified)
// configures the input channels for the ADC based on the specified positive and negative channel numbers
void configureChannels(SPI_HandleTypeDef *hspi, uint8_t pos_Input_Channel, uint8_t neg_Input_Channel){
	uint8_t cmd[4] = {0,0,0,0};
	cmd[0]  = MCP3561_MUX_WRITE;
	cmd[1]  = (pos_Input_Channel << 4) | neg_Input_Channel;   // [7..4] VIN+ / [3..0] VIN-
	//cmd[1]  = (MCP3561_MUX_CH_IntTemp_P << 4) | MCP3561_MUX_CH_IntTemp_M;   // [7..4] VIN+ / [3..0] VIN-
	insideWrite(hspi, cmd, 2);
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
	// Tell the barometer to convert the pressure to a digital value with an over-sampling ratio of 512

	//--- block of interaction using SPI protocol

	// port and pin are ADC specific
	// spi handle is slave specific
	// set GPIO port and pin
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);

	//
	HAL_SPI_Transmit(spi_Handle, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);

	//
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);

	osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(spi_Handle, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

	return true;
}

// start of github
/**
 * @brief prints the configuration registers content
 */
void printRegisters(SPI_HandleTypeDef *hspi){
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

	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, resp, 4, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);

	printf("SCAN : %02x %02x %02x\n", resp[1], resp[2], resp[3]);

	cmd[0] = MCP3561_TIMER_SREAD;

	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, cmd, resp, 4, MCP3561_HAL_TIMEOUT);
	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_SET);

	printf("TIMER: %02x %02x %02x\n", resp[1], resp[2], resp[3]);

	/* @todo all the remaining registers */
}
// end of github
