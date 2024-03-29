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
	SPI_Handle spi_Handle;
	GPIO_Port gpio_port;
	GPIO_pin gpio_pin;

}

/**
 * @brief Initializes SPI driver for the specific slave (barometer,...).
*/
void SPIDriver::Init(SPI_HandleTypeDef *hspi, GPIO_Port gpio_Port, GPIO_Pin gpio_Pin)
{
		// set driver's variables to given slave's parameters
		spi_Handle = hspi;
		gpio_port = gpio_Port;
		gpio_pin = gpio_Pin;

}

/**
 *  Sets the specific configuration for each ADC
 */
void SPIDriver::setConfiguration(CONFIG config){
	// config gives us oversampling ratio
		// adc mode

	// Author - Aly Masani
	uint32_t tempConfigReg0 = MCP3561_USERCONF_REG0;
	uint32_t tempConfigReg1 = MCP3561_USERCONF_REG1;
	uint32_t tempConfigReg2 = MCP3561_USERCONF_REG2;
	uint32_t tempConfigReg3 = MCP3561_USERCONF_REG3;
	uint32_t tempConfigIrqReg = MCP3561_USERCONF_IRQ_REG;
	uint32_t tempConfigScanReg = MCP3561_USERCONF_SCAN_REG;

	//Configure Register 0
	switch (conf.Mode) {
		case "Standby":
			tempConfigReg0 &= ~MCP3561_CONFIG0_ADC_MODE_MASK;
			tempConfigReg0 |= MCP3561_CONFIG0_MODE_STANDBY;
		case "CONV":
			tempConfigReg0 &= ~MCP3561_CONFIG0_ADC_MODE_MASK;
			tempConfigReg0 |= MCP3561_CONFIG0_MODE_CONV;
		case "OFF":
			tempConfigReg0 &= ~MCP3561_CONFIG0_ADC_MODE_MASK;
			tempConfigReg0 |= MCP3561_CONFIG0_MODE_OFF;

	    }

	switch(conf.csSel){
	    	case 0:
	            tempConfigReg0 &= ~MCP3561_CONFIG0_CS_SEL_MASK;
	            tempConfigReg0 |= MCP3561_CONFIG0_CS_SEL_NONE;
	    	case 9:
	            tempConfigReg0 &= ~MCP3561_CONFIG0_CS_SEL_MASK;
	            tempConfigReg0 |= MCP3561_CONFIG0_CS_SEL_0_9uA;
	    	case 37:
	            tempConfigReg0 &= ~MCP3561_CONFIG0_CS_SEL_MASK;
	            tempConfigReg0 |= MCP3561_CONFIG0_CS_SEL_3_7uA;
	    	case 15:
	            tempConfigReg0 &= ~MCP3561_CONFIG0_CS_SEL_MASK;
	            tempConfigReg0 |= MCP3561_CONFIG0_CS_SEL_15uA;

	    }

	//    Configure Register 1
	    switch (conf.OSR) {
	        case 32:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_32;
	            break;
	        case 64:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_64;
	            break;
	        case 128:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_128;
	            break;
	        case 256:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_256;
	            break;
	        case 512:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_512;
	            break;
	        case 1024:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_1024;
	            break;
	        case 2048:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_2048;
	            break;
	        case 4096:
	            tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
	            tempConfigReg1 |= MCP3561_CONFIG1_OSR_4096;
	            break;
	        case 8192:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_8192;
				break;
	        case 16384:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_16384;
				break;
	        case 20480:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_20480;
				break;
	        case 24576:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_24576;
				break;
	        case 40960:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_40960;
				break;
	        case 49152:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_49152;
				break;
	        case 81920:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_81920;
				break;
	        case 98304:
				tempConfigReg1 &= ~MCP3561_CONFIG1_OSR_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_OSR_98304;
				break;
	}
	    switch(conf.amclk){
			case 0:
				tempConfigReg1 &= ~MCP3561_CONFIG1_AMCLK_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_AMCLK_DIV8;
				break;
			case 2:
				tempConfigReg1 &= ~MCP3561_CONFIG2_AMCLK_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_AMCLK_DIV2;
				break;
			case 4:
				tempConfigReg1 &= ~MCP3561_CONFIG1_AMCLK_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_AMCLK_DIV4;
				break;
			case 8:
				tempConfigReg1 &= ~MCP3561_CONFIG2_AMCLK_MASK;
				tempConfigReg1 |= MCP3561_CONFIG1_AMCLK_DIV8;
				break;
	    }

	//Configure Register 2

	    switch(conf.gain){
	    	case 1:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x1;
	    	case 2:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x2;
	    	case 4:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x4;
	    	case 8:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x8;
	    	case 16:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x16;
	    	case 32:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x32;
	    	case 64:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_x64;
	    	case 13:
	    		tempConfigReg2 &= ~MCP3561_CONFIG2_GAIN_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_GAIN_DIV3;
	    }

	    switch(conf.boost){
			case 1:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_BOOST_x1;
			case 2:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_BOOST_x2;
			case 23:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_BOOST_2DIV3;
			case 12:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG2_BOOST_DIV2;
	    }

	//Configure Register 3

	    //	int convMode = 0; //0 is one-shot and shutdown, 1 is one-shot and standby, 2 is continuous.
	    switch(conf.convMode){
			case 0:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_CONV_MODE_ONE_SHOT_OFF;
			case 1:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_CONV_MODE_ONE_SHOT_STANDBY;
			case 2:
				tempConfigReg2 &= ~MCP3561_CONFIG2_BOOST_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_CONV_MODE_CONTINUOUS;
	    }

	    switch(conf.dataFormat){
			case 0:
				tempConfigReg2 &= ~MCP3561_CONFIG3_DATA_FORMAT_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_DATA_FORMAT_24BIT;
			case 1:
				tempConfigReg2 &= ~MCP3561_CONFIG3_DATA_FORMAT_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_DATA_FORMAT_32BIT;
			case 2:
				tempConfigReg2 &= ~MCP3561_CONFIG3_DATA_FORMAT_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_DATA_FORMAT_32BIT_SGN;
			case 3:
				tempConfigReg2 &= ~MCP3561_CONFIG3_DATA_FORMAT_MASK;
				tempConfigReg2 != MCP3561_CONFIG3_DATA_FORMAT_32BIT_CHID_SGN;
	    }

	// end of co-Author


	// // mcp github start - commands need to be changed
	uint8_t cmd[4] = {0,0,0,0};

	// 8-bit CONFIG registers
	cmd[0]  = MCP3561_CONFIG0_WRITE;
	cmd[1]  = MCP3561_USERCONF_REG0;
	internalWrite(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG1_WRITE;
	cmd[1]  = MCP3561_USERCONF_REG1;
	internalWrite(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG2_WRITE;
	cmd[1]  = MCP3561_USERCONF_REG2;
	cmd[1] += 3; // last two bits must always be '11'
	internalWrite(hspi, cmd, 2);

	cmd[0]  = MCP3561_CONFIG3_WRITE;
	cmd[1]  = MCP3561_USERCONF_REG3;
	internalWrite(hspi, cmd, 2);

	cmd[0]  = MCP3561_IRQ_WRITE;
	cmd[1]  = MCP3561_USERCONF_IRQ_REG;
	internalWrite(hspi, cmd, 2);

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
	cmd[0]  = config;
	cmd[1]  = (pos_Input_Channel << 4) | neg_Input_Channel;   // [7..4] VIN+ / [3..0] VIN-

	// might have to have special layout commands for special cases like below
	//cmd[1]  = (MCP3561_MUX_CH_IntTemp_P << 4) | MCP3561_MUX_CH_IntTemp_M;   // [7..4] VIN+ / [3..0] VIN-
	internalWrite(hspi, cmd, 2);
}

/**
 * @brief Transmits data via polling
 * @param data The data to transmit
 * @param len The length of the data to transmit
 * @return True if the transmission was successful, false otherwise
 */
bool SPIDriver::WriteADC(int channel, uint8_t data)
{
	// take needed params from the given param of the upper function
	configureChannels(hspi, pos_Input_Channel, neg_Input_Channel);

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
	// take needed params from the given param of the upper function
	configureChannels(hspi, pos_Input_Channel, neg_Input_Channel);
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

// another github
SPIDriver::MCP3x6x(const uint16_t MCP3x6x_DEVICE_TYPE, const uint8_t pinCS, SPIClass *theSPI,
                 const uint8_t pinMOSI, const uint8_t pinMISO, const uint8_t pinCLK) {
  switch (MCP3x6x_DEVICE_TYPE) {
    case MCP3461_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 2;
      break;
    case MCP3462_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 4;
      break;
    case MCP3464_DEVICE_TYPE:
      _resolution_max = 16;
      _channels_max   = 8;
      break;
    case MCP3561_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 2;
      break;
    case MCP3562_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 4;
      break;
    case MCP3564_DEVICE_TYPE:
      _resolution_max = 24;
      _channels_max   = 8;
      break;
    default:
#warning "undefined MCP3x6x_DEVICE_TYPE"
      break;
  }

  //  settings.id = MCP3x6x_DEVICE_TYPE;

  _spi        = theSPI;
  _pinMISO    = pinMISO;
  _pinMOSI    = pinMOSI;
  _pinCLK     = pinCLK;
  _pinCS      = pinCS;

  _resolution = _resolution_max;
  _channel_mask |= 0xff << _channels_max;  // todo use this one
};
//
