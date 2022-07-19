/**
  ******************************************************************************
  * @file           : Macros.h
  * @brief          : Macros and wrappers
  ******************************************************************************
  *
  * Contains system wide macros, defines, and wrappers
  *
  ******************************************************************************
  */
#ifndef SOAR_MAIN_SYSTEM_DEFINES_H
#define SOAR_MAIN_SYSTEM_DEFINES_H

//--------------------- TYPE DEFINITIONS --------------------------


//--------------------- SYSTEM DEFINES --------------------------


//---------------------STM32 HAL C++ WRAPPERS--------------------------


inline //Wraps HAL_GPIO_WritePin with a non-explicit GPIO_PinState
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int state)
{
	// Call the underlying HAL_GPIO_WritePin function with the value explicitly cast
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, (GPIO_PinState)state);
}

//TODO: Check this wrapper is valid!
inline //Wraps HAL_UART_Transmit with a pointer to a data pointer
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t **pData, uint16_t Size, uint32_t Timeout)
{
	// Call the underlying HAL_UART_Transmit function with the dereferenced pointer
	return HAL_UART_Transmit(huart, *pData, Size, Timeout);
}




#endif // SOAR_MAIN_SYSTEM_DEFINES_H
