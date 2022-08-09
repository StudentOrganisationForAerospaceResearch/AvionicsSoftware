/**
 ******************************************************************************
 * File Name          : GPIO.hpp
 * Description        :
 *
 *	GPIO contains all GPIO pins wrapped in a namespace and corresponding functions
 *
 *	All GPIO pins should be controlled through this abstraction layer to ensure readable control.
 *
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_GPIO_H
#define AVIONICS_INCLUDE_SOAR_CORE_GPIO_H
#include "SystemDefines.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"

namespace GPIO
{
	namespace LED1
	{
		GPIO_TypeDef* GPIOx = LED_1_GPIO_Port;
		constexpr uint16_t GPIO_Pin = LED_1_Pin;

		void On() { HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); }
		void Off() { HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); }
		void Toggle() { HAL_GPIO_TogglePin(GPIOx, GPIO_Pin); }
	}
	
}


#endif /* AVIONICS_INCLUDE_SOAR_CORE_GPIO_H */