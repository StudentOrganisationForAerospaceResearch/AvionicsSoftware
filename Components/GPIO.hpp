/**
 ******************************************************************************
 * File Name          : GPIO.hpp
 * Description        :
 *
 *    GPIO contains all GPIO pins wrapped in a namespace and corresponding functions
 *
 *    All GPIO pins should be controlled through this abstraction layer to ensure readable control.
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
        inline void On() { HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); }
        inline void Off() { HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); }
        inline void Toggle() { HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin); }

        inline bool IsOn() { return HAL_GPIO_ReadPin(LED_1_GPIO_Port, LED_1_Pin) == GPIO_PIN_SET; }
    }

    namespace LED2
    {
        inline void On() { HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); }
        inline void Off() { HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); }
        inline void Toggle() { HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin); }

        inline bool IsOn() { return HAL_GPIO_ReadPin(LED_2_GPIO_Port, LED_2_Pin) == GPIO_PIN_SET; }
    }
    
    namespace LED3
    {
        inline void On() { HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); }
        inline void Off() { HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); }
        inline void Toggle() { HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin); }

        inline bool IsOn() { return HAL_GPIO_ReadPin(LED_3_GPIO_Port, LED_3_Pin) == GPIO_PIN_SET; }
    }

	namespace LED3
	{
		inline void On() { HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(LED_3_GPIO_Port, LED_3_Pin) == GPIO_PIN_SET; }
	}
	
	namespace Vent
	{
		inline void Open() { HAL_GPIO_WritePin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin, GPIO_PIN_RESET); }
		inline void Close() { HAL_GPIO_WritePin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin, GPIO_PIN_SET); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin) == GPIO_PIN_RESET; }
	}	

	namespace Drain
	{
		inline void Open() { HAL_GPIO_WritePin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin, GPIO_PIN_RESET); }
		inline void Close() { HAL_GPIO_WritePin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin, GPIO_PIN_SET); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL_CTRL_GPIO_Port, SOL_CTRL_Pin) == GPIO_PIN_RESET; }
	}	

	namespace PowerSelect
	{
		inline void InternalPower() { HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_SET); }
		inline void UmbilicalPower() { HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(BAT_EN_GPIO_Port, BAT_EN_Pin); }
		
		inline bool IsInternal() { return HAL_GPIO_ReadPin(BAT_EN_GPIO_Port, BAT_EN_Pin) == GPIO_PIN_SET; }
	}
}

#endif /* AVIONICS_INCLUDE_SOAR_CORE_GPIO_H */
