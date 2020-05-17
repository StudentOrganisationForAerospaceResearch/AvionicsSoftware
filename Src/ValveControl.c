/**
  ******************************************************************************
  * File Name          : ValveControl.c
  * Description        : Code for...
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ValveControl.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
int ventValveIsOpen = 0;
int injectionValveIsOpen = 0;
int lowerVentValveIsOpen = 0;

/* Functions -----------------------------------------------------------------*/
// Injection Valve is Normally Closed (NC)
void openInjectionValve()
{
    // Powered is open
    HAL_GPIO_WritePin(INJECTION_VALVE_GPIO_Port, INJECTION_VALVE_Pin, GPIO_PIN_SET);
    injectionValveIsOpen = 1;
}

void closeInjectionValve()
{
    // Unpowered is closed
    HAL_GPIO_WritePin(INJECTION_VALVE_GPIO_Port, INJECTION_VALVE_Pin, GPIO_PIN_RESET);
    injectionValveIsOpen = 0;
}

// Lower vent valve is Normally Open (NO)
void openLowerVentValve()
{
    HAL_GPIO_WritePin(LOWER_VENT_VALVE_GPIO_Port, LOWER_VENT_VALVE_Pin, GPIO_PIN_RESET);      //Temporary pin until we change to using new board
    lowerVentValveIsOpen = 1;
}

void closeLowerVentValve()
{
    HAL_GPIO_WritePin(LOWER_VENT_VALVE_GPIO_Port, LOWER_VENT_VALVE_Pin, GPIO_PIN_SET);    //Temporary pin until we change to using new board
    lowerVentValveIsOpen = 0;
}
