#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ValveControl.h"

int upperVentValveIsOpen = 0;
int injectionValveIsOpen = 0;
int lowerVentValveIsOpen = 0;

void closeUpperVentValve()
{
    // Unpowered is closed
    HAL_GPIO_WritePin(VENT_VALVE_GPIO_Port, VENT_VALVE_Pin, GPIO_PIN_RESET);
    upperVentValveIsOpen = 0;
}
// High pulse is sent to change state of injection valve.
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

void openLowerVentValve()
{
    HAL_GPIO_WritePin(LAUNCH_GPIO_Port, LAUNCH_Pin, GPIO_PIN_SET);      //Temporary pin until we change to using new board
    lowerVentValveIsOpen = 1;
}

void closeLowerVentValve()
{
    HAL_GPIO_WritePin(LAUNCH_GPIO_Port, LAUNCH_Pin, GPIO_PIN_RESET);    //Temporary pin until we change to using new board
    lowerVentValveIsOpen = 0;  
}
