#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ValveControl.h"

int ventValveIsOpen = 0;
int injectionValveIsOpen = 0;

void openVentValve()
{
    // Powered is open
    HAL_GPIO_WritePin(VENT_VALVE_GPIO_Port, VENT_VALVE_Pin, GPIO_PIN_SET);
    ventValveIsOpen = 1;
}

void closeVentValve()
{
    // Unpowered is closed
    HAL_GPIO_WritePin(VENT_VALVE_GPIO_Port, VENT_VALVE_Pin, GPIO_PIN_RESET);
    ventValveIsOpen = 0;
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