#pragma once

#define PRELAUNCH_VALVE_PULSE_MAX_TIME (2000)
#define POST_BURN_VALVE_PULSE_MAX_TIME (8000)

extern UART_HandleTypeDef huart1;
extern uint8_t launchCmdReceived;
extern uint8_t systemIsArmed;
extern uint8_t pulseVentValveRequested;

void engineControlTask(void const* arg);
