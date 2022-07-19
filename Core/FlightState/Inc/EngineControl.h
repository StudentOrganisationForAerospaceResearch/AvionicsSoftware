#pragma once

extern UART_HandleTypeDef huart1;
extern uint8_t launchCmdReceived;
extern uint8_t systemIsArmed;
extern uint8_t pulseVentValveRequested;

void engineControlTask(void const* arg);
