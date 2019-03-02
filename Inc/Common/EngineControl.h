#pragma once

extern UART_HandleTypeDef huart1;
extern uint8_t launchCmdReceived;

void engineControlTask(void const* arg);
