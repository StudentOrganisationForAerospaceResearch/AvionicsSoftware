#pragma once

#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t launchSystemsRxChar;

void debugTask(void const* arg);
