#pragma once

#include "main.h"

#define DEBUG_RX_BUFFER_SZ_B 16

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t debugRxChar;
extern uint8_t debugMsg[DEBUG_RX_BUFFER_SZ_B + 1];
extern uint8_t isDebugMsgReady;
extern uint8_t debugMsgIdx;

void debugTask(void const* arg);
