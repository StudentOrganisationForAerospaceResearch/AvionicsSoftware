#pragma once

#include "main.h"

void transmitDataTask(void const* arg);

#define GS_CMD_SZ_B 5

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
extern int injectionValveIsOpen;
extern int lowerVentValveIsOpen;
extern uint8_t launchSystemsRxBuf[GS_CMD_SZ_B];
