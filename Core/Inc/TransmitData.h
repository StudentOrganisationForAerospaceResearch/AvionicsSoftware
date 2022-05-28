#pragma once

void transmitDataTask(void const* arg);

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
extern int injectionValveIsOpen;
extern int lowerVentValveIsOpen;
