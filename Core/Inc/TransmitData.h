#pragma once

#include "Globals.h"

void transmitDataTask(void const* arg);

extern UART_HandleTypeDef RADIO_UART;
extern UART_HandleTypeDef GS_UART;
extern CRC_HandleTypeDef hcrc;
extern int injectionValveIsOpen;
extern int lowerVentValveIsOpen;
