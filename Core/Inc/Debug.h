#pragma once

#include "main.h"
#include "Globals.h"

extern UART_HandleTypeDef DEBUG_UART;

void debugTask(void const* arg);
