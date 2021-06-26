#pragma once

#include "main.h"
#include "LogData.h"

extern UART_HandleTypeDef huart5;

void debugTask(void const* arg);
