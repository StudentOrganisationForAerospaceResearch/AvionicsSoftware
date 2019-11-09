#pragma once
#include "Data.h"
void readGpsTask(void const* arg);

extern UART_HandleTypeDef huart4;

extern GpsData* gpsData;
extern int arrayNumber;
extern char test;
