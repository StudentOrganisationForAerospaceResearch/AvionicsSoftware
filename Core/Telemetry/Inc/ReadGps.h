#pragma once
#include "../../Telemetry/Inc/Data.h"

void readGpsTask(void const *arg);

extern UART_HandleTypeDef huart4;

extern char dma_rx_buffer[NMEA_MAX_LENGTH + 1];
extern GpsData *gpsData;
