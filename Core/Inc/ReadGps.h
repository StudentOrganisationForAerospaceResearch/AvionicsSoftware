#pragma once
#include "Globals.h"

void readGpsTask(void const* arg);

extern UART_HandleTypeDef GPS_UART;

extern char dma_rx_buffer[NMEA_MAX_LENGTH + 1];
extern GpsData* gpsData;
