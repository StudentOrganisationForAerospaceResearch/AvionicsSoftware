#pragma once

#include "main.h"

extern UART_HandleTypeDef huart5;

typedef struct{
    int32_t accelX;
    int32_t accelY;
    int32_t accelZ;
    int32_t gyroX;
    int32_t gyroY;
    int32_t gyroZ;
    int32_t magnetoX;
    int32_t magnetoY;
    int32_t magnetoZ;
    int32_t barometerPressure;
    int32_t barometerTemperature;
    int32_t combustionChamberPressure;
    int32_t oxidizerTankPressure;
    int32_t gps_time;
    int32_t latitude_degrees;
    int32_t latitude_minutes;
    int32_t longitude_degrees;
    int32_t longitude_minutes;
    int32_t antennaAltitude;
    int32_t geoidAltitude;
    int32_t altitude;
    int32_t currentFlightPhase;
    int32_t tick;
} LogEntry;

void writeLogEntryToEEPROM(uint16_t memAddress, LogEntry* givenLog);
void readLogEntryFromEEPROM(uint16_t memAddress, LogEntry* givenLog);
void initializeLogEntry(LogEntry* givenLog);

void debugTask(void const* arg);

