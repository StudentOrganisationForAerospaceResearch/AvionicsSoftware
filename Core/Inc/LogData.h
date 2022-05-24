#pragma once

#include "main.h"
#include "../../Drivers/w25qxx/w25qxx.h"

#define SPIMODE 1

//#ifdef I2CMODE
//extern I2C_HandleTypeDef hi2c1;
//#endif

extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi2;
extern w25qxx_t w25qxx;

/* Structs -------------------------------------------------------------------*/
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

    //LogEntry(){
    //}
} LogEntry; // LogEntry holds data from AllData that is to be logged

void initializeLogEntry(LogEntry* givenLog);
void logDataTask(void const* arg);
