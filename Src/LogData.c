/**
 ******************************************************************************
 * File Name          : LogData.c
 * Description        : Code handles logging of AllData as per FlightPhase
 ******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "defines.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "FlightPhase.h"


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
    int32_t tick;
} logEntry; // logEntry holds data from AllData that is to be logged


/* Constants -----------------------------------------------------------------*/
static int SLOW_LOG_DATA_PERIOD = 700;
static int FAST_LOG_DATA_PERIOD = 200;
static uint8_t softwareVersion = 104;
static uint8_t deviceAddress = 0x50;
static uint8_t currMemAddress = 0x07; //TODO: update this as needed
static uint32_t timeout = 10; // Time required to wait before ending read/write 
static logEntry theLogEntry; // This is the global log entry that is updated
static uint8_t logEntrySize = 80;

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Writes data to the EEPROM over I2C.
 * @param buffer, pointer to the data buffer.
 * @param bufferSize, size of data buffer.
 */
void writeToEEPROM(uint8_t* buffer, uint16_t bufferSize, uint16_t memAddress)
{
	HAL_I2C_Mem_Write(&hi2c1, deviceAddress<<1, memAddress, uint16_t(sizeof(memAddress)), buffer, bufferSize, timeout);
}

/**
 * @brief Reads data from the EEPROM over I2C.
 * @param receiveBuffer, pointer to the buffer to store received data.
 * @param bufferSize, size of data to be read.
 */
void readFromEEPROM(uint8_t* receiveBuffer, uint16_t bufferSize, uint16_t memAddress)
{
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress<<1, memAddress, sizeof(memAddress), receiveBuffer, bufferSize, timeout);
}

/**
 * @brief Blocks the current thread until the I2C state is ready for reading/writing.
 */
void checkEEPROMBlocking()
{
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){};
	while (HAL_I2C_IsDeviceReady(&hi2c1, deviceAddress<<1, 3, timeout) != HAL_OK){};
}

/**
 * @brief puts a logEntry into a char array and sends to EEPROM for writing
 * @param memAddress, the address at which the EEPROM is instructed to start writing
 */
void writeLogEntryToEEPROM(uint16_t memAddress)
{
    char dataToWrite[logEntrySize]; 

    dataToWrite[0] = theLogEntry.accelX;
    dataToWrite[4] = theLogEntry.accelY;
    dataToWrite[8] = theLogEntry.accelZ;
    dataToWrite[12] = theLogEntry.gyroX;
    dataToWrite[16] = theLogEntry.gyroY;
    dataToWrite[20] = theLogEntry.gyroZ;
    dataToWrite[24] = theLogEntry.magnetoX;
    dataToWrite[28] = theLogEntry.magnetoY;
    dataToWrite[32] = theLogEntry.magnetoZ;
    dataToWrite[36] = theLogEntry.barometerPressure;
    dataToWrite[40] = theLogEntry.barometerTemperature;
    dataToWrite[44] = theLogEntry.combustionChamberPressure;
    dataToWrite[48] = theLogEntry.oxidizerTankPressure;
    dataToWrite[52] = theLogEntry.gps_time;
    dataToWrite[56] = theLogEntry.latitude_degrees;
    dataToWrite[60] = theLogEntry.latitude_minutes;
    dataToWrite[64] = theLogEntry.longitude_degrees;
    dataToWrite[68] = theLogEntry.longitude_minutes;
    dataToWrite[72] = theLogEntry.altitude;
    dataToWrite[76] = theLogEntry.tick;

    checkEEPROMBlocking();
    writeToEEPROM(dataToWrite, sizeof(dataToWrite), memAddress);
    //TODO: make sure memAddress is correct, increment address
}

/**
 * @brief reads logEntry-sized char array from EEPROM and updates theLogEntry
 * @param memAddress, the address at which the EEPROM is instructed to start reading
 */
void readLogEntryToEEPROM(uint16_t memAddress)
{
    char dataRead[logEntrySize];
  
    checkEEPROMBlocking();
    readFromEEPROM(dataRead, sizeof(dataRead), memAddress);
    //TODO: memAdress may not be corrrect bc it's never incremented

    theLogEntry.accelX = dataRead[0];
    theLogEntry.accelY = dataRead[4];
    theLogEntry.accelY = dataRead[8];
    theLogEntry.gyroX = dataRead[12];
    theLogEntry.gyroY = dataRead[16];
    theLogEntry.gyroZ = dataRead[20];
    theLogEntry.magnetoX = dataRead[24];
    theLogEntry.magnetoY = dataRead[28];
    theLogEntry.magnetoZ = dataRead[32];
    theLogEntry.barometerPressure = dataRead[36];
    theLogEntry.barometerTemperature = dataRead[40];
    theLogEntry.combustionChamberPressure = dataRead[44];
    theLogEntry.oxidizerTankPressure = dataRead[48];
    theLogEntry.gps_time = dataRead[52];
    theLogEntry.latitude_degrees = dataRead[56];
    theLogEntry.latitude_minutes = dataRead[60];
    theLogEntry.longitude_degrees = dataRead[64];
    theLogEntry.longitude_minutes = dataRead[68];
    theLogEntry.altitude = dataRead[72];
    theLogEntry.tick = dataRead[76];
}

/**
 * @brief this method initializes the log entry struct
 * and is only called during the start of the task
 */
void initializeLogEntry()
{
    theLogEntry.accelX = -1;
    theLogEntry.accelY = -1;
    theLogEntry.accelZ = -1;
    theLogEntry.gyroX = -1;
    theLogEntry.gyroY = -1;
    theLogEntry.gyroZ = -1;
    theLogEntry.magnetoX = -1;
    theLogEntry.magnetoY = -1;
    theLogEntry.magnetoZ = -1;
    theLogEntry.barometerPressure = -1;
    theLogEntry.barometerTemperature = -1;
    theLogEntry.combustionChamberPressure = -1;
    theLogEntry.oxidizerTankPressure = -1;
    theLogEntry.gps_time = 0xFFFF;
    theLogEntry.latitude_degrees = -1;
    theLogEntry.latitude_minutes = 0xFFFF;
    theLogEntry.longitude_degrees = -1;
    theLogEntry.longitude_minutes = 0xFFFF;
    theLogEntry.antennaAltitude = -1;
    theLogEntry.geoidAltitude = -1;
    theLogEntry.altitude = -1;
    theLogEntry.tick = HAL_GetTick();
}
/**
 * @brief Simply updates theLogEntry field with CURRENT values in AllData struct
 * @param data, pointer to AllData struct
 */
void updateLogEntry(AllData* data)
{
    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        theLogEntry.accelX = data->accelGyroMagnetismData_->accelX_;
        theLogEntry.accelY = data->accelGyroMagnetismData_->accelY_;
        theLogEntry.accelZ = data->accelGyroMagnetismData_->accelZ_;
        theLogEntry.gyroX = data->accelGyroMagnetismData_->gyroX_;
        theLogEntry.gyroY = data->accelGyroMagnetismData_->gyroY_;
        theLogEntry.gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        theLogEntry.magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        theLogEntry.magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        theLogEntry.magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        theLogEntry.pressure = data->barometerData_->pressure_;
        theLogEntry.temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        theLogEntry.combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

     if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        theLogEntry.oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        theLogEntry.gps_time = data->gpsData_->time_;
        theLogEntry.latitude_degrees = data->gpsData_->latitude_.degrees_;
        theLogEntry.latitude_minutes = data->gpsData_->latitude_.minutes_;
        theLogEntry.longitude_degrees = data->gpsData_->longitude_.degrees_;
        theLogEntry.longitude_minutes = data->gpsData_->longitude_.minutes_;
        theLogEntry.antennaAltitude = data->gpsData_->totalAltitude_.altitude_;
        theLogEntry.geoidAltitude = data->gpsData_->totalAltitude_.altitude_;
        theLogEntry.altitude = data->gpsData_->totalAltitude_.altitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }
    //TODO: add FlightPhase to logEntry and update it
    theLogEntry.tick = HAL_GetTick();
}

/**
 * @brief Used for logging entries to the EEPROM. First updates the struct, then writes it
 * the wait times are all managed in the main for loop so highFrequencyLog = lowFrequencyLog
 * @param data, pointer to AllData Struct sent to updateLogEntry to update the logEntry struct
 */
void logEntryOnceRoutine(AllData* data)
{
    updateLogEntry(data);
    writeLogEntryToEEPROM(currMemAddress);
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    initializeLogEntry();
    uint32_t prevWakeTime;
    clock_t beforeLogTime, afterLogTime, totalTime;
    for (;;)
    {
        beforeLogTime = clock();
        logEntryOnceRoutine();
        afterLogTime = clock();
        totalTime = (double)(afterLogTime - beforeLogTime) / CLOCKS_PER_SEC / 1000; //this also takes time!
        prevWakeTime = osKernelSysTick();//assume it is atomic: runs in one cycle
        switch (getCurrentFlightPhase())
        {
            case BURN:
            case COAST:
            case DROGUE_DESCENT:
                osDelayUntil(&prevWakeTime, (uint32_t)(FAST_LOG_DATA_PERIOD - totalTime));
                break;

            default:
                osDelayUntil(&prevWakeTime, (uint32_t)(SLOW_LOG_DATA_PERIOD - totalTime));
                break;
        }
    }
}
