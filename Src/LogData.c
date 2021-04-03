#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "defines.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "FlightPhase.h"

static int SLOW_LOG_DATA_PERIOD = 700;
static int FAST_LOG_DATA_PERIOD = 200;
static uint8_t softwareVersion = 104;
static uint8_t deviceAddress = 0x50;
static uint8_t currMemAddress = 0x07;
static uint32_t timeout = 10;//what sbould this be?
//NOTE: the functions pass around a timeout but i made a static declaration here

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
    int32_t altitude;
} logEntry;


//SUPER COOL THING BELOW!
static logEntry theLogEntry;


/**
 * @brief Writes data to the EEPROM over I2C.
 * @param buffer, pointer to the data buffer.
 * @param bufferSize, size of data buffer.
 * @param timeout, time to wait for write operation before timing out in ms.
 */
void writeToEEPROM(uint8_t* buffer, uint16_t bufferSize, uint16_t memAddress, uint16_t timeout)
{
	HAL_I2C_Mem_Write(&hi2c1, deviceAddress<<1, memAddress, uint16_t(sizeof(memAddress)), buffer, bufferSize, timeout);
}

/**
 * @brief Reads data from the EEPROM over I2C.
 * @param receiveBuffer, pointer to the buffer to store received data.
 * @param bufferSize, size of data to be read.
 * @param timeout, time to wait for read operation before timing out in ms.
 */
void readFromEEPROM(uint8_t* receiveBuffer, uint16_t bufferSize, uint16_t memAddress, uint16_t timeout)
{
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress<<1, memAddress, sizeof(memAddress), receiveBuffer, bufferSize, timeout);
}

/**
 * @brief Blocks the current thread until the I2C state is ready for reading/writing.
 * @param timeout Timeout duration
 */
void checkEEPROMBlocking(uint16_t timeout)
{
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){};
	while (HAL_I2C_IsDeviceReady(&hi2c1, deviceAddress<<1, 3, timeout) != HAL_OK){};
}

/**
 * @brief puts a logEntry into a char array and sends to EEPROM for writing
 * @param memAddress, needed by the writeToEEPROM function
 * @param timeout, time to wait for write operation
 */
void writeLogEntryToEEPROM(uint16_t memAddress, uint16_t timeout)
{
    char dataToWrite[200]; 

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

    checkEEPROMBlocking(timeout);
    writeToEEPROM(dataToWrite, 76, memAddress, timeout);
    // need: memAddress and timeout??????
    // increment address
}

/**
 * @brief reads logEntry-sized char array from EEPROM and updates theLogEntry
 * @param addr, I dont know where this is used
 * @param memAddress, needed by the readFromEEPROM function
 * @param timeout, time to wait for read operation
 */
void readLogEntryToEEPROM(uint16_t addr, uint16_t memAddress, uint16_t timeout)
{
    char dataRead[200];
  
    checkEEPROMBlocking(timeout);
    readFromEEPROM(dataRead, 80, memAddress, timeout);
    // need: memAdress and timeout

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
}

/**
 * @brief
 */
void initializeLogEntry(){
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
    theLogEntry.oxidizerTankPressure = -1; //this too
    // GPS
    theLogEntry.gps_time = 0xFFFF;
    theLogEntry.latitude_degrees = -1;
    theLogEntry.latitude_minutes = 0xFFFF;
    theLogEntry.longitude_degrees = -1;
    theLogEntry.longitude_minutes = 0xFFFF;
    theLogEntry.altitude = -1;
}
/**
 * @brief Simply updates theLogEntry field with CURRENT values in AllData struct, formerly buildLogEntry
 * @param data, pointer to AllData struct
 */
void updateLogEntry(AllData* data)//updateLogEntry, pointer to the log entry
{
    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK) //this is how we do it!
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

        theLogEntry.altitude = data->gpsData_->totalAltitude_.altitude_;

        osMutexRelease(data->gpsData_->mutex_);
    }
}

/**
 * Used for logging entries to the EEPROM. First updates the struct, then writes it
 * the wait times are all managed in the main for loop so highFrequencyLog = lowFrequencyLog
 * @param data, sent to updateLogEntry
 */
void logEntryOnceRoutine(AllData* data)
{
    updateLogEntry(data);
    writeLogEntryToEEPROM(currMemAddress, timeout);
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
        logEntryOnceRoutine();//should have a delay arugument
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
