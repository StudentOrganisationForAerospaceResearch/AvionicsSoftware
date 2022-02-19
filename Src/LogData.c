/**
 ******************************************************************************
 * File Name          : LogData.c
 * Description        : Code handles logging of AllData for different routines
 ******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

//#include "defines.h"
//#include "tm_stm32_delay.h"
//#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "FlightPhase.h"
#include "Utils.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/
//CHECK: was max implemented somewhere else?
#define max(a,b) \
    ({ __typeof__ (a) _a = (a); \
        __typeof__ (b) _b = (b); \
        _a > _b ? _a : _b; })


/* Constants -----------------------------------------------------------------*/
static const int32_t SLOW_LOG_DATA_PERIOD_ms = 700; // logging period for slow routine
static const int32_t FAST_LOG_DATA_PERIOD_ms = 200; // logging period for fast routine
static const uint8_t SOFTWARE_VERSION = 104; // this is not used at all !!
static const uint8_t DEVICE_ADDRESS = 0x50; // used for reading/writing to EEPROM
static const uint8_t EEPROM_START_ADDRESS = 0x07; // start address for reading/writing
static const uint32_t TIMEOUT_MS = 10; // Time required to wait before ending read/write
static const uint8_t LOG_ENTRY_SIZE = sizeof(LogEntry); //size of LogEntry = 92 bytes
static const uint32_t LOGGING_BUSY_RETRIES = 3; //Number of times to retry if EEPROM is Busy.

/* Variables -----------------------------------------------------------------*/
static uint16_t preFlightAddressOffset = 0; // offset updated after writing in logEntryOnceRoutine
static uint16_t inFlightAddressOffset = 14*sizeof(LogEntry); // largest address to write to before flight, updated during flight
/* Functions -----------------------------------------------------------------*/
/**
 * @brief Writes data to the EEPROM over I2C.
 * @param buffer, pointer to the data buffer.
 * @param bufferSize, size of data buffer.
 * @return HAL Status of the read operation (HAL_OK, HAL_ERROR ,HAL_BUSY).
 */
HAL_StatusTypeDef writeToEEPROM(uint8_t* buffer, uint16_t bufferSize, uint16_t memAddress)
{
	// if HAL_I2C_Mem_Write returns HAL_OK then return HAL_OK.
	// if it returns HAL_ERROR, return HAL_ERROR.
	// if it returns HAL_BUSY, retry LOGGING_BUSY_RETRIES times with longer timeout.

#ifdef SPIMODE
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit (&hspi2, (uint8_t *)buffer, bufferSize, TIMEOUT_MS);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
#else
    //    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
    //    		&hi2c1, DEVICE_ADDRESS<<1, memAddress, (sizeof(memAddress)), buffer, bufferSize, TIMEOUT_MS);

#endif
    if(status == HAL_BUSY)
    {
    	for(int i=0; i<LOGGING_BUSY_RETRIES; i++)
    	{
#ifdef SPIMODE
    	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    	    HAL_StatusTypeDef retryStatus = HAL_SPI_Transmit (&hspi2, (uint8_t *)buffer, bufferSize, (i + 2) * TIMEOUT_MS);
    	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
#else
    	    HAL_StatusTypeDef retryStatus = HAL_I2C_Mem_Write(
					&hi2c1, DEVICE_ADDRESS<<1, memAddress, (sizeof(memAddress)), buffer, bufferSize, (i + 2) * TIMEOUT_MS);
#endif
    		if(retryStatus == HAL_OK || retryStatus == HAL_ERROR)
    		{
    			return 1; //retryStatus;
    		}
    	}
    	return HAL_BUSY;
    }
    return status;
}

/**
 * @brief Reads data from the EEPROM over I2C.
 * @param receiveBuffer, pointer to the buffer to store received data.
 * @param bufferSize, size of data to be read.
 * @return HAL Status of the read operation (HAL_OK, HAL_ERROR ,HAL_BUSY).
 */
HAL_StatusTypeDef readFromEEPROM(uint8_t* receiveBuffer, uint16_t bufferSize, uint16_t memAddress)
{
	// if HAL_I2C_Mem_Read returns HAL_OK then return HAL_OK.
	// if it returns HAL_ERROR, return HAL_ERROR.
	// if it returns HAL_BUSY, retry LOGGING_BUSY_RETRIES times with longer timeout.
#ifdef SPIMODE
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi2, (uint8_t *)receiveBuffer, bufferSize, TIMEOUT_MS);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

#else
    //    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
    //    		&hi2c1, DEVICE_ADDRESS<<1, memAddress, sizeof(memAddress), receiveBuffer, bufferSize, TIMEOUT_MS);
#endif
    if( status == HAL_BUSY)
    {
    	for(int i=0; i<LOGGING_BUSY_RETRIES; i++)
    	{
#ifdef SPIMODE
    	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    	    HAL_StatusTypeDef retryStatus = HAL_SPI_Receive(&hspi2, (uint8_t *)receiveBuffer, bufferSize, TIMEOUT_MS);
    	    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

#else
    	    HAL_StatusTypeDef retryStatus = HAL_I2C_Mem_Read(
    	        				&hi2c1, DEVICE_ADDRESS<<1, memAddress, sizeof(memAddress), (uint8_t *)receiveBuffer, bufferSize, TIMEOUT_MS);
#endif
    		if( retryStatus == HAL_OK || retryStatus == HAL_ERROR)
    		{
    			return 1;// retryStatus;
    		}
    	}
    	return HAL_BUSY;
    }
    return status;
}

/**
 * @brief Blocks the current thread until the I2C state is ready for reading/writing.
 */
void checkEEPROMBlocking()
{
//	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){};
//	while (HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS<<1, 3, TIMEOUT_MS) != HAL_OK){};
    // TODO: add a delay ~100us?
    // should this return a boolean?
}

/**
 * @brief puts a LogEntry into a char array and sends to EEPROM for writing.
 * @param memAddress, the address at which the EEPROM is instructed to start writing.
 * @param givenLog, pointer to a log initialized at the start of task.
 * @return HAL Status of the read operation (HAL_OK, HAL_ERROR ,HAL_BUSY).
 */
HAL_StatusTypeDef writeLogEntryToEEPROM(uint16_t memAddress, LogEntry* givenLog)
{
    checkEEPROMBlocking();
    //Note: writeToEEPROM expects a uint8_t*
    //if that's not a problem, can we send the LogEntry pointer without casting?
    return (writeToEEPROM(givenLog, LOG_ENTRY_SIZE, memAddress));
}

/**
 * @brief reads LogEntry-sized char array from EEPROM and updates the log entry
 * @param memAddress, the address at which the EEPROM is instructed to start reading
 * @param givenLog, pointer to a log initialized at the start of task
 */
void readLogEntryFromEEPROM(uint16_t memAddress, LogEntry* givenLog)
{
    char dataRead[LOG_ENTRY_SIZE];
    memset(dataRead, 1, LOG_ENTRY_SIZE);

    checkEEPROMBlocking();
    HAL_StatusTypeDef statusCode = readFromEEPROM((uint8_t*)dataRead, sizeof(dataRead), memAddress);
    // If read is not successful, fill dataRead with error code.
    if(statusCode != HAL_OK)
    {
    	for(int i=0; i<sizeof(dataRead); i++)
    	{
    		dataRead[i] = statusCode;
    	}
    }
    /* what if we did:

    uint32_t* readingAddress = &(givenLog->accelX);
    for(int i = 0; i < LOG_ENTRY_SIZE - 4; i+=4, readingAddress++){
        readUInt32FromUInt8Array((uint8_t*)dataRead, i, readingAddress);
    }

    because structs are contiguous memory, would incrementing address like this work?
    readingAddress is a uint32_t pointer so doing ++ should move it to the next uint32_t ???
    */
    readUInt32FromUInt8Array((uint8_t*)dataRead, 0, &(givenLog->accelX));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 4, &(givenLog->accelY));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 8, &(givenLog->accelZ));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 12, &(givenLog->gyroX));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 16, &(givenLog->gyroY));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 20, &(givenLog->gyroZ));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 24, &(givenLog->magnetoX));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 28, &(givenLog->magnetoY));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 32, &(givenLog->magnetoZ));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 36, &(givenLog->barometerPressure));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 40, &(givenLog->barometerTemperature));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 44, &(givenLog->combustionChamberPressure));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 48, &(givenLog->oxidizerTankPressure));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 52, &(givenLog->gps_time));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 56, &(givenLog->latitude_degrees));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 60, &(givenLog->latitude_minutes));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 64, &(givenLog->longitude_degrees));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 68, &(givenLog->longitude_minutes));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 72, &(givenLog->antennaAltitude));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 76, &(givenLog->geoidAltitude));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 80, &(givenLog->altitude));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 84, &(givenLog->currentFlightPhase));
    readUInt32FromUInt8Array((uint8_t*)dataRead, 88, &(givenLog->tick));
}

/**
 * @brief this method initializes the log entry struct
 * @param givenLog, pointer to a log created at the start of the task
 */
void initializeLogEntry(LogEntry* givenLog)
{
//    givenLog->accelX = 0;
//    givenLog->accelY = 0;
//    givenLog->accelZ = 0;
//    givenLog->gyroX = 0;
//    givenLog->gyroY = 0;
//    givenLog->gyroZ = 0;
//    givenLog->magnetoX = 0;
//    givenLog->magnetoY = 0;
//    givenLog->magnetoZ = 0;
    memset(givenLog, 3, LOG_ENTRY_SIZE);
//	bzero(givenLog, LOG_ENTRY_SIZE);
    givenLog->currentFlightPhase = getCurrentFlightPhase();
    givenLog->tick = osKernelSysTick();
}

/**
 * @brief Simply updates theLogEntry field with CURRENT values in AllData struct
 * @param data, pointer to AllData struct
 * @param givenLog, pointer to a log created at the start of the task
 */
void updateLogEntry(AllData* data, LogEntry* givenLog)
{
    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        givenLog->accelX = data->accelGyroMagnetismData_->accelX_;
        givenLog->accelY = data->accelGyroMagnetismData_->accelY_;
        givenLog->accelZ = data->accelGyroMagnetismData_->accelZ_;
        givenLog->gyroX = data->accelGyroMagnetismData_->gyroX_;
        givenLog->gyroY = data->accelGyroMagnetismData_->gyroY_;
        givenLog->gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        givenLog->magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        givenLog->magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        givenLog->magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        givenLog->barometerPressure = data->barometerData_->pressure_;
        givenLog->barometerTemperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        givenLog->combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

     if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        givenLog->oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        givenLog->gps_time = data->gpsData_->time_;
        givenLog->latitude_degrees = data->gpsData_->latitude_.degrees_;
        givenLog->latitude_minutes = data->gpsData_->latitude_.minutes_;
        givenLog->longitude_degrees = data->gpsData_->longitude_.degrees_;
        givenLog->longitude_minutes = data->gpsData_->longitude_.minutes_;
        givenLog->antennaAltitude = data->gpsData_->antennaAltitude_.altitude_;
        givenLog->geoidAltitude = data->gpsData_->geoidAltitude_.altitude_;
        givenLog->altitude = data->gpsData_->totalAltitude_.altitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }
    givenLog->currentFlightPhase = getCurrentFlightPhase();
    givenLog->tick = HAL_GetTick();
}

/**
 * @brief Used for logging entries to the EEPROM. First updates the struct, then writes it
 * the wait times are all managed in the main for loop so highFrequencyLog = lowFrequencyLog
 * @param data, pointer to AllData Struct sent to updateLogEntry to update the LogEntry struct
 * @param givenLog, pointer to a log initialized at the start of task
 * @param logStartAddress, pointer to the start of memory for flight logging
 */
void logEntryOnceRoutine(AllData* data, LogEntry* givenLog, uint16_t* logStartAddress)
{
    updateLogEntry(data, givenLog);
    writeLogEntryToEEPROM(EEPROM_START_ADDRESS + (*logStartAddress), givenLog);
    (*logStartAddress) += sizeof(LogEntry);
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    LogEntry log;
    initializeLogEntry(&log);
    char flightStartflag[] = "**flight**";
    writeToEEPROM((uint8_t*)flightStartflag, sizeof(flightStartflag), inFlightAddressOffset-sizeof(flightStartflag));
    uint32_t prevWakeTime, beforeLogTime;
    for (;;)
    {
    	beforeLogTime = osKernelSysTick();
        switch (getCurrentFlightPhase())
        {
        	case PRELAUNCH:
        	case ARM:
        		logEntryOnceRoutine(data, &log, &preFlightAddressOffset);
        		if ((inFlightAddressOffset - preFlightAddressOffset) < sizeof(LogEntry))
        			preFlightAddressOffset = 0;
        		prevWakeTime = osKernelSysTick(); //assume it is atomic: runs in one cycle
        		osDelayUntil(&prevWakeTime, (uint32_t)(max(SLOW_LOG_DATA_PERIOD_ms - (osKernelSysTick() - beforeLogTime), 0)));
        		break;

        	case BURN:
        	case COAST:
        	case DROGUE_DESCENT:
        		logEntryOnceRoutine(data, &log, &inFlightAddressOffset);
        		prevWakeTime = osKernelSysTick();
        		osDelayUntil(&prevWakeTime, (uint32_t)(max(FAST_LOG_DATA_PERIOD_ms - (osKernelSysTick() - beforeLogTime), 0)));
        		break;

        	default:
        		logEntryOnceRoutine(data, &log, &inFlightAddressOffset);
        		prevWakeTime = osKernelSysTick();
        		osDelayUntil(&prevWakeTime, (uint32_t)(max(SLOW_LOG_DATA_PERIOD_ms - (osKernelSysTick() - beforeLogTime), 0)));
        		break;
        }
    }
}
