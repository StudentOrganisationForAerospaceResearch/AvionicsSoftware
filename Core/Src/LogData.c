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

#include "LogData.h"
#include "FlightPhase.h"
#include "Utils.h"
#include <string.h>

/* Macros --------------------------------------------------------------------*/
#define max(a,b) \
    ({ __typeof__ (a) _a = (a); \
        __typeof__ (b) _b = (b); \
        _a > _b ? _a : _b; })

#define min(a,b) \
    ({ __typeof__ (a) _a = (a); \
        __typeof__ (b) _b = (b); \
        _a < _b ? _a : _b; })


/* Constants -----------------------------------------------------------------*/
static const int32_t SLOW_LOG_DATA_PERIOD_ms = 700; // logging period for slow routine
static const int32_t FAST_LOG_DATA_PERIOD_ms = 200; // logging period for fast routine

/* Variables -----------------------------------------------------------------*/
static uint16_t preFlightAddressOffset = 0; // offset updated after writing in logEntryOnceRoutine
static uint16_t inFlightAddressOffset = 14*sizeof(LogEntry); // largest address to write to before flight, updated during flight
uint32_t currentSectorAddr = 0; // counts up; rewrites if needed
uint32_t currentSectorOffset_B = 0; // two per page, 0 or 92
uint8_t flash_filled = 0;
uint8_t remaining_overwrite_pages = 16; // counts down from 16
bool current_sector_erased = false;

/* Functions -----------------------------------------------------------------*/
/**
 * @brief this method initializes the log entry struct
 * @param givenLog, pointer to a log created at the start of the task
 */
void initializeLogEntry(LogEntry* givenLog)
{
	memset(givenLog, -1, sizeof(LogEntry));
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

    if (osMutexWait(data->batteryVoltageData_->mutex_, 0) == osOK)
    {
        givenLog->batteryVoltage = data->batteryVoltageData_->voltage_;
        osMutexRelease(data->batteryVoltageData_->mutex_);
    }
    
    givenLog->currentFlightPhase = getCurrentFlightPhase();
    givenLog->tick = HAL_GetTick();
}

/**
 * @brief Writes a single LogEntry struct to the on-board W25Qxx flash chip.
 * @param data, pointer to AllData struct sent to updateLogEntry to update the LogEntry struct
 * @param givenLog, pointer to a log initialized at the start of task
 * @param logStartAddress, pointer to the start of memory for flight logging
 * @return True if the log was written, false if the chip is full
 */
bool logEntryOnceRoutine(AllData* data, LogEntry* givenLog, uint16_t* logStartAddress)
{
    updateLogEntry(data, givenLog);

    uint8_t* logPtr = (uint8_t*)(givenLog);
    uint32_t internalLogOffset = 0;
    uint32_t numBytesLeftInLog = sizeof(LogEntry);
    while (numBytesLeftInLog > 0) {
		if (currentSectorOffset_B == w25qxx.SectorSize) {
			// Current sector is full, move to the next sector
			currentSectorOffset_B = 0;
			currentSectorAddr += 1;
		}

		if (currentSectorAddr >= w25qxx.SectorCount) {
			// Chip is full, can't log anymore!
			return false;
		}

		// Write next portion of log into current flash sector,
		// # free bytes in sector or rest of the log, whichever is lowest.
		uint32_t numFreeBytesInSector = w25qxx.SectorSize - currentSectorOffset_B;
		uint32_t numBytesToWrite = min(numFreeBytesInSector, numBytesLeftInLog) ;
		W25qxx_WriteSector(&logPtr[internalLogOffset], currentSectorAddr, currentSectorOffset_B, numBytesToWrite);

		currentSectorOffset_B += numBytesToWrite;
		numBytesLeftInLog -= numBytesToWrite;
		internalLogOffset += numBytesToWrite;
		logPtr = &logPtr[internalLogOffset];

		return true;
    }
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    LogEntry log;
    initializeLogEntry(&log);
    char flightStartflag[] = "**flight**";
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
