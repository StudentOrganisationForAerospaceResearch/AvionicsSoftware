#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "defines.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_fatfs.h"

#include "LogData.h"
#include "Data.h"
#include "FlightPhase.h"
#include "SoftwareVersion.h"

static int SLOW_LOG_DATA_PERIOD = 1000;
static int FAST_LOG_DATA_PERIOD = 50;

static FATFS fatfs;
static FIL file;

void buildLogEntry(AllData* data, char* buffer)
{

    int32_t accelX = -1;
    int32_t accelY = -1;
    int32_t accelZ = -1;
    int32_t gyroX = -1;
    int32_t gyroY = -1;
    int32_t gyroZ = -1;
    int32_t magnetoX = -1;
    int32_t magnetoY = -1;
    int32_t magnetoZ = -1;
    int32_t pressure = -1;
    int32_t temperature = -1;
    int32_t combustionChamberPressure = -1;
    int32_t altitude = -1;
    int32_t epochTimeMsec = -1;
    int32_t latitude = -1;
    int32_t longitude = -1;
    int32_t oxidizerTankPressure = -1;

    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        accelX = data->accelGyroMagnetismData_->accelX_;
        accelY = data->accelGyroMagnetismData_->accelY_;
        accelZ = data->accelGyroMagnetismData_->accelZ_;
        gyroX = data->accelGyroMagnetismData_->gyroX_;
        gyroY = data->accelGyroMagnetismData_->gyroY_;
        gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        pressure = data->barometerData_->pressure_;
        temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        altitude = data->gpsData_->altitude_;
        epochTimeMsec = data->gpsData_->epochTimeMsec_;
        latitude = data->gpsData_->latitude_;
        longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    sprintf(
        buffer,
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d\n",
        accelX,
        accelY,
        accelZ,
        gyroX,
        gyroY,
        gyroZ,
        magnetoX,
        magnetoY,
        magnetoZ,
        pressure,
        temperature,
        combustionChamberPressure,
        altitude,
        epochTimeMsec,
        latitude,
        longitude,
        oxidizerTankPressure,
        getCurrentFlightPhase(),
        SOFTWARE_VERSION
    );
}

void lowFrequencyLogToSdRoutine(AllData* data, char* buffer, FlightPhase entryPhase)
{
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, SLOW_LOG_DATA_PERIOD);

        if (getCurrentFlightPhase() != entryPhase)
        {
            // New phase has started, exit low frequency logging
            return;
        }

        buildLogEntry(data, buffer);

        if (f_mount(&fatfs, "SD:", 1) == FR_OK)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

            if (f_open(&file, "SD:AvionicsSoftware.csv", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
            {
                f_puts(buffer, &file);
                f_close(&file);
            }

            f_mount(NULL, "SD:", 1);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
        }
    }
}

void highFrequencyLogToSdRoutine(AllData* data, char* buffer)
{
    // Get card mounted
    uint8_t mounted = 0;

    while (!mounted)
    {
        // Keep trying, really need to log during this time
        mounted = (f_mount(&fatfs, "SD:", 1) == FR_OK);

        FlightPhase flightPhase = getCurrentFlightPhase();

        if ( flightPhase != BURN &&
                flightPhase != COAST &&
                flightPhase != DROGUE_DESCENT)
        {
            // couldn't mount during important phases, too bad :(
            return;
        }
    }

    // Card mounted, start writing at high frequency
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
    uint32_t prevWakeTime = osKernelSysTick();

    for (;;)
    {
        osDelayUntil(&prevWakeTime, FAST_LOG_DATA_PERIOD);

        FlightPhase flightPhase = getCurrentFlightPhase();

        if ( flightPhase != BURN &&
                flightPhase != COAST &&
                flightPhase != DROGUE_DESCENT)
        {
            // done important phases, unmont card and exit high frequency logging
            break;
        }

        buildLogEntry(data, buffer);

        if (f_open(&file, "SD:AvionicsSoftware.csv", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
        {
            f_puts(buffer, &file);
            f_close(&file); // close to save the file
        }
    }

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    f_mount(NULL, "SD:", 1);
}

void logDataTask(void const* arg)
{
    AllData* data = (AllData*) arg;
    char buffer[256];

    sprintf(
        buffer,
        "accelX,"
        "accelY,"
        "accelZ,"
        "gyroX,"
        "gyroY,"
        "gyroZ,"
        "magnetoX,"
        "magnetoY,"
        "magnetoZ,"
        "pressure,"
        "temperature,"
        "combustionChamberPressure,"
        "altitude,"
        "epochTimeMsec,"
        "latitude,"
        "longitude,"
        "oxidizerTankPressure,"
        "currentFlightPhase,"
        "softwareVersion\n"
    );

    if (f_mount(&fatfs, "SD:", 1) == FR_OK)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

        if (f_open(&file, "SD:AvionicsSoftware.csv", FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
        {
            f_puts(buffer, &file);
            f_close(&file);
        }

        f_mount(NULL, "SD:", 1);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    }

    for (;;)
    {
        switch (getCurrentFlightPhase())
        {
            case PRELAUNCH:
                lowFrequencyLogToSdRoutine(data, buffer, PRELAUNCH);
                break;

            case BURN:
            case COAST:
            case DROGUE_DESCENT:
                highFrequencyLogToSdRoutine(data, buffer);
                break;

            case MAIN_DESCENT:
                lowFrequencyLogToSdRoutine(data, buffer, MAIN_DESCENT);
                break;

            case ABORT:
                lowFrequencyLogToSdRoutine(data, buffer, ABORT);
                break;

            default:
                lowFrequencyLogToSdRoutine(data, buffer, MAIN_DESCENT); // won't work
                break;
        }
    }
}
