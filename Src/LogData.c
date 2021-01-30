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

static FATFS fatfs;
static FIL file;

char fileName[32];

/**
 * @brief Writes data to the EEPROM over I2C.
 * @param buffer, pointer to the data buffer.
 * @param bufferSize, size of data buffer.
 * @param timeout, time to wait for write operation before timing out in ms.
 */
void writeToEEPROM(uint8_t* buffer, uint16_t bufferSize, uint16_t memAddress, uint16_t timeout)
{
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){};
	while (HAL_I2C_IsDeviceReady(&hi2c1, deviceAddress<<1, 3, timeout) != HAL_OK){};
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
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){};
	while (HAL_I2C_IsDeviceReady(&hi2c1, deviceAddress<<1, 3, timeout) != HAL_OK){};
	HAL_I2C_Mem_Read(&hi2c1, deviceAddress<<1, memAddress, sizeof(memAddress), receiveBuffer, bufferSize, timeout);
}

void writeAllDataToEEPROM(AllData* data, uint16_t memAddress, uint16_t timeout){

    char dataToWrite[200]; 

    int index = 0;
    dataToWrite[0] = data->accelGyroMagnetismData_ ->accelX;
    dataToWrite[4] = data->accelGyroMagnetismData_ ->accelY;
    dataToWrite[8] = data->accelGyroMagnetismData_ ->accelZ;
    dataToWrite[12] = data->accelGyroMagnetismData_ ->gyroX;
    dataToWrite[16] = data->accelGyroMagnetismData_ ->gyroY;
    dataToWrite[20] = data->accelGyroMagnetismData_ ->gyroZ;
    dataToWrite[24] = data->accelGyroMagnetismData_ ->magnetoX;
    dataToWrite[28] = data->accelGyroMagnetismData_ ->magnetoY;
    dataToWrite[32] = data->accelGyroMagnetismData_ ->magnetoZ;

    dataToWrite[36] = data->barometerData_ ->pressure_;
    dataToWrite[40] = data->barometerData_ ->temperature_;

    dataToWrite[44] = data->combustionChamberPressureData_ ->pressure_;

    dataToWrite[48] = data->gpsData_ ->time_;
    dataToWrite[52] = data->gpsData_ ->latitude_->degrees_;
    dataToWrite[56] = data->gpsData_ ->latitude_->minutes_;
    dataToWrite[60] = data->gpsData_ ->longitude_->degrees_;
    dataToWrite[64] = data->gpsData_ ->longitude_->minutes_;

    dataToWrite[68] = data->gpsData_ ->antennaAltitude_->altitude_;
    dataToWrite[72] = data->gpsData_ ->geoidAltitude_->altitude_;
    dataToWrite[76] = data->gpsData_ ->totalAltitude_->altitude_;


    dataToWrite[80] = data->oxidizerTankPressureData_ ->pressure_;

   
    writeToEEPROM(dataToWrite, 80, uint16_t memAddress, uint16_t timeout);
    // need: memAddress and timeout??????
    
}

void readAllDataFromEEPROM(uint16_t addr, AllData* rtn, uint16_t memAddress, uint16_t timeout){
    char dataRead[200];
  
   readFromEEPROM(dataRead, 80, uint16_t memAddress, uint16_t timeout);
   // need: memAdress and timeout

    rtn->accelGyroMagnetismData_ ->accelX = dataRead[0];
    rtn->accelGyroMagnetismData_ ->accelY = dataRead[4];
    rtn->accelGyroMagnetismData_ ->accelZ = dataRead[8];
    rtn->accelGyroMagnetismData_ ->gyroX = dataRead[12];
    rtn->accelGyroMagnetismData_ ->gyroY = dataRead[16];
    rtn->accelGyroMagnetismData_ ->gyroZ = dataRead[20];
    rtn->accelGyroMagnetismData_ ->magnetoX = dataRead[24];
    rtn->accelGyroMagnetismData_ ->magnetoY = dataRead[28];
    rtn->accelGyroMagnetismData_ ->magnetoZ = dataRead[32];

    rtn->barometerData_ ->pressure_ = dataRead[36];
    rtn->barometerData_ ->temperature_ = dataRead[40];

    rtn->combustionChamberPressureData_ ->pressure_ = dataRead[44];

    rtn->gpsData_ ->time_ = dataRead[48];
    rtn->gpsData_ ->latitude_->degrees_ = dataRead[52];
    rtn->gpsData_ ->latitude_->minutes_ = dataRead[56];
    rtn->gpsData_ ->longitude_->degrees_ = dataRead[60];
    rtn->gpsData_ ->longitude_->minutes_ = dataRead[64];

    rtn->gpsData_ ->antennaAltitude_->altitude_ = dataRead[68];
    rtn->gpsData_ ->geoidAltitude_->altitude_ = dataRead[72];
    rtn->gpsData_ ->totalAltitude_->altitude_ = dataRead[76];

    rtn->oxidizerTankPressureData_ ->pressure_ = dataRead[80];

}


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
    int32_t oxidizerTankPressure = -1;
    // GPS
    static uint32_t gps_time = 0xFFFF;
    static int32_t latitude_degrees = -1;
    static uint32_t latitude_minutes = 0xFFFF;
    static int32_t longitude_degrees = -1;
    static uint32_t longitude_minutes = 0xFFFF;
    static int32_t altitude = -1;

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
        gps_time = data->gpsData_->time_;

        latitude_degrees = data->gpsData_->latitude_.degrees_;
        latitude_minutes = data->gpsData_->latitude_.minutes_;

        longitude_degrees = data->gpsData_->longitude_.degrees_;
        longitude_minutes = data->gpsData_->longitude_.minutes_;

        altitude = data->gpsData_->totalAltitude_.altitude_;

        osMutexRelease(data->gpsData_->mutex_);
    }

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    sprintf(
        buffer,
        "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%ld,%ld,%ld,%ld,%ld,%d,%ld,%d\n",
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
        oxidizerTankPressure,
        gps_time,
        latitude_degrees,
        latitude_minutes,
        longitude_degrees,
        longitude_minutes,
        altitude,
        getCurrentFlightPhase(),
        HAL_GetTick(),
        softwareVersion
    );
}

void lowFrequencyLogToSdRoutine(AllData* data, char* buffer)
{
    uint32_t prevWakeTime = osKernelSysTick();
    FlightPhase entryPhase = getCurrentFlightPhase();

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

            if (f_open(&file, fileName, FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
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
            // done important phases, unmount card and exit high frequency logging
            break;
        }

        buildLogEntry(data, buffer);

        if (f_open(&file, fileName, FA_OPEN_APPEND | FA_READ | FA_WRITE) == FR_OK)
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
    char buffer[500];

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
        "temperature(100C),"
        "combustionChamberPressure(1000psi),"
        "oxidizerTankPressure(1000psi),"
        "GPS_time,"
        "GPS_latitude_degrees,"
        "GPS_latitude_minutes,"
        "GPS_longitude_degrees,"
        "GPS_longitude_minutes,"
        "GPS_altitude,"
        "currentFlightPhase,"
        "elapsedTime(ms),"
        "softwareVersion\n"
    );

    if (f_mount(&fatfs, "SD:", 1) == FR_OK)
    {
        if (f_open(&file, "SD:AvionicsData1.csv", FA_OPEN_EXISTING) == FR_NO_FILE)
        {
            f_open(&file, "SD:AvionicsData1.csv", FA_CREATE_NEW | FA_READ | FA_WRITE);
            f_puts(buffer, &file);
            f_close(&file);
        }
        else
        {
            uint8_t fileExists = 1;

            for (uint8_t index = 2; fileExists; index++)
            {
                sprintf(fileName, "SD:AvionicsData%i.csv", index);

                if (f_open(&file, fileName, FA_OPEN_EXISTING) == FR_NO_FILE)
                {
                    f_open(&file, fileName, FA_CREATE_NEW | FA_READ | FA_WRITE);
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

                    if (f_size(&file) == 0)
                    {
                        f_puts(buffer, &file);
                    }

                    f_close(&file);
                    fileExists = 0;
                }
            }
        }

        f_mount(NULL, "SD:", 1);
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
    }

    for (;;)
    {
        switch (getCurrentFlightPhase())
        {
            case BURN:
            case COAST:
            case DROGUE_DESCENT:
                highFrequencyLogToSdRoutine(data, buffer);
                break;

            default:
                lowFrequencyLogToSdRoutine(data, buffer);
                break;
        }
    }
}
