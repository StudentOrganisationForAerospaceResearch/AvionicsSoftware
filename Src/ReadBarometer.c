#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadBarometer.h"
#include "Data.h"

static const int READ_BAROMETER_PERIOD = 20;    // Sampling delay set to 50 Hz to match high frequency logging

static const int CMD_SIZE = 1;
static const int CMD_TIMEOUT = 150;

static const uint8_t ADC_D1_512_CONV_CMD = 0x42;
static const uint8_t ADC_D2_512_CONV_CMD = 0x52;
static const uint8_t ADC_READ_CMD = 0x00;
static const uint8_t PROM_READ_SENS_CMD = 0xA2;
static const uint8_t PROM_READ_OFF_CMD = 0xA4;
static const uint8_t PROM_READ_TCS_CMD = 0xA6;
static const uint8_t PROM_READ_TCO_CMD = 0xA8;
static const uint8_t PROM_READ_TREF_CMD = 0xAA;
static const uint8_t PROM_READ_TEMPSENS_CMD = 0xAC;
static const uint8_t RESET_CMD = 0x1E;

uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD);

void readBarometerTask(void const* arg)
{
    BarometerData* data = (BarometerData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3);   // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    uint8_t dataInBuffer;

    // PROM read for calibration coefficients
    uint16_t c1Sens = readCalibrationCoefficient(PROM_READ_SENS_CMD);
    uint16_t c2Off = readCalibrationCoefficient(PROM_READ_OFF_CMD);
    uint16_t c3Tcs = readCalibrationCoefficient(PROM_READ_TCS_CMD);
    uint16_t c4Tco = readCalibrationCoefficient(PROM_READ_TCO_CMD);
    uint16_t c5Tref = readCalibrationCoefficient(PROM_READ_TREF_CMD);
    uint16_t c6Tempsens = readCalibrationCoefficient(PROM_READ_TEMPSENS_CMD);

    uint32_t pressureReading = 0;   // Stores a 24 bit value
    uint32_t temperatureReading = 0;   // Stores a 24 bit value

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_BAROMETER_PERIOD);

        // Read D1 pressure
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        pressureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // Read D2 temperature
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for OSR 512

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        temperatureReading = 0;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer << 16;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer << 8;

        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer;

        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        // calculate 1st order pressure and temperature (1st order algorithm from MS5607-02BA03 datasheet)
        double dT = temperatureReading - c5Tref * pow(2, 8);
        double temp = 2000 + dT * c6Tempsens / pow(2, 23);
        double off = c2Off * pow(2, 17) + dT * c4Tco / pow(2, 6);
        double sens = c1Sens * pow(2, 16) + dT * c3Tcs / pow(2, 7);
        // pressure and temperature were not divided by 100 to keep the decimal places
        double pressure = (pressureReading * sens / pow(2, 21) - off) / pow(2, 15); // need to divide P by 100 to get mbar
        double temperature = (temp); // need to divide T by 100 to get degrees Celcius

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) pressure;
        data->temperature_ = (int32_t) temperature;
        osMutexRelease(data->mutex_);
    }
}

uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD)
{
    uint8_t dataInBuffer;
    uint16_t coefficient = 0;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD, CMD_SIZE, CMD_TIMEOUT);
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient = dataInBuffer << 8;
    HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient += dataInBuffer;
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return coefficient;
}
