/**
  ******************************************************************************
  * File Name          : ReadBarometer.c
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadBarometer.h"
#include "Data.h"

/* Macros --------------------------------------------------------------------*/

#define READ_BAROMETER_PERIOD   20  // Sampling delay set to 50 Hz to match high frequency logging
#define TEMP_VERY_LOW           -1500
#define TEMP_LOW                2000
#define CMD_SIZE                1
#define CMD_TIMEOUT             150

/* Constants -----------------------------------------------------------------*/

static const uint8_t ADC_D1_512_CONV_CMD    = 0x42;
static const uint8_t ADC_D2_512_CONV_CMD    = 0x52;
static const uint8_t ADC_READ_CMD           = 0x00;
static const uint8_t PROM_READ_SENS_CMD     = 0xA2;
static const uint8_t PROM_READ_OFF_CMD      = 0xA4;
static const uint8_t PROM_READ_TCS_CMD      = 0xA6;
static const uint8_t PROM_READ_TCO_CMD      = 0xA8;
static const uint8_t PROM_READ_TREF_CMD     = 0xAA;
static const uint8_t PROM_READ_TEMPSENS_CMD = 0xAC;
static const uint8_t READ_BYTE_CMD          = 0x00;
static const uint8_t RESET_CMD              = 0x1E;

/* Variables -----------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD);

/* Functions -----------------------------------------------------------------*/

/**
 * This function is to be used as a thread task that reads and updates
 * pressure and temperature readings from the barometer.
 */
void readBarometerTask(void const* arg)
{
    /**
     * Variable Descriptions from MS5607-02BA03 Data Sheet:
     *
     * C1 (SENSt1)      - Pressure sensitivity
     * C2 (OFFt1)       - Pressure offset
     * C3 (TCS)         - Temperature coefficient of pressure sensitivity
     * C4 (TCO)         - Temperature coefficient of pressure offset
     * C5 (Tref)        - Reference temperature
     * C6 (TEMPSENS)    - Temperature coefficient of the temperature
     *
     * D1   - Digital pressure value
     * D2   - Digital temperature value
     *
     * dT   - Difference between actual and reference temperature
     *          dT = D2 - Tref = D2 - (C5 * 2^8)
     * TEMP - Actual temperature (-40...85°C with 0.01°C resolution)
     *          TEMP = 20°C + (dT * TEMPSENS) = 2000 + (dT * C6)/2^23
     * OFF  - Offset at actual temperature
     *          OFF = OFFt1 + (TCO * dT) = (C2 * 2^17) + (C4 * dT)/2^6
     * SENS - Sensitivity at actual temperature
     *          SENS = SENSt1 + (TCS * dT) = (C1 * 2^16) + (C3 * dT)/2^7
     * P    - Temperature compensated pressure (10...1200mbar with 0.01mbar resolution)
     *          P = (D1 * SENS) - OFF = ((D1 * SENS)/2^21 - OFF)/2^15
     */

    // Variables
    BarometerData* data         = (BarometerData*) arg;
    uint32_t prevWakeTime       = osKernelSysTick();
    uint32_t pressureReading    = 0;    // Stores a 24 bit value
    uint32_t temperatureReading = 0;    // Stores a 24 bit value
    uint8_t dataInBuffer;

    // Reset the barometer
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    // Read PROM for calibration offsets
    uint16_t c1Sens     = readCalibrationCoefficient(PROM_READ_SENS_CMD);
    uint16_t c2Off      = readCalibrationCoefficient(PROM_READ_OFF_CMD);
    uint16_t c3Tcs      = readCalibrationCoefficient(PROM_READ_TCS_CMD);
    uint16_t c4Tco      = readCalibrationCoefficient(PROM_READ_TCO_CMD);
    uint16_t c5Tref     = readCalibrationCoefficient(PROM_READ_TREF_CMD);
    uint16_t c6Tempsens = readCalibrationCoefficient(PROM_READ_TEMPSENS_CMD);

    /**
     * Repeatedly read digital pressure and temperature.
     * Convert these values into their calibrated counterparts.
     * Finally, update the data globally if the mutex is available.
     *
     * Restricted to 50Hz.
     */
    while(1)
    {
        // Delay so that the loop operates at 50Hz
        osDelayUntil(&prevWakeTime, READ_BAROMETER_PERIOD);

        /* Read Digital Pressure (D1) ----------------------------------------*/

        // Tell the barometer to convert the pressure to a digital value with an over-sampling ratio of 512
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

        // Read the pressure value

        // Chip select to start a new command
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

        // Tell the chip we want to read from the adc
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        // Read the first byte (bits 23-16)
        HAL_SPI_TransmitReceive(&hspi2, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading = dataInBuffer << 16;

        // Read the second byte (bits 15-8)
        HAL_SPI_TransmitReceive(&hspi2, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer << 8;

        // Read the third byte (bits 7-0)
        HAL_SPI_TransmitReceive(&hspi2, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        pressureReading += dataInBuffer;

        // Release chip select to end the command
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        /* Read Digital Temperature (D2) -------------------------------------*/

        // Tell the barometer to convert the temperature to a digital value with an over-sampling ratio of 512
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi2, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

        // Read the temperature value

        // Chip select to start a new command
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

        // Tell the chip we want to read from the adc
        HAL_SPI_Transmit(&hspi2, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

        // Read the first byte (bits 23-16)
        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading = dataInBuffer << 16;

        // Read the second byte (bits 15-8)
        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer << 8;

        // Read the third byte (bits 7-0)
        HAL_SPI_TransmitReceive(&hspi2, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
        temperatureReading += dataInBuffer;

        // Release chip select to end the command
        HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

        /* Calculate First-Order Temperature and Pressure --------------------*/

        // Equations provided by MS5607-02BA03 data sheet

        int32_t dT      = temperatureReading - (c5Tref << 8);
        int32_t temp    = 2000 + ((dT * c6Tempsens) >> 23);                 // Divide this value by 100 to get degrees Celcius
        int64_t off     = (c2Off << 17) + ((dT * c4Tco) >> 6);
        int64_t sens    = (c1Sens << 16) + ((dT * c3Tcs) >> 7);
        int32_t p       = (((pressureReading * sens) >> 21) - off) >> 15;   // Divide this value by 100 to get millibars

        /* Calculate Second-Order Temperature and Pressure -------------------*/

        // Equations provided by MS5607-02BA03 data sheet

        if (temp < TEMP_LOW)    // If the temperature is below 20°C
        {
            int32_t t2      = (dT * dT) >> 31;
            int32_t off2    = 61 * (((temp - 2000) * (temp - 2000)) >> 4);
            int32_t sens2   = 2 * ((temp - 2000) * (temp - 2000));

            if (temp < TEMP_VERY_LOW)   // If the temperature is below -15°C
            {
                off2    = off2 + (15 * ((temp + 1500) * (temp + 1500)));
                sens2   = sens2 + (8 * ((temp + 1500) * (temp + 1500)));
            }

            temp    = temp  - t2;
            off     = off   - off2;
            sens    = sens  - sens2;
        }

        /* Store Data --------------------------------------------------------*/

        if (osMutexWait(data->mutex_, 0) == osOK)
        {
            // Assign new values if mutex was available
            data->pressure_     = p;
            data->temperature_  = temp;
            osMutexRelease(data->mutex_);
        }

        // PRESSURE AND TEMPERATURE VALUES ARE STORED AT 100x VALUE TO MAINTAIN
        // 2 DECIMAL POINTS OF PRECISION AS AN INTEGER!
        // E.x. The value 1234 should be interpreted as 12.34
    }
}

/**
 * This function reads and returns a 16-bit coefficient from the barometer.
 * @param   PROM_READ_CMD   The command to send in order to read the desired
 *                          coefficient. See the data sheet for the commands.
 * @return                  The read coefficient.
 */
uint16_t readCalibrationCoefficient(const uint8_t PROM_READ_CMD)
{
    // Variables
    uint16_t    coefficient;
    uint8_t     dataInBuffer;

    /* Read Coefficient ------------------------------------------------------*/

    // Chip select to start a new command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

    // Tell the chip we want to read from PROM
    HAL_SPI_Transmit(&hspi2, &PROM_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

    // Read the first byte (bits 15-8)
    HAL_SPI_TransmitReceive(&hspi2, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient = dataInBuffer << 8;

    // Read the second byte (bits 7-0)
    HAL_SPI_TransmitReceive(&hspi2, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient += dataInBuffer;

    // Release chip select to end the command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    // Return the read coefficient
    return coefficient;
}
