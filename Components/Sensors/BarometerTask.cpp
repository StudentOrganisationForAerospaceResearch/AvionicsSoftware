/**
  ******************************************************************************
  * File Name          : BarometerTask.cpp
  *
  *	Source Info		   : Based on Andromeda V3.31 Implementation
  *						 Andromeda_V3.31_Legacy/Core/Src/ReadBarometer.c
  *
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the flight board. A
  *                      thread task is included that will constantly loop,
  *                      reading and updating the passed BarometerData struct.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "BarometerTask.hpp"
#include "main.h"
#include "Data.h"
#include "DebugTask.hpp"
#include "Task.hpp"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
constexpr int READ_BAROMETER_PERIOD = 25;
constexpr int TEMP_LOW = 2000;
constexpr int TEMP_VERY_LOW = -1500;
constexpr int CMD_SIZE = 1;
constexpr int CMD_TIMEOUT = 150;

static uint8_t ADC_D1_512_CONV_CMD = 0x42;
static uint8_t ADC_D2_512_CONV_CMD = 0x52;
static uint8_t ADC_READ_CMD = 0x00;
static uint8_t PROM_READ_SENS_CMD = 0xA2;
static uint8_t PROM_READ_OFF_CMD = 0xA4;
static uint8_t PROM_READ_TCS_CMD = 0xA6;
static uint8_t PROM_READ_TCO_CMD = 0xA8;
static uint8_t PROM_READ_TREF_CMD = 0xAA;
static uint8_t PROM_READ_TEMPSENS_CMD = 0xAC;
static uint8_t READ_BYTE_CMD = 0x00;
static uint8_t RESET_CMD = 0x1E;


/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Default constructor, sets and sets up storage for member variables
 */
BarometerTask::BarometerTask() : Task(TASK_DEBUG_STACK_DEPTH_WORDS)
{
    data = (BarometerData*)soar_malloc(sizeof(BarometerData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void BarometerTask::InitTask()
{
	// Make sure the task is not already initialized
	SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Debug task twice");

	// Start the task
	BaseType_t rtValue =
		xTaskCreate((TaskFunction_t)BarometerTask::RunTask,
			(const char*)"BaroTask",
			(uint16_t)TASK_BAROMETER_STACK_DEPTH_WORDS,
			(void*)this,
			(UBaseType_t)TASK_BAROMETER_PRIORITY,
			(TaskHandle_t*)&rtTaskHandle);

	//Ensure creation succeded
	SOAR_ASSERT(rtValue == pdPASS, "BarometerTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief BarometerTask run loop
 * @param pvParams Currently unused task context
 */
void BarometerTask::Run(void * pvParams)
{
	while (1) {
		Command cm;

		//Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

		//Process the command
		HandleCommand(cm);
	}
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void BarometerTask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a while, we need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

	//Switch for the GLOBAL_COMMAND
	switch (cm.GetCommand()) {
	case REQUEST_COMMAND: {
		HandleRequestCommand(cm.GetTaskCommand());
	}
	case TASK_SPECIFIC_COMMAND: {
		break;
	}
	default:
		SOAR_PRINT("BarometerTask - Received Unsupported Command {%d}\n", cm.GetCommand());
		break;
	}

	//No matter what we happens, we must reset allocated data
	cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void BarometerTask::HandleRequestCommand(uint16_t taskCommand)
{
	//Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case BARO_REQUEST_NEW_SAMPLE:
        SampleBarometer();
        break;
    case BARO_REQUEST_TRANSMIT:
        SOAR_PRINT("Stubbed: Barometer task transmit not implemented\n");
        break;
    case BARO_REQUEST_DEBUG:
        SOAR_PRINT("\t-- Barometer Data --\n");
        SOAR_PRINT(" Temp (C)       : %d.%d\n", data->temperature_ / 100, data->temperature_ % 100);
        SOAR_PRINT(" Pressure (mbar): %d.%d\n", data->pressure_ / 100, data->pressure_ % 100);
        SOAR_PRINT(" Pressure (kPa) : %d.%d\n\n", data->pressure_ / 1000, data->pressure_ % 1000);
        break;
    default:
        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief This function reads and updates pressure and temperature readings
 *		  from the barometer.
 */
void BarometerTask::SampleBarometer()
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
 * TEMP - Actual temperature (-40...85�C with 0.01�C resolution)
 *          TEMP = 20�C + (dT * TEMPSENS) = 2000 + (dT * C6)/2^23
 * OFF  - Offset at actual temperature
 *          OFF = OFFt1 + (TCO * dT) = (C2 * 2^17) + (C4 * dT)/2^6
 * SENS - Sensitivity at actual temperature
 *          SENS = SENSt1 + (TCS * dT) = (C1 * 2^16) + (C3 * dT)/2^7
 * P    - Temperature compensated pressure (10...1200mbar with 0.01mbar resolution)
 *          P = (D1 * SENS) - OFF = ((D1 * SENS)/2^21 - OFF)/2^15
 */

	// Variables
    uint32_t pressureReading = 0;    // Stores a 24 bit value
    uint32_t temperatureReading = 0;    // Stores a 24 bit value
    uint8_t dataInBuffer;

    // Reset the barometer
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &RESET_CMD, CMD_SIZE, CMD_TIMEOUT);
    osDelay(3); // 2.8ms reload after Reset command
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    // Read PROM for calibration coefficients
    uint16_t c1Sens = ReadCalibrationCoefficients(PROM_READ_SENS_CMD);
    uint16_t c2Off = ReadCalibrationCoefficients(PROM_READ_OFF_CMD);
    uint16_t c3Tcs = ReadCalibrationCoefficients(PROM_READ_TCS_CMD);
    uint16_t c4Tco = ReadCalibrationCoefficients(PROM_READ_TCO_CMD);
    uint16_t c5Tref = ReadCalibrationCoefficients(PROM_READ_TREF_CMD);
    uint16_t c6Tempsens = ReadCalibrationCoefficients(PROM_READ_TEMPSENS_CMD);

    /**
     * Repeatedly read digital pressure and temperature.
     * Convert these values into their calibrated counterparts.
     * Finally, update the data globally if the mutex is available.
     */
    /* Read Digital Pressure (D1) ----------------------------------------*/

    // Tell the barometer to convert the pressure to a digital value with an over-sampling ratio of 512
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &ADC_D1_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

    // Read the first byte (bits 23-16)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    pressureReading = dataInBuffer << 16;

    // Read the second byte (bits 15-8)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    pressureReading += dataInBuffer << 8;

    // Read the third byte (bits 7-0)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    pressureReading += dataInBuffer;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    /* Read Digital Temperature (D2) -------------------------------------*/

    // Tell the barometer to convert the temperature to a digital value with an over-sampling ratio of 512
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &ADC_D2_512_CONV_CMD, CMD_SIZE, CMD_TIMEOUT);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    osDelay(2); // 1.17ms max conversion time for an over-sampling ratio of 512

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &ADC_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

    // Read the first byte (bits 23-16)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    temperatureReading = dataInBuffer << 16;

    // Read the second byte (bits 15-8)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    temperatureReading += dataInBuffer << 8;

    // Read the third byte (bits 7-0)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &ADC_READ_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    temperatureReading += dataInBuffer;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    /* Calculate First-Order Temperature and Parameters ------------------*/

    // Calibration coefficients need to be type cast to int64_t to avoid overflow during intermediate calculations
    int32_t dT = temperatureReading - ((int32_t)c5Tref << 8);
    int32_t temp = 2000 + ((dT * (int64_t)c6Tempsens) >> 23); // Divide this value by 100 to get degrees Celsius
    int64_t off = ((int64_t)c2Off << 17) + ((dT * (int64_t)c4Tco) >> 6);
    int64_t sens = ((int64_t)c1Sens << 16) + ((dT * (int64_t)c3Tcs) >> 7);

    /* Calculate Second-Order Temperature and Pressure -------------------*/

    if (temp < TEMP_LOW)    // If the temperature is below 20�C
    {
        int32_t t2 = ((int64_t)dT * dT) >> 31;
        int64_t off2 = 61 * (((int64_t)(temp - 2000) * (temp - 2000)) >> 4);
        int64_t sens2 = 2 * ((int64_t)(temp - 2000) * (temp - 2000));

        if (temp < TEMP_VERY_LOW)   // If the temperature is below -15�C
        {
            off2 = off2 + (15 * ((int64_t)(temp + 1500) * (temp + 1500)));
            sens2 = sens2 + (8 * ((int64_t)(temp + 1500) * (temp + 1500)));
        }

        temp = temp - t2;
        off = off - off2;
        sens = sens - sens2;
    }

    int32_t p = (((pressureReading * sens) >> 21) - off) >> 15;   // Divide this value by 100 to get millibars

    /* Store Data --------------------------------------------------------*/
    data->pressure_ = p;
    data->temperature_ = temp;

    // All equations provided by MS5607-02BA03 data sheet

    // PRESSURE AND TEMPERATURE VALUES ARE STORED AT 100x VALUE TO MAINTAIN
    // 2 DECIMAL POINTS OF PRECISION AS AN INTEGER!
    // E.x. The value 1234 should be interpreted as 12.34
}

/**
 * @brief   This function reads and returns a 16-bit coefficient from the barometer.
 * @param   PROM_READ_CMD   The command to send in order to read the desired
 *                          coefficient. See the data sheet for the commands.
 * @return                  The read coefficient.
 */
uint16_t BarometerTask::ReadCalibrationCoefficients(uint8_t PROM_READ_CMD)
{
    uint16_t coefficient;
    uint8_t dataInBuffer;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(SystemHandles::SPI_Barometer, &PROM_READ_CMD, CMD_SIZE, CMD_TIMEOUT);

    // Read the first byte (bits 15-8)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient = dataInBuffer << 8;

    // Read the second byte (bits 7-0)
    HAL_SPI_TransmitReceive(SystemHandles::SPI_Barometer, &READ_BYTE_CMD, &dataInBuffer, CMD_SIZE, CMD_TIMEOUT);
    coefficient += dataInBuffer;

    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

    return coefficient;
}
