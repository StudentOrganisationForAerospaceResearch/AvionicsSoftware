#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadCombustionChamberPressure.h"

#include "Data.h"
#include "Utils.h"

#define QUEUE_SIZE 5

static int READ_COMBUSTION_CHAMBER_PRESSURE_PERIOD = 80;

static const int COMBUSTION_CHAMBER_ADC_POLL_TIMEOUT = 50;
static const double R1 = 100;    // Resistor values in kOhms
static const double R2 = 133;

static uint16_t combustionChamberValuesQueue[QUEUE_SIZE] = {0};

void readCombustionChamberPressureTask(void const* arg)
{
    CombustionChamberPressureData* data = (CombustionChamberPressureData* ) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    double vo = 0;  // The voltage across the 133k resistor
    double vi = 0;  // The pressure sensor output
    double chamberPressure = 0;

    int combustionChamberQueueIndex = 0;

    HAL_ADC_Start(&hadc1);  // Enables ADC and starts conversion of regular channels

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_COMBUSTION_CHAMBER_PRESSURE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc1, COMBUSTION_CHAMBER_ADC_POLL_TIMEOUT) == HAL_OK)
        {
            combustionChamberValuesQueue[combustionChamberQueueIndex++] = HAL_ADC_GetValue(&hadc1);
        }

        uint16_t adcRead = averageArray(combustionChamberValuesQueue, QUEUE_SIZE);

        vo = 3.3 / pow(2, 12) * adcRead;    // Calculate voltage from the 12 bit ADC reading

        // vi to voltage divider varies between 0.5V-4.5V, but the board requires a voltage less than 3.3V.
        // After the voltage divider, the voltage varies between 0.285V-2.57V
        vi = (R2 + R1) / R2 * vo;   // Calculate the original voltage output of the sensor

        // The pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0.5V, and is 1000
        // psi when the voltage is 4.5V. The equation is derived from this information.
        chamberPressure = (vi - 0.5) * 1000 / 4;  // Tank pressure in psi
        chamberPressure = chamberPressure * 1000;   // Multiply by 1000 to keep decimal places

        combustionChamberQueueIndex %= QUEUE_SIZE;

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) chamberPressure;
        osMutexRelease(data->mutex_);
    }
}
