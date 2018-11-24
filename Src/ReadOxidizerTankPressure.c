#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"
#include "math.h"

#include "ReadOxidizerTankPressure.h"

#include "Data.h"
#include "Utils.h"

#define QUEUE_SIZE 5

static int READ_OXIDIZER_TANK_PRESSURE_PERIOD = 80;

static const int OXIDIZER_TANK_POLL_TIMEOUT = 50;

static uint16_t oxidizerTankValuesQueue[QUEUE_SIZE] = {0};

void readOxidizerTankPressureTask(void const* arg)
{
    OxidizerTankPressureData* data = (OxidizerTankPressureData* ) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    double vo = 0;  // The pressure sensor voltage after amplification
    double vi = 0;  // The original pressure sensor output
    double tankPressure = 0;

    int oxidizerTankQueueIndex = 0;

    HAL_ADC_Start(&hadc2);  // Enables ADC and starts conversion of regular channels

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_OXIDIZER_TANK_PRESSURE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc2, OXIDIZER_TANK_POLL_TIMEOUT) == HAL_OK)
        {
            oxidizerTankValuesQueue[oxidizerTankQueueIndex++] = HAL_ADC_GetValue(&hadc2);
        }

        uint16_t adcRead = averageArray(oxidizerTankValuesQueue, QUEUE_SIZE);

        vo = 3.3 / pow(2, 12) * adcRead;    // Calculate voltage from the 12 bit ADC reading

        // Since the voltage output of the pressure sensor is very small ( below 0.1V ), an opamp was used to amplify
        // the voltage to be more accuractely read by the ADC. See AndromedaV2 PCB schematic for details.
        vi = (double) (13.0 / 400.0) * vo * 1000; // Calculate the original voltage output of the sensor * 1000 to keep decimal places

        // The pressure sensor is ratiometric. The pressure is 0 psi when the voltage is 0V, and is 1000
        // psi when the voltage is 0.1V. The equation is derived from this information.
        tankPressure = vi * 1000 / 0.1;  // Tank pressure in psi

        oxidizerTankQueueIndex %= QUEUE_SIZE;

        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->pressure_ = (int32_t) tankPressure;
        osMutexRelease(data->mutex_);
    }
}