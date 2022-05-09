#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadBatteryVoltage.h"

#include "Data.h"

static int READ_BATTERY_VOLTAGE_PERIOD = 1000;

static const int BATTERY_VOLTAGE_ADC_POLL_TIMEOUT = 50;

void readBatteryVoltageTask(void const* arg) {
    BatteryVoltageData* data = (BatteryVoltageData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    uint32_t batteryVoltageValue = 0;

    HAL_ADC_Start(&hadc2);  // Enables ADC and starts conversion of regular channels

    for (;;) {
        osDelayUntil(&prevWakeTime, READ_BATTERY_VOLTAGE_PERIOD);

        if (HAL_ADC_PollForConversion(&hadc2, BATTERY_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK)
        {
            batteryVoltageValue = HAL_ADC_GetValue(&hadc2); // TODO: Test!
        }

        batteryVoltageValue = batteryVoltageValue * 4;   // Multiply by 4 for the voltage divider calculation

         if (osMutexWait(data->mutex_, 0) == osOK)
        {
            data->voltage_ = (int32_t) batteryVoltageValue;
            osMutexRelease(data->mutex_);
        }
    }
}
