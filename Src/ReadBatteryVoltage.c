#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadBatteryVoltage.h"

#include "Data.h"

static int READ_BATTERY_VOLTAGE_PERIOD = 250;

void readBatteryVoltageTask(void const* arg) {
    BatteryVoltageData* data = (BatteryVoltageData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();
}