/**
 ******************************************************************************
 * File Name          : ReadBatteryVoltage.c
 * Description        : This file contains constants and functions designed to
 *                      read the voltage of the battery used to power the DMB.
 *                      The voltage first goes through a voltage divider, so
 *                      the reading =will initially be 1/4 the actual value.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "ReadBatteryVoltage.h"

#include "Data.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"

/* Constants -----------------------------------------------------------------*/

static const int BATTERY_VOLTAGE_ADC_POLL_TIMEOUT = 50;
static const int READ_BATTERY_VOLTAGE_PERIOD = 1000;

static const double ADC_SCALE = 2000.0 / 2797.0;  // TODO: Figure out why
static const double VOLTAGE_DIVIDER_SCALE = 4;

/* Functions -----------------------------------------------------------------*/

/**
 * This function is to be used as a thread task that reads and updates
 * the voltage level of the main battery powering the DMB.
 *
 * @param   arg A pointer to the BatteryVoltageData struct that will be updated.
 */
void readBatteryVoltageTask(void const* arg) {
  BatteryVoltageData* data = (BatteryVoltageData*)arg;
  uint32_t prevWakeTime = osKernelSysTick();

  uint32_t batteryVoltageValue = 0;

  HAL_ADC_Start(&hadc2);  // Enables ADC and starts conversion of regular channels

  for (;;) {
    osDelayUntil(&prevWakeTime, READ_BATTERY_VOLTAGE_PERIOD);

    if (HAL_ADC_PollForConversion(&hadc2, BATTERY_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) {
      batteryVoltageValue = HAL_ADC_GetValue(&hadc2);
      batteryVoltageValue = batteryVoltageValue * ADC_SCALE * VOLTAGE_DIVIDER_SCALE;

      if (osMutexWait(data->mutex_, 0) == osOK) {
        data->voltage_ = batteryVoltageValue;
        osMutexRelease(data->mutex_);
      }
    }
  }
}
