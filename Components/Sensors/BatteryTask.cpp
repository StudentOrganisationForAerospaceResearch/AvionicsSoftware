/*
  ******************************************************************************
  * File Name          : BatteryTask.cpp
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "BatteryTask.hpp"
#include "main.h"
#include "Data.h"
#include "DebugTask.hpp"
#include "Task.hpp"
#include <time.h>
#include "DMBProtocolTask.hpp"
#include "GPIO.hpp"
#include "TelemetryMessage.hpp"

/* Macros --------------------------------------------------------------------*/
constexpr double ADC_CONVERSION_FACTOR = 3.3/4095;
constexpr int VOLTAGE_DIVIDER_SCALE = 4; // Value to scale voltage back to original value
constexpr int BATTERY_VOLTAGE_ADC_POLL_TIMEOUT = 50;
/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Default constructor, sets and sets up storage for member variables
 */
BatteryTask::BatteryTask() : Task(BATTERY_TASK_QUEUE_DEPTH_OBJS)
{
    data = (BatteryData*)soar_malloc(sizeof(BatteryData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void BatteryTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize battery task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)BatteryTask::RunTask,
            (const char*)"BatTask",
            (uint16_t)BATTERY_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)BATTERY_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "BatteryTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief BatteryTask run loop
 * @param pvParams Currently unused task context
 */
void BatteryTask::Run(void * pvParams)
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
void BatteryTask::HandleCommand(Command& cm)
{
    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand());
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    default:
        SOAR_PRINT("BatteryTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void BatteryTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within REQUEST_COMMAND
    switch (taskCommand) {
    case BATTERY_REQUEST_NEW_SAMPLE:
        SampleBatteryVoltage();
        break;
    case BATTERY_REQUEST_TRANSMIT:
    	TransmitProtocolBatteryData();
        break;
    case BATTERY_REQUEST_DEBUG:
        SOAR_PRINT("|VOLTAGE_TASK| Battery Voltage (V): %d.%d, MCU Timestamp: %u\r\n", data->voltage_ / 1000, data->voltage_ % 1000,
        timestampPT);
        SOAR_PRINT("Power State: %d, \r\n", GetPowerState());
        break;
    default:
        SOAR_PRINT("BATTERYTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief This function reads and updates battery voltage reading
 *          from the battery.
 */
void BatteryTask::SampleBatteryVoltage()
{
	double adcVal[1] = {};
	double batteryVoltageValue = 0;
	double vi = 0;

	HAL_ADC_Start(&hadc2);  // Enables ADC and starts conversion of regular channels
	if(HAL_ADC_PollForConversion(&hadc2, BATTERY_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
		adcVal[0] = HAL_ADC_GetValue(&hadc2); // Get ADC Value
		HAL_ADC_Stop(&hadc2);
		}

	vi = (ADC_CONVERSION_FACTOR * (adcVal[0])); // Converts 12 bit ADC value into voltage
	batteryVoltageValue = (vi * VOLTAGE_DIVIDER_SCALE) * 1000; // Multiply by 1000 to keep decimal places
	data->voltage_ = (uint32_t) batteryVoltageValue; // Battery Voltage in volts

	timestampPT = HAL_GetTick();
}

Proto::Battery::PowerSource BatteryTask::GetPowerState() {
	if (GPIO::PowerSelect::IsInternal()) {
		return Proto::Battery::PowerSource::ROCKET;
	}
	else {
		return Proto::Battery::PowerSource::GROUND;
	}
}

/**
 * @brief Transmits a protocol battery voltage data sample
 */
void BatteryTask::TransmitProtocolBatteryData()
{
    //SOAR_PRINT("Battery Transmit...\n");

    Proto::TelemetryMessage msg;
	msg.set_source(Proto::Node::NODE_DMB);
	msg.set_target(Proto::Node::NODE_RCU);
	Proto::Battery bat;
	bat.set_voltage(data->voltage_);
	bat.set_power_source(GetPowerState());
	msg.set_battery(bat);

	EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
	msg.serialize(writeBuffer);

    // Send the battery voltage data
    DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
