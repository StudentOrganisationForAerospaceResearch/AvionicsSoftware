/**
  ******************************************************************************
  * File Name          : BatteryTask.cpp
  *
  *    Source Info           : Based on Andromeda V3.31 Implementation
  *                         Andromeda_V3.31_Legacy/Core/Src/ReadBarometer.c
  *
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the flight board. A
  *                      thread task is included that will constantly loop,
  *                      reading and updating the passed BarometerData struct.
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
            (const char*)"PTTask",
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
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
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
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case BATTERY_REQUEST_NEW_SAMPLE:
        SampleBatteryVoltage();
        GetPowerState();
        break;
    case BATTERY_REQUEST_TRANSMIT:
    	TransmitProtocolBatteryData();
        break;
    case BATTERY_REQUEST_DEBUG:
        SOAR_PRINT("|VOLTAGE_TASK| Battery Voltage (V): %d.%d, MCU Timestamp: %u\r\n", data->voltage_ / 1000, data->voltage_ % 1000,
        timestampPT);
        SOAR_PRINT("Battery Power State: %d, \r\n", GetPowerState());
        break;
    default:
        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief This function reads and updates battery voltage reading
 *          from the battery.
 */
void BatteryTask::SampleBatteryVoltage()
{
	static const int BATTERY_VOLTAGE_ADC_POLL_TIMEOUT = 50;
	double adcVal[1] = {};
	double batteryVoltageValue = 0;
	double vi = 0;

	/* Functions -----------------------------------------------------------------*/
	HAL_ADC_Start(&hadc2);  // Enables ADC and starts conversion of regular channels
	if(HAL_ADC_PollForConversion(&hadc2, BATTERY_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
		adcVal[0] = HAL_ADC_GetValue(&hadc2); // Get ADC Value
		HAL_ADC_Stop(&hadc2);
		}

	vi = ((3.3/4095) * (adcVal[0])); // Converts 12 bit ADC value into voltage
	batteryVoltageValue = (vi * 4) * 1000; // Multiply by 1000 to keep decimal places
	data->voltage_ = (uint32_t) batteryVoltageValue; // Battery Voltage in volts

	timestampPT = HAL_GetTick();
}

enum Proto::Battery::power_source BatteryTask::GetPowerState() {
	if (GPIO::PowerSelect::IsInternal()) {
		return Proto::Battery::power_source::ROCKET;
	}
	else if (!GPIO::PowerSelect::IsInternal()) {
		return Proto::Battery::power_source::GROUND;
	}
	else {
		return Proto::Battery::power_source::INVALID;
	}
}

/**
 * @brief Transmits a protocol battery voltage data sample
 */
void BatteryTask::TransmitProtocolBatteryData()
{
    SOAR_PRINT("Battery Transmit...\n");

    Proto::TelemetryMessage msg;
	msg.set_source(Proto::Node::NODE_DMB);
	msg.set_target(Proto::Node::NODE_RCU);
	Proto::Battery bat;
	bat.set_volt(data->voltage_);
	bat.set_p_source(GetPowerState());
	msg.set_bat(bat);

	EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
	msg.serialize(writeBuffer);

    // Send the battery voltage data
    DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
