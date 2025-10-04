/**
 ********************************************************************************
 * @file    Logging_Task_M.cpp
 * @author  jaddina
 * @date    Sep 6, 2025
 * @brief
 *
 * * Setup Steps
 * 1. Define the Task Queue Depth in SystemDefines.hpp
 * 2. Define the Task Stack Depth in SystemDefines.hpp
 * 3. Define the Task Priority in SystemDefines.hpp
 * 4. Replace all placeholders marked with a $ sign
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "LoggingTest.hpp"
#include "SystemDefines.hpp"
#include "Command.hpp"
#include "DataBroker.hpp"
#include "DataBrokerMessageTypes.hpp"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/************************************
 * FUNCTION DEFINITIONS
 ************************************/
/**
 * @brief Constructor for LoggingTask
 */
LoggingTask::LoggingTask()
: Task(LOGGING_TASK_DEPTH_OBJS)
{

}

/**
 * @brief Initialize the LoggingTask
 *        Do not modify this function aside from adding the task name
 */
void LoggingTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LoggingTask::RunTask,
            (const char*)"LoggingTask",
            (uint16_t)LOGGING_TASK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)LOGGING_TASK_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "LoggingTask::InitTask() - xTaskCreate() failed");
}


/**
 * @brief Instance Run loop for the Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void LoggingTask::Run(void * pvParams)
{
	DataBroker::Subscribe<IMUData>(this);
	DataBroker::Subscribe<PressureData>(this);
	DataBroker::Subscribe<ThermocoupleData>(this);


    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res){
        	HandleCommand(cm);
        }
    }
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void LoggingTask::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {

    case DATA_BROKER_COMMAND:
          HandleDataBrokerCommand(cm);
          break;

    default:
        SOAR_PRINT("LoggingTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

bool LoggingTask::HandleDataBrokerCommand(Command& cm){

	DataBrokerMessageTypes messageType = DataBroker::getMessageType(cm);
	IMUData imu_data;
	PressureData pressure_data;
	ThermocoupleData thermocouple_data;

	switch (messageType){

	case DataBrokerMessageTypes :: IMU_DATA:
		imu_data = DataBroker::ExtractData<IMUData>(cm);
		SOAR_PRINT("Data Recieved\n");
		SOAR_PRINT("accelX: %d\n", imu_data.accelX);
		SOAR_PRINT("accelY: %d\n", imu_data.accelY);
		SOAR_PRINT("accelZ: %d\n", imu_data.accelZ);

		//access IMU data, then write data to a file in the fs
		//Use FreeRTOS FATFS wrapper
		break;
	case DataBrokerMessageTypes::PRESSURE_DATA:
		pressure_data = DataBroker::ExtractData<PressureData>(cm);
		//access PressureData data, then write data to a file in the fs
		//Use FreeRTOS FATFS wrapper
		SOAR_PRINT("Data Recieved");
		SOAR_PRINT("pressure: %f", pressure_data.pressure);
		break;
	case DataBrokerMessageTypes::THERMOCOUPLE_DATA:
		thermocouple_data = DataBroker::ExtractData<ThermocoupleData>(cm);
		//access Thermocouple data, then write data to a file in the fs
		//Use FreeRTOS FATFS wrapper
		SOAR_PRINT("Data Recieved\n");
		SOAR_PRINT("temperature: %f", thermocouple_data.temperature);

		break;
	case DataBrokerMessageTypes :: INVALID:
		SOAR_PRINT("Invalid data type");
	default:
		break;


	}


}
