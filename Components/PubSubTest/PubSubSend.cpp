/**
 ********************************************************************************
 * @file    PubSubSend.cpp
 * @author  shiva
 * @date    Dec 14, 2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "PubSubSend.hpp"
#include "SystemDefines.hpp"
#include "SensorDataTypes.hpp"
#include "DataBroker.hpp"

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
 * @brief Constructor for PubSubSend
 */
PubSubSend::PubSubSend() : Task(PUBSUB_SEND_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the PubSubSend
 *        Do not modify this function aside from adding the task name
 */
void PubSubSend::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PubSubSend::RunTask,
            (const char*)"PubSubSend",
            (uint16_t)PUBSUB_SEND_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)PUBSUB_SEND_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

						SOAR_ASSERT(rtValue == pdPASS, "PubSubSend::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void PubSubSend::Run(void * pvParams)
{
	SOAR_PRINT("\nPUBSUB SEND STARTED\n");

	while(1) {
		Command cm;
		if(qEvtQueue->Receive(cm, 2000)) {
			HandleCommand(cm);
		}
		else {
			IMUData imuData = {
					.accelX = 1,
					.accelY = 2,
					.accelZ = 3,
			};
			DataBroker::PublishData<IMUData>(&imuData);
		}
	}
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void PubSubSend::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {

    case DATA_BROKER_COMMAND:
    	break;
//    	IMUData* newData = parseData(&data);



    default:
        SOAR_PRINT("PubSubSend - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}
