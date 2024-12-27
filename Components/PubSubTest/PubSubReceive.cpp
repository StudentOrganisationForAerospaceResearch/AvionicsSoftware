/**
 ********************************************************************************
 * @file    PubSubReceive.cpp
 * @author  shiva
 * @date    Dec 14, 2024
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "PubSubReceive.hpp"
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
 * @brief Constructor for PubSubReceive
 */
PubSubReceive::PubSubReceive() : Task(PUBSUB_RECEIVE_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the PubSubReceive
 *        Do not modify this function aside from adding the task name
 */
void PubSubReceive::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PubSubReceive::RunTask,
            (const char*)"PubSubReceive",
            (uint16_t)PUBSUB_RECEIVE_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)PUBSUB_RECEIVE_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

						SOAR_ASSERT(rtValue == pdPASS, "PubSubReceive::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void PubSubReceive::Run(void * pvParams)
{
	SOAR_PRINT("\nPUBSUB RECIEVE STARTED\n");
    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res) {
            HandleCommand(cm);
        }
    }
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void PubSubReceive::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {

    default:
        SOAR_PRINT("PubSubReceive - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}
