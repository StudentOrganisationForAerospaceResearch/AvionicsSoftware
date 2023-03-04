/**
 ******************************************************************************
 * File Name          : DMBProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#include "DMBProtocolTask.hpp"

#include "FlightTask.hpp"

/**
 * @brief Initialize the DMBProtocolTask
 */
void DMBProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)DMBProtocolTask::RunTask,
            (const char*)"ProtocolTask",
            (uint16_t)TASK_PROTOCOL_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_PROTOCOL_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "ProtocolTask::InitTask - xTaskCreate() failed");
}

/**
 * @brief Default constructor
 */
DMBProtocolTask::DMBProtocolTask() : ProtocolTask(Proto::Node::NODE_DMB)
{
}

/**
 * @brief Handle protocol message
 */
void DMBProtocolTask::HandleProtocolMessage(Command& cmd)
{
    SOAR_PRINT("We're totally handling a protocol message! (TODO - implement this)\n");
}
