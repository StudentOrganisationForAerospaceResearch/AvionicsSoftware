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
 * @brief Handle a command message
 */
void DMBProtocolTask::HandleProtobufCommandMessage(uint8_t* data, uint16_t size)
{

}

/**
 * @brief Handle a control message
 */
void DMBProtocolTask::HandleProtobufControlMesssage(uint8_t* data, uint16_t size)
{

}

/**
 * @brief Handle a telemetry message
 */
void DMBProtocolTask::HandleProtobufTelemetryMessage(uint8_t* data, uint16_t size)
{

}
