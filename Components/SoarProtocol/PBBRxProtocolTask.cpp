/**
 ******************************************************************************
 * File Name          : PBBRxProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#include "PBBRxProtocolTask.hpp"
#include "DMBProtocolTask.hpp"
#include "MEVManager.hpp"
#include "ReadBufferFixedSize.h"
#include "TelemetryMessage.hpp"
#include "UARTTask.hpp"

/**
 * @brief Initialize the PBBRxProtocolTask
 */
void PBBRxProtocolTask::InitTask() {
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue = xTaskCreate((TaskFunction_t)PBBRxProtocolTask::RunTask, (const char*)"PbbProtocol",
                                     (uint16_t)TASK_PROTOCOL_STACK_DEPTH_WORDS, (void*)this,
                                     (UBaseType_t)TASK_PROTOCOL_PRIORITY, (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "ProtocolTask::InitTask - xTaskCreate() failed");
}

/**
 * @brief Default constructor
 */
PBBRxProtocolTask::PBBRxProtocolTask()
    : ProtocolTask(Proto::Node::NODE_DMB, UART::Conduit_PBB, UART_TASK_COMMAND_SEND_PBB) {}

/**
 * @brief Handle a command message
 */
void PBBRxProtocolTask::HandleProtobufCommandMessage(
    EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer) {}

/**
 * @brief Handle a control message
 */
void PBBRxProtocolTask::HandleProtobufControlMesssage(
    EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer) {}

/**
 * @brief Handle a telemetry message
 */
void PBBRxProtocolTask::HandleProtobufTelemetryMessage(
    EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer) {
    Proto::TelemetryMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source node is the PBB
    if (msg.get_source() != Proto::Node::NODE_PBB)
        return;

    // If the target is the DMB, forward it to the RCU
    if (msg.get_target() == Proto::Node::NODE_DMB)
        msg.set_target(Proto::Node::NODE_RCU);

    // Prints for specific message contents
    if (msg.has_mevstate()) {
        SOAR_PRINT("PROTO-MEV-STATE: %d\n", msg.get_mevstate().get_mev_open());
        MEVManager::HandleMEVTelemetry(msg);
    }

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
