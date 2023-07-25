/**
 ******************************************************************************
 * File Name          : PBBRxProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#include "PBBRxProtocolTask.hpp"
#include "DMBProtocolTask.hpp"
#include "ReadBufferFixedSize.h"
#include "TelemetryMessage.hpp"
#include "UARTTask.hpp"

/**
 * @brief Initialize the PBBRxProtocolTask
 */
void PBBRxProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PBBRxProtocolTask::RunTask,
            (const char*)"PbbProtocol",
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
PBBRxProtocolTask::PBBRxProtocolTask() : ProtocolTask(Proto::Node::NODE_DMB, 
    SystemHandles::UART_PBB,
    UART_TASK_COMMAND_SEND_PBB)
{
}

/**
 * @brief Handle a command message
 */
void PBBRxProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{

}

/**
 * @brief Handle a control message
 */
void PBBRxProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{

}

/**
 * @brief Handle a telemetry message
 */
void PBBRxProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    Proto::TelemetryMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source node is the PBB
    if (msg.get_source() != Proto::Node::NODE_PBB)
        return;

    // If the target is the DMB, forward it to the RCU
    if(msg.get_target() == Proto::Node::NODE_DMB)
    	msg.set_target(Proto::Node::NODE_RCU);

    // Prints for specific message contents
    if(msg.has_mevstate()) {
    	SOAR_PRINT("PROTO-MEV-STATE: %d\n", msg.get_mevstate().get_mev_open());
    }

    if(msg.has_presspbb()) {
//    	SOAR_PRINT("PBB-IB-PRESSURE : %d.%d \n", (msg.get_presspbb().get_ib_pressure())/1000, (msg.get_presspbb().get_ib_pressure())%1000);
		SOAR_PRINT("PBB-PV-PRESSURE : %d.%d \n", (msg.get_presspbb().get_lower_pv_pressure())/1000, (msg.get_presspbb().get_lower_pv_pressure())%1000);
    }

    if(msg.has_temppbb()) {
    	SOAR_PRINT("PBB-PV-TEMPERATURE : %d.%d \n", (msg.get_temppbb().get_ib_temperature())/100, (msg.get_temppbb().get_ib_temperature())%100);
    	// TODO : Change IB and PV name sin protobufs
    	//    	SOAR_PRINT("PBB-PV-TEMPERATURE : %d.%d \n", (msg.get_temppbb().get_pv_temperature())/100, (msg.get_temppbb().get_pv_temperature())%100);
    }

    // Copy the message to the read buffer
	SOAR_PRINT("PROTO-INFO: Received PBB Rx Telemetry Message\n");

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

	DMBProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
