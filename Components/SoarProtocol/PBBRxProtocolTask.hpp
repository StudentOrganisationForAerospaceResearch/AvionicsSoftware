/**
 ******************************************************************************
 * File Name          : PBBRxProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#ifndef SOAR_PBBRXPROTOCOL_HPP_
#define SOAR_PBBRXPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class PBBRxProtocolTask : public ProtocolTask
{
public:
    static PBBRxProtocolTask& Inst() {
        static PBBRxProtocolTask inst;
        return inst;
    }

    void InitTask();

    static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
		Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

    static void SendPbbCommand(Proto::PbbCommand::Command cmd)
    {
        Proto::CommandMessage cmdMsg;
        Proto::PbbCommand pbbCmd;
        cmdMsg.set_source(Proto::Node::NODE_DMB);
        cmdMsg.set_target(Proto::Node::NODE_PBB);
        pbbCmd.set_command_enum(cmd);
        cmdMsg.set_pbb_command(pbbCmd);
        EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
        cmdMsg.serialize(writeBuffer);
        PBBRxProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_COMMAND);
    }

    static void SendFastLogCommand(Proto::FastLog::FastLogCommand cmd)
    {
        Proto::ControlMessage ctrlMsg;
        Proto::FastLog fastLogCmd;
        ctrlMsg.set_source(Proto::Node::NODE_DMB);
        ctrlMsg.set_target(Proto::Node::NODE_PBB);
        fastLogCmd.set_cmd(cmd);
        ctrlMsg.set_fast_log(fastLogCmd);
        EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
        ctrlMsg.serialize(writeBuffer);
        PBBRxProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
    }



protected:
    static void RunTask(void* pvParams) { PBBRxProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    
    // Member variables

private:
    PBBRxProtocolTask();        // Private constructor
    PBBRxProtocolTask(const PBBRxProtocolTask&);                        // Prevent copy-construction
    PBBRxProtocolTask& operator=(const PBBRxProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_PBBRXPROTOCOL_HPP_
