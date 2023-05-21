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

protected:
    static void RunTask(void* pvParams) { PBBRxProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    
    // Member variables

private:
    PBBRxProtocolTask();        // Private constructor
    PBBRxProtocolTask(const PBBRxProtocolTask&);                        // Prevent copy-construction
    PBBRxProtocolTask& operator=(const PBBRxProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_PBBRXPROTOCOL_HPP_
