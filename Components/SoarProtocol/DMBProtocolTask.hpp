/**
 ******************************************************************************
 * File Name          : DMBProtocolTask.hpp
 * Description        : Protocol task, specific to DMB
 ******************************************************************************
*/
#ifndef SOAR_DMBPROTOCOL_HPP_
#define SOAR_DMBPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class DMBProtocolTask : public ProtocolTask
{
public:
    static DMBProtocolTask& Inst() {
        static DMBProtocolTask inst;
        return inst;
    }

    void InitTask();

    //Functions exposed to HAL callbacks
    void InterruptRxData();

protected:
    // NOTE: This must be in the derived class
    static void RunTask(void* pvParams) { DMBProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void* pvParams);    // Main run code

    void ConfigureUART();
    // This will receive a (DMB_PROTO_COMMAND, DMB_PROTO_RX_DECODED_DATA) with the data pointer allocated, COBS decoded (but in the SOAR Message Format)
    void HandleProtocolMessage(Command& cmd) override;
    //void HandleCommand(Command& cm);

    bool ReceiveData();

    // Helper functions

    // Member variables
    uint8_t* protocolRxBuffer;
    uint16_t protocolMsgIdx;
    bool isDMBProtocolMsgReady;

    uint8_t protocolRxChar; // Character received from UART Interrupt

private:
    DMBProtocolTask();        // Private constructor
    DMBProtocolTask(const DMBProtocolTask&);                        // Prevent copy-construction
    DMBProtocolTask& operator=(const DMBProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_DMBPROTOCOL_HPP_
