/**
 ******************************************************************************
 * File Name          : DMBProtocolTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SYSTEM_DMB_PROTO_TASK_HPP_
#define SOAR_SYSTEM_DMB_PROTO_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "ProtocolTask.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class DMBProtocolTask : public Task
{
public:
    static DMBProtocolTask& Inst() {
        static DMBProtocolTask inst;
        return inst;
    }

    DMBProtocolTask();

    void InitTask();

    //Functions exposed to HAL callbacks
    void InterruptRxData();

protected:
    // NOTE: This must be in the derived class
    //    static void RunTask(void* pvParams) { DMBProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void* pvParams);    // Main run code

    void ConfigureUART();
    // This will receive a (DMB_PROTO_COMMAND, DMB_PROTO_RX_DECODED_DATA) with the data pointer allocated, COBS decoded (but in the SOAR Message Format)
    virtual void HandleDMBProtocolMessage(Command& cmd) = 0;   // This MUST be implemented in the derived board-specific DMBProtocolTask object
    //void HandleCommand(Command& cm);

    bool ReceiveData();

    // Helper functions

    // Member variables
    uint8_t* protocolRxBuffer;
    uint16_t protocolMsgIdx;
    bool isDMBProtocolMsgReady;

    uint8_t protocolRxChar; // Character received from UART Interrupt

private:
    // NOTE: This must be in the derived class
    //    DMBProtocolTask(); // Private constructor
    //    DMBProtocolTask(const DMBProtocolTask&);                    // Prevent copy-construction
    //    DMBProtocolTask& operator=(const DMBProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_SYSTEM_DMB_PROTO_TASK_HPP_
