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

protected:
    static void RunTask(void* pvParams) { DMBProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // This will receive a (DMB_PROTO_COMMAND, DMB_PROTO_RX_DECODED_DATA) with the data pointer allocated, COBS decoded (but in the SOAR Message Format)
    void HandleProtocolMessage(Command& cmd) override;

    // Helper functions

    // Member variables

private:
    DMBProtocolTask();        // Private constructor
    DMBProtocolTask(const DMBProtocolTask&);                        // Prevent copy-construction
    DMBProtocolTask& operator=(const DMBProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_DMBPROTOCOL_HPP_
