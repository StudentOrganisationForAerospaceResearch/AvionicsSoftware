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
#include "RocketSM.hpp"

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

    void HandleProtocolMessage(Command& cmd) override;

private:
    // Private Functions
    DMBProtocolTask();        // Private constructor
    DMBProtocolTask(const DMBProtocolTask&);                        // Prevent copy-construction
    DMBProtocolTask& operator=(const DMBProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_DMBPROTOCOL_HPP_
