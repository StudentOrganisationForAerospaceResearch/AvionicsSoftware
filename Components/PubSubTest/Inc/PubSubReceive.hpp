/**
 ********************************************************************************
 * @file    PubSubReceive.hpp
 * @author  shiva
 * @date    Dec 14, 2024
 * @brief
 ********************************************************************************
 */

#ifndef PUBSUBRECEIEVE_HPP_
#define PUBSUBRECEIEVE_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
#include "SystemDefines.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
class PubSubReceive : public Task
{
public:
    static PubSubReceive& Inst() {
        static PubSubReceive inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { PubSubReceive::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);

private:
    // Private Functions
    PubSubReceive();        // Private constructor
    PubSubReceive(const PubSubReceive&);                        // Prevent copy-construction
    PubSubReceive& operator=(const PubSubReceive&);            // Prevent assignment
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* PUBSUBRECEIEVE_HPP_ */
