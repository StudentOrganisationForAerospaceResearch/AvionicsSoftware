/**
 ********************************************************************************
 * @file    PubSubSend.hpp
 * @author  shiva
 * @date    Dec 14, 2024
 * @brief
 ********************************************************************************
 */

#ifndef PUBSUBSEND_HPP_
#define PUBSUBSEND_HPP_

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
class PubSubSend : public Task
{
public:
    static PubSubSend& Inst() {
        static PubSubSend inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { PubSubSend::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);

private:
    // Private Functions
    PubSubSend();        // Private constructor
    PubSubSend(const PubSubSend&);                        // Prevent copy-construction
    PubSubSend& operator=(const PubSubSend&);            // Prevent assignment
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* PUBSUBSEND_HPP_ */
