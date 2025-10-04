/**
 ********************************************************************************
 * @file    FSBProtocolTask.hpp
 * @author  jaddina
 * @date    Sep 13, 2025
 * @brief
 ********************************************************************************
 */

#ifndef FSBPROTOCOL_INC_FSBPROTOCOLTASK_HPP_
#define FSBPROTOCOL_INC_FSBPROTOCOLTASK_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
/************************************
 * MACROS AND DEFINES
 ************************************/
enum FSB_TASK_COMMANDS {
    PUBLISH_IMU =0,
    PUBLISH_PRESSURE,
	PUBLISH_THERMOCOUPLE,
};

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/
class FSBProtocolTask: public Task
{
	public:
		static FSBProtocolTask& Inst() {
			static FSBProtocolTask inst;
			return inst;
		}

		void InitTask();



	protected:
		bool RecieveData();
		static void RunTask(void* pvParams) { FSBProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void * pvParams); // Main run code
		void HandleCommand(Command& cm);
		//uint8_t debugBuffer[LOGGING_RX_BUFFER_SZ_BYTES + 1];

	private:
		// Private Functions
		FSBProtocolTask();        // Private constructor
		FSBProtocolTask(const FSBProtocolTask&);                        // Prevent copy-construction
		FSBProtocolTask& operator=(const FSBProtocolTask&);														// Prevent assignment
};
#endif /* FSBPROTOCOL_INC_FSBPROTOCOLTASK_HPP_ */
