/**
 ********************************************************************************
 * @file    Logging_Task_M.hpp
 * @author  jaddina
 * @date    Sep 6, 2025
 * @brief
 ********************************************************************************
 */

#ifndef CORE_INC_LOGGING_TASK_M_HPP_
#define CORE_INC_LOGGING_TASK_M_HPP_

/************************************
 * INCLUDES
 ************************************/
//#include "SensorDataTypes.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/
constexpr uint16_t LOGGING_RX_BUFFER_SZ_BYTES = 16;
/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/
class LoggingTask : public Task
{
	public:
		static LoggingTask& Inst() {
			static LoggingTask inst;
			return inst;
		}



		void InitTask();

	protected:
		bool RecieveData();
		static void RunTask(void* pvParams) { LoggingTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
		void Run(void * pvParams); // Main run code
		void HandleCommand(Command& cm);
		void WriteData();//put argue
		bool HandleDataBrokerCommand(Command& cm);

	private:
		// Private Functions
		LoggingTask();        // Private constructor
		LoggingTask(const LoggingTask&);          // Prevent copy-construction
		LoggingTask& operator=(const LoggingTask&);		// Prevent assignment
};

#endif /* CORE_INC_LOGGING_TASK_M_HPP_ */
