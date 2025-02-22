/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   This is a template header file to create a new task in our firmware
 *
 * Setup Steps
 * 1. Define the Task Queue Depth in SystemDefines.hpp
 * 2. Define the Task Stack Depth in SystemDefines.hpp
 * 3. Define the Task Priority in SystemDefines.hpp
 * 4. Replace all placeholders marked with a $ sign
 ********************************************************************************
 */
#ifndef LFSTaskTEST123_INC
#define LFSTaskTEST123_INC
/************************************
 * INCLUDES
 ************************************/
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "SPIFlash.hpp"
/************************************
 * MACROS AND DEFINES
 ************************************/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/************************************
 * TYPEDEFS
 ************************************/
enum LFS_TASK_COMMANDS {
		LFS_TASK_NONE = 0,
		WRITE_DATA_TO_LFS,
};
/************************************
 * CLASS DEFINITIONS
 ************************************/
class LFSTask : public Task
{
public:
    static LFSTask& Inst() {
        static LFSTask inst;
        return inst;
    }
    void InitTask();
protected:
    static void RunTask(void* pvParams) { LFSTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();
    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);
    void LFSTest();
    void WriteTest();
private:
    // Private Functions
    LFSTask();        // Private constructor
    LFSTask(const LFSTask&);                        // Prevent copy-construction
    LFSTask& operator=(const LFSTask&);            // Prevent assignment

    void Test();
    // Offsets
		struct Offsets
		{
				uint32_t writeDataOffset;
		};
		uint8_t writesSinceLastOffsetUpdate_;
};
/************************************
 * FUNCTION DECLARATIONS
 ************************************/
#endif /* LFSTaskTEST123_INC */
