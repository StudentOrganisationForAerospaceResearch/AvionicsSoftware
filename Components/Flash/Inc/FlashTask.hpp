/**
 ******************************************************************************
 * File Name          : FlashTask.hpp
 * Description        : Flash interface task. Used for data logging and state recovery
 ******************************************************************************
*/
#ifndef SOAR_FLASHTASK_HPP_
#define SOAR_FLASHTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "SystemStorage.hpp"
#include "SPIFlash.hpp"
#include "Timer.hpp"
#include "Data.h"
#include "FlashLogHandler.hpp"

/* Macros/Enums ------------------------------------------------------------*/
constexpr uint16_t MAX_FLASH_TASK_WAIT_TIME_MS = 5000; // The max time to wait for a command before maintenance is checked


enum FLASH_COMMANDS {
    WRITE_STATE_TO_FLASH = 0,
    WRITE_DATA_TO_FLASH = 0x01, // The top 3 bits of this contain the ID of the log type to be logged
    DUMP_FLASH_DATA,
    ERASE_ALL_FLASH,
	GET_FLASH_OFFSET,
	FLASH_DUMP_AT,
	FLASH_RESET_AND_ERASE,
	GET_LOGS_PAST_SECOND,
	GET_PAGE_OFFSET,
	FLASH_READ_FIRST_LOGS,
	TOG_BUFLOGS
};


class FlashTask : public Task
{
public:
    static FlashTask& Inst() {
        static FlashTask inst;
        return inst;
    }

    void InitTask();


    // Will put cmd in regular queue, unless it is full, where it will put cmd in the
    // separate priority queue
    void SendPriorityCommand(Command& cmd);

protected:
    static void RunTask(void* pvParams) { FlashTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);

    Timer* benchmarktimer;

    Queue* qPriorityQueue;

    static void benchmarkcallback(TimerHandle_t x) {


        Command cmd((uint16_t)GET_LOGS_PAST_SECOND);
        FlashTask::Inst().SendPriorityCommand(cmd);

    	Timer::DefaultCallback(x);
    }

private:


    // Private Functions
    FlashTask();        // Private constructor
    FlashTask(const FlashTask&);                        // Prevent copy-construction
    FlashTask& operator=(const FlashTask&);            // Prevent assignment

    // Offsets
    struct Offsets
    {
        uint32_t writeDataOffset;
    };

    FlashLogHandler* loghandler;



    bool flashDumpVerbose;

};

#endif    // SOAR_FLASHTASK_HPP_
