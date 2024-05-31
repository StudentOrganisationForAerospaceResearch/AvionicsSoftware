/**
 ******************************************************************************
 * File Name          : FlashTask.cpp
 * Description        : Flash interface task. Used for logging, writing to system
 *                      state, and flash maintenance operations for the system.
 ******************************************************************************
*/
#include "FlashTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Utils.hpp"
#include "Timer.hpp"
#include "RocketSM.hpp"
#include "SPIFlash.hpp"
#include "Data.h"
#include <cstring>
#include "FlashLogHandler.hpp"


/**
 * @brief Constructor for FlashTask
 */
FlashTask::FlashTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS) {

    flashDumpVerbose = false;

    qPriorityQueue = new Queue(FLASH_TASK_QUEUE_DEPTH_OBJS);

    benchmarktimer = new Timer(benchmarkcallback);
    benchmarktimer->SetAutoReload(true);
    benchmarktimer->Start();
    benchmarktimer->ResetTimerAndStart();

    loghandler = nullptr;
}

/**
 * @brief Initialize the FlashTask
 */
void FlashTask::InitTask() {
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flash task twice");

    BaseType_t rtValue = xTaskCreate((TaskFunction_t) FlashTask::RunTask,
    		(const char*) "FlashTask", (uint16_t) FLASH_TASK_STACK_DEPTH_WORDS,
    		(void*) this, (UBaseType_t) FLASH_TASK_RTOS_PRIORITY,
    		(TaskHandle_t*) &rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS,
    		"FlashTask::InitTask() - xTaskCreate() failed");

    SOAR_PRINT("Flash Task initialized");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlashTask::Run(void *pvParams) {
    // Wait until the flash has been initialized by flight task
    while (!SPIFlash::Inst().GetInitialized())
        osDelay(1);

    loghandler = new FlashLogHandler(SPI_FLASH_PAGE_OFFSET_STORAGE/SPIFlash::Inst().GetSectorSize(),
                                     SPI_FLASH_LOGGING_STORAGE_START_ADDR/SPIFlash::Inst().GetSectorSize(),
                                     (w25qxx.CapacityInKiloByte*1024-SPI_FLASH_LOGGING_STORAGE_START_ADDR)/SPIFlash::Inst().GetSectorSize(),
                                     SPIFlash::Inst().GetSectorSize());

    while (1) {
    	//Process any commands in the queue
    	Command cm;
    	bool res = qEvtQueue->Receive(cm, MAX_FLASH_TASK_WAIT_TIME_MS); // all sensor logs in here
    	if (res)
    		HandleCommand(cm);

    	res = qPriorityQueue->Receive(cm, 0);
    	if(res) {
    		HandleCommand(cm);
    		SOAR_PRINT("Flash Priority Received\n");
    	}
    	//Run maintenance on dual sector storages
    	SystemStorage::Inst().Maintain();

    }
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void FlashTask::HandleCommand(Command &cm) {
	// Can't have this if we try to erase then reset?
	//SOAR_ASSERT(w25qxx.UniqID[0] != 0, "Flash command received before flash was initialized");

	switch (cm.GetCommand()) {
	case TASK_SPECIFIC_COMMAND: {
		if (cm.GetTaskCommand() == WRITE_STATE_TO_FLASH) {
			// Read the current state from the system storage, and change the rocket state to the new state
			SystemState sysState;
			SystemStorage::Inst().Read(sysState);
			sysState.rocketState = (RocketState) (cm.GetDataPointer()[0]);
			SystemStorage::Inst().Write(sysState);
			SOAR_PRINT("System state written to flash\n");

		} else if (cm.GetTaskCommand() == ERASE_ALL_FLASH) {
			// Erase the system storage to make sure it isn't being used anymore
			SystemStorage::Inst().Erase();
			loghandler->ResetLogging();

		} else if (cm.GetTaskCommand() == GET_PAGE_OFFSET) {
			SOAR_PRINT("Flash page offset is at %luth page\n", loghandler->GetCurrentPage());

		} else if (cm.GetTaskCommand() == FLASH_DUMP_AT) {
			uint32_t dumptarget = *(uint32_t*) cm.GetDataPointer();
			SOAR_PRINT("Dumping 256 bytes at %d\n", dumptarget);
			uint8_t databuf[256];
			W25qxx_ReadBytes(databuf, dumptarget, sizeof(databuf));
			for (size_t i = 0; i < sizeof(databuf); i++) {
				SOAR_PRINT("%02x ", databuf[i]);
				if (i % 16 == 0) {
					SOAR_PRINT("\n");
				}
			}
			SOAR_PRINT("\n");

		} else if (cm.GetTaskCommand() == FLASH_READ_FIRST_LOGS) {
			int32_t numfirst = *(int32_t*) cm.GetDataPointer();
			SOAR_PRINT("first %d logs: \n", numfirst);
			loghandler->DumpFirstNLogs(numfirst, flashDumpVerbose);

		} else if (cm.GetTaskCommand() == GET_LOGS_PAST_SECOND) {
			SOAR_PRINT("%d logs in past second\n", loghandler->GetLogBenchmark());

		} else if(cm.GetTaskCommand() == TOG_BUFLOGS) {
			loghandler->ToggleMessageOnWrite();

		} else if (cm.GetTaskCommand() == FLASH_RESET_AND_ERASE) {
		    int32_t eraseSectors = *(int32_t*)cm.GetDataPointer();
		    loghandler->PartialResetLogging(eraseSectors);

		}
		else {
			SOAR_PRINT("FlashTask Received Unsupported Task Command: %d\n", cm.GetTaskCommand());
		}
		break;
	}
	case DATA_COMMAND: {
		// If the command is not a WRITE_DATA_TO_FLASH command do nothing
		if ((cm.GetTaskCommand()&0b00011111) != WRITE_DATA_TO_FLASH) {
			SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n",
					cm.GetTaskCommand());
			break;
		}

		FLASH_LOG_TYPE thisType = (FLASH_LOG_TYPE)((cm.GetTaskCommand()>>5)&0b00000111);
		if(thisType == LTYPE_INVAL || thisType >= LTYPE_OTHER) {
			SOAR_PRINT("Unknown log type asked to flash %d\n",thisType);
			break;
		}
		loghandler->AddFlashLog(thisType,cm.GetDataPointer(), cm.GetDataSize());
		break;
	}

	default:
		SOAR_PRINT("FlashTask Received Unsupported Command: %d\n",
				cm.GetCommand());
		break;
	}

	cm.Reset();
}


void FlashTask::SendPriorityCommand(Command& cmd) {
	if(qEvtQueue->GetQueueMessageCount() == qEvtQueue->GetQueueDepth()) {
		qPriorityQueue->Send(cmd);
	} else {
		qEvtQueue->Send(cmd);
	}
}
