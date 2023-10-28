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


/**
 * @brief Constructor for FlashTask
 */
FlashTask::FlashTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS)
{
	logbufA = (uint8_t*)soar_malloc(FLASH_HEAP_BUF_SIZE);
	logbufB = (uint8_t*)soar_malloc(FLASH_HEAP_BUF_SIZE);
	currbuf = logbufA;
	offsetWithinBuf=0;
	SOAR_ASSERT(logbufA && logbufB, "Failed to allocate flash log bufs.\n");
	currentLogPage = SPI_FLASH_LOGGING_STORAGE_START_ADDR / 256;
	logsInLastSecond = 0;
}

/**
 * @brief Initialize the FlashTask
 */
void FlashTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flash task twice");
    
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FlashTask::RunTask,
            (const char*)"FlashTask",
            (uint16_t)FLASH_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)FLASH_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "FlashTask::InitTask() - xTaskCreate() failed");
    benchmarktimer = new Timer(benchmarkcallback);
    benchmarktimer->SetAutoReload(true);
    benchmarktimer->Start();
    benchmarktimer->ResetTimerAndStart();
    SOAR_PRINT("Flash Task initialized");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void FlashTask::Run(void * pvParams)
{
    // Wait until the flash has been initialized by flight task
    while (!SPIFlash::Inst().GetInitialized())
        osDelay(1);

    // Initialize the offsets storage
    offsetsStorage_ = new SimpleDualSectorStorage<Offsets>(&SPIFlash::Inst(), SPI_FLASH_OFFSETS_SDSS_START_ADDR);
    offsetsStorage_->Read(currentOffsets_);

    while (1) {
        //Process any commands in the queue
        Command cm;
        bool res = qEvtQueue->Receive(cm, MAX_FLASH_TASK_WAIT_TIME_MS);
        if(res)
            HandleCommand(cm);

        //Run maintenance on dual sector storages
        SystemStorage::Inst().Maintain();
        offsetsStorage_->Maintain();
    }
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void FlashTask ::HandleCommand(Command& cm)
{
    // Can't have this if we try to erase then reset?
    //SOAR_ASSERT(w25qxx.UniqID[0] != 0, "Flash command received before flash was initialized");

    switch (cm.GetCommand()) {
    case TASK_SPECIFIC_COMMAND: {
        if (cm.GetTaskCommand() == WRITE_STATE_TO_FLASH)
        {
            // Read the current state from the system storage, and change the rocket state to the new state
            SystemState sysState;
            SystemStorage::Inst().Read(sysState);
            sysState.rocketState = (RocketState)(cm.GetDataPointer()[0]);
            SystemStorage::Inst().Write(sysState);

            SOAR_PRINT("System state written to flash\n");
        }
        else if (cm.GetTaskCommand() == DUMP_FLASH_DATA)
        {
            ReadLogDataFromFlash();
        }
        else if (cm.GetTaskCommand() == ERASE_ALL_FLASH)
        {
            // Erase the system storage to make sure it isn't being used anymore
            SystemStorage::Inst().Erase();

            // Erase the chip
            W25qxx_EraseChip();
            currentOffsets_.writeDataOffset = 0;
        }
        else if (cm.GetTaskCommand() == GET_FLASH_OFFSET) {
        	SOAR_PRINT("Flash offset is at 0x%04x\n%d writes since update\n",currentOffsets_.writeDataOffset,writesSinceLastOffsetUpdate_);
        } else if (cm.GetTaskCommand() == GET_PAGE_OFFSET) {
        	SOAR_PRINT("Flash page offset is at %dth page\n",currentLogPage);
        }
        else if (cm.GetTaskCommand() == FLASH_DUMP_AT) { // WIP
        	uint32_t dumptarget = *(uint32_t*)cm.GetDataPointer();
        	SOAR_PRINT("we dumpin at %d\n", dumptarget);
        	uint8_t databuf[256];
        	W25qxx_ReadBytes(databuf, dumptarget, sizeof(databuf));
        	for(size_t i = 0; i < sizeof(databuf); i++) {
        		SOAR_PRINT("%02x ",databuf[i]);
        		if(i%16==0) {
        			SOAR_PRINT("\n");
        		}
        	}
        	SOAR_PRINT("\n");
        } else if (cm.GetTaskCommand() == FLASH_DEBUGWRITE) {

        	SOAR_PRINT("yeah debug write at 0xA0000\n");


        	//uint8_t debugdata[256]; // allocate on heap?

        	W25qxx_EraseSector(W25qxx_PageToSector((0xA0000)/256));
        	uint32_t lastCurrentLogPage = currentLogPage;
        	currentLogPage = 0xA0000 / 256;

        	SOAR_PRINT("Writing: ");

        	SOAR_PRINT("\n30 TIMES doing it...");

        	auto starttick = HAL_GetTick();

        	uint8_t incomingdatatest[40];

        	//int p = 0; // pages written
    		offsetWithinBuf = 0;
    		currbuf = logbufA;


        	for(int log = 0; log < 30; log++) {

            	for(size_t j = 0; j < 40; j++) {
            		// listen i dont know how to get rand working, just set it to something
            		//incomingdatatest[j]=((j*2)^(starttick*111^j^(starttick*111)^(starttick^j)*254^log*10^(log*starttick)*j^0b1010101010101010^(starttick>>5)^(log-starttick-j)^(j<<p)^(j > 0 ? incomingdatatest[j-1]>>1 : j))*111)&0xff; // just. yeah
            		incomingdatatest[j]=log;
            	}
            	AddLog(incomingdatatest,40);
        	}

        	auto endtick = HAL_GetTick();

        	SOAR_PRINT("completed in %d ms\n", (endtick-starttick));
        	currentLogPage = lastCurrentLogPage;
        }
        else if(cm.GetTaskCommand() == GET_LOGS_PAST_SECOND) {
        	SOAR_PRINT("%d logs in past second\n",logsInLastSecond);
        	logsInLastSecond=0;
        	break;
        }
        else 
        {
            SOAR_PRINT("FlashTask Received Unsupported Task Command: %d\n", cm.GetTaskCommand());
        }
        break;
    }
    case DATA_COMMAND: {
        // If the command is not a WRITE_DATA_TO_FLASH command do nothing
        if (cm.GetTaskCommand() != WRITE_DATA_TO_FLASH) {
            SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n", cm.GetTaskCommand());
            break;
        }

        //break; // dont log from sensors for now
        AddLog(cm.GetDataPointer(), cm.GetDataSize());
        //WriteLogDataToFlash(cm.GetDataPointer(), cm.GetDataSize()); // replace with AddLog when ready
        break;
    }


    default:
        SOAR_PRINT("FlashTask Received Unsupported Command: %d\n", cm.GetCommand());
        break;
    }

    cm.Reset();
}

/**
 * @brief writes data to flash with the size of the data written as the header, increases offset by size + 1 to account for size, currently only handles size < 255
 */
void FlashTask::WriteLogDataToFlash(uint8_t* data, uint16_t size)
{
    uint8_t buff[size + 1];

    buff[0] = (uint8_t)(size & 0xff);
    memcpy(buff + 1, data, size);

    SPIFlash::Inst().Write(SPI_FLASH_LOGGING_STORAGE_START_ADDR + currentOffsets_.writeDataOffset, buff, size + 1);
    currentOffsets_.writeDataOffset += size + 1;

//    SOAR_PRINT("log size %d\n",size);

    //TODO: Consider adding a readback to check if it was successful

    //If the number of writes since the last offset update has exceeded the threshold, update the offsets in storage
    if(++writesSinceLastOffsetUpdate_ >= FLASH_OFFSET_WRITES_UPDATE_THRESHOLD) {

        offsetsStorage_->Write(currentOffsets_);
        writesSinceLastOffsetUpdate_ = 0;
    }

}


/**
 * @brief Adds data to current buffer, plus header byte, and writes and switches buffers if full
 * WIP
 */
void FlashTask::AddLog(const uint8_t* datain, uint32_t size) {
	// want to have this called by queue msg, so that while writing pages,
	// other readings can pile up.
	// or make it async somehow

	logsInLastSecond++;
#if 0
	SOAR_PRINT("Logging size %d (",size);
	switch (size) {
	case sizeof(BarometerData):
			SOAR_PRINT("baro!");
		break;
	case sizeof(AccelGyroMagnetismData):
			SOAR_PRINT("imu!");
	break;
	case 44:
			SOAR_PRINT("GPS!");
	break;
	default:
		SOAR_PRINT("idk what this is");
		break;
	}
	SOAR_PRINT(") at page %d\n",currentLogPage);
#endif

	if(offsetWithinBuf + size + 1 >= FLASH_HEAP_BUF_SIZE) {
		// this data wont fit, write current buf and start on other buf


		//SOAR_PRINT("\nyeah, filled buf!! wow!!!\n");

		WriteLogDataToFlashPageAligned(currbuf, offsetWithinBuf, currentLogPage);

		offsetWithinBuf = 0;
		currbuf = (currbuf == logbufA) ? (logbufB) : (logbufA);
		currentLogPage++;
	}

	currbuf[offsetWithinBuf] = size;
	offsetWithinBuf++;

	memcpy(currbuf+offsetWithinBuf,datain,size);
	offsetWithinBuf+=size;

}

/**
 * @brief currently just writes. nothing fancy :) help
 */
void FlashTask::WriteLogDataToFlashPageAligned(uint8_t* data, uint16_t size,uint32_t pageAddr)
{


//    SPIFlash::Inst().Write(byteAddr, data, size);
    W25qxx_WritePage(data, pageAddr, 0, size);


}



/**
 * @brief reads all data and prints it through UART up until offset read from struct
 *        currently unimplemented
 */
bool FlashTask::ReadLogDataFromFlash()
{
    //unused
    bool res = true;

    uint8_t length;

    for (unsigned int i = 0; i < currentOffsets_.writeDataOffset + SPI_FLASH_LOGGING_STORAGE_START_ADDR; i++) {
        W25qxx_ReadByte(&length, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i);

        if (length == sizeof(AccelGyroMagnetismData)) {
            uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(AccelGyroMagnetismData));
            AccelGyroMagnetismData* IMURead = (AccelGyroMagnetismData*)dataRead;
            SOAR_PRINT("%03d %08d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\n",
                length, IMURead->time, IMURead->accelX_, IMURead->accelY_, IMURead->accelZ_,
                IMURead->gyroX_, IMURead->gyroY_, IMURead->gyroZ_, IMURead->magnetoX_,
                IMURead->magnetoY_, IMURead->magnetoZ_);
        }
        else if (length == sizeof(BarometerData)) {
            uint8_t dataRead[sizeof(BarometerData)];
            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(BarometerData));
            BarometerData* baroRead = (BarometerData*)dataRead;
            SOAR_PRINT("%3d %08d   %04d   %04d\n",
                length, baroRead->time, baroRead->pressure_, baroRead->temperature_);
        }
        else {
            SOAR_PRINT("Unknown length, readback brokedown: %d\n", length);
        }
        i = i + length;
    }
    return res;
}

//void FlashTask::benchmarkcallback(TimerHandle_t x) {
	//SOAR_PRINT("timer\n");
//}

