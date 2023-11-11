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
FlashTask::FlashTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS) {
	logbufA = (uint8_t*) soar_malloc(FLASH_HEAP_BUF_SIZE);
	logbufB = (uint8_t*) soar_malloc(FLASH_HEAP_BUF_SIZE);
	currbuf = logbufA;
	offsetWithinBuf = 0;
	SOAR_ASSERT(logbufA && logbufB, "Failed to allocate flash log bufs.\n");
	currentLogPage = SPI_FLASH_LOGGING_STORAGE_START_ADDR / 256;
	logsInLastSecond = 0;
	lastBaroData = {0};
	lastIMUData = {0};
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
void FlashTask::Run(void *pvParams) {
	// Wait until the flash has been initialized by flight task
	while (!SPIFlash::Inst().GetInitialized())
		osDelay(1);

	// Initialize the offsets storage
	offsetsStorage_ = new SimpleDualSectorStorage<Offsets>(&SPIFlash::Inst(),
			SPI_FLASH_OFFSETS_SDSS_START_ADDR);
	offsetsStorage_->Read(currentOffsets_);
	SPIFlash::Inst().Erase(SPI_FLASH_LOGGING_STORAGE_START_ADDR);

	while (1) {
		//Process any commands in the queue
		Command cm;
		bool res = qEvtQueue->Receive(cm, MAX_FLASH_TASK_WAIT_TIME_MS);
		if (res)
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
		} else if (cm.GetTaskCommand() == DUMP_FLASH_DATA) {
			ReadLogDataFromFlash();
		} else if (cm.GetTaskCommand() == ERASE_ALL_FLASH) {
			// Erase the system storage to make sure it isn't being used anymore
			SystemStorage::Inst().Erase();

			// Erase the chip
			W25qxx_EraseChip();
			currentOffsets_.writeDataOffset = 0;
		} else if (cm.GetTaskCommand() == GET_FLASH_OFFSET) {
			SOAR_PRINT("Flash offset is at 0x%04x\n%d writes since update\n",
					currentOffsets_.writeDataOffset,
					writesSinceLastOffsetUpdate_);
		} else if (cm.GetTaskCommand() == GET_PAGE_OFFSET) {
			SOAR_PRINT("Flash page offset is at %dth page\n", currentLogPage);
		} else if (cm.GetTaskCommand() == FLASH_DUMP_AT) { // WIP
			uint32_t dumptarget = *(uint32_t*) cm.GetDataPointer();
			SOAR_PRINT("we dumpin at %d\n", dumptarget);
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
			DebugReadLogs(numfirst);
		}

		else if (cm.GetTaskCommand() == FLASH_DEBUGWRITE) {

			SOAR_PRINT("yeah debug write at 0xA0000\n");

			//uint8_t debugdata[256]; // allocate on heap?

			W25qxx_EraseSector(W25qxx_PageToSector((0xA0000) / 256));
			uint32_t lastCurrentLogPage = currentLogPage;
			currentLogPage = 0xA0000 / 256;

			SOAR_PRINT("Writing: ");

			SOAR_PRINT("\nWriting 300 random AccelGyroMagnetism:");

			auto starttick = HAL_GetTick();

			uint8_t incomingdatatest[40];

			//int p = 0; // pages written
			offsetWithinBuf = 0;
			currbuf = logbufA;

			for (int log = 0; log < 300; log++) {

				for (size_t j = 0; j < 40; j++) {
					// listen i dont know how to get rand working, just set it to something
					incomingdatatest[j]=((j*2)^(starttick*111^j^(starttick*111)^(starttick^j)*254^log*10^(log*starttick)*j^0b1010101010101010^(starttick>>5)^(log-starttick-j)^(j<<(currentLogPage&0b11))^(j > 0 ? incomingdatatest[j-1]>>1 : j))*111)&0xff; // just. yeah
					//incomingdatatest[j] = log;
				}
				AddLog(incomingdatatest, 40);
			}

			auto endtick = HAL_GetTick();

			SOAR_PRINT("completed AccelGyroMag in %d ms\n", (endtick - starttick));


			SOAR_PRINT("\nWriting 300 random BarometerData:");
			starttick = HAL_GetTick();


			//int p = 0; // pages written
			offsetWithinBuf = 0;
			currbuf = logbufA;

			for (int log = 0; log < 300; log++) {

				for (size_t j = 0; j < 12; j++) {
					// listen i dont know how to get rand working, just set it to something
					incomingdatatest[j]=((j*2)^(starttick*111^j^(starttick*111)^(starttick^j)*254^log*10^(log*starttick)*j^0b1010101010101010^(starttick>>5)^(log-starttick-j)^(j<<(currentLogPage&0b11))^(j > 0 ? incomingdatatest[j-1]>>1 : j))*111)&0xff; // just. yeah
					//incomingdatatest[j] = log;
				}
				AddLog(incomingdatatest, 12);
			}

			endtick = HAL_GetTick();

			SOAR_PRINT("completed BarometerData in %d ms\n", (endtick - starttick));
			currentLogPage = lastCurrentLogPage;

		} else if (cm.GetTaskCommand() == GET_LOGS_PAST_SECOND) {
			SOAR_PRINT("%d logs in past second\n", logsInLastSecond);
			logsInLastSecond = 0;
			break;

		} else if(cm.GetTaskCommand() == TOG_BUFLOGS) {
			writebuftimemsg = !writebuftimemsg;
		}
		else {
			SOAR_PRINT("FlashTask Received Unsupported Task Command: %d\n",
					cm.GetTaskCommand());
		}
		break;
	}
	case DATA_COMMAND: {
		// If the command is not a WRITE_DATA_TO_FLASH command do nothing
		if (cm.GetTaskCommand() != WRITE_DATA_TO_FLASH) {
			SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n",
					cm.GetTaskCommand());
			break;
		}

		//break; // dont log from sensors for now
		AddLog(cm.GetDataPointer(), cm.GetDataSize());
		//WriteLogDataToFlash(cm.GetDataPointer(), cm.GetDataSize()); // replace with AddLog when ready
		break;
	}

	default:
		SOAR_PRINT("FlashTask Received Unsupported Command: %d\n",
				cm.GetCommand());
		break;
	}

	cm.Reset();
}

/**
 * @brief writes data to flash with the size of the data written as the header, increases offset by size + 1 to account for size, currently only handles size < 255
 */
void FlashTask::WriteLogDataToFlash(uint8_t *data, uint16_t size) {
	uint8_t buff[size + 1];

	buff[0] = (uint8_t) (size & 0xff);
	memcpy(buff + 1, data, size);

	SPIFlash::Inst().Write(
			SPI_FLASH_LOGGING_STORAGE_START_ADDR
					+ currentOffsets_.writeDataOffset, buff, size + 1);
	currentOffsets_.writeDataOffset += size + 1;

//    SOAR_PRINT("log size %d\n",size);

	//TODO: Consider adding a readback to check if it was successful

	//If the number of writes since the last offset update has exceeded the threshold, update the offsets in storage
	if (++writesSinceLastOffsetUpdate_
			>= FLASH_OFFSET_WRITES_UPDATE_THRESHOLD) {

		offsetsStorage_->Write(currentOffsets_);
		writesSinceLastOffsetUpdate_ = 0;
	}

}

/**
 * @brief Adds data to current buffer, plus header byte, and writes and switches buffers if full.
 * Will reject duplicate logs (same readings and timestamp)
 * When a value in two consecutive logs of the same type differs by less than 256 between the two logs,
 * only the delta will be logged as 8-bit signed char and the corresponding bit in the header will yeah.
 *
 * WIP
 */
void FlashTask::AddLog(const uint8_t *datain, uint32_t size) {

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

	uint8_t extraheaderbyte = 0;
	bool useextraheaderbyte = false;
	//auto starttime = HAL_GetTick();

	enum LogType {
		BAROMETER,
		ACCELGYROMAG,
		OTHER
	};
	LogType thisLogType;

	switch (size) {
	case sizeof(BarometerData):
		thisLogType = BAROMETER;
		break;
	case sizeof(AccelGyroMagnetismData):
		thisLogType = ACCELGYROMAG;
		break;
	default:
		thisLogType = OTHER;
		//SOAR_PRINT("Received other log type!\n");
	}

	if(thisLogType == BAROMETER or thisLogType == ACCELGYROMAG) {
		useextraheaderbyte = true;
	} else {
		useextraheaderbyte = false;
	}

 // VERY WIP!!!!!!!!!!!!

#define min(a,b) ((a)>(b) ? (b):(a))

	uint8_t maxbytesAbleToWriteInCurrbuf = FLASH_HEAP_BUF_SIZE - (offsetWithinBuf); // Num of bytes until end of buf

	uint8_t reducedLog[50];

	reducedLog[0] = size; // First byte of log is size of log

	int j = 1 + useextraheaderbyte;
	const int32_t* lastData = nullptr;
	size_t numfields = 0;
	if(thisLogType == BAROMETER) {
		lastData = (const int32_t*)(&lastBaroData.pressure_);
		numfields = sizeof(BarometerData)/sizeof(int32_t);
	} else if (thisLogType == ACCELGYROMAG) {
		lastData = (const int32_t*)(&lastIMUData.accelX_);
		numfields = sizeof(AccelGyroMagnetismData)/sizeof(int32_t);
	}


	if(useextraheaderbyte) {

		// Iterate through each int32_t member of the struct.
		// If the member differs from the matching member in the last
		// log of this type by an amount representable by only
		// 1 signed byte, mark it for delta storage to save 3 bytes reducing writes hopefully :)
		for(unsigned int i = 0; i < numfields; i++) {
			int32_t lastValOfThisElement;
			memcpy(&lastValOfThisElement,lastData+i,sizeof(int32_t));
			int32_t currentValOfThisElement;
			memcpy(&currentValOfThisElement, (const int32_t*)(datain)+i,sizeof(int32_t));
			int32_t thisdelta = currentValOfThisElement-lastValOfThisElement;
			if(i < 8 and thisdelta > -128 and thisdelta < 127) {
				extraheaderbyte |= (1 << i);
				int32_t absdelta = thisdelta > 0 ? thisdelta : -thisdelta;

				reducedLog[j] = ((int8_t)(absdelta & 0x7f)) * (thisdelta > 0 ? 1 : -1);
				j += 1;
			} else {

				memcpy(reducedLog+j,&currentValOfThisElement,sizeof(int32_t));
				j += sizeof(int32_t);

			}
		}

		//memcpy(&lastBaroData,datain,size);

		reducedLog[1] = extraheaderbyte;

	} else {

		memcpy(reducedLog+j, datain, size);
		j += size;
	}



	uint8_t bytesToWrite = min(maxbytesAbleToWriteInCurrbuf == 0 ? 255 : maxbytesAbleToWriteInCurrbuf,j); // Num of bytes that should be written in this buf

	memcpy(currbuf + offsetWithinBuf, reducedLog, bytesToWrite);

	offsetWithinBuf += bytesToWrite;
	bytesToWrite = j - bytesToWrite;

	if(offsetWithinBuf == 0) {

		uint32_t writestarttime;
		if(writebuftimemsg) {
			writestarttime = HAL_GetTick();
		}
		WriteLogDataToFlashPageAligned(currbuf,	offsetWithinBuf > 0 ? offsetWithinBuf : 256, currentLogPage);
		if(writebuftimemsg) {
			auto writeendtime = HAL_GetTick();

			SOAR_PRINT("Write buffer in %dms\n", writeendtime-writestarttime);
		}
		currbuf = (currbuf == logbufA) ? (logbufB) : (logbufA);
		currentLogPage++;
	}



	if(bytesToWrite > 0) { // still stuff to write

		memcpy(currbuf + offsetWithinBuf, reducedLog + (j-bytesToWrite), bytesToWrite);
		offsetWithinBuf += bytesToWrite;
	}


	if (thisLogType == BAROMETER and (!extraheaderbyte or true)) {
		memcpy(&lastBaroData,datain,size); // WIP

	} else if (thisLogType == ACCELGYROMAG and (!extraheaderbyte or true)) {
		memcpy(&lastIMUData,datain,size); // WIP

	}

	//auto endtime = HAL_GetTick();
	//SOAR_PRINT("Added log in %dms\n", endtime-starttime);

}

/**
 * @brief currently just writes. nothing fancy :) help
 */
void FlashTask::WriteLogDataToFlashPageAligned(uint8_t *data, uint16_t size,
		uint32_t pageAddr) {

//    SPIFlash::Inst().Write(byteAddr, data, size);
	//W25qxx_WritePage(data, pageAddr, 0, size);
	W25qxx_WritePageScary(data, pageAddr);
}

bool FlashTask::DebugReadLogs(uint32_t numOfLogs) {

	uint8_t logbuf[128];
	enum LogType {
		BAROMETER,
		ACCELGYROMAG,
		OTHER
	};

	uint32_t readOffsetBytes = SPI_FLASH_LOGGING_STORAGE_START_ADDR;
	//W25qxx_ReadPage(pagebuf, readOffsetBytes/256, 0, 256);
	BarometerData lastBaroRead = {0};
	AccelGyroMagnetismData lastIMURead = {0};
	const int32_t* lastRead = nullptr;

	do {

		uint8_t thisLogSize = 0;
		W25qxx_ReadByte(&thisLogSize, readOffsetBytes);
		SOAR_PRINT("yeah! %d\n", thisLogSize);
		readOffsetBytes++;
		LogType thistype = OTHER;


		switch(thisLogSize) {
		case sizeof(BarometerData):
			thistype = BAROMETER;
			lastRead = (const int32_t*)(&lastBaroRead);
			break;
		case sizeof(AccelGyroMagnetismData):
			thistype = ACCELGYROMAG;
			lastRead = (const int32_t*)(&lastIMURead);
			break;
		default:
			thistype=OTHER;
			return false;
			break;
		}
		size_t thisDataSize = thisLogSize / sizeof(int32_t);
		if(thistype == BAROMETER or thistype == ACCELGYROMAG) {
			thisLogSize++;
		}

		W25qxx_ReadBytes(logbuf, readOffsetBytes, thisLogSize);
		//readOffsetBytes += thisLogSize;



		int32_t* startData = nullptr;
		uint8_t extraheaderbyte = logbuf[0];
		BarometerData thisBaroRead = {0};
		AccelGyroMagnetismData thisIMURead = {0};
		switch(thistype) {
		case BAROMETER:
			startData = (int32_t*)&thisBaroRead;
			break;
		case ACCELGYROMAG:
			startData = (int32_t*)&thisIMURead;
			break;
		default:
			break;
		}



			int i = 1;
			for(unsigned int headerbit = 0; headerbit < thisDataSize; headerbit++) {

				if(extraheaderbyte & (1<<headerbit)) {
					int32_t lastFieldValue;
					memcpy(&lastFieldValue,lastRead+headerbit,sizeof(int32_t));
					int32_t currentFieldValue = (int8_t)logbuf[i]+lastFieldValue;
					memcpy(startData+headerbit,&currentFieldValue,sizeof(int32_t));
					i++;
				} else {
					memcpy(startData+headerbit,logbuf+i,sizeof(int32_t));
					i += sizeof(int32_t);
				}
			}


			switch(thistype) {
			case BAROMETER:

			SOAR_PRINT("ExtraHeader: %x\nPressure: %d\nTemp: %d\nTime: %d\n\n",
					extraheaderbyte,thisBaroRead.pressure_, thisBaroRead.temperature_,
					thisBaroRead.time);
			readOffsetBytes += i;
			lastBaroRead = thisBaroRead;
			break;
			case ACCELGYROMAG:

			SOAR_PRINT(
					"ExtraHeader: %x\nAccel: %d %d %d\nGyro: %d %d %d\nMagnet: %d %d %d\nTime: %d\n\n",
					extraheaderbyte, thisIMURead.accelX_, thisIMURead.accelY_, thisIMURead.accelZ_,
					thisIMURead.gyroX_, thisIMURead.gyroY_, thisIMURead.gyroZ_,
					thisIMURead.magnetoX_, thisIMURead.magnetoY_, thisIMURead.magnetoZ_,
					thisIMURead.time);
			readOffsetBytes += i;
			lastIMURead = thisIMURead;

			break;
			default:

			SOAR_PRINT("unknown!! uh oh!\n");


			}

		numOfLogs--;

	} while (numOfLogs > 0);

}

/**
 * @brief reads all data and prints it through UART up until offset read from struct
 *        currently unimplemented
 */
bool FlashTask::ReadLogDataFromFlash() {
	//unused
	bool res = true;

	uint8_t length;

	for (unsigned int i = 0; i < currentOffsets_.writeDataOffset + SPI_FLASH_LOGGING_STORAGE_START_ADDR; i++) {
		W25qxx_ReadByte(&length, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i);

		if (length == sizeof(AccelGyroMagnetismData)) {
			uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
			W25qxx_ReadBytes(dataRead,
					SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1,
					sizeof(AccelGyroMagnetismData));
			AccelGyroMagnetismData *IMURead = (AccelGyroMagnetismData*) dataRead;
			SOAR_PRINT(
					"%03d %08d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\n",
					length, IMURead->time, IMURead->accelX_, IMURead->accelY_,
					IMURead->accelZ_, IMURead->gyroX_, IMURead->gyroY_,
					IMURead->gyroZ_, IMURead->magnetoX_, IMURead->magnetoY_,
					IMURead->magnetoZ_);
		} else if (length == sizeof(BarometerData)) {
			uint8_t dataRead[sizeof(BarometerData)];
			W25qxx_ReadBytes(dataRead,
					SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1,
					sizeof(BarometerData));
			BarometerData *baroRead = (BarometerData*) dataRead;
			SOAR_PRINT("%3d %08d   %04d   %04d\n", length, baroRead->time,
					baroRead->pressure_, baroRead->temperature_);
		} else {
			SOAR_PRINT("Unknown length, readback brokedown: %d\n", length);
		}
		i = i + length;
	}
	return res;
}

//void FlashTask::benchmarkcallback(TimerHandle_t x) {
//SOAR_PRINT("timer\n");
//}

