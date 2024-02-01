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
	lastPTC = {0};
	currentPageStorageByte = 0;
	currentPageStoragePage = 0;
	currentPageStorageSector = 0;
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
	//offsetsStorage_ = new SimpleDualSectorStorage<Offsets>(&SPIFlash::Inst(), SPI_FLASH_OFFSETS_SDSS_START_ADDR);



	//erase first sectors for testing
	for(auto i = 0; i < 256; i++) {

		SPIFlash::Inst().Erase(SPI_FLASH_LOGGING_STORAGE_START_ADDR+i*SPIFlash::Inst().GetSectorSize());

	}

	{ // Recover last written sensor page
		uint32_t highestPage = 0;
		for(uint32_t sec = 0; sec < 2; sec++) {
			for(uint32_t page = 0; page < 16; page++) {
				for(uint32_t byte = 0; byte < 256-sizeof(currentLogPage)*2; byte+=sizeof(currentLogPage)*2) {
					uint32_t pageread1;
					uint32_t pageread2;
					SPIFlash::Inst().Read(SPI_FLASH_PAGE_OFFSET_STORAGE + sec*w25qxx.SectorSize + page*w25qxx.PageSize + byte, (uint8_t*)(&pageread1), sizeof(pageread1));
					SPIFlash::Inst().Read(SPI_FLASH_PAGE_OFFSET_STORAGE + sec*w25qxx.SectorSize + page*w25qxx.PageSize + byte + sizeof(currentLogPage), (uint8_t*)(&pageread2), sizeof(pageread2));
					if(pageread1 == 0xffffffff or pageread2 == 0xffffffff) {
						continue;
					}
					if(pageread1 == pageread2) {
						if(pageread1 > highestPage) {
							highestPage = pageread1;
						}
					} else {
						SOAR_PRINT("Encountered corrupt lastpage data\n");
					}
				}
			}
		}

		SOAR_PRINT("The last written page was %d\n",highestPage); // currently does not continue from this page

		//currentLogPage = highestPage; // uncomment this to actually continue from old page

		W25qxx_EraseSector(SPI_FLASH_PAGE_OFFSET_STORAGE/w25qxx.SectorSize);
		W25qxx_EraseSector(SPI_FLASH_PAGE_OFFSET_STORAGE/w25qxx.SectorSize+1);
	}

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
			DumpFirstNLogs(numfirst);
		}

		else if (cm.GetTaskCommand() == FLASH_DEBUGWRITE) { // probably remove

			SOAR_PRINT("removed\n");

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


		AddLog(cm.GetDataPointer(), cm.GetDataSize());
		//WriteLogDataToFlash(cm.GetDataPointer(), cm.GetDataSize());
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
	case 8:
		SOAR_PRINT("PTC!");
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


	FLASH_LOG_TYPE thisLogType;

	switch (size) {
	case sizeof(BarometerData):
		thisLogType = LTYPE_BAROMETER;
		break;
	case sizeof(AccelGyroMagnetismData):
		thisLogType = LTYPE_ACCELGYROMAG;
		break;
	case sizeof(PressureTransducerFlashLogData):
		thisLogType = LTYPE_PTC;
		break;
	case sizeof(GPSDataFlashLog):
		thisLogType = LTYPE_GPS;
		break;
	case sizeof(PBBPressureFlashLogData):
		thisLogType = LTYPE_PBB_PRES;
		break;
	default:
		thisLogType = LTYPE_OTHER;
		//SOAR_PRINT("Received other log type!\n");
	}

#define usesExtraHeader(type) ((type) == LTYPE_BAROMETER or (type) == LTYPE_ACCELGYROMAG or (type) == LTYPE_PTC or (type) == LTYPE_PBB_PRES)

	useextraheaderbyte = usesExtraHeader(thisLogType);



#define min(a,b) ((a)>(b) ? (b):(a))

	uint8_t maxbytesAbleToWriteInCurrbuf = FLASH_HEAP_BUF_SIZE - (offsetWithinBuf); // Num of bytes until end of buf

	uint8_t reducedLog[50];

	reducedLog[0] = size; // First byte of log is size of log

	int j = 1 + useextraheaderbyte;
	const int32_t* lastData = nullptr;
	size_t numfields = size/sizeof(int32_t);
	if(thisLogType == LTYPE_BAROMETER) {
		lastData = (const int32_t*)(&lastBaroData.pressure_);
	} else if (thisLogType == LTYPE_ACCELGYROMAG) {
		lastData = (const int32_t*)(&lastIMUData.accelX_);
	} else if (thisLogType == LTYPE_PTC) {
		lastData = (const int32_t*)(&lastPTC.pressure);
	} else if (thisLogType == LTYPE_PBB_PRES){
		lastData = (const int32_t*)(&lastPBBPres);
	} else if (thisLogType == LTYPE_GPS) {
		numfields = 8; // some fields of GPS are units
	}


	if(useextraheaderbyte) {

		// Iterate through each int32_t member of the struct.
		// If the member differs from the matching member in the last
		// log of this type by an amount representable by only
		// 1 signed byte, mark it for delta storage to save 3 bytes reducing writes hopefully :)
		size_t exactsFound = 0;
		for(unsigned int i = 0; i < numfields; i++) {
			int32_t lastValOfThisElement;
			memcpy(&lastValOfThisElement,lastData+i,sizeof(int32_t));
			int32_t currentValOfThisElement;
			memcpy(&currentValOfThisElement, (const int32_t*)(datain)+i,sizeof(int32_t));
			int32_t thisdelta = currentValOfThisElement-lastValOfThisElement;
			if(i < 8 and thisdelta > -128 and thisdelta < 127) {
				extraheaderbyte |= (1 << i);
				int32_t absdelta = thisdelta > 0 ? thisdelta : -thisdelta;
				exactsFound += !absdelta;

				reducedLog[j] = ((int8_t)(absdelta & 0x7f)) * (thisdelta > 0 ? 1 : -1);
				j += 1;
			} else {

				memcpy(reducedLog+j,&currentValOfThisElement,sizeof(int32_t));
				j += sizeof(int32_t);

			}
		}

		// reject exact dupes at same timestamps
		if(exactsFound == numfields) {
			return;
		}

		reducedLog[1] = extraheaderbyte;

	} else {

		memcpy(reducedLog+j, datain, size);
		j += size;
	}



	uint8_t bytesToWrite = min(maxbytesAbleToWriteInCurrbuf == 0 ? 255 : maxbytesAbleToWriteInCurrbuf,j); // Num of bytes that should be written in this buf

	memcpy(currbuf + offsetWithinBuf, reducedLog, bytesToWrite);

	offsetWithinBuf += bytesToWrite;
	bytesToWrite = j - bytesToWrite;

	if(offsetWithinBuf == 0) { // just filled the buf

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

		// Logs two currentLogPages immediately after each other for offset recovery,
		// the second one acts as a checksum to the first
		uint8_t offsetStorageData[sizeof(currentLogPage)*2];
		memcpy(offsetStorageData,(const uint8_t*)&currentLogPage,sizeof(currentLogPage));
		memcpy(offsetStorageData+sizeof(currentLogPage),(const uint8_t*)&currentLogPage,sizeof(currentLogPage));

		SPIFlash::Inst().Write(SPI_FLASH_PAGE_OFFSET_STORAGE+currentPageStorageSector*w25qxx.SectorSize+currentPageStoragePage*w25qxx.PageSize + currentPageStorageByte,
				&(offsetStorageData[0]), sizeof(offsetStorageData));

		currentPageStorageByte+=sizeof(offsetStorageData);

		if(currentPageStorageByte >= 256-sizeof(offsetStorageData)) {
			currentPageStorageByte = 0;
			currentPageStoragePage++;
		}

		if(currentPageStoragePage >= 16) {
			currentPageStorageSector = !currentPageStorageSector;
			currentPageStoragePage = 0;
			SOAR_PRINT("Switching recovery sector...\n");
			// erase the new sector to prepare
			W25qxx_EraseSector(SPI_FLASH_PAGE_OFFSET_STORAGE/w25qxx.SectorSize+currentPageStorageSector);

		}

	}



	if(bytesToWrite > 0) { // still stuff to write

		memcpy(currbuf + offsetWithinBuf, reducedLog + (j-bytesToWrite), bytesToWrite);
		offsetWithinBuf += bytesToWrite;
	}


	if (thisLogType == LTYPE_BAROMETER) {
		memcpy(&lastBaroData,datain,size);

	} else if (thisLogType == LTYPE_ACCELGYROMAG) {
		memcpy(&lastIMUData,datain,size);

	} else if(thisLogType == LTYPE_PTC) {
		memcpy(&lastPTC,datain,size);
	} else if (thisLogType == LTYPE_PBB_PRES) {
		memcpy(&lastPBBPres,datain,size);
	}

	//auto endtime = HAL_GetTick();
	//SOAR_PRINT("Added log in %dms\n", endtime-starttime);

}

/**
 * @brief Writes data to the beginning of a specified page
 */
void FlashTask::WriteLogDataToFlashPageAligned(uint8_t *data, uint16_t size,
		uint32_t pageAddr) {

	W25qxx_WritePage(data, pageAddr, 0, size);
	//W25qxx_WritePageScary(data, pageAddr);
}

/**
 * @brief Prints through UART the first N logs in flash
 *
 */

bool FlashTask::DumpFirstNLogs(uint32_t numOfLogs) {

	uint8_t logbuf[128];


	uint32_t readOffsetBytes = SPI_FLASH_LOGGING_STORAGE_START_ADDR;
	BarometerData lastBaroRead = {0};
	AccelGyroMagnetismData lastIMURead = {0};
	PressureTransducerFlashLogData lastPTCRead = {0};
	PBBPressureFlashLogData lastPBBPresRead = {0};
	const int32_t* lastRead = nullptr;

	SOAR_PRINT("<p>\n"); // used for python testing

	do {

		uint8_t thisLogSize = 0;
		W25qxx_ReadByte(&thisLogSize, readOffsetBytes);
		SOAR_PRINT("Log Size %d\n", thisLogSize);
		readOffsetBytes++;
		FLASH_LOG_TYPE thistype = LTYPE_OTHER;


		switch(thisLogSize) {
		case sizeof(BarometerData):
			thistype = LTYPE_BAROMETER;
			lastRead = (const int32_t*)(&lastBaroRead);
			break;
		case sizeof(AccelGyroMagnetismData):
			thistype = LTYPE_ACCELGYROMAG;
			lastRead = (const int32_t*)(&lastIMURead);
			break;
		case sizeof(PressureTransducerFlashLogData):
			thistype = LTYPE_PTC;
			lastRead = (const int32_t*)(&lastPTCRead);
			break;
		case sizeof(PBBPressureFlashLogData):
			thistype = LTYPE_PBB_PRES;
			lastRead = (const int32_t*)(&lastPBBPresRead);
			break; // todo finish implementing id instead of size byte
		case sizeof(GPSDataFlashLog):
			thistype = LTYPE_GPS;
			// no delta storage
			break;
		default:
			thistype=LTYPE_OTHER;
			SOAR_PRINT("Encountered unknown log type %d while dumping! <\\p>\n",thisLogSize);
			return false;

		}

		size_t thisNumFields = thisLogSize / sizeof(int32_t);
		bool extraHeaderExists = usesExtraHeader(thistype);
		if(extraHeaderExists) {
			thisLogSize++;
		}

		W25qxx_ReadBytes(logbuf, readOffsetBytes, thisLogSize);

		int32_t* startData = nullptr;
		uint8_t extraheaderbyte = logbuf[0];
		union {
			BarometerData thisBaroRead;
			AccelGyroMagnetismData thisIMURead;
			PressureTransducerFlashLogData thisPTCRead;
			GPSDataFlashLog thisGPSRead;
			PBBPressureFlashLogData thisPBBPresRead;

		} thisRead;


		switch(thistype) {
		case LTYPE_BAROMETER:
			startData = (int32_t*)&(thisRead.thisBaroRead);
			break;
		case LTYPE_ACCELGYROMAG:
			startData = (int32_t*)&(thisRead.thisIMURead);
			break;
		case LTYPE_PTC:
			startData = (int32_t*)&(thisRead.thisPTCRead);
			break;
		case LTYPE_GPS:
			startData = (int32_t*)&(thisRead.thisGPSRead);
			break;
		case LTYPE_PBB_PRES:
			startData = (int32_t*)&(thisRead.thisPBBPresRead);
			break;
		default:
			break;
		}

		int i = 0;
		if(extraHeaderExists) {
			i=1;
			for(unsigned int field = 0; field < thisNumFields; field++) {

				if(extraheaderbyte & (1<<field)) {
					int32_t lastFieldValue;
					memcpy(&lastFieldValue,lastRead+field,sizeof(int32_t));
					int32_t currentFieldValue = (int8_t)logbuf[i]+lastFieldValue;
					memcpy(startData+field,&currentFieldValue,sizeof(int32_t));
					i++;
				} else {
					memcpy(startData+field,logbuf+i,sizeof(int32_t));
					i += sizeof(int32_t);
				}
			}
		} else {
			memcpy(startData,logbuf,thisLogSize);
			i=thisLogSize;
		}


		switch(thistype) {
		case LTYPE_BAROMETER:

			SOAR_PRINT("ExtraHeader: %x\nPressure: %d\nTemp: %d\nTime: %d\n\n",
					extraheaderbyte,thisRead.thisBaroRead.pressure_, thisRead.thisBaroRead.temperature_,
					thisRead.thisBaroRead.time);
			lastBaroRead = thisRead.thisBaroRead;
			break;

		case LTYPE_ACCELGYROMAG:

			SOAR_PRINT(
					"ExtraHeader: %x\nAccel: %d %d %d\nGyro: %d %d %d\nMagnet: %d %d %d\nTime: %d\n\n",
					extraheaderbyte, thisRead.thisIMURead.accelX_, thisRead.thisIMURead.accelY_, thisRead.thisIMURead.accelZ_,
					thisRead.thisIMURead.gyroX_, thisRead.thisIMURead.gyroY_, thisRead.thisIMURead.gyroZ_,
					thisRead.thisIMURead.magnetoX_, thisRead.thisIMURead.magnetoY_, thisRead.thisIMURead.magnetoZ_,
					thisRead.thisIMURead.time);
			lastIMURead = thisRead.thisIMURead;
			break;

		case LTYPE_PTC:

			SOAR_PRINT("ExtraHeader: %x\nPTC Pressure: %d, Time: %d\n\n",extraheaderbyte,thisRead.thisPTCRead.pressure,thisRead.thisPTCRead.time);
			lastPTCRead = thisRead.thisPTCRead;
			break;

		case LTYPE_GPS:
			//SOAR_PRINT("GPS: yeah\n");

			SOAR_PRINT("Time: %d\nLat: %d deg, %d mins\nLong: %d deg, %d mins\nAntenna Alt: %d %c\nGeoid Alt: %d %c\nTotal Alt: %d %c\n\n",
					thisRead.thisGPSRead.time_, thisRead.thisGPSRead.latitude_.degrees_, thisRead.thisGPSRead.latitude_.minutes_,
					thisRead.thisGPSRead.longitude_.degrees_, thisRead.thisGPSRead.longitude_.minutes_,
					thisRead.thisGPSRead.antennaAltitude_.altitude_, thisRead.thisGPSRead.antennaAltitude_.unit_,
					thisRead.thisGPSRead.geoidAltitude_.altitude_,thisRead.thisGPSRead.geoidAltitude_.unit_,
					thisRead.thisGPSRead.totalAltitude_.altitude_,thisRead.thisGPSRead.totalAltitude_.unit_);

			break;

		case LTYPE_PBB_PRES:


			SOAR_PRINT("PBB PRES!!! Pressure: %d\n, Time: %d\n\nPressure: %d\n, Time: %d\n\nPressure: %d\n, Time: %d\n\nPressure: %d\n, Time: %d\n\n",
					thisRead.thisPBBPresRead.a.pressure, thisRead.thisPBBPresRead.a.time,
					thisRead.thisPBBPresRead.b.pressure, thisRead.thisPBBPresRead.b.time,
					thisRead.thisPBBPresRead.c.pressure, thisRead.thisPBBPresRead.c.time,
					thisRead.thisPBBPresRead.d.pressure, thisRead.thisPBBPresRead.d.time);
			break;




		default:

			SOAR_PRINT("unknown!! uh oh!\n");


		}
		readOffsetBytes += i;

		numOfLogs--;

	} while (numOfLogs > 0);

	SOAR_PRINT("<\\p>\n");

	return true;
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
