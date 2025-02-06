/**
 ********************************************************************************
 * @file    LFSTask.cpp
 * @author  spiro
 * @date    Jan 25, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "LFSTask.hpp"
#include "FlashTask.hpp"
#include <W25Qxx.hpp>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RunInterface.hpp"
#include "SystemDefines.hpp"
#include <cstdio>
#include <cstring>
#include "lfs.h"
#include "lfs_util.h"
#include "W25Qxx.h"



LFSTask::LFSTask() : Task(FLASH_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the LFSTask
 */
void LFSTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flash task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LFSTask::RunTask,
            (const char*)"LFSTask",
            (uint16_t)FLASH_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)FLASH_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "LFSTask::InitTask() - xTaskCreate() failed");

    SOAR_PRINT("LFS Task initialized");
}

/**
 * @brief Instance Run loop for the Flash Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void LFSTask::Run(void * pvParams)
{
    // Wait until the flash has been initialized by flight task


    while (1) {
        //Process any commands in the queue
        Command cm;
        bool res = qEvtQueue->Receive(cm, MAX_FLASH_TASK_WAIT_TIME_MS);
        if(res){
            HandleCommand(cm);
        	cm.Reset();
        }

        //Run maintenance on dual sector storages
//        SystemStorage::Inst().Maintain();
//        offsetsStorage_->Maintain();
    }
}

/**
 * @brief Handles current command
 * @param cm The command to handle
 */
void LFSTask::HandleCommand(Command& cm)
{



	const char* fn_templ1 = "F%u.tst";
	const char* fn_templ2 = "R%u.tst";
	char fn[32], fn2[32];
	lfs_file_t fp;


//	 HAL_TIM_Base_Start(&htim2);										// Not used

	  SOAR_PRINT("\n\nlittlefs version %x\n",LFS_VERSION);

	  W25Q_Reset();

	  SOAR_PRINT("Flash Identifier = 0x%08lx\n",W25Q_ReadID());

	  W25Q_ReadUniqueID();

	  SOAR_PRINT("StatusReg1=%02x\n",W25Q_ReadStatus(1));
	  SOAR_PRINT("StatusReg2=%02x\n",W25Q_ReadStatus(2));
	  SOAR_PRINT("StatusReg3=%02x\n",W25Q_ReadStatus(3));

	  SOAR_PRINT("\nRead SFDP Table:\n");
	  uint8_t sfdp[256]={0};
	  W25Q_ReadSFDP(sfdp);
	  SOAR_PRINT("%c %c %c %c ",sfdp[0],sfdp[1],sfdp[2],sfdp[3]);
	  for (int i=4;i<256;i++) SOAR_PRINT("%02x ",sfdp[i]);
	  SOAR_PRINT("\n\n");


	  // test file system
	  SOAR_PRINT("\n\n ********************* Mount lfs ***********************\n\n");
	  stmlfs_mount(true);

	  SOAR_PRINT("mounted\n");

	  //---------------------------------------------------------------------------------------------
	  // We'll create 32 files, verify them, rename them, reverify, and delete them.
	  //---------------------------------------------------------------------------------------------

	  for (int i = 0; i < 32; i++) {

		  sprintf(fn, fn_templ1,i);                                  	// Create file name string

	      int err = stmlfs_file_open(&fp, fn, LFS_O_WRONLY | LFS_O_CREAT);// Create the fp
	      if (err < 0) {
	          SOAR_PRINT("open failed\n");
	          fflush(stdout);
	          Error_Handler();
	      }

	      SOAR_PRINT("Write to File %s\n",fn);
	      if ((strlen(fn) + 1) != (uint32_t)stmlfs_file_write(&fp, fn, strlen(fn) + 1)) {// Write the file name to the file
	          SOAR_PRINT("write fails\n");
	          fflush(stdout);
	          Error_Handler();
	      }

	      if (stmlfs_file_close(&fp)<0){                          		// flush and close the file
	          SOAR_PRINT("closed failed\n");
	          fflush(stdout);
	          Error_Handler();
	      }
	  }

	  dump_dir();														// Show directory

	  //stmlfs_unmount();                                               	// Unmount & remount
	  //stmlfs_mount(false);

	  struct littlfs_fsstat_t stat;                                   	// Display file system sizes
	  stmlfs_fsstat(&stat);
	  SOAR_PRINT("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);

	  for (int i = 0; i < 32; i++) {
	  	  sprintf(fn, fn_templ1, i);
	      sprintf(fn2, fn_templ2, i);

	      SOAR_PRINT("Rename from %s to %s\n",fn,fn2);
	      if (stmlfs_rename(fn, fn2) < 0) {                           	// rename
	          SOAR_PRINT("rename failed\n");
	          fflush(stdout);
	          Error_Handler();
	      }
	  }
	  dump_dir();														// Show directory

	  stmlfs_fsstat(&stat);                                           	// Display file system sizes
	  SOAR_PRINT("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);

	  char buf[32];
	  for (int i = 0; i < 32; i++) {

	      sprintf(fn, fn_templ1, i);
	      sprintf(fn2, fn_templ2, i);

	      SOAR_PRINT("Reopen Filename=%s\n",fn2);
	      int err = stmlfs_file_open(&fp, fn2, LFS_O_RDONLY);       	// verify the file's content
	      if (err < 0) {
	          SOAR_PRINT("lfs open failed\n");
	          fflush(stdout);
	          Error_Handler();
	      } else {
	          stmlfs_file_read(&fp, buf, sizeof(buf));
	          if (strcmp(fn, buf) != 0) {
	              SOAR_PRINT("lfs read failed\n");
	              fflush(stdout);
	              Error_Handler();
	          }
	          stmlfs_file_close(&fp);

	          if (stmlfs_remove(fn2) < 0) {                             // Delete the file
	              SOAR_PRINT("remove failed\n");
	              fflush(stdout);
	              Error_Handler();
	          } else SOAR_PRINT("File %s removed\n",fn2);
	      }
	  }
	  dump_dir();

	  stmlfs_fsstat(&stat);                                         	// Display file system sizes
	  SOAR_PRINT("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);

	  stmlfs_unmount();                                             	// Release any resources we were using
	  fflush(stdout);
	  SOAR_PRINT("lfs test done\n");


//	  PUTCHAR_PROTOTYPE
//	  {
//	    /* Place your implementation of fputc here */
//	    /* e.g. write a character to the USART1 and Loop until the end of transmission */
//	    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
//
//	    return ch;
//	  }
//


}
//
///**
// * @brief writes data to flash with the size of the data written as the header, increases offset by size + 1 to account for size, currently only handles size < 255
// */
//void LFSTask::WriteLogDataToFlash(uint8_t* data, uint16_t size)
//{
//    uint8_t buff[size + 1];
//
//    buff[0] = (uint8_t)(size & 0xff);
//    memcpy(buff + 1, data, size);
//
//    SPIFlash::Inst().Write(SPI_FLASH_LOGGING_STORAGE_START_ADDR + currentOffsets_.writeDataOffset, buff, size + 1);
//    currentOffsets_.writeDataOffset += size + 1;
//
//    //TODO: Consider adding a readback to check if it was successful
//
//    //If the number of writes since the last offset update has exceeded the threshold, update the offsets in storage
//    if(++writesSinceLastOffsetUpdate_ >= FLASH_OFFSET_WRITES_UPDATE_THRESHOLD)
//        offsetsStorage_->Write(currentOffsets_);
//}
//
///**
// * @brief reads all data and prints it through UART up until offset read from struct
// *        currently unimplemented
// */
//bool LFSTask::ReadLogDataFromFlash()
//{
//    //unused
//    bool res = true;
//
//    uint8_t length;
//
//    for (unsigned int i = 0; i < currentOffsets_.writeDataOffset + SPI_FLASH_LOGGING_STORAGE_START_ADDR; i++) {
//        W25qxx_ReadByte(&length, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i);
//
//        if (length == sizeof(AccelGyroMagnetismData)) {
//            uint8_t dataRead[sizeof(AccelGyroMagnetismData)];
//            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(AccelGyroMagnetismData));
//            AccelGyroMagnetismData* IMURead = (AccelGyroMagnetismData*)dataRead;
//            SOAR_PRINT("%03d %08d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d   %04d\n",
//                length, IMURead->time, IMURead->accelX_, IMURead->accelY_, IMURead->accelZ_,
//                IMURead->gyroX_, IMURead->gyroY_, IMURead->gyroZ_, IMURead->magnetoX_,
//                IMURead->magnetoY_, IMURead->magnetoZ_);
//        }
//        else if (length == sizeof(BarometerData)) {
//            uint8_t dataRead[sizeof(BarometerData)];
//            W25qxx_ReadBytes(dataRead, SPI_FLASH_LOGGING_STORAGE_START_ADDR + i + 1, sizeof(BarometerData));
//            BarometerData* baroRead = (BarometerData*)dataRead;
//            SOAR_PRINT("%3d %08d   %04d   %04d\n",
//                length, baroRead->time, baroRead->pressure_, baroRead->temperature_);
//        }
//        else {
//            SOAR_PRINT("Unknown length, readback brokedown: %d\n", length);
//        }
//        i = i + length;
//    }
//    return res;
//}
//
