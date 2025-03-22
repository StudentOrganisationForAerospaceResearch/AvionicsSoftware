/**
 ********************************************************************************
 * @file    ${file_name}
 * @author  ${user}
 * @date    ${date}
 * @brief   This is a template source file to create a new task in our firmware
 *
 * Setup Steps
 * 1. Define the Task Queue Depth in SystemDefines.hpp
 * 2. Define the Task Stack Depth in SystemDefines.hpp
 * 3. Define the Task Priority in SystemDefines.hpp
 * 4. Replace all placeholders marked with a $ sign
 ********************************************************************************
 */
/************************************
 * INCLUDES
 ************************************/
#include "LFSTask.hpp"
#include "SystemDefines.hpp"
#include <cstring>
#include "lfs.h"
#include "lfs_util.h"
#include "W25Qxx.h"
#include <W25Qxx.hpp>
#include "FlashTask.hpp"
#include "cmsis_os.h"
#include "lfsWrapper.hpp"
/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
/************************************
 * VARIABLES
 ************************************/
/************************************
 * FUNCTION DECLARATIONS
 ************************************/
/************************************
 * FUNCTION DEFINITIONS
 ************************************/
/**
 * @brief Constructor for LFSTaskTEST123
 */
LFSTask::LFSTask() : Task(LFS_TASK_QUEUE_DEPTH_OBJS)
{
}
/**
 * @brief Initialize the LFSTaskTEST123
 *        Do not modify this function aside from adding the task name
 */
void LFSTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LFSTask::RunTask,
            (const char*)"LFSTask",
            (uint16_t)LFS_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)LFS_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);
                SOAR_ASSERT(rtValue == pdPASS, "LFSTask::InitTask() - xTaskCreate() failed");
}
/**
 * @brief Instance Run loop for the Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void LFSTask::Run(void * pvParams)
{
    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res) {
            HandleCommand(cm);
        }
    }
}
/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void LFSTask::HandleCommand(Command& cm)
{
    switch (cm.GetCommand()) {
			case TASK_SPECIFIC_COMMAND: {
				if (cm.GetTaskCommand() == WRITE_DATA_TO_LFS) {
						LFSTest();
						SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n", cm.GetTaskCommand());
						break;
				}
				if (cm.GetTaskCommand() == WRITE_TEST_DATA) {
						WriteTest();
						SOAR_PRINT("FlashTask Received Unsupported Data Command: %d\n", cm.GetTaskCommand());
						break;
				}
			}
			default:
				SOAR_PRINT("LFSTask - Received Unsupported Command {%d}\n", cm.GetCommand());
				break;
    }
    //No matter what we happens, we must reset allocated data
    cm.Reset();
}
void LFSTask::LFSTest() {
	const char* fn_templ1 = "F%u.tst";
		const char* fn_templ2 = "R%u.tst";
		char fn[32], fn2[32];
		lfs_file_t fp;
	//	 HAL_TIM_Base_Start(&htim2);										// Not used
		  SOAR_PRINT("\n\nlittlefs version %x\n",LFS_VERSION);
		  W25Q_Reset();

		  /*
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
		   *
		   */
		  SOAR_PRINT("\n\n ********************* Mount lfs ***********************\n\n");
		  if (stmlfs_mount(true))
			  SOAR_PRINT("Failed to mount\n");

		  SOAR_PRINT("Mounted\n");



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
		  SOAR_PRINT("lfs test done\n");
		  fflush(stdout);

}

void LFSTask::WriteTest(void){

<<<<<<< HEAD
	LFS fs = LFS::getLFS();

	char buff[32];
	fs.writeToFile("spirofile", buff, 32);

	// read data back from file
	fs.readFromFile("spirofile", buff, 32);

	char pbuff[6];

	// partially read data back from file
	fs.readFromFile("spirofile", pbuff, 5);
	pbuff[5] = 0;



=======
	Lfs fs;
	char sample[] = "sample text\n";
	SOAR_PRINT("Writing sample data: %s\n", sample);


	char buff[32];
	fs.writeToFile("spirofile", sample, 12);
	fs.readFromFile("spirofile", buff, 32);

	SOAR_PRINT("Printing buffer data: %s\n", buff);


	char pbuff[7];

	fs.readFromFile("spirofile", pbuff, 6);
	pbuff[6] = 0;


	SOAR_PRINT("Printing partial buffer data: %s\n", pbuff);


	SOAR_PRINT("write test done\n");
>>>>>>> 41cec5c31b288429e6a951e7290681af3379e47a

}
