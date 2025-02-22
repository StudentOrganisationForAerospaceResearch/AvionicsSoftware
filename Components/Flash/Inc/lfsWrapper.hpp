/** ********************************************************************************
 * * @file    lfsWrapper.hpp
 * * @author  root
 * * @date    Feb 22, 2025
 * * @brief ******************************************************************************** */
#ifndef FLASH_INC_LFSWRAPPER_HPP_
#define FLASH_INC_LFSWRAPPER_HPP_
/************************************ * INCLUDES ************************************/
#include "lfs.h"
#include "lfs_util.h"
#include "W25Qxx.h"
/************************************ * MACROS AND DEFINES ************************************/
#define MAX_FILE_SIZE_BYTES 256
/************************************ * TYPEDEFS ************************************/
/************************************ * CLASS DEFINITIONS ************************************/
class Lfs {
public:
	enum LFS_ERROR {
		LFS_OK = 0,
		LFS_MOUNT_FAILED,
		LFS_FOPEN_ERR,
		LFS_FCLOSE_ERR,
		LFS_UNLABELED_ERR,						// you will have to look through the LFS code to find what this error is
		LFS_RECIEVER_TOO_SMALL_WARNING = 0,
	};


	LFS_ERROR mount();
	void unmount();

	LFS_ERROR writeToFile(const char* filepath, const void* buffer, uint8_t datasize);						// do i have to open a certain directory
	LFS_ERROR readFromFile(const char* filepath, void* receiverBuffer, uint8_t recieverSize);
	LFS_ERROR moveFile(const char* filepath, const char* newPath);

	LFS_ERROR fastWrite(const char* filepath, const void* buffer, uint8_t datasize);
	LFS_ERROR fastRead(const char* filepath, void* receiverBuffer, uint8_t recieverSize);

	LFS_ERROR moveFile(const char* filepath, const char* newPath);

private:
	lfs_file_t fileptr = nullptr;
	uint8_t redundancyLevel; // can be used in the future to implement a CRC
	bool mounted;
};
/************************************ * FUNCTION DECLARATIONS ************************************/
#endif /* EXAMPLE_TASK_HPP_ */
