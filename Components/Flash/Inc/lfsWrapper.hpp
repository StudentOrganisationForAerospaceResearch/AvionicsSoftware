/** ********************************************************************************
 * * @file    lfsWrapper.hpp
 * * @author  NoahVickerson, Spiro Douvais
 * * @date    Feb 22, 2025
 * * @brief ******************************************************************************** */
#ifndef FLASH_INC_LFSWRAPPER_HPP_
#define FLASH_INC_LFSWRAPPER_HPP_
/************************************ * INCLUDES ************************************/
#include "lfs.h"
#include "lfs_util.h"
#include "W25Qxx.h"
/************************************ * MACROS AND DEFINES ************************************/
/************************************ * TYPEDEFS ************************************/
/************************************ * CLASS DEFINITIONS ************************************/
class Lfs {
public:
	enum LFS_ERROR {
		LFS_OK = 0,
		LFS_MOUNT_FAILED = 1,					// failed attempt to mount LFS
		LFS_FOPEN_ERR,							// error on file open
		LFS_FCLOSE_ERR,							// error on file close
		LFS_INSUFFICIENT_MEMORY_ERR,			// there is not enough memory available to support the creation of a file
		LFS_MOUNT_STATE_ERR,					// a mount/unmount is requested when LFS is alreadey mounted/unmounted
		LFS_UNLABELED_ERR,						// you will have to look through the LFS code to find what this error is
		LFS_RECIEVER_TOO_SMALL_WARNING = 0,		// the data buffer to store the filedata in is smaller than the file size, and file data will be truncated
	};

	Lfs(uint8_t rl = 0);


	LFS_ERROR mount();
	LFS_ERROR unmount();

	LFS_ERROR writeToFile(const char* filepath, const void* buffer, uint32_t datasize);						// do i have to open a certain directory
	LFS_ERROR readFromFile(const char* filepath, void* receiverBuffer, uint32_t recieverSize);
	LFS_ERROR moveFile(const char* filepath, const char* newPath);

	LFS_ERROR fastWrite(const char* filepath, const void* buffer, uint32_t datasize);
	LFS_ERROR fastRead(const char* filepath, void* receiverBuffer, uint32_t recieverSize);

private:
	lfs_file_t fileptr;
	uint8_t redundancyLevel; // can be used in the future to implement a CRC
	bool mounted;
};
/************************************ * FUNCTION DECLARATIONS ************************************/
#endif /* EXAMPLE_TASK_HPP_ */
