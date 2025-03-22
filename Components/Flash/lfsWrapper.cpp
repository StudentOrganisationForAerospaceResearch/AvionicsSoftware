/** ********************************************************************************
 * * @file    lfsWrapper.cpp
 * * @author  Noah Vickerson, Spiro Douvais
 * * @date    Feb 22, 2025
 * * @brief *********************************************************************************/
/************************************ * INCLUDES ************************************/
#include "lfsWrapper.hpp"
#include "lfs.h"
#include "lfs_util.h"
#include "W25Qxx.h"
#include "SystemDefines.hpp"
/************************************ * PRIVATE MACROS AND DEFINES ************************************/
/************************************ * VARIABLES ************************************/
/************************************ * FUNCTION DECLARATIONS ************************************/
/************************************ * FUNCTION DEFINITIONS ************************************/

/*
 * @brief 	init mounts and formats the filesystem
 * @param	rl: optional redundancy level for a crc
 * @note 	the format is at the start of W25Qxx.c
 * */
<<<<<<< HEAD
LFS::LFS(uint8_t rl) : redundancyLevel(rl), mounted(false) {
	if(instantiated){
		SOAR_PRINT("Cannot have more than 1 filesystem\n");
	}

	instantiated = true;

=======
Lfs::Lfs(uint8_t rl) : redundancyLevel(rl), mounted(false) {
>>>>>>> 41cec5c31b288429e6a951e7290681af3379e47a
	stmlfs_mount(true);
	unmount();
}

LFS LFS::getLFS(){
	SOAR_ASSERT(instantiated);

	return filesystem;
}

/*
 * @brief	mount the lfs file system. Must mount and unmount before and after fast writes/reads
 * @return 	err: LFS_OK corresponds to a successful mount, LFS_MOUNT_FAILED corresponds to a failed mount
 * */
LFS::LFS_ERROR LFS::mount(){
	if(mounted){
		return LFS_MOUNT_STATE_ERR;
	}

	if(stmlfs_mount(false)){
		return LFS_MOUNT_FAILED;
	}

	mounted = true;

	return LFS_OK;
}

/*
 * @brief	umount the lfs file system. Must mount and unmount before and after fast writes/reads
 * */
LFS::LFS_ERROR LFS::unmount(){
	if(!mounted){
		return LFS_MOUNT_STATE_ERR;
	}

	stmlfs_unmount();

	mounted = false;

	return LFS_OK;
}

/*
 * @brief	write data to a new file in the file system
 * @param	filepath: a string filepath to the location of the file
 * @param 	buffer: a buffer of the data to write to the file
 * @param 	datasize: the number of bytes in the buffer to write
 * @return	err
 * */
LFS::LFS_ERROR LFS::writeToFile(const char* filepath, const void* buffer, uint32_t datasize){
	// mount the filesystem
	uint8_t err = mount();
	if(err == 1)
		return LFS_MOUNT_FAILED;
	else
		return LFS_MOUNT_STATE_ERR;

	// open the file in create mode
	err = stmlfs_file_open(&fileptr, filepath, LFS_O_WRONLY | LFS_O_CREAT);
	if(err < 0)
		return LFS_FOPEN_ERR;

	// write the data to the file
	uint32_t written_amnt = stmlfs_file_write(&fileptr, buffer, datasize);
	if(written_amnt < datasize)
		return LFS_INSUFFICIENT_MEMORY_ERR;

	// close the file
	err = stmlfs_file_close(&fileptr);
	if(err < 0)
		return LFS_FCLOSE_ERR;

	// unmount the filesystem
	unmount();

	return LFS_OK;
}

/*
 * @brief 	read data from an existing file
 * @param	filepath: a string filepath to the location of the file
 * @param 	recieverBuffer: the buffer that the file data is copied into
 * @param	recieverSize: the size of the reciever buffer. Will get a warning if this is smaller than the size of the file
 * @return	err: LFS_RECIEVER_TOO_SMALL_ERROR will still copy the file data to the buffer, just not all of it
 * */
LFS::LFS_ERROR LFS::readFromFile(const char* filepath, void* receiverBuffer, uint32_t recieverSize){
	// mount the filesystem
	uint8_t err = mount();
	if(err)
		return LFS_MOUNT_FAILED;

	// open the file in readonly mode
	err = stmlfs_file_open(&fileptr, filepath, LFS_O_RDONLY);
	if(err < 0)
		return LFS_FOPEN_ERR;

	// read the file contents to the buffer
	stmlfs_file_read(&fileptr, receiverBuffer, recieverSize);

	// close the file
	err = stmlfs_file_close(&fileptr);
	if(err < 0)
		return LFS_FCLOSE_ERR;

	// unmount the filesystem
	err = unmount();
	if(err)
		return LFS_MOUNT_STATE_ERR;

	return LFS_OK;
}

/*
 * @brief	write data to a new file. Does not mount and unmount filesystem before an after
 * @attention	user must mount the filesystem before a series of writes, and unmount it when completed
 * @param 	buffer: a buffer of the data to write to the file
 * @param 	datasize: the number of bytes in the buffer to write
 * */
LFS::LFS_ERROR LFS::fastWrite(const char* filepath, const void* buffer, uint32_t datasize){
	if(!mounted)
		return LFS_MOUNT_STATE_ERR;

	uint8_t err;
	// open the file in create mode
	err = stmlfs_file_open(&fileptr, filepath, LFS_O_WRONLY | LFS_O_CREAT);
	if(err < 0)
		return LFS_FOPEN_ERR;

	// write the data to the file
	uint32_t written_amnt = stmlfs_file_write(&fileptr, buffer, datasize);
	if(written_amnt < datasize)
		return LFS_INSUFFICIENT_MEMORY_ERR;

	// close the file
	err = stmlfs_file_close(&fileptr);
	if(err < 0)
		return LFS_FCLOSE_ERR;

	return LFS_OK;
}

/*
 * @brief 	read data from an existing file Does not mount and unmount filesystem before an after
 * @attention	user must mount the filesystem before a series of reads, and unmount it when completed
 * @param	filepath: a string filepath to the location of the file
 * @param 	recieverBuffer: the buffer that the file data is copied into
 * @param	recieverSize: the size of the reciever buffer. Will get a warning if this is smaller than the size of the file
 * @return	err: LFS_RECIEVER_TOO_SMALL_ERROR will still copy the file data to the buffer, just not all of it
 * */
LFS::LFS_ERROR LFS::fastRead(const char* filepath, void* receiverBuffer, uint32_t recieverSize){
	if(!mounted)
		return LFS_MOUNT_STATE_ERR;

	uint8_t err;
	// open the file in readonly mode
	err = stmlfs_file_open(&fileptr, filepath, LFS_O_RDONLY);
	if(err < 0)
		return LFS_FOPEN_ERR;

	// read the file contents to the buffer
	stmlfs_file_read(&fileptr, receiverBuffer, recieverSize);

	// close the file
	err = stmlfs_file_close(&fileptr);
	if(err < 0)
		return LFS_FCLOSE_ERR;

	return LFS_OK;
}

/*
 * @brief	move a file from one location to another
 * @return 	err
 * */
LFS::LFS_ERROR LFS::moveFile(const char* filepath, const char* newPath){
	uint8_t err;
	bool justMounted = false;

	if(!mounted){
		err = mount();
		justMounted = true;
		if(err)
			return LFS_MOUNT_FAILED;
	}

	err = stmlfs_rename(filepath, newPath);
	if(err < 0)
		return LFS_UNLABELED_ERR;

	if(justMounted)
		unmount();

	return LFS_OK;
}

int LFS::getBlockCount(){
	stmlfs_fsstat(&stat);
	return (int)stat.block_count;
}













