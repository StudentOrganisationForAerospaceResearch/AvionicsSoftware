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
Lfs::Lfs(uint8_t rl) : redundancyLevel(rl), mounted(false) {
	stmlfs_mount(true);
	unmount();
}

/*
 * @brief	mount the lfs file system. Must mount and unmount before and after fast writes/reads
 * @return 	err: LFS_OK corresponds to a successful mount, LFS_MOUNT_FAILED corresponds to a failed mount
 * */
Lfs::LFS_ERROR Lfs::mount(){
	if(stmlfs_mount(false)){
		return LFS_MOUNT_FAILED;
	}

	mounted = true;

	return LFS_OK;
}

/*
 * @brief	umount the lfs file system. Must mount and unmount before and after fast writes/reads
 * */
void Lfs::unmount(){
	stmlfs_unmount();

	mounted = false;
}

/*
 * @brief	write data to a new file in the file system
 * @param	filepath: a string filepath to the location of the file
 * @param 	buffer: a buffer of the data to write to the file
 * @param 	datasize: the number of bytes in the buffer to write
 * @return	err
 * */
Lfs::LFS_ERROR Lfs::writeToFile(const char* filepath, const void* buffer, uint32_t datasize){
	// mount the filesystem
	uint8_t err = mount();
	if(err)
		return LFS_MOUNT_FAILED;

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
Lfs::LFS_ERROR Lfs::readFromFile(const char* filepath, void* receiverBuffer, uint32_t recieverSize){
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
	unmount();

	return LFS_OK;
}

/*
 * @brief	write data to a new file. Does not mount and unmount filesystem before an after
 * @attention	user must mount the filesystem before a series of writes, and unmount it when completed
 * @param 	buffer: a buffer of the data to write to the file
 * @param 	datasize: the number of bytes in the buffer to write
 * */
Lfs::LFS_ERROR Lfs::fastWrite(const char* filepath, const void* buffer, uint32_t datasize){
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
Lfs::LFS_ERROR Lfs::fastRead(const char* filepath, void* receiverBuffer, uint32_t recieverSize){
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
Lfs::LFS_ERROR Lfs::moveFile(const char* filepath, const char* newPath){
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













