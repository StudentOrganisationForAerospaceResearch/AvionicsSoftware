/**
 ********************************************************************************
 * @file    W25Qxx.h
 * @author  spiro
 * @date    Jan 18, 2025
 * @brief
 ********************************************************************************
 */

#ifndef FLASH_INC_W25QXX_H_
#define FLASH_INC_W25QXX_H_

/************************************
 * INCLUDES
 ************************************/

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/*
 * W25Qxx.h
 *
 *  Created on: Apr 6, 2024
 *      Author: hans6
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

// Comment out for some extra debugging info
//#define SPIDEBUG					1

#define FS_SIZE                 (1024 * 1024 * 8)                   // 8Mbyte
#define FS_PAGE_SIZE            256									// Winbond W25Qxx 256 Page program
#define FS_SECTOR_SIZE          4096								// Winbond W25Qxx minimum erase size

#include "lfs_util.h"
#include "lfs.h"

struct littlfs_fsstat_t {
    lfs_size_t block_size;
    lfs_size_t block_count;
    lfs_size_t blocks_used;
};


#ifdef SPIDEBUG
	#define dprintf(...)    printf(__VA_ARGS__)		                // Debug messages on UART0
#else
	#define dprintf(...)
#endif



int stmlfs_mount(bool format);
int stmlfs_file_open(lfs_file_t *file, const char *path, int flags);
int stmlfs_file_read(lfs_file_t *file,void *buffer, lfs_size_t size);
int stmlfs_file_rewind(lfs_file_t *file);
lfs_ssize_t stmlfs_file_write(lfs_file_t *file,const void *buffer, lfs_size_t size);
int stmlfs_file_close(lfs_file_t *file);
int stmlfs_unmount(void);
int stmlfs_remove(const char* path);
int stmlfs_rename(const char* oldpath, const char* newpath);
int stmlfs_fflush(lfs_file_t *file);
int stmlfs_dir_open(const char* path);
int stmlfs_dir_close(int dir);
int stmlfs_dir_read(int dir, struct lfs_info* info);
int stmlfs_dir_seek(int dir, lfs_off_t off);
lfs_soff_t stmlfs_dir_tell(int dir);
int stmlfs_dir_rewind(int dir);
lfs_soff_t stmlfs_lseek(lfs_file_t *file, lfs_soff_t off, int whence);
int stmlfs_truncate(lfs_file_t *file, lfs_off_t size);
lfs_soff_t stmlfs_tell(lfs_file_t *file);
int stmlfs_stat(const char* path, struct lfs_info* info);
int stmlfs_fsstat(struct littlfs_fsstat_t* stat);
lfs_ssize_t stmlfs_getattr(const char* path, uint8_t type, void* buffer, lfs_size_t size);
int stmlfs_setattr(const char* path, uint8_t type, const void* buffer, lfs_size_t size);
int stmlfs_removeattr(const char* path, uint8_t type);
int stmlfs_opencfg(lfs_file_t *file, const char* path, int flags, const struct lfs_file_config* config);
lfs_soff_t stmlfs_size(lfs_file_t *file);
int stmlfs_mkdir(const char* path);
const char* stmlfs_errmsg(int err);
void dump_dir(void);


int stmlfs_hal_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size);
int stmlfs_hal_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size);
int stmlfs_hal_erase(const struct lfs_config *c, lfs_block_t sector);
int stmlfs_hal_sync(const struct lfs_config *c);


void W25Q_Reset (void);
uint32_t W25Q_ReadID (void);
uint64_t W25Q_ReadUniqueID(void);
void W25Q_ReadSFDP(uint8_t *rData);
uint8_t W25Q_ReadStatus(int reg);
void W25Q_WriteStatus(int reg, uint8_t status);
void W25Q_Read(uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead(uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData);
void W25Q_Write_block(uint32_t block, uint16_t offset, uint32_t size, const uint8_t *data);
void W25Q_Erase_Chip(void);
void W25Q_Erase_Sector(uint16_t numsector);
void write_enable(void);
void write_disable(void);
void delay_us(uint16_t us);

#endif /* INC_W25QXX_H_ */

#endif /* EXAMPLE_TASK_HPP_ */
 
