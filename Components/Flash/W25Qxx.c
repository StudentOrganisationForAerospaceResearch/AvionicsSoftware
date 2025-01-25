/**
 ********************************************************************************
 * @file    W25Qxx.c
 * @author  spiro
 * @date    Jan 18, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/

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
 /*
 * W25Qxx.c
 *
 *  Created on: Apr 6, 2024
 *      Author: hans6
 */

#include "W25Qxx.h"
#include "main.h"
//#include "SystemDefines.hpp"

extern TIM_HandleTypeDef htim1;										// Not used for this demo
extern SPI_HandleTypeDef hspi1;

#define W25Q_SPI hspi1

static lfs_t lfs;													// Littlefs


const struct lfs_config stmconfig = {
    // block device operations
    .read  = stmlfs_hal_read,
    .prog  = stmlfs_hal_prog,
    .erase = stmlfs_hal_erase,
    .sync  = stmlfs_hal_sync,

    // block device configuration
    .read_size      = FS_PAGE_SIZE,
    .prog_size      = FS_PAGE_SIZE,
    .block_size     = FS_SECTOR_SIZE,
    .block_count    = FS_SIZE/FS_SECTOR_SIZE,
    .cache_size     = FS_SECTOR_SIZE/4,
    .lookahead_size = 32,                                           // must be multiple of 8
    .block_cycles   = 100,                                          // 100(better wear levelling)-1000(better performance)
};

int save_and_disable_interrupts(void) {								// Not used
    uint32_t store_primask = __get_PRIMASK();
    __disable_irq();
    return store_primask;
}

void restore_interrupts(int mask) {									// Not used
    __set_PRIMASK(mask);
}

int stmlfs_hal_sync(const struct lfs_config *c)
{
    UNUSED(*c);
    return LFS_ERR_OK;
}

int stmlfs_mount(bool format)
{
//	SOAR_PRINT("mounted");

	int err=-1;

	assert(FS_SIZE<16777216);										// Chip < 16Mbyte, change R/W to 32bits address


    if (format) {
    	err=lfs_format(&lfs,&stmconfig);
//    	printf("lfs_format - returned: %d\n",err);
    }


    err=lfs_mount(&lfs,&stmconfig);                              	// mount the filesystem
//    printf("lfs_mount  - returned: %d\n",err);
    return err;
}

int stmlfs_hal_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size)
{
	assert(block < c->block_count);
    assert(off + size <= c->block_size);

    dprintf("stmlfs_hal_read(block=%ld off=%ld size=%ld)\n",block,off,size);
    W25Q_Read(block,off,size,buffer);

    return LFS_ERR_OK;
}

int stmlfs_hal_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size)
{
	assert(block < c->block_count);

    dprintf("stmlfs_hal_prog(block=%ld off=%ld size=%ld)\n",block,off,size);
    W25Q_Write_block(block,off,size,buffer);

    return LFS_ERR_OK;
}

int stmlfs_hal_erase(const struct lfs_config *c, lfs_block_t block)
{
	assert(block < c->block_count);

    dprintf("stmlfs_hal_erase(block=%ld)\n",block);
    W25Q_Erase_Sector(block);

    return LFS_ERR_OK;
}



int stmlfs_file_open(lfs_file_t *file, const char *path, int flags)
{
    return lfs_file_open(&lfs, file, path, flags);
}

int stmlfs_file_read(lfs_file_t *file,void *buffer, lfs_size_t size)
{
    return lfs_file_read(&lfs, file, buffer, size);
}

int stmlfs_file_rewind(lfs_file_t *file)
{
    return lfs_file_rewind(&lfs, file);
}

lfs_ssize_t stmlfs_file_write(lfs_file_t *file,const void *buffer, lfs_size_t size)
{
    return lfs_file_write(&lfs, file,buffer,size);
}

int stmlfs_file_close(lfs_file_t *file)
{
    return lfs_file_close(&lfs, file);
}

int stmlfs_unmount(void)
{
    return lfs_unmount(&lfs);
}

int stmlfs_remove(const char* path)
{
    return lfs_remove(&lfs, path);
}

int stmlfs_rename(const char* oldpath, const char* newpath)
{
    return lfs_rename(&lfs, oldpath, newpath);
}

int stmlfs_fflush(lfs_file_t *file)
{
    return lfs_file_sync(&lfs, file);
}

int stmlfs_fsstat(struct littlfs_fsstat_t* stat)
{
    stat->block_count = stmconfig.block_count;
    stat->block_size  = stmconfig.block_size;
    stat->blocks_used = lfs_fs_size(&lfs);
    return LFS_ERR_OK;
}



lfs_soff_t stmlfs_lseek(lfs_file_t *file, lfs_soff_t off, int whence)
{
    return lfs_file_seek(&lfs, file, off, whence);
}

int stmlfs_truncate(lfs_file_t *file, lfs_off_t size)
{
    return lfs_file_truncate(&lfs, file, size);
}

lfs_soff_t stmlfs_tell(lfs_file_t *file)
{
    return lfs_file_tell(&lfs, file);
}

int stmlfs_stat(const char* path, struct lfs_info* info)
{
    return lfs_stat(&lfs, path, info);
}

lfs_ssize_t stmlfs_getattr(const char* path, uint8_t type, void* buffer, lfs_size_t size)
{
    return lfs_getattr(&lfs, path, type, buffer, size);
}

int stmlfs_setattr(const char* path, uint8_t type, const void* buffer, lfs_size_t size)
{
    return lfs_setattr(&lfs, path, type, buffer, size);
}

int stmlfs_removeattr(const char* path, uint8_t type)
{
    return lfs_removeattr(&lfs, path, type);
}

int stmlfs_opencfg(lfs_file_t *file, const char* path, int flags, const struct lfs_file_config* config)
{
    return lfs_file_opencfg(&lfs, file, path, flags, config);
}

lfs_soff_t stmlfs_size(lfs_file_t *file)
{
    return lfs_file_size(&lfs, file);
}

int stmlfs_mkdir(const char* path)
{
    return lfs_mkdir(&lfs, path);
}




int stmlfs_dir_open(const char* path)
{
	lfs_dir_t* dir = lfs_malloc(sizeof(lfs_dir_t));
	if (dir == NULL)
		return -1;
	if (lfs_dir_open(&lfs, dir, path) != LFS_ERR_OK) {
		lfs_free(dir);
		return -1;
	}
	return (int)dir;
}

int stmlfs_dir_close(int dir)
{
	return lfs_dir_close(&lfs, (lfs_dir_t*)dir);
	lfs_free((void*)dir);
}

int stmlfs_dir_read(int dir, struct lfs_info* info)
{
    return lfs_dir_read(&lfs, (lfs_dir_t*)dir, info);
}

int stmlfs_dir_seek(int dir, lfs_off_t off)
{
    return lfs_dir_seek(&lfs, (lfs_dir_t*)dir, off);
}

lfs_soff_t stmlfs_dir_tell(int dir)
{
    return lfs_dir_tell(&lfs, (lfs_dir_t*)dir);
}

int stmlfs_dir_rewind(int dir)
{
    return lfs_dir_rewind(&lfs, (lfs_dir_t*)dir);
}

const char* stmlfs_errmsg(int err)
{
    static const struct {
        int err;
        char* text;
    } mesgs[] = {{LFS_ERR_OK, "No error"},
                 {LFS_ERR_IO, "Error during device operation"},
                 {LFS_ERR_CORRUPT, "Corrupted"},
                 {LFS_ERR_NOENT, "No directory entry"},
                 {LFS_ERR_EXIST, "Entry already exists"},
                 {LFS_ERR_NOTDIR, "Entry is not a dir"},
                 {LFS_ERR_ISDIR, "Entry is a dir"},
                 {LFS_ERR_NOTEMPTY, "Dir is not empty"},
                 {LFS_ERR_BADF, "Bad file number"},
                 {LFS_ERR_FBIG, "File too large"},
                 {LFS_ERR_INVAL, "Invalid parameter"},
                 {LFS_ERR_NOSPC, "No space left on device"},
                 {LFS_ERR_NOMEM, "No more memory available"},
                 {LFS_ERR_NOATTR, "No data/attr available"},
                 {LFS_ERR_NAMETOOLONG, "File name too long"}};

    for (unsigned int i = 0; i < sizeof(mesgs) / sizeof(mesgs[0]); i++)
        if (err == mesgs[i].err)
            return mesgs[i].text;
    return "Unknown error";
}


//-------------------------------------------------------------------------------------------------
// display each directory entry name
//-------------------------------------------------------------------------------------------------
void dump_dir(void)
{
    int dir = stmlfs_dir_open("/");
    if (dir < 0) {
    	printf("\nstmlfs_dir_open failed\n");
    	return;
    }

    struct lfs_info info;
    while (stmlfs_dir_read(dir, &info) > 0) {
        printf("%16.16s ", info.name);
        if (info.type==LFS_TYPE_REG) {
            printf(" %04ld\n",info.size);
            // static const char *prefixes[] = {"", "K", "M", "G"};
            // for (int i = sizeof(prefixes)/sizeof(prefixes[0])-1; i >= 0; i--) {
            //     if (info.size >= (1 << 10*i)-1) {
            //         printf("%*u%sB\n", 4-(i != 0), info.size >> 10*i, prefixes[i]);
            //         break;
            //     }
            // }
        } else {
            printf("\n");
        }
    }
    stmlfs_dir_close(dir);

    struct littlfs_fsstat_t stat;                                      // Show file system sizes
    stmlfs_fsstat(&stat);
    printf("\nBlocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);

}



//-------------------------------------------------------------------------------------------------
// STM32 SPI Driver
//-------------------------------------------------------------------------------------------------

void W25Q_Delay(uint32_t time)
{
	HAL_Delay(time);
}

void csLOW (void)
{
	HAL_GPIO_WritePin(SPI_FLASH_CS_GPIO_Port, SPI_FLASH_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH (void)
{
	HAL_GPIO_WritePin(SPI_FLASH_CS_GPIO_Port, SPI_FLASH_CS_Pin, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint16_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint16_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

/**************************************************************************************************/

void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;  												// enable Reset
	tData[1] = 0x99;  												// Reset
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	W25Q_Delay(100);
}

uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9F;  // Read JEDEC ID
	uint8_t rData[3];
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);
	csHIGH();
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}

uint8_t W25Q_ReadStatus(int reg)									// Read status reg1,2,3
{
	uint8_t tData,rData;
	switch(reg){
		case 1: tData = 0x05; break;
		case 2: tData = 0x35; break;
		case 3: tData = 0x15; break;
		default:
			printf("Invalid status register 0\n");
			return 0;
	}
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(&rData, 1);
	csHIGH();
	return (rData);
}

void W25Q_WriteStatus(int reg, uint8_t newstatus)
{
	uint8_t tData[2];
	switch(reg){
		case 1: tData[0] = 0x01; break;
		case 2: tData[0] = 0x31; break;
		case 3: tData[0] = 0x11; break;
		default: return;
	}

	tData[1]=newstatus;
	write_enable();
	csLOW();
	SPI_Write(tData, 2);
	csHIGH();
	write_disable();
}

uint64_t W25Q_ReadUniqueID(void)
{
	uint8_t tData[5];  												// Read Unique 64bits  ID
	uint8_t rData[8];

	tData[0] = 0x4B;
	csLOW();
	SPI_Write(tData, 5);
	SPI_Read(rData, 8);
	csHIGH();
	printf("64bits Identifier = 0x");
	for (int i=0;i<8;i++) printf("%02x",rData[i]);
	printf("\n");

	return (((uint64_t)rData[0]<<56)|((uint64_t)rData[1]<<48)|((uint64_t)rData[2]<<40)|((uint64_t)rData[3]<<32)|
			((uint64_t)rData[4]<<24)|((uint64_t)rData[5]<<16)|((uint64_t)rData[6]<<8)|(uint64_t)rData[7]);
}

void W25Q_ReadSFDP(uint8_t *rData)
{
	uint8_t tData[5]={0x5A,0,0,0,0};
	csLOW();  														// pull the CS Low
	SPI_Write(tData, 5);
	SPI_Read(rData, 256);  											// Read the data
	csHIGH();  														// pull the CS High
}

void W25Q_Read (uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (block*FS_SECTOR_SIZE) + offset;

	tData[0] = 0x03;  												// enable Read
	tData[1] = (memAddr>>16)&0xFF;  								// MSB of the memory Address
	tData[2] = (memAddr>>8)&0xFF;
	tData[3] = (memAddr)&0xFF; 										// LSB of the memory Address

	csLOW();  														// pull the CS Low
	SPI_Write(tData, 4);  											// 24 bit memory address
	SPI_Read(rData, size);  										// Read the data
	csHIGH();  														// pull the CS High
}

void W25Q_FastRead (uint32_t block, uint16_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (block*FS_SECTOR_SIZE) + offset;

	tData[0] = 0x0B;  												// enable Fast Read
	tData[1] = (memAddr>>16)&0xFF;  								// MSB of the memory Address
	tData[2] = (memAddr>>8)&0xFF;
	tData[3] = (memAddr)&0xFF; 										// LSB of the memory Address
	tData[4] = 0;  													// Dummy clock

	csLOW();  														// pull the CS Low
	SPI_Write(tData, 5);  											// 24 bit memory address
	SPI_Read(rData, size);  										// Read the data
	csHIGH();  														// pull the CS High
}

void write_enable(void)
{
	uint8_t tData = 0x06;  											// enable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  												// 5ms delay
}

void write_disable(void)
{
	uint8_t tData = 0x04;  											// disable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_Delay(5);  												// 5ms delay
}

void W25Q_Erase_Chip(void)
{
	uint8_t tData = 0x60;  											// Chip Erase

	write_enable();
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	write_disable();

	while(W25Q_ReadStatus(1)&0x01);									// Wait for BUSY to go low, TODO add timeout?
}

void W25Q_Erase_Sector(uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector*FS_SECTOR_SIZE;					// Each sector contains 16 pages * 256 bytes

	write_enable();

	tData[0] = 0x20;  												// Erase sector
	tData[1] = (memAddr>>16)&0xFF;  								// MSB of the memory Address
	tData[2] = (memAddr>>8)&0xFF;
	tData[3] = (memAddr)&0xFF; 										// LSB of the memory Address

	csLOW();
	SPI_Write(tData, 4);
	csHIGH();

	while(W25Q_ReadStatus(1)&0x01);									// Check BUSY is low, if not wait, TODO add timeout?

	write_disable();
}

void Write_page(uint32_t page, uint16_t offset, uint32_t size, const uint8_t *data)
{
	uint8_t tData[266];
	uint32_t memAddr = (page*FS_PAGE_SIZE)+offset;
	uint32_t indx = 0;

	dprintf("W25Q_Write(page=%ld, offset=%d, memaddr=%ld) memAddr=%08lx\n",page,offset,size,memAddr);

	write_enable();

	tData[0] = 0x02;  												// block program
	tData[1] = (memAddr>>16)&0xFF;  								// MSB of the memory Address
	tData[2] = (memAddr>>8)&0xFF;
	tData[3] = (memAddr)&0xFF; 										// LSB of the memory Address
	indx = 4;

	memcpy(&tData[indx],data,size);

	csLOW();
	SPI_Write(tData, indx+size);
	csHIGH();

	while(W25Q_ReadStatus(1)&0x01);									// Check BUSY is low, if not wait, TODO add timeout?
	write_disable();
}

void W25Q_Write_block(uint32_t block, uint16_t offset, uint32_t size, const uint8_t *data)
{
	uint32_t startpage=((block*FS_SECTOR_SIZE)+offset)/FS_PAGE_SIZE;
	uint32_t bytesleft=size;
	uint32_t newoff=offset%256;
	uint32_t bufptr=0;

	dprintf("W25Q_Write_block(%ld,%d,%ld)  startpage=%ld newoff=%ld\n",block,offset,size,startpage,newoff);

	dprintf("First %ld,%04ld,%03ld ",startpage,newoff,(FS_PAGE_SIZE-newoff));
	Write_page(startpage, newoff, (FS_PAGE_SIZE-newoff), data);				// First block
	bufptr=(FS_PAGE_SIZE-newoff);
	bytesleft-=(FS_PAGE_SIZE-newoff);

	startpage++;
	while (bytesleft) {

		if (bytesleft>256) {
			dprintf("Page %ld,%04d,%03d ",startpage,0,FS_PAGE_SIZE);
			Write_page(startpage, 0, FS_PAGE_SIZE, &data[bufptr]);
			bytesleft-=FS_PAGE_SIZE;
			bufptr+=FS_PAGE_SIZE;
		} else {
			if (newoff) {
				dprintf("Last  %ld,%04d,%03ld ",startpage,0,newoff);
				Write_page(startpage, 0, newoff, &data[bufptr]);	// Last block
			} else {
				dprintf("Last  %ld,%04d,%03ld ",startpage,0,bytesleft);
				Write_page(startpage, 0, bytesleft, &data[bufptr]);
				bufptr+=bytesleft;
			}
			bytesleft=0;
		}
		startpage++;
	}
	dprintf("\n");
}


void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  								// clear counter
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  					// wait
}


// Software CRC implementation with small lookup table
uint32_t lfs_crc(uint32_t crc, const void* buffer, size_t size) {
    static const uint32_t rtable[16] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4,
        0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };

    const uint8_t* data = buffer;

    for (size_t i = 0; i < size; i++) {
        crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 0)) & 0xf];
        crc = (crc >> 4) ^ rtable[(crc ^ (data[i] >> 4)) & 0xf];
    }

    return crc;
}
