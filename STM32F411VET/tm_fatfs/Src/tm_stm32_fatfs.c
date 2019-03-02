/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_fatfs.h"

/* Private functions */
static FRESULT scan_files(char* path, uint16_t tmp_buffer_size, TM_FATFS_Search_t* FindStructure);

FRESULT TM_FATFS_GetDriveSize(char* str, TM_FATFS_Size_t* SizeStruct) {
	FATFS *fs;
    DWORD fre_clust;
	FRESULT res;

    /* Get volume information and free clusters of drive */
    if ((res = f_getfree(str, &fre_clust, &fs)) != FR_OK) {
		return res;
	}

    /* Get total sectors and free sectors */
    SizeStruct->Total = (fs->n_fatent - 2) * fs->csize * 0.5;
    SizeStruct->Free = fre_clust * fs->csize * 0.5;
	
	/* Return OK */
	return FR_OK;
}

FRESULT TM_FATFS_TruncateBeginning(FIL* fil, uint32_t index) {
	uint8_t Buffer[FATFS_TRUNCATE_BUFFER_SIZE];				/* Buffer for temporary data */

	uint32_t FileSize = f_size(fil);						/* Size of file */
	uint32_t ReadIndex = index;								/* Starting read index */
	uint32_t WriteIndex = 0;								/* We have to write at beginning */
	uint32_t TotalSize = FileSize - ReadIndex;				/* New file size after truncate */
	uint32_t NewSize = TotalSize;							/* Save new file size */
	uint32_t BlockSize;										/* Block size for read operation */
	uint32_t Read;											/* Read bytes */
	uint32_t Written;										/* Written bytes */
	FRESULT fr;												/* Result typedef */
	
	/* Index is 0 or file is empty, nothing to do */
	if (index == 0 || FileSize == 0) {
		return FR_OK;
	}
	
	/* Check if index is more than file size, truncate all */
	if (index > FileSize) {
		fr = f_lseek(fil, 0);								/* Go to beginning */
		if (fr) return fr;									/* Check for success */
		return f_truncate(fil);								/* Truncate file from new end to actual end */
	}
	
	/* Until we have available data in file after user specific index */
	while (TotalSize > 0) {
		/* Calculate new block size for new read operation */
		BlockSize = (TotalSize > FATFS_TRUNCATE_BUFFER_SIZE) ? (FATFS_TRUNCATE_BUFFER_SIZE) : (TotalSize);
	
		fr = f_lseek(fil, ReadIndex);						/* Go to the read index */
		if (fr) return fr;									/* Check for success */
		fr = f_read(fil, &Buffer, BlockSize, &Read);		/* Read data */
		if (fr) return fr;									/* Check for success */

		fr = f_lseek(fil, WriteIndex);						/* Go back to the write index */
		if (fr) return fr;									/* Check for success */
		fr = f_write(fil, &Buffer, BlockSize, &Written);    /* Write data */
		if (fr) return fr;									/* Check for success */

		TotalSize -= BlockSize;								/* Calculate new total size we have more to move everything */
		ReadIndex += Read;									/* Calculate new read pointer */
		WriteIndex += Written;								/* Calculate new write pointer */
	}
	
	fr = f_lseek(fil, NewSize);								/* Move pointer to the "end" of new file */
	if (fr) return fr;										/* Check for success */
	fr = f_truncate(fil);									/* Truncate file from new end to actual end */
	return f_lseek(fil, 0);									/* Move pointer to the beginning */
}

uint8_t TM_FATFS_CheckCardDetectPin(void) {
	uint8_t status = 1;
	
#if FATFS_USE_DETECT_PIN > 0
	if (TM_GPIO_GetInputPinValue(FATFS_DETECT_PORT, FATFS_DETECT_PIN) != 0) {
		status = 0;
	}
#endif
	
	/* Return status */
	return status;
}

FRESULT TM_FATFS_Search(char* Folder, char* tmp_buffer, uint16_t tmp_buffer_size, TM_FATFS_Search_t* FindStructure) {
	uint8_t malloc_used = 0;
	FRESULT res;
	
	/* Reset values first */
	FindStructure->FilesCount = 0;
	FindStructure->FoldersCount = 0;
	
	/* Check for buffer */
	if (tmp_buffer == NULL) {
		/* Try to allocate memory */
		tmp_buffer = (char *) LIB_ALLOC_FUNC(tmp_buffer_size);
		
		/* Check for success */
		if (tmp_buffer == NULL) {
			return FR_NOT_ENOUGH_CORE;
		}
	}
	
	/* Check if there is a lot of memory allocated */
	if (strlen(Folder) < tmp_buffer_size) {
		/* Reset TMP buffer */
		tmp_buffer[0] = 0;
		
		/* Format path first */
		strcpy(tmp_buffer, Folder);
		
		/* Call search function */
		res = scan_files(tmp_buffer, tmp_buffer_size, FindStructure);
	} else {
		/* Not enough memory */
		res = FR_NOT_ENOUGH_CORE;
	}
	
	/* Check for malloc */
	if (malloc_used) {
		LIB_FREE_FUNC(tmp_buffer);
	}
	
	/* Return result */
	return res;
}

/*******************************************************************/
/*                      FATFS SEARCH CALLBACK                      */
/*******************************************************************/
__weak uint8_t TM_FATFS_SearchCallback(char* path, uint8_t is_file, TM_FATFS_Search_t* FindStructure) {
	/* NOTE: This function Should not be modified, when the callback is needed,
             the TM_FATFS_SearchCallback could be implemented in the user file
	*/
	
	/* Allow next search */
	return 1;
}

/*******************************************************************/
/*                    FATFS PRIVATE FUNCTIONS                      */
/*******************************************************************/
static FRESULT scan_files(char* path, uint16_t tmp_buffer_size, TM_FATFS_Search_t* FindStructure) {
	FRESULT res;
	static FILINFO fno;
	DIR dir;
	int i;
	uint8_t gonext;
	char* fn;

	/* Try to open folder */
	if ((res = f_opendir(&dir, path)) == FR_OK) {
		/* Get length of current path */
		i = strlen(path);

		/* Read item from card */
		while ((res = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0) {

			/* Ignore files */
			if (fno.fname[0] == '.') {
				continue;
			}

			/* Format name */
			fn = fno.fname;

			/* Check if available memory for tmp buffer */
			/* + 1 is for "/" used for path formatting */
			if ((i + strlen(fn) + 1) >= tmp_buffer_size) { 
				/* Not enough memory */
				res = FR_NOT_ENOUGH_CORE;

				break;
			}

			/* Format temporary path */
			sprintf(&path[i], "/%s", fn);

			/* Check for folder or file */
			if (fno.fattrib & AM_DIR) {
				/* We are folder */

				/* Increase number of folders */
				FindStructure->FoldersCount++;

				/* Call user function */
				gonext = TM_FATFS_SearchCallback(path, 0, FindStructure);

				/* Stop execution if user wants that */
				if (gonext) {
					/* Call recursive */
					res = scan_files(path, tmp_buffer_size, FindStructure);

					/* Check recursive scan result */
					if (res != FR_OK) {
						break;
					}
				}
			} else {
				/* We are file */

				/* Increase number of files */
				FindStructure->FilesCount++;

				/* Call user function */
				gonext = TM_FATFS_SearchCallback(path, 1, FindStructure);
			}
			
			/* Set path back */
			path[i] = 0;

			/* Stop execution if user wants that */
			if (!gonext || res != FR_OK) {
				break;
			}
		}

		/* Close directory */
		f_closedir(&dir);
	}

	/* Return result */
	return res;
}
