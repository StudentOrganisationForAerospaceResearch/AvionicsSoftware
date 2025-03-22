/** ********************************************************************************
 * * @file    umsDriver.cpp
 * * @author  root
 * * @date    Mar 15, 2025
 * * @brief
 * ******************************************************************************** */
/************************************ * INCLUDES ************************************/
#include "umsDriver.hpp"
#include "lfs.h"
/************************************ * PRIVATE MACROS AND DEFINES ************************************/
/************************************ * VARIABLES ************************************/
/************************************ * FUNCTION DECLARATIONS ************************************/
/************************************ * FUNCTION DEFINITIONS ************************************/
UMSDriver::UMSDriver()
{
	return;
	//lfs = LFS::getLFS();
}


void UMSDriver::respondToCBW(uint8_t* cbw, uint8_t* data)
{
	// asser the first 4 bytes are 55 53 42 43
	SOAR_ASSERT(USBCSIG == getByteRange(cbw, 0, 4));

	// skip the 4 ID bytes


	// record the data transfer length
	uint16_t transferLength = (uint16_t) getByteRange(cbw, 0x08, 4);

	// record if this is in or out
	bool in = getByteRange(cbw, 0x0C, 1) && IN;

	// LUN should be 0?
	SOAR_ASSERT(!getByteRange(cbw, 0x0D, 1));

	// record the command length
	uint8_t commandLength = getByteRange(cbw, 0x0E, 1);

	// parse and handle the individual commands
	uint8_t scsi[16];
	getByteRange(scsi, cbw, 0x0F, commandLength);

	handleCommand(scsi, data);

}

void UMSDriver::handleCommand(uint8_t* scsi, uint8_t* data)
{
	// get the opcode
	uint8_t opcode = scsi[0];
	uint8_t FlagLUN = scsi[1];

	// command fields
	uint8_t lbaHigh;
	uint8_t lbaLow;
	uint8_t length;
	uint8_t control;
	int lba;
	uint8_t group;
	int transferLength;


	// handle the command based on the opcode
	switch(opcode)
	{
		case INQUIRY: // 6 byte
			lbaHigh = scsi[2];
			lbaLow = scsi[3];
			length = scsi[4];
			control = scsi[5];

			respInquiry(data);

			break;
		case MODSENSE6: // 6 byte
			lbaHigh = scsi[2];
			lbaLow = scsi[3];
			length = scsi[4];
			control = scsi[5];

			break;
		case MODSENSE10: // 10 byte
			lba = getByteRange(scsi, 2, 4);
			group = scsi[6]; // usually reserved 0x00
			transferLength = (uint16_t) getByteRange(scsi, 7, 2);
			control = scsi[9];

			break;
		case READCAPACITY: // 10 byte
			lba = getByteRange(scsi, 2, 4);
			group = scsi[6]; // usually reserved 0x00
			transferLength = (uint16_t) getByteRange(scsi, 7, 2);
			control = scsi[9];

			respReadCapacity(data);

			break;
		case READ10: // 10 byte
			lba = getByteRange(scsi, 2, 4);
			group = scsi[6]; // usually reserved 0x00
			transferLength = (uint16_t) getByteRange(scsi, 7, 2);
			control = scsi[9];



			break;
		case WRITE10: // 10 byte
			lba = getByteRange(scsi, 2, 4);
			group = scsi[6]; // usually reserved 0x00
			transferLength = (uint16_t) getByteRange(scsi, 7, 2);
			control = scsi[9];

			break;
		default:
			SOAR_PRINT("Unsupported command: %d", opcode);
	}
}

void UMSDriver::respInquiry(uint8_t* data)	// only implemented with example data
{
	// bulk in 36 bytes of inquiry data

	*data = 0x00; // peripheral device type is direct access 0x00?
	data++;
	*data = 0x80; // removable media is yes
	data++;
	*data = 0x06; // version is SCSI-6
	data++;
	*data = 0x02; // response data format is SCSI-2
	data++;
	*data = 0x1F; // remaining bytes is 31
	data++;
	uint8_t deviceData[] = "USBSTORFlash Drive1.00";
	data = setByteRange(data, deviceData, sizeof(deviceData));
}

void UMSDriver::respReadCapacity(uint8_t* data){
	int blockCount = lfs->getBlockCount();
	int blockSize = lfs->getBlockSize();

	*data = blockCount & 0xff;
	data++;
	*data = (blockCount & 0xff00) >> 8;
	data++;
	*data = (blockCount & 0xff0000) >> 16;
	data++;
	*data = (blockCount & 0xff000000) >> 24;
	data++;

	*data = blockSize & 0xff;
	data++;
	*data = (blockSize & 0xff00) >> 8;
	data++;
	*data = (blockSize & 0xff0000) >> 16;
	data++;
	*data = (blockSize & 0xff000000) >> 24;
	data++;
}

void UMSDriver::respRead10(uint8_t* data, int lba, int transferLength)
{

}

uint32_t UMSDriver::getByteRange(uint8_t* command, uint8_t offset, uint8_t range)
{
	int bytes = 0;
	for(int i = 0; i < range; i++){
		uint8_t byteIndex = offset + i;
		bytes = bytes << 0x1;
		bytes += command[byteIndex];
	}

	return bytes;
}

void UMSDriver::getByteRange(uint8_t* dest, uint8_t* src, uint8_t offset, uint8_t range)
{
	for(int i = 0; i < range; i++){
		uint8_t byteIndex = offset + i;
		src[i] = dest[byteIndex];
	}
}

uint8_t* UMSDriver::setByteRange(uint8_t* data, uint8_t* cpy, uint8_t cpylen)
{
	for(; cpylen >= 0; cpylen--){
		*data = *cpy;
		data++;
		cpy++;
	}

	return data;
}








