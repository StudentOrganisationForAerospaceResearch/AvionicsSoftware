/** ********************************************************************************
 * * @file    umsDriver.hpp
 * * @author  root
 * * @date    Mar 15, 2025
 * * @brief ******************************************************************************** */
#ifndef FLASH_UMSDRIVER_HPP_
#define FLASH_UMSDRIVER_HPP_
/************************************ * INCLUDES ************************************/
#include "SystemDefines.hpp"
#include "lfsWrapper.hpp"
/************************************ * MACROS AND DEFINES ************************************/

// CBW layout: 55 53 42 43 (4: ID) (4: Data transfer length in bytes) (1: 0x80 for IN, 0x00 for OUT) (1: LUN) (1: Command length ≤ 16) (command length: SCSI command)
constexpr int USBCSIG = 0x55'53'42'43; // offset 0x00
constexpr int IN = 0x80; // offset 0x08
constexpr int OUT = 0x00; // 0x08 offset

// scsi commands
// command layout: (1: opcode) (1: flags and LUN) (LBA logical block address) (transfer length) (control byte) (zero bytes)
constexpr int INQUIRY = 0x12;
constexpr int MODSENSE6 = 0x1a;
constexpr int MODSENSE10 = 0x5a;
constexpr int READCAPACITY = 0x25;
constexpr int READ10 = 0x28;
constexpr int WRITE10 = 0x2a;

/************************************ * TYPEDEFS ************************************/
/************************************ * CLASS DEFINITIONS ************************************/
class UMSDriver {
public:
	UMSDriver();

	void respondToCBW(uint8_t* cbw, uint8_t* data); // takes 32 byte cbw
	void handleCommand(uint8_t* command, uint8_t* data); // takes 16 byte scsi command
	void respInquiry(uint8_t* data);
	void respReadCapacity(uint8_t* data);
	void respRead10(uint8_t* data, int lba, int transferLength);

	// must read partition table
	// must read root directory
	// must read file


private:
	void getByteRange(uint8_t* dest, uint8_t* src, uint8_t offset, uint8_t range);
	uint32_t getByteRange(uint8_t* data, uint8_t offset, uint8_t range);
	uint8_t* setByteRange(uint8_t* data, uint8_t* cpy, uint8_t cpylen);

	LFS* lfs;

};
/************************************ * FUNCTION DECLARATIONS ************************************/
#endif /* EXAMPLE_TASK_HPP_ */
