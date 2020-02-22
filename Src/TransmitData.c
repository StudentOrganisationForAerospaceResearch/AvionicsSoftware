#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "TransmitData.h"
#include "Utils.h"
#include "FlightPhase.h"
#include "Data.h"

static const int TRANSMIT_DATA_PERIOD = 500;

static const int8_t IMU_HEADER_BYTE = 0x31;
static const int8_t BAROMETER_HEADER_BYTE = 0x32;
static const int8_t GPS_HEADER_BYTE = 0x33;
static const int8_t OXIDIZER_TANK_HEADER_BYTE = 0x34;
static const int8_t COMBUSTION_CHAMBER_HEADER_BYTE = 0x35;
static const int8_t FLIGHT_PHASE_HEADER_BYTE = 0x36;
static const int8_t INJECTION_VALVE_STATUS_HEADER_BYTE = 0x38;
static const int8_t LOWER_VALVE_STATUS_HEADER_BYTE = 0x39;

#define IMU_SERIAL_MSG_SIZE (41)
#define BAROMETER_SERIAL_MSG_SIZE (13)
#define GPS_SERIAL_MSG_SIZE (21)
#define OXIDIZER_TANK_SERIAL_MSG_SIZE (9)
#define COMBUSTION_CHAMBER_SERIAL_MSG_SIZE (9)
#define FLIGHT_PHASE_SERIAL_MSG_SIZE (6)

static const uint8_t UART_TIMEOUT = 100;

void transmitImuData(AllData* data)
{
    int32_t accelX = -1;
    int32_t accelY = -1;
    int32_t accelZ = -1;
    int32_t gyroX = -1;
    int32_t gyroY = -1;
    int32_t gyroZ = -1;
    int32_t magnetoX = -1;
    int32_t magnetoY = -1;
    int32_t magnetoZ = -1;

    if (osMutexWait(data->accelGyroMagnetismData_->mutex_, 0) == osOK)
    {
        accelX = data->accelGyroMagnetismData_->accelX_;
        accelY = data->accelGyroMagnetismData_->accelY_;
        accelZ = data->accelGyroMagnetismData_->accelZ_;
        gyroX = data->accelGyroMagnetismData_->gyroX_;
        gyroY = data->accelGyroMagnetismData_->gyroY_;
        gyroZ = data->accelGyroMagnetismData_->gyroZ_;
        magnetoX = data->accelGyroMagnetismData_->magnetoX_;
        magnetoY = data->accelGyroMagnetismData_->magnetoY_;
        magnetoZ = data->accelGyroMagnetismData_->magnetoZ_;
        osMutexRelease(data->accelGyroMagnetismData_->mutex_);
    }

    uint8_t buffer[IMU_SERIAL_MSG_SIZE] = {0};

    buffer[0] = IMU_HEADER_BYTE;
    buffer[1] = IMU_HEADER_BYTE;
    buffer[2] = IMU_HEADER_BYTE;
    buffer[3] = IMU_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, accelX);
    writeInt32ToArray(&buffer, 8, accelY);
    writeInt32ToArray(&buffer, 12, accelZ);
    writeInt32ToArray(&buffer, 16, gyroX);
    writeInt32ToArray(&buffer, 20, gyroY);
    writeInt32ToArray(&buffer, 24, gyroZ);
    writeInt32ToArray(&buffer, 28, magnetoX);
    writeInt32ToArray(&buffer, 32, magnetoY);
    writeInt32ToArray(&buffer, 36, magnetoZ);
    buffer[IMU_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitBarometerData(AllData* data)
{
    int32_t pressure = -1;
    int32_t temperature = -1;

    if (osMutexWait(data->barometerData_->mutex_, 0) == osOK)
    {
        pressure = data->barometerData_->pressure_;
        temperature = data->barometerData_->temperature_;
        osMutexRelease(data->barometerData_->mutex_);
    }

    uint8_t buffer[BAROMETER_SERIAL_MSG_SIZE] = {0};

    buffer[0] = BAROMETER_HEADER_BYTE;
    buffer[1] = BAROMETER_HEADER_BYTE;
    buffer[2] = BAROMETER_HEADER_BYTE;
    buffer[3] = BAROMETER_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, pressure);
    writeInt32ToArray(&buffer, 8, temperature);
    buffer[BAROMETER_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);	// Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitGpsData(AllData* data)
{
    int32_t altitude = -1;
    int32_t epochTimeMsec = -1;
    int32_t latitude = -1;
    int32_t longitude = -1;

    if (osMutexWait(data->gpsData_->mutex_, 0) == osOK)
    {
        altitude = data->gpsData_->altitude_;
        epochTimeMsec = data->gpsData_->epochTimeMsec_;
        latitude = data->gpsData_->latitude_;
        longitude = data->gpsData_->longitude_;
        osMutexRelease(data->gpsData_->mutex_);
    }

    uint8_t buffer[GPS_SERIAL_MSG_SIZE] = {0};

    buffer[0] = GPS_HEADER_BYTE;
    buffer[1] = GPS_HEADER_BYTE;
    buffer[2] = GPS_HEADER_BYTE;
    buffer[3] = GPS_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, altitude);
    writeInt32ToArray(&buffer, 8, epochTimeMsec);
    writeInt32ToArray(&buffer, 12, latitude);
    writeInt32ToArray(&buffer, 16, longitude);
    buffer[GPS_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitOxidizerTankData(AllData* data)
{
    int32_t oxidizerTankPressure = -1;

    if (osMutexWait(data->oxidizerTankPressureData_->mutex_, 0) == osOK)
    {
        oxidizerTankPressure = data->oxidizerTankPressureData_->pressure_;
        osMutexRelease(data->oxidizerTankPressureData_->mutex_);
    }

    uint8_t buffer[OXIDIZER_TANK_SERIAL_MSG_SIZE] = {0};

    buffer[0] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[1] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[2] = OXIDIZER_TANK_HEADER_BYTE;
    buffer[3] = OXIDIZER_TANK_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, oxidizerTankPressure);
    buffer[OXIDIZER_TANK_SERIAL_MSG_SIZE - 1] = 0x00;


    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitCombustionChamberData(AllData* data)
{
    int32_t combustionChamberPressure = -1;

    if (osMutexWait(data->combustionChamberPressureData_->mutex_, 0) == osOK)
    {
        combustionChamberPressure = data->combustionChamberPressureData_->pressure_;
        osMutexRelease(data->combustionChamberPressureData_->mutex_);
    }

    uint8_t buffer[COMBUSTION_CHAMBER_SERIAL_MSG_SIZE] = {0};

    buffer[0] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[1] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[2] = COMBUSTION_CHAMBER_HEADER_BYTE;
    buffer[3] = COMBUSTION_CHAMBER_HEADER_BYTE;
    writeInt32ToArray(&buffer, 4, combustionChamberPressure);
    buffer[COMBUSTION_CHAMBER_SERIAL_MSG_SIZE - 1] = 0x00;

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitFlightPhaseData(AllData* data)
{
    uint8_t flightPhase = getCurrentFlightPhase();

    uint8_t buffer [] = {FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         FLIGHT_PHASE_HEADER_BYTE,
                         flightPhase,
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);  // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);	// Radio
}

void transmitInjectionValveStatus()
{
    uint8_t injectionValveStatus = injectionValveIsOpen;

    uint8_t buffer [] = {INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         INJECTION_VALVE_STATUS_HEADER_BYTE,
                         (uint8_t) ((injectionValveStatus)),
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
}

void transmitLowerVentValveStatus()
{
    uint8_t ventValveStatus = lowerVentValveIsOpen;

    uint8_t buffer [] = {LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         LOWER_VALVE_STATUS_HEADER_BYTE,
                         (uint8_t) ((ventValveStatus)),
                         0x00
                        };

    if ((getCurrentFlightPhase() == PRELAUNCH) || (getCurrentFlightPhase() == ARM) || (getCurrentFlightPhase() == BURN) || (IS_ABORT_PHASE))
    {
        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    }

    HAL_UART_Transmit(&huart1, &buffer, sizeof(buffer), UART_TIMEOUT);  // Radio
}

static uint8_t is_ready = 1;
void transmitStringTest()
{

    char buffer[] = "This is a really long string. Sending this will be complicated!\r\n";

    if (is_ready == 1) {
    	is_ready = 0;
    	HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT); // Ground Systems
    	is_ready = 1;
    }

}





//Struct to store all the variables of a specific HAL call
typedef struct
{

	//Keep track of what kind of element it is
	char type; // a: transmit, b: togglePin, c: writePin, d: transmitReceive, e: Receive

	//HAL Transmit, Transmit Receive and Receive variables
	uint16_t Size;
	uint32_t Timeout;

	UART_HandleTypeDef *huart;
	uint8_t *pData;

	SPI_HandleTypeDef *hspi;
	uint8_t *pTxData;
	uint8_t *pRxData;


	//HAL TogglePin and WritePin Variables
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	GPIO_PinState PinState;

	//struct for the next node in the list
	struct HALCall *next;

} HALCall;

//if 0 will not transmit any data, will just add any replaced HAL calls to the linked List
static int transmitRealData = 0;
//Start and end of the linked list
static HALCall* listHead;
static HALCall* listTail;

//Print all elements of the linked list
//Will have to change how the data is sent/printed once the python script works
void printLinkedList()
{
	char msg[] = "\r\n\r\nList Currently Contains:\r\n";
	HAL_UART_Transmit(&huart2, &msg, sizeof(msg), UART_TIMEOUT);

	HALCall *listItem = listHead;
	int i = 0;
	while(listItem != NULL)
	{

		printListItem(listItem);

		listItem = listItem->next;

		i++;
	}

	char data[] = "End of the list\r\n\r\n";
	HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);
}

//Return the address of the first item from the list and remove it, will have to de alocate the memory elsewhere
HALCall* getFirstListItem()
{
	HALCall* firstItem = listHead;
	listHead = firstItem->next;
	return firstItem;
}

//Print the data contained within the specific list item
void printListItem(HALCall* listItem)
{

	//if Transmit Call Print data
	if (listItem->type == 'a')
	{
		char data[] = "HAL Transmit Call\r\nData Sent: ";
		HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);
		HAL_UART_Transmit(&huart2, listItem->pData, listItem->Size, UART_TIMEOUT);
		char newLine[] = "\r\n\r\n";
		HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), UART_TIMEOUT);
	}

	//if TogglePin print the pin to toggle
	else if (listItem->type == 'b')
	{
		char data[] = "HAL Toggle Pin Call\r\nPin Toggled: ";
		HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);
		HAL_UART_Transmit(&huart2, listItem->GPIO_Pin, sizeof(listItem->GPIO_Pin), UART_TIMEOUT);
		char newLine[] = "\r\n\r\n";
		HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), UART_TIMEOUT);
	}

	//if WritePin print the pin and write value
	else if (listItem->type == 'c')
	{
		char data[] = "HAL Write Pin Call\r\nPin Written To: ";
		HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);
		HAL_UART_Transmit(&huart2, listItem->GPIO_Pin, sizeof(listItem->GPIO_Pin), UART_TIMEOUT);
		char newLine[] = "\r\n\r\n";
		HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), UART_TIMEOUT);
	}

	//if TransmitReceive print data sent out
	else if (listItem->type == 'd')
	{
		char data[] = "HAL Transmit Receive Call\r\nData Sent: ";
		HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);
		HAL_UART_Transmit(&huart2, listItem->pTxData, listItem->Size, UART_TIMEOUT);
		char newLine[] = "\r\n\r\n";
		HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), UART_TIMEOUT);
	}

	//if Receive call not going to print anything
	else if (listItem->type == 'e')
	{
		char data[] = "HAL Receive Call\r\n";
		HAL_UART_Transmit(&huart2, &data, sizeof(data), UART_TIMEOUT);

		//HAL_UART_Transmit(&huart2, listItem->pData, listItem->Size, UART_TIMEOUT);

		char newLine[] = "\r\n\r\n";
		HAL_UART_Transmit(&huart2, &newLine, sizeof(newLine), UART_TIMEOUT);
	}
}

//Enters the struct into the linked list
void enterIntoList(HALCall* listItem)
{
	if(listHead == NULL)
	{
		listHead = listItem;
		listTail = listItem;
	}
	else
	{
		listTail->next = listItem;
		listTail = listItem;
	}
}

//Copy the from string to the to string
void copyString(uint8_t **to, uint8_t *from, uint16_t Size)
{
	*(to) = (uint8_t*) malloc(Size);

	for (int i = 0; i < Size; i++)
	{
		(*(to))[i] = from[i];
	}
}

//Create a struct for a Hall Transmit
void createHALTransmitCall(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	//Dynamic memory allocation
	HALCall *newCall = NULL;
	newCall = (HALCall*) malloc(sizeof(HALCall));
	newCall->type = 'a';
	newCall->huart = huart;

	copyString(&(newCall->pData), pData, Size);

	newCall->Size = Size;
	newCall->Timeout = Timeout;
	newCall->next = NULL;

	enterIntoList(newCall);

}

//Replaces the HAL transmit function
static inline void HALTransmitReplacement(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{

    if (transmitRealData)
    {
    	HAL_UART_Transmit(huart, pData, Size, Timeout); // Transmit the actual data
    }
    else
    {
    	createHALTransmitCall(huart, pData, Size, Timeout); //Create struct and add it to the linked list

    	printLinkedList(); //Print the linked list as a test
    }

}

//Create a struct for a Hall TogglePin
void createHALTogglePinCall(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	//Dynamic memory allocation
	HALCall *newCall = NULL;
	newCall = (HALCall*) malloc(sizeof(HALCall));
	newCall->type = 'b';
	newCall->GPIOx = GPIOx;
	newCall->GPIO_Pin = GPIO_Pin;
	newCall->next = NULL;

	enterIntoList(newCall);

}

//Replaces HAL togglePin
static inline void HALTogglePinReplacement(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{

    if (transmitRealData)
    {
    	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin); // Toggle pin as normal
    }
    else
    {
    	createHALTogglePinCall(GPIOx, GPIO_Pin); //Create a struct on the linked list
    }

}

//Create a struct for a Hall write Pin
void createHALWritePinCall(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	//Dynamic memory allocation
	HALCall *newCall = NULL;
	newCall = (HALCall*) malloc(sizeof(HALCall));
	newCall->type = 'b';
	newCall->GPIOx = GPIOx;
	newCall->GPIO_Pin = GPIO_Pin;
	newCall->PinState = PinState;
	newCall->next = NULL;

	enterIntoList(newCall);

}

//Replaces HAL write Pin
static inline void HALWritePinReplacement(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{

    if (transmitRealData)
    {
    	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState); // Toggle pin as normal
    }
    else
    {
    	createHALWritePinCall(GPIOx, GPIO_Pin, PinState); //Create a struct in the linked list

    }

}

//Create a struct for a Hall TransmitReceive
void createHALTransmitReceiveCall(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	//Dynamic memory allocation
	HALCall *newCall = NULL;
	newCall = (HALCall*) malloc(sizeof(HALCall));
	newCall->type = 'd';
	newCall->hspi = hspi;

	copyString(&(newCall->pTxData), pTxData, Size);
	copyString(&(newCall->pRxData), pRxData, Size); //Might need to double check if both are the same size

	newCall->Size = Size;
	newCall->Timeout = Timeout;
	newCall->next = NULL;

	enterIntoList(newCall);

	}

//Replaces the HAL transmitReceive function
static inline void HALTransmitReceiveReplacement(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{

	if (transmitRealData)
	{
		HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout); // Transmit the actual data
	}
	else
	{
		createHALTransmitReceiveCall(hspi, pTxData, pRxData, Size, Timeout); //Create a struct in the linked list
	}

}

//Create a struct for a Hall TransmitReceive
void createHALReceiveCall(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	//Dynamic memory allocation
	HALCall *newCall = NULL;
	newCall = (HALCall*) malloc(sizeof(HALCall));
	newCall->type = 'e';
	newCall->hspi = hspi;
	newCall->pData = pData;
	newCall->Size = Size;
	newCall->Timeout = Timeout;
	newCall->next = NULL;

	enterIntoList(newCall);

}

//Replaces the HAL transmitReceive function
static inline void HALReceiveReplacement(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{

	if (transmitRealData)
	{
		HAL_SPI_Receive(hspi, pData, Size, Timeout); // Transmit the actual data
	}
	else
	{
		createHALReceiveCall(hspi, pData, Size, Timeout); //Create a struct in the linked list
	}

}

//#define HAL_UART_Transmit HALTransmitReplacement

void transmitDataTask(void const* arg)
{

	char welcomeMsg[] = " -- SOAR Andromeda Dev Console -- \r\n> ";
	HAL_UART_Transmit(&huart2, &welcomeMsg, sizeof(welcomeMsg), UART_TIMEOUT);

	char rxChar = 0;
	unsigned char cmdSize = 0;
	char cmdStr[258];
	char* cmd = &cmdStr[2];
	cmdStr[0] = '\r';
	cmdStr[1] = '\n';
	cmd[0] = 0;

	while (1) {

		HAL_UART_Receive(&huart2, &rxChar, 1, 30000);
		if (rxChar == '\n' || rxChar == '\r') {
			cmd[cmdSize++] = '\r';
			cmd[cmdSize++] = '\n';
			cmd[cmdSize++] = '>';
			cmd[cmdSize++] = ' ';
			cmd[cmdSize] = 0;

			//HAL_UART_Transmit(&huart2, &cmdStr, cmdSize + 2, UART_TIMEOUT);
			HALTransmitReplacement(&huart2, &cmdStr, cmdSize + 2, UART_TIMEOUT); //Replaced Transmit with transmit replacement

			cmdSize = 0;
			cmd[0] = 0;
		} else if (rxChar == 127) {
			cmd[--cmdSize] = 0;
			HAL_UART_Transmit(&huart2, 0x8, 1, UART_TIMEOUT);
			rxChar = 0;
		} else if (rxChar != 0) {
			cmd[cmdSize++] = rxChar;
			cmd[cmdSize] = 0;
			HAL_UART_Transmit(&huart2, &rxChar, 1, UART_TIMEOUT);
			rxChar = 0;
		}


	}

//    AllData* data = (AllData*) arg;
//    uint32_t prevWakeTime = osKernelSysTick();
//
//    for (;;)
//    {
//        osDelayUntil(&prevWakeTime, 2000);
//
//        transmitImuData(data);
//        transmitBarometerData(data);
//        transmitGpsData(data);
//        transmitOxidizerTankData(data);
//        transmitCombustionChamberData(data);
//        transmitFlightPhaseData(data);
//        transmitInjectionValveStatus();
//        transmitLowerVentValveStatus();
//
//        HAL_UART_Receive_IT(&huart2, &launchSystemsRxChar, 1);
//
//        char buffer[] = "\r\nHAL is alive!\r\n";
//        HAL_UART_Transmit(&huart2, &buffer, sizeof(buffer), UART_TIMEOUT);
//
//        HAL_UART_Transmit(&huart2, &launchSystemsRxChar, 1, UART_TIMEOUT);
//
//    }

}






