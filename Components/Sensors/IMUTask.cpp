/**
  ******************************************************************************
  * File Name          : IMUTask.cpp
  *
  *    Source Info           : Based on Andromeda V3.31 Implementation
  *                         Andromeda_V3.31_Legacy/Core/Src/ReadAccelGyroMagnetism.c
  *
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the IMU on the flight board.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "IMUTask.hpp"

#include <string.h>
#include "DMBProtocolTask.hpp"
#include "Data.h"
#include "DebugTask.hpp"
#include "FlashTask.hpp"
#include "Task.hpp"
#include "main.h"

/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/
/* Values should not be modified, non-const due to HAL and C++ strictness) ---*/
constexpr int CMD_TIMEOUT = 150;

#define READ_CMD_MASK 0x80
#define WRITE_CMD_MASK 0x00
#define ACCEL_GYRO_MASK 0x00
#define MAGNETO_MASK 0x40

// Register addresses
#define G1_CTRL_REGISTER_ADDR 0x10   // CTRL_REG1_G (10h)
#define XL6_CTRL_REGISTER_ADDR 0x20  // CTRL_REG6_XL (20h)
#define M3_CTRL_REGISTER_ADDR 0x22   // CTRL_REG3_M (22h)
#define WHOAMI_REGISTER_ADDR \
    0x0F  // WHO_AM_I_A/G (Accel/Gyro - expected value is 104)
#define WHOAMIM_REGISTER_ADDR \
    0x0F  // WHO_AM_I_M (Magnetometer - expected value is 61)

#define GYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCEL_X_LOW_REGISTER_ADDR 0x28
#define MAGNETO_X_LOW_REGISTER_ADDR 0x28

#define ACCEL_SENSITIVITY 0.732   // Unit is mg/LSB
#define GYRO_SENSITIVITY 8.75     // Unit is mdps/LSB
#define MAGENTO_SENSITIVITY 0.14  // Unit is mgauss/LSB

// Full Commands
static uint8_t ACTIVATE_GYRO_ACCEL_CMD = G1_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 00 0 00 -> ODR 119, 245 DPS
static uint8_t ACTIVATE_GYRO_ACCEL_DATA = 0x60;  // ODR 119 has cutoff of 38Hz

static uint8_t SET_ACCEL_SCALE_CMD = XL6_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 01 0 00 -> ODR 119, +/- 16G
static uint8_t SET_ACCEL_SCALE_DATA = 0x68;

static uint8_t ACTIVATE_MAGNETO_CMD = M3_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 1 0 0 00 0 00 -> I2C Disable, Low power mode disabled, SPI write enable, Continuous-conversion mode
static uint8_t ACTIVATE_MAGNETO_DATA = 0x80;

static uint8_t READ_GYRO_X_G_LOW_CMD =
    GYRO_X_G_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_ACCEL_X_LOW_CMD =
    ACCEL_X_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_MAGNETO_X_LOW_CMD =
    MAGNETO_X_LOW_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;
static uint8_t READ_WHOAMI_CMD =
    WHOAMI_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_WHOAMIM_CMD =
    WHOAMIM_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Default constructor, sets and sets up storage for member variables
 */
IMUTask::IMUTask() : Task(TASK_IMU_QUEUE_DEPTH_OBJS) {
    data = (AccelGyroMagnetismData*)soar_malloc(sizeof(AccelGyroMagnetismData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void IMUTask::InitTask() {
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize IMU task twice");

    // Start the task
    BaseType_t rtValue = xTaskCreate(
        (TaskFunction_t)IMUTask::RunTask, (const char*)"IMUTask",
        (uint16_t)TASK_IMU_STACK_DEPTH_WORDS, (void*)this,
        (UBaseType_t)TASK_IMU_PRIORITY, (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS,
                "IMUTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief IMUTask run loop
 * @param pvParams Currently unused task context
 */
void IMUTask::Run(void* pvParams) {
    //Delay before IMU init
    osDelay(100);

    //Setup the IMU
    SetupIMU();

    //Task run loop
    while (1) {
        Command cm;

        //Wait forever for a command
        qEvtQueue->ReceiveWait(cm);

        //Process the command
        HandleCommand(cm);
    }
}

/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void IMUTask::HandleCommand(Command& cm) {
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
        case REQUEST_COMMAND: {
            HandleRequestCommand(cm.GetTaskCommand());
            break;
        }
        case TASK_SPECIFIC_COMMAND: {
            break;
        }
        default:
            SOAR_PRINT("IMUTask - Received Unsupported Command {%d}\n",
                       cm.GetCommand());
            break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void IMUTask::HandleRequestCommand(uint16_t taskCommand) {
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
        case IMU_REQUEST_NEW_SAMPLE:
            SampleIMU();
            break;
        case IMU_REQUEST_TRANSMIT:
            TransmitProtocolData();
            LogDataToFlash();
            break;
        case IMU_REQUEST_FLASH_LOG:
            LogDataToFlash();
            break;
        case IMU_REQUEST_DEBUG:
            SOAR_PRINT("\t-- IMU Data --\n");
            SOAR_PRINT(" Accel (x,y,z) : (%d, %d, %d) milli-Gs\n",
                       data->accelX_, data->accelY_, data->accelZ_);
            SOAR_PRINT(" Gyro (x,y,z)  : (%d, %d, %d) milli-deg/s\n",
                       data->gyroX_, data->gyroY_, data->gyroZ_);
            SOAR_PRINT(" Mag (x,y,z)   : (%d, %d, %d) milli-gauss\n",
                       data->magnetoX_, data->magnetoY_, data->magnetoZ_);
            break;
        default:
            SOAR_PRINT("IMUTask - Received Unsupported REQUEST_COMMAND {%d}\n",
                       taskCommand);
            break;
    }
}

/**
 * @brief Transmits protocol data
 */
void IMUTask::TransmitProtocolData() {
    // Transmits protocol data
    //SOAR_PRINT("IMU Task Transmit...\n");

    Proto::TelemetryMessage msg;
    msg.set_source(Proto::Node::NODE_DMB);
    msg.set_target(Proto::Node::NODE_RCU);
    msg.set_message_id((uint32_t)Proto::MessageID::MSG_TELEMETRY);
    Proto::IMU imuData;
    imuData.set_accelx(data->accelX_);
    imuData.set_accely(data->accelY_);
    imuData.set_accelz(data->accelZ_);
    imuData.set_gyrox(data->gyroX_);
    imuData.set_gyroy(data->gyroY_);
    imuData.set_gyroz(data->gyroZ_);
    imuData.set_magx(data->magnetoX_);
    imuData.set_magy(data->magnetoY_);
    imuData.set_magz(data->magnetoZ_);
    msg.set_imu(imuData);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>
        writeBuffer;
    msg.serialize(writeBuffer);

    // Send the barometer data
    DMBProtocolTask::SendProtobufMessage(writeBuffer,
                                         Proto::MessageID::MSG_TELEMETRY);
}

/**
 * @brief Logs the IMU data to flash
 */
void IMUTask::LogDataToFlash() {
    Command flashCommand(DATA_COMMAND, WRITE_DATA_TO_FLASH);
    flashCommand.CopyDataToCommand((uint8_t*)data,
                                   sizeof(AccelGyroMagnetismData));
    FlashTask::Inst().GetEventQueue()->Send(flashCommand);
}

/**
 * @brief This function reads and updates the IMU data
 */
void IMUTask::SampleIMU() {
    //Storable data
    uint8_t dataBuffer[6];
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t magnetoX, magnetoY, magnetoZ;

    data->time = TICKS_TO_MS(xTaskGetTickCount());  // ms

    //READ------------------------------------------------------
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_GYRO_X_G_LOW_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);
    gyroX = (dataBuffer[1] << 8) | (dataBuffer[0]);
    gyroY = (dataBuffer[3] << 8) | (dataBuffer[2]);
    gyroZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_ACCEL_X_LOW_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);
    accelX = (dataBuffer[1] << 8) | (dataBuffer[0]);
    accelY = (dataBuffer[3] << 8) | (dataBuffer[2]);
    accelZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_MAGNETO_X_LOW_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_SET);
    magnetoX = (dataBuffer[1] << 8) | (dataBuffer[0]);
    magnetoY = (dataBuffer[3] << 8) | (dataBuffer[2]);
    magnetoZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

    // Write to storage
    data->accelX_ = accelX * ACCEL_SENSITIVITY;        // mg
    data->accelY_ = accelY * ACCEL_SENSITIVITY;        // mg
    data->accelZ_ = accelZ * ACCEL_SENSITIVITY;        // mg
    data->gyroX_ = gyroX * GYRO_SENSITIVITY;           // mdps
    data->gyroY_ = gyroY * GYRO_SENSITIVITY;           // mdps
    data->gyroZ_ = gyroZ * GYRO_SENSITIVITY;           // mdps
    data->magnetoX_ = magnetoX * MAGENTO_SENSITIVITY;  // mgauss
    data->magnetoY_ = magnetoY * MAGENTO_SENSITIVITY;  // mgauss
    data->magnetoZ_ = magnetoZ * MAGENTO_SENSITIVITY;  // mgauss
}

/**
 * @brief Sets up IMU
 * @return WHOAMI_M register value if read
 */
uint8_t IMUTask::SetupIMU() {
    /* Setup the Accel / Gyro */
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_GYRO_X_G_LOW_CMD, 1,
                     CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &ACTIVATE_GYRO_ACCEL_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &ACTIVATE_GYRO_ACCEL_DATA, 1,
                     CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &SET_ACCEL_SCALE_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &SET_ACCEL_SCALE_DATA, 1,
                     CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);

    /* Read WHO AM I register for verification, should read 104. */
    uint8_t whoami = 0xDE;
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_WHOAMI_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Receive(SystemHandles::SPI_IMU, &whoami, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);

    /* Setup the Magnetometer */
    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &ACTIVATE_MAGNETO_CMD, 1,
                     CMD_TIMEOUT);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &ACTIVATE_MAGNETO_DATA, 1,
                     CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_SET);

    /* Read WHO AM I MAG register for verification, should read 61. */
    uint8_t whoami_m = 0xDE;
    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_WHOAMIM_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Receive(SystemHandles::SPI_IMU, &whoami_m, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_SET);

    /* Verify the two WHOAMI registers */
    if (whoami != 104)
        SOAR_PRINT("Non-Fatal-Warning: IMU WHOAMI failed   [%d]\n", whoami);
    if (whoami_m != 61)
        SOAR_PRINT("Non-Fatal-Warning: IMU WHOAMI_M failed [%d]\n", whoami_m);

    return whoami;
}
