/**
  ******************************************************************************
  * File Name          : KalmanFilter.cpp
  *
  *Author			   : Andrey Dimanchev
  * Description        : Filters and fuses sensor data based on a Kalman Filter
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "KalmanFilterTask.hpp"

#include "main.h"
#include "Data.h"
#include "Task.hpp"
#include <string.h>

/* Constants -----------------------------------------------------------------*/
/* Values should not be modified, non-const due to HAL and C++ strictness) ---*/
constexpr int CMD_TIMEOUT = 150;

#define READ_CMD_MASK 0x80
#define WRITE_CMD_MASK 0x00
#define ACCEL_GYRO_MASK 0x00
#define MAGNETO_MASK 0x40

// Register addresses
#define G1_CTRL_REGISTER_ADDR 0x10 // CTRL_REG1_G (10h)
#define XL6_CTRL_REGISTER_ADDR 0x20 // CTRL_REG6_XL (20h)
#define M3_CTRL_REGISTER_ADDR 0x22 // CTRL_REG3_M (22h)
#define WHOAMI_REGISTER_ADDR 0x0F  // WHO_AM_I_A/G (Accel/Gyro - expected value is 104)
#define WHOAMIM_REGISTER_ADDR 0x0F // WHO_AM_I_M (Magnetometer - expected value is 61)

#define GYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCEL_X_LOW_REGISTER_ADDR 0x28
#define MAGNETO_X_LOW_REGISTER_ADDR 0x28

#define ACCEL_SENSITIVITY 0.732 // Unit is mg/LSB
#define GYRO_SENSITIVITY 8.75  // Unit is mdps/LSB
#define MAGENTO_SENSITIVITY 0.14 // Unit is mgauss/LSB

// Full Commands
static uint8_t ACTIVATE_GYRO_ACCEL_CMD = G1_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 00 0 00 -> ODR 119, 245 DPS
static uint8_t ACTIVATE_GYRO_ACCEL_DATA = 0x60;   // ODR 119 has cutoff of 38Hz

static uint8_t SET_ACCEL_SCALE_CMD = XL6_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 01 0 00 -> ODR 119, +/- 16G
static uint8_t SET_ACCEL_SCALE_DATA = 0x68;

static uint8_t ACTIVATE_MAGNETO_CMD = M3_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 1 0 0 00 0 00 -> I2C Disable, Low power mode disabled, SPI write enable, Continuous-conversion mode
static uint8_t ACTIVATE_MAGNETO_DATA = 0x80;

static uint8_t READ_GYRO_X_G_LOW_CMD = GYRO_X_G_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_MAGNETO_X_LOW_CMD = MAGNETO_X_LOW_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;
static uint8_t READ_WHOAMI_CMD = WHOAMI_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static uint8_t READ_WHOAMIM_CMD = WHOAMIM_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;

KalmanFilterTask::KalmanFilterTask(){
	data = (AccelGyroMagnetismData*)soar_malloc(sizeof(AccelGyroMagnetismData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void KalmanFilterTask::InitTask() : Task(TASK_KALMAN_FILTER_QUEUE_DEPTH_OBJS)
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Kalman Filter task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)KalmanFilterTask::RunTask,
            (const char*)"KalmanFilterTask",
            (uint16_t)TASK_KALMAN_FILTER_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_KALMAN_FILTER_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "KalmanFilterTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief KalmanFilterTask run loop
 * @param pvParams Currently unused task context
 */
void KalmanFilterTask::Run(void* pvParams)
{
    //Delay before IMU init
    osDelay(100);

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
void IMUTask::HandleCommand(Command& cm)
{
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
        SOAR_PRINT("IMUTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void IMUTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case IMU_REQUEST_NEW_SAMPLE:
        SampleIMU();
        break;

    default:
        SOAR_PRINT("IMUTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

void KalmanFilterTask::ReadSensors(){
	//READ------------------------------------------------------
	    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
	    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
	    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);
	    gyroX = (dataBuffer[1] << 8) | (dataBuffer[0]);
	    gyroY = (dataBuffer[3] << 8) | (dataBuffer[2]);
	    gyroZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

	    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_ACCEL_X_LOW_CMD, 1, CMD_TIMEOUT);
	    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
	    HAL_GPIO_WritePin(IMU_XL_GY_CS_GPIO_Port, IMU_XL_GY_CS_Pin, GPIO_PIN_SET);
	    accelX = (dataBuffer[1] << 8) | (dataBuffer[0]);
	    accelY = (dataBuffer[3] << 8) | (dataBuffer[2]);
	    accelZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

	    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_RESET);
	    HAL_SPI_Transmit(SystemHandles::SPI_IMU, &READ_MAGNETO_X_LOW_CMD, 1, CMD_TIMEOUT);
	    HAL_SPI_Receive(SystemHandles::SPI_IMU, &dataBuffer[0], 6, CMD_TIMEOUT);
	    HAL_GPIO_WritePin(IMU_MAG_CS_GPIO_Port, IMU_MAG_CS_Pin, GPIO_PIN_SET);
	    magnetoX = (dataBuffer[1] << 8) | (dataBuffer[0]);
	    magnetoY = (dataBuffer[3] << 8) | (dataBuffer[2]);
	    magnetoZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

	    // Write to storage
	    data->accelX_ = accelX * ACCEL_SENSITIVITY; // mg
	    data->accelY_ = accelY * ACCEL_SENSITIVITY; // mg
	    data->accelZ_ = accelZ * ACCEL_SENSITIVITY; // mg
	    data->gyroX_ = gyroX * GYRO_SENSITIVITY; // mdps
	    data->gyroY_ = gyroY * GYRO_SENSITIVITY; // mdps
	    data->gyroZ_ = gyroZ * GYRO_SENSITIVITY; // mdps
	    data->magnetoX_ = magnetoX * MAGENTO_SENSITIVITY; // mgauss
	    data->magnetoY_ = magnetoY * MAGENTO_SENSITIVITY; // mgauss
	    data->magnetoZ_ = magnetoZ * MAGENTO_SENSITIVITY; // mgauss

}
