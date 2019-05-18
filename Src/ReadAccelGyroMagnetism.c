#include "stm32f4xx.h"
#include "stm32f4xx_hal_conf.h"
#include "cmsis_os.h"

#include "ReadAccelGyroMagnetism.h"

#include "Data.h"

static int READ_ACCEL_GYRO_MAGNETISM = 25;

static const int CMD_TIMEOUT = 150;

#define READ_CMD_MASK 0x80
#define WRITE_CMD_MASK 0x00
#define ACCEL_GYRO_MASK 0x00
#define MAGNETO_MASK 0x40

// Register addresses
#define G1_CTRL_REGISTER_ADDR 0x10 // CTRL_REG1_G (10h)
#define XL6_CTRL_REGISTER_ADDR 0x20 // CTRL_REG6_XL (20h)
#define M3_CTRL_REGISTER_ADDR 0x22 // CTRL_REG3_M (22h)
// #define WHOAMI_REGISTER_ADDR 0x0F
// #define WHOAMIM_REGISTER_ADDR 0x0F

#define GYRO_X_G_LOW_REGISTER_ADDR 0x18
#define ACCEL_X_LOW_REGISTER_ADDR 0x28
#define MAGNETO_X_LOW_REGISTER_ADDR 0x28

#define ACCEL_SENSITIVITY 0.732 // Unit is mg/LSB
#define GYRO_SENSITIVITY 8.75  // Unit is mdps/LSB
#define MAGENTO_SENSITIVITY 0.14 // Unit is mgauss/LSB

// Full Commands
static const uint8_t ACTIVATE_GYRO_ACCEL_CMD = G1_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 00 0 00 -> ODR 119, 245 DPS
static const uint8_t ACTIVATE_GYRO_ACCEL_DATA = 0x60;   // ODR 119 has cutoff of 38Hz

static const uint8_t SET_ACCEL_SCALE_CMD = XL6_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 011 01 0 00 -> ODR 119, +/- 16G
static const uint8_t SET_ACCEL_SCALE_DATA = 0x68;

static const uint8_t ACTIVATE_MAGNETO_CMD = M3_CTRL_REGISTER_ADDR | WRITE_CMD_MASK;
// 1 0 0 00 0 00 -> I2C Disable, Low power mode disabled, SPI write enable, Continuous-conversion mode
static const uint8_t ACTIVATE_MAGNETO_DATA = 0x80;

static const uint8_t READ_GYRO_X_G_LOW_CMD = GYRO_X_G_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static const uint8_t READ_ACCEL_X_LOW_CMD = ACCEL_X_LOW_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
static const uint8_t READ_MAGNETO_X_LOW_CMD = MAGNETO_X_LOW_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;
// static const uint8_t READ_WHOAMI_CMD = WHOAMI_REGISTER_ADDR | READ_CMD_MASK | ACCEL_GYRO_MASK;
// static const uint8_t READ_WHOAMIM_CMD = WHOAMIM_REGISTER_ADDR | READ_CMD_MASK | MAGNETO_MASK;

void readAccelGyroMagnetismTask(void const* arg)
{
    AccelGyroMagnetismData* data = (AccelGyroMagnetismData*) arg;
    uint32_t prevWakeTime = osKernelSysTick();

    osDelay(1000);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_ACCEL_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_GYRO_ACCEL_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &SET_ACCEL_SCALE_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &SET_ACCEL_SCALE_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_MAGNETO_CMD, 1, CMD_TIMEOUT);
    HAL_SPI_Transmit(&hspi1, &ACTIVATE_MAGNETO_DATA, 1, CMD_TIMEOUT);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

    /* Read WHO AM I register for verification, should read 104. */
    // uint8_t whoami;
    // HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    // HAL_SPI_Transmit(&hspi1, &READ_WHOAMIM_CMD, 1, CMD_TIMEOUT);
    // HAL_SPI_Receive(&hspi1, &whoami, 1, CMD_TIMEOUT);
    // HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

    uint8_t dataBuffer[6];

    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t magnetoX, magnetoY, magnetoZ;

    for (;;)
    {
        osDelayUntil(&prevWakeTime, READ_ACCEL_GYRO_MAGNETISM);

        //READ------------------------------------------------------
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_GYRO_X_G_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &dataBuffer[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        gyroX = (dataBuffer[1] << 8) | (dataBuffer[0]);
        gyroY = (dataBuffer[3] << 8) | (dataBuffer[2]);
        gyroZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &READ_ACCEL_X_LOW_CMD, 1, CMD_TIMEOUT);
        HAL_SPI_Receive(&hspi1, &dataBuffer[0], 6, CMD_TIMEOUT);
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        accelX = (dataBuffer[1] << 8) | (dataBuffer[0]);
        accelY = (dataBuffer[3] << 8) | (dataBuffer[2]);
        accelZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

        // HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
        // HAL_SPI_Transmit(&hspi1, &READ_MAGNETO_X_LOW_CMD, 1, CMD_TIMEOUT);
        // HAL_SPI_Receive(&hspi1, &dataBuffer[0], 6, CMD_TIMEOUT);
        // HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
        // magnetoX = (dataBuffer[1] << 8) | (dataBuffer[0]);
        // magnetoY = (dataBuffer[3] << 8) | (dataBuffer[2]);
        // magnetoZ = (dataBuffer[5] << 8) | (dataBuffer[4]);

        /* Writeback */
        if (osMutexWait(data->mutex_, 0) != osOK)
        {
            continue;
        }

        data->accelX_ = accelX * ACCEL_SENSITIVITY; // mg
        data->accelY_ = accelY * ACCEL_SENSITIVITY; // mg
        data->accelZ_ = accelZ * ACCEL_SENSITIVITY; // mg
        data->gyroX_ = gyroX * GYRO_SENSITIVITY; // mdps
        data->gyroY_ = gyroY * GYRO_SENSITIVITY; // mdps
        data->gyroZ_ = gyroZ * GYRO_SENSITIVITY; // mdps
        // data->magnetoX_ = magnetoX * MAGENTO_SENSITIVITY; // mgauss
        // data->magnetoY_ = magnetoY * MAGENTO_SENSITIVITY; // mgauss
        // data->magnetoZ_ = magnetoZ * MAGENTO_SENSITIVITY; // mgauss
        osMutexRelease(data->mutex_);
    }
}
