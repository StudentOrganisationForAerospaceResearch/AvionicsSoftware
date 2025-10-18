/**
 ********************************************************************************
 * @file    SensorDataTypes.hpp
 * @author  Shivam Desai
 * @date    Nov 3, 2024
 * @brief   General sensor data structure to pass around in the system or
 *log it to flash memory
 ********************************************************************************
 */

#ifndef SENSORDATATYPES_HPP_
#define SENSORDATATYPES_HPP_

#include <stdint.h>

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/**
 * @param accelX The acceleration in the X axis relative to the sensor
 * @param accelY The acceleration in the Y axis relative to the sensor
 * @param accelZ The acceleration in the Z axis relative to the sensor
 */
struct AccelerometerData {
  uint32_t accelX;
  uint32_t accelY;
  uint32_t accelZ;
};

struct GyroscopeData {
  uint32_t gyroX;
  uint32_t gyroY;
  uint32_t gyroZ;
};
/**
 * @param Temperature. Can be any where from -2147483648 to 2147483647
 */
struct ThermocoupleData {
  int32_t temperature;
};

struct PressureData {
  int32_t pressure;
};

#endif /* SENSORDATATYPES_HPP_ */
