/**
 ********************************************************************************
 * @file    FSBProtocolTask.cpp
 * @author  jaddina
 * @date    Sep 13, 2025
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "FSBTest.hpp"
#include "SystemDefines.hpp"
#include "Command.hpp"
#include "SensorData.h"
#include "DataBroker.hpp"
#include "Task.hpp"
#include "WriteBufferFixedSize.h"
#include "ReadBufferFixedSize.h"
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
FSBProtocolTask::FSBProtocolTask():Task(TASK_FSB_PROTOCOL_DEPTH_OBJS)
{

}

/**
 * @brief Initialize the FSBProtocolTask
 *        Do not modify this function aside from adding the task name
 */
void FSBProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize watchdog task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)FSBProtocolTask::RunTask,
            (const char*)"FSBProtocolTask",
            (uint16_t)TASK_FSB_PROTOCOL_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_FSB_PROTOCOL_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

                SOAR_ASSERT(rtValue == pdPASS, "FSBProtocolTask::InitTask() - xTaskCreate() failed");
}

void FSBProtocolTask::Run(void * pvParams){

    while (1) {
        /* Process commands in blocking mode */
        Command cm;
        bool res = qEvtQueue->ReceiveWait(cm);
        if(res){

        	HandleCommand(cm);
        }
    }
}

void FSBProtocolTask::HandleCommand(Command& cm){


	switch(cm.GetTaskCommand()){

		case PUBLISH_PRESSURE:
		{

			PressureData pd;
			pd.pressure = 140;
			SOAR_PRINT("Pressure published");

			DataBroker::Publish<PressureData>(&pd);
			break;
		}

		case PUBLISH_IMU:
		{
			//serialize
			EmbeddedProto::WriteBufferFixedSize<64> buffer;

			Proto :: ImuSixAxis imu_msg;

			Proto :: Accelerometer accel;
			accel.set_accel_x(100);
			accel.set_accel_y(150);
			accel.set_accel_z(200);
			imu_msg.set_accelerometer(accel);

			Proto :: Gyroscope gyro;
			gyro.set_gyro_x(1);
			gyro.set_gyro_y(2);
			gyro.set_gyro_z(3);
			imu_msg.set_gyroscope(gyro);


			Proto :: SensorLoggingRate rate;
			rate.set_newSensorLoggingRate(100);
			imu_msg.set_sensorLoggingRate(rate);



			EmbeddedProto::Error err = imu_msg.serialize(buffer);

			//deserialize
			EmbeddedProto::ReadBufferFixedSize<64> read_buffer;
			memcpy(read_buffer.get_data(), buffer.get_data(), buffer.get_size());
			read_buffer.set_bytes_written(buffer.get_size());

			Proto::ImuSixAxis imu_received;
			err = imu_received.deserialize(read_buffer);
			if(err == EmbeddedProto::Error::NO_ERRORS)
			{
				SOAR_PRINT("Deserialized IMU message:\n");
				SOAR_PRINT("Accel X: %d\n", imu_received.get_accelerometer().get_accel_x());
				SOAR_PRINT("Accel Y: %d\n", imu_received.get_accelerometer().get_accel_y());
				SOAR_PRINT("Accel Z: %d\n", imu_received.get_accelerometer().get_accel_z());

				SOAR_PRINT("Gyro X: %d\n", imu_received.get_gyroscope().get_gyro_x());
				SOAR_PRINT("Gyro Y: %d\n", imu_received.get_gyroscope().get_gyro_y());
				SOAR_PRINT("Gyro Z: %d\n", imu_received.get_gyroscope().get_gyro_z());

				SOAR_PRINT("Logging Rate: %d", imu_received.get_sensorLoggingRate().get_newSensorLoggingRate());
			}
			else
			{
				SOAR_PRINT("Deserialization failed: %d", static_cast<int>(err));
			}

			AccelerometerData acceleration = {
				(uint32_t)imu_received.get_accelerometer().get_accel_x(),
				(uint32_t)imu_received.get_accelerometer().get_accel_y(),
				(uint32_t)imu_received.get_accelerometer().get_accel_z()
			};






			SOAR_PRINT("IMU published");
			DataBroker::Publish<AccelerometerData>(&acceleration);
			break;
		}

		case PUBLISH_THERMOCOUPLE:
		{

			ThermocoupleData tc;
			tc.temperature = 150;
			SOAR_PRINT("Temperature published");

			DataBroker::Publish<ThermocoupleData>(&tc);
			break;
		}
	}

}


