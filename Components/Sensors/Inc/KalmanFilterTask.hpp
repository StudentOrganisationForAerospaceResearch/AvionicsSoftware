/**
 ******************************************************************************
 * File Name          : KalmanFilterTask.hpp
 * Description        :
 ******************************************************************************
*/
#ifndef SOAR_SENSOR_KALMAN_FILTER_TASK_HPP_
#define SOAR_SENSOR_KALMAN_FILTER_TASK_HPP_
/* Includes ------------------------------------------------------------------*/
#include "Task.hpp"
#include "Data.h"
#include "SystemDefines.hpp"

/* Macros/Enums ------------------------------------------------------------*/
enum KALMAN_TASK_COMMANDS {
    KALMAN_NONE = 0,
    READ_NEW_SAMPLE,// Get a new IMU sample, task will be blocked for polling time
};


/* Class ------------------------------------------------------------------*/
class KalmanFilterTask : public Task
{
	public:
		static KalmanFilterTask& Inst() {
			static KalmanFilterTask inst;
			return inst;
		}
		void InitTask();
	protected:
	    static void RunTask(void* pvParams) { KalmanFilterTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

	    void Run(void* pvParams);    // Main run code

	    void HandleCommand(Command& cm);
	    void HandleRequestCommand(uint16_t taskCommand);

	    void ReadSensors();

	    void Test();



	    // Data
	    AccelGyroMagnetismData* data;

	private:
	    KalmanFilterTask();                                        // Private constructor
	    KalmanFilterTask(const KalmanFilterTask&);                    // Prevent copy-construction
	    KalmanFilterTask& operator=(const KalmanFilterTask&);            // Prevent assignment



};

#endif
