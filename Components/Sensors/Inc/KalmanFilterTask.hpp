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


	    // Data
	    AccelGyroMagnetismData* data;


};

#endif
