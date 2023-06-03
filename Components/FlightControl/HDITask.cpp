/**
 ******************************************************************************
 * File Name          : HDITask.cpp
 * Description        : Primary flight task, default task for the system.
 ******************************************************************************
*/
#include "HDITask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "RocketSM.hpp"
#include "FlightTask.hpp"
#include <map>
#include "Command.hpp"



/**
 * @brief Constructor for FlightTask
 */

Command variable;

struct BLINK{
	uint8_t numBlinks;
	uint16_t delayMs;
};

std::map <RocketState, BLINK> stateBlinks = {
  {RS_PRELAUNCH, {1, 1000}},
  {RS_ARM, {2, 1000}},
  {RS_LAUNCH, {3, 500}},
  {RS_DESCENT, {10, 5000}}
};

extern TIM_HandleTypeDef htim2;

HDITask::HDITask():Task(HDI_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the FlightTask
 */
void HDITask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)HDITask::RunTask,
            (const char*)"HDITask",
            (uint16_t)HDI_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)HDI_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "HDITask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Flight Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void HDITask::Run(void * pvParams)
{
	// NOTE: refactor out init

    uint32_t tempSecondCounter = 0; // TODO: Temporary counter, would normally be in HeartBeat task or HID Task, unless FlightTask is the HeartBeat task

    //uint8_t value = 0;
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


    GPIO::LED1::Off();

    while (1) {
    	//Every cycle, print something out (for testing)

    	BLINK blinksToDo = {.numBlinks = 3, .delayMs = 5000};
//    	if(qEvtQueue->GetQueueMessageCount() > 0){
////    		Command cm;
////    		bool rxed = qEvtQueue->Receive(cm);
////
////    		if(rxed)
////    		{
////
////    		}
////
////    		cm.Reset();
//    	}
//    	else{
//    		RocketState currentHDIState = FlightTask::Inst().GetCurrentState();
//    		blinksToDo = stateBlinks[currentHDIState];
//    	}

    	for (uint8_t i = 0; i < blinksToDo.numBlinks; i++) {
    		GPIO::LED1::On();
//    		uint8_t value = 0; // the value for the duty cycle
//    		while (value<210)
//    		{
//    		  htim2.Instance->CCR1 = value; // vary the duty cycle
//    		  value += 20; // increase the duty cycle by 20
//    		  osDelay (500); // wait for 500 ms
//    		}
//    		value = 0;
    		SOAR_PRINT("LED PLS");
    	      osDelay(blinksToDo.delayMs);

    	      GPIO::LED1::Off();
    	      SOAR_PRINT("LED OFF");
    	      //BUZZER_OFF();
    	      osDelay(blinksToDo.delayMs);
    	    }

    		osDelay(20000);

    	  }

//    	RocketState currentHDIState = FlightTask::Inst().GetCurrentState();
//
//        switch(currentHDIState){
//        case RS_PRELAUNCH:
//        	while(value < 210){
//			  htim2.Instance -> CCR1 = value; // duty cycle
//			  value += 20;
//			  HAL_Delay (500);
//        	}
//
//        	//pattern build for buzzer
//        	value = 0;
//			HAL_GPIO_WritePin(GPIOC, HDI_PWM_BUZZER_Pin, GPIO_PIN_SET);
//
//			// 2 Hz - blink twice
//			GPIO::LED1::On();
//			osDelay(500);
//			GPIO::LED1::Off();
//			osDelay(500);
//			break;
//
//		case RS_ABORT:
//			GPIO::LED2::On();
//			//HAL_GPIO_WritePin(GPIOC, HDI_PWM_BUZZER, GPIO_PIN_RESET);
//
//			osDelay(500);
//			GPIO::LED2::Off();
//
//			osDelay(500);
//			break;
//
////		case RS_OPEN_VENT:
////			GPIO::LED3::On();
////
////			osDelay(500);
////			GPIO::LED3::Off();
////
////			osDelay(500);
////			break;
////
////		case RS_CLOSE_VENT:
////			GPIO::LED2::On();
////
////			osDelay(500);
////			GPIO::LED2::Off();
////
////			osDelay(500);
////			break;
////
////		case RS_OPEN_DRAIN:
////			GPIO::LED2::On();
////
////			osDelay(500);
////			GPIO::LED2::Off();
////
////			osDelay(500);
////			break;
////
////		case RS_CLOSE_DRAIN:
////			GPIO::LED2::On();
////
////			osDelay(500);
////			GPIO::LED2::Off();
////
////			osDelay(500);
////			break;
//
//		default:
//
//			GPIO::LED1::Off();
//			GPIO::LED2::Off();
//			GPIO::LED3::Off();
//
//			HAL_GPIO_WritePin(GPIOC, HDI_PWM_BUZZER_Pin, GPIO_PIN_RESET);
//
//
//		break;
//	}


//    }
}
