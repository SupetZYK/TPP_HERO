#include "rtos_task.h"
#include "rtos_init.h"

#include "drivers_buzzer_low.h"
#include "drivers_imu_low.h"

//#include "utilities_iopool.h"
#include "drivers_led_low.h"
//#include "tasks_remotecontrol.h"
#include "tasks_upper.h"
#include "drivers_canmotor_low.h"
#include "tasks_motor.h"
#include "drivers_sonar_low.h"
#include "task_quaternion.h"
#include "application_remotecontrol.h"
//#include "drivers_mpu6050_low.h"
//#include "tasks_mpu6050.h"

//#include "tasks_testtasks.h"
#include "tasks_Hero.h"
#include "utilities_debug.h"

osThreadId ledGreenTaskHandle;
osThreadId ledRedTaskHandle;
//osThreadId ledStateTaskHandle;

osThreadId buzzerTaskHandle;
//IMU
osThreadId printIMUTaskHandle;
//UART
osThreadId getCtrlUartTaskHandle;
osThreadId RControlTaskHandle;
//Motor
osThreadId CMGMControlTaskHandle;
osThreadId AMControlTaskHandle;

osThreadId CMGMCanTransmitTaskHandle;
osThreadId AMCanTransmitTaskHandle;

osThreadId sonarTaskHandle;

osThreadId updateQuaternionTaskHandle;

osThreadId TimerTaskHandle;

//osThreadId EngineeringTaskHandle;

osThreadId HeroTaskHandle;
//osThreadId WaveTaskHandle;
//extern osThreadId testFlashTaskHandle;

//#include "drivers_flash.h"
//osThreadId testFlashTaskHandle;

void rtos_addThreads(){
//	osThreadDef(testFlashTask, testFlashTask, osPriorityNormal, 0, 128);
//  testFlashTaskHandle = osThreadCreate(osThread(testFlashTask), NULL);
	
	osThreadDef(ledGreenTask, ledGreenTask, osPriorityNormal, 0, 128);
  ledGreenTaskHandle = osThreadCreate(osThread(ledGreenTask), NULL);
	osThreadDef(ledRedTask, ledRedTask, osPriorityNormal, 0, 128);
  ledRedTaskHandle = osThreadCreate(osThread(ledRedTask), NULL);

//	osThreadDef(buzzerTask, buzzerTask, osPriorityNormal, 0, 128);
//  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);
	
	osThreadDef(printIMUTask, printIMUTask, osPriorityHigh, 0, 128);
  printIMUTaskHandle = osThreadCreate(osThread(printIMUTask), NULL);

	osThreadDef(getCtrlUartTask, getCtrlUartTask, osPriorityAboveNormal, 0, 256);
  getCtrlUartTaskHandle = osThreadCreate(osThread(getCtrlUartTask), NULL);

	osThreadDef(CMGMC_Task, CMGMControlTask, osPriorityAboveNormal, 0, 512);
  CMGMControlTaskHandle = osThreadCreate(osThread(CMGMC_Task), NULL);
	
	osThreadDef(AMC_Task, AMControlTask, osPriorityAboveNormal, 0, 512);
  AMControlTaskHandle = osThreadCreate(osThread(AMC_Task), NULL);
	
	osThreadDef(CMGMC_T_Task, CMGMCanTransmitTask, osPriorityRealtime, 0, 512);
  CMGMCanTransmitTaskHandle = osThreadCreate(osThread(CMGMC_T_Task), NULL);
	
	osThreadDef(AMC_T_Task, AMCanTransmitTask, osPriorityRealtime, 0, 512);
  AMCanTransmitTaskHandle = osThreadCreate(osThread(AMC_T_Task), NULL);
	

	osThreadDef(updateQ_Task, updateQuaternionTask, osPriorityNormal, 0, 256);
  updateQuaternionTaskHandle = osThreadCreate(osThread(updateQ_Task), NULL);
	
	osThreadDef(Timer_Task, Timer_1ms_lTask, osPriorityAboveNormal, 0, 256);
  TimerTaskHandle = osThreadCreate(osThread(Timer_Task), NULL);
	
	osThreadDef(Hero_Task, HeroTask, osPriorityNormal, 0, 256);
	HeroTaskHandle = osThreadCreate(osThread(Hero_Task), NULL);

//	osThreadDef(Wave_Task, wave_task, osPriorityNormal, 0, 128);
//  WaveTaskHandle = osThreadCreate(osThread(Wave_Task), NULL);
	
//	osThreadDef(RControlTask, RControlTask, osPriorityAboveNormal , 0, 512);
//  RControlTaskHandle = osThreadCreate(osThread(RControlTask), NULL);

//	osThreadDef(sonarTask, sonarTask, osPriorityNormal, 0, 128);
//  sonarTaskHandle = osThreadCreate(osThread(sonarTask), NULL);

//	fw_printfln("ledGreenTaskHandle = %x", (uint16_t)ledGreenTaskHandle);
//	fw_printfln("ledRedTaskHandle = %x", (uint16_t)ledRedTaskHandle);
//	fw_printfln("buzzerTaskHandle = %x", (uint16_t)buzzerTaskHandle);
//	fw_printfln("printIMUTaskHandle = %x", (uint16_t)printIMUTaskHandle);
//	fw_printfln("printRcTaskHandle = %x", (uint16_t)printRcTaskHandle);
//	fw_printfln("getCtrlUartTaskHandle = %x", (uint16_t)getCtrlUartTaskHandle);
//	fw_printfln("CMGMControlTaskHandle = %x", (uint16_t)CMGMControlTaskHandle);
//	fw_printfln("AMControlTaskHandle = %x", (uint16_t)AMControlTaskHandle);
//	fw_printfln("CMGMCanTransmitTaskHandle = %x", (uint16_t)CMGMCanTransmitTaskHandle);
//	fw_printfln("AMCanTransmitTaskHandle = %x", (uint16_t)AMCanTransmitTaskHandle);
//	fw_printfln("updateQuaternionTaskHandle = %x", (uint16_t)updateQuaternionTaskHandle);
	
}
