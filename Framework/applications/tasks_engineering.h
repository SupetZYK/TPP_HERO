#ifndef __TASKS_ENGINEERING_H
#define __TASKS_ENGINEERING_H

#include "stdint.h"
typedef enum
{
	ENGINEER_NO_MOVE = 1,
	ENGINEER_ADJUSTING = 2,
	ENGINEER_DISTANCE_OK = 3,
	ENGINEER_DISTANCE_AND_ROTATION_OK = 4,
	ENGINEER_PLACING = 5,
	ENGINEER_FETCHING = 6,
	ENGINEER_PLACING_COMPLETE =7,
	ENGINEER_FETCHING_COMPLETE =8,
	ENGINEER_DISCASDING =9,
	ENGINEER_REPLACING =10,
	ENGINEER_GRABING =11,//抬高，伸出，抓住的状态（没有考虑是否抓住）
	
	ENGINEER_MANUAL_PLACING=12,//手动控制抓取时的运动过程中的状态，运动完成后的状态为ENGINEER_GRABING
	ENGINEER_MANUAL_FETCHING=13,//手动控制抓取的运动过程中
	ENGINEER_MANUAL_STRETCHED=14,//手动抓取运动结束的状态
	
	ENGINEER_BELT_MOVING=15,//皮带轮手动控制
	
	ENGINEER_RECOVERING=16,
}Engineer_State_t;

typedef enum
{
	//STOP只允许外部置位(rc)，库内部不会对其置位,
	ENGINEER_STOP = 1,
	ENGINEER_ADJUSTDISTANCE=2,
	ENGINEER_ADJUSTDISTANCE_AND_ROTATION=3,
	ENGINEER=4,
	ENGINEER_DISCARD_STUFF=5,
	ENGINEER_REPLACE_STUFF=6,
	//发生错误，内部可以用pause，置位
	ENGINEER_PAUSE=7,
	
	//手动操作命令
	ENGINEER_MANUAL_PLACE=8,
	ENGINEER_MANUAL_FETCH=9,
	//空命令
	ENGINEER_STANDBY=10,

	//手动皮带轮向后
	ENGINEER_BELT_BACK=11,
	ENGINEER_BELT_FORWARD=12,
	
}Engineer_Order_t;

extern Engineer_State_t Engineering_State;
extern Engineer_Order_t Engineering_Order;
//extern uint8_t engineering_task_on;
//外部调用
uint8_t engineer_grab_somthing();
void StartNewPlaceTask();
void StartNewFetchTask();
void StopEngineerTask();
void EngineeringTask(void const * argument);
void OrderReplace();
void EngineerPrintState();
void SetFetchHeight(uint8_t HeightIndex);
#endif

