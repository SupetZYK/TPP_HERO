#ifndef TASKS_HERO_H
#define TASKS_HERO_H
#include "stdint.h"


typedef enum
{
	HERO_NORMAL_STATE = 1,
	HERO_PREPARE_GET_BULLET=2,
	HERO_GETTING_BULLET=3,
	HERO_RECOVERING =4,
}Hero_State_t;

typedef enum
{
	HERO_GETBULLET=1,
	HERO_STOP=3,
	HERO_STANDBY=4,
}Hero_Order_t;



extern Hero_Order_t Hero_Order;
extern Hero_State_t Hero_State;

void HeroTask(void const * argument);
#endif