#ifndef TASKS_HERO_H
#define TASKS_HERO_H
#include "stdint.h"


typedef enum
{
	HERO_NORMAL_STATE = 1,
	HERO_PREPARE_GET_BULLET=2,
	HERO_GETTING_BULLET=3,
	HERO_AUTO_GETTING_BULLET=4,
	HERO_RECOVERING =5,
	HERO_SHOOTING =6,
}Hero_State_t;

typedef enum
{
	HERO_GETBULLET=1,
	HERO_AUTO_GETBULLET=2,
	HERO_STOP=3,
	HERO_STANDBY=4,
	HERO_SHOOT_1=5,
	HERO_SHOOT_4=6,
	HERO_STOP_AUTO_GETBULLET=7,
}Hero_Order_t;

void Hero_Prepare_Get_Bullet();
void Hero_Recover();

extern Hero_Order_t Hero_Order;
extern Hero_State_t Hero_State;
extern double aut_get_bullet_base_height;
void HeroTask(void const * argument);
#endif