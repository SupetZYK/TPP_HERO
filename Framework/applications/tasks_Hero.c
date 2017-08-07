#include "tasks_motor.h"
#include "tasks_Hero.h"
#include "cmsis_os.h"
#include "utilities_debug.h"
#include "application_auxmotorcontrol.h"
#include "math.h"
#include "application_remotecontrol.h"
Hero_Order_t Hero_Order=HERO_STANDBY;
Hero_State_t Hero_State=HERO_NORMAL_STATE;

//function called
void Hero_Recover();
uint8_t Hero_Stretch(float value, uint32_t time_milis);
uint8_t Hero_Lift(float value, uint32_t time_milis);
void Hero_Prepare_Get_Bullet();
void Hero_Auto_Get_Bullet();
void Hero_Shoot_1();
void Hero_Shoot_4();
void Stuck_Process();
void Hero_Stop_Auto_Get_Bullet();

void HeroTask(void const * argument){
	while(1)
	{
		switch(Hero_Order)
		{
			case HERO_GETBULLET:
			{
				Hero_Prepare_Get_Bullet();
			}break;
			case HERO_AUTO_GETBULLET:
			{
				Hero_Auto_Get_Bullet();
			}break;
			case HERO_STOP:
			{
				Hero_Recover();
			}break;
			case HERO_STOP_AUTO_GETBULLET:
			{
				Hero_Stop_Auto_Get_Bullet();
			}break;
			case HERO_SHOOT_1:
			{
				Hero_Shoot_1();
			}break;
			case HERO_SHOOT_4:
			{
				Hero_Shoot_4();
			}break;
			case HERO_STANDBY:
			{
				Stuck_Process();
				osDelay(1);
			}break;
			
			
		}
	}
}

//执行升起，伸出，打开摩擦轮的动作
void Hero_Prepare_Get_Bullet()
{
	Hero_Order=HERO_STANDBY;
	if(Hero_State==HERO_NORMAL_STATE)
	{
		Hero_State=HERO_PREPARE_GET_BULLET;
		if(!Hero_Lift(aux34_limit,1500)){Hero_Order=HERO_STOP;return;}
		if(!Hero_Stretch(getBullet_limit,1000)){Hero_Order=HERO_STOP;return;}
		//StartBulletFrictionWheel();
		Hero_State=HERO_GETTING_BULLET;
	}
}

uint8_t Hero_Strech_and_Lift(float stretch_valu,float lift_value,uint32_t time_milis);
double aut_get_bullet_base_height;
void Hero_Auto_Get_Bullet()
{
	Hero_Order=HERO_STANDBY;
	if(Hero_State==HERO_GETTING_BULLET)
	{
		Hero_State=HERO_AUTO_GETTING_BULLET;
		fw_printfln("start auto move!");
		while(1)
		{
			StopBulletFrictionWheel();
			if(!Hero_Strech_and_Lift(getBullet_limit-15000,aut_get_bullet_base_height-4000,300)) break;
			StartBulletFrictionWheel();
//			if(!Hero_Strech_and_Lift(getBullet_limit-14000,aux34_limit-800,350)) break;
//			if(!Hero_Strech_and_Lift(getBullet_limit-12000,aux34_limit-2400,350)) break;
//			if(!Hero_Strech_and_Lift(getBullet_limit-9000,aux34_limit-4800,350)) break;
//			if(!Hero_Strech_and_Lift(getBullet_limit-5000,aux34_limit-8000,350)) break;
			if(!Hero_Strech_and_Lift(getBullet_limit-10000,aut_get_bullet_base_height-8000,1500)) break;
			if(!Hero_Strech_and_Lift(getBullet_limit,aut_get_bullet_base_height-12000,1500)) break; 
		}
		fw_printfln("end auto move!");
	}
}


//伸出的闭环控制任务函数，
uint8_t Hero_Stretch(float value, uint32_t time_milis)
{
	float original=getBullet_angle_target;
	//使用五次插值
	//float tmp=(value-original)/time_milis;
	for(uint32_t i=1;i<time_milis+1;i++)
	{
		float a_b=original-value;
		float tmp=((float)i/time_milis);
		float tmp3=tmp*tmp*tmp;
		float tmp4=tmp3*tmp;
		float tmp5=tmp4*tmp;
		getBullet_angle_target=-6*a_b*tmp5+15*a_b*tmp4-10*a_b*tmp3+original;
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech!");
			return 0;
		}
		//fw_printfln("%f",aux_motor2_position_target);
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(6)-value) >1000 || (GetAuxMotorRealAngle(6)-value)<-1000)
	{
		if(cnt++>500)
		{
			fw_printfln("stretch not reach!target %f  real %f", value, GetAuxMotorRealAngle(6));
			Hero_Order=HERO_STOP;
			return 0;
		}
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech! ");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

//升起的闭环控制函数
uint8_t Hero_Lift(float value, uint32_t time_milis)
{
	float original=aux_motor34_position_target;
	fw_printfln("original is %f",original);
	float tmp=(value-original)/time_milis;
	for(uint32_t i=1;i<=time_milis+1;i++)
	{
		aux_motor34_position_target=original + i*tmp;
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	aux_motor34_position_target=value;
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(3)-value) > 2000 || (GetAuxMotorRealAngle(3)-value)<-2000)
	{
		if(cnt++>800)
		{
			Hero_Order=HERO_STOP;
			fw_printfln("lift not reach! target %f  real %f", value, GetAuxMotorRealAngle(4));
			return 0;
		}
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}


uint8_t Hero_Strech_and_Lift(float stretch_value,float lift_value,uint32_t time_milis)
{
	float original_l=aux_motor34_position_target;
	float tmp_l=(lift_value-original_l)/time_milis;
	float original_s=getBullet_angle_target;
	float tmp_s=(stretch_value-original_s)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor34_position_target=original_l + i*tmp_l;
		getBullet_angle_target=original_s+i*tmp_s;
		if(Hero_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift and stretch!");
			return 0;
		}
		if(Hero_Order==HERO_STOP_AUTO_GETBULLET)
		{	
			fw_printfln("stop called when lift and stretch!");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}
//no check, only used in recovering!
void HeroForceLift(float value, uint32_t time_milis)
{
	float original=aux_motor34_position_target;
	float step=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis;i++)
	{
		aux_motor34_position_target+=step;
		osDelay(1);
	}
	aux_motor34_position_target=value;
}
//no check, only used in recovering!
void HeroForceStretch(float value, uint32_t time_milis)
{
	float original=getBullet_angle_target;
	float step=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis;i++)
	{
		getBullet_angle_target+=step;
		osDelay(1);
	}
	getBullet_angle_target=value;
}

void Hero_Stop_Auto_Get_Bullet()
{
		//StopBulletFrictionWheel();
		Hero_Order=HERO_GETBULLET;
		Hero_State=HERO_GETTING_BULLET;
		HeroForceLift(aux34_limit,1000);
		HeroForceStretch(getBullet_limit,1000);
}
//执行恢复动作，伸缩，降落，关闭摩擦轮
void Hero_Recover()
{
		Hero_Order=HERO_STANDBY;
		Hero_State=HERO_RECOVERING;
		StopBulletFrictionWheel();
		HeroForceLift(aux34_limit,500);
		HeroForceStretch(0,1500);
		HeroForceLift(0,2000);
		Hero_State=HERO_NORMAL_STATE;
}
void Hero_Shoot_1()
{
	Hero_Order=HERO_STANDBY;
	Hero_State=HERO_SHOOTING;
	plate_angle_target-=90.0*95.8;
	osDelay(300);
	
	Hero_Order=HERO_STANDBY;
	Hero_State=HERO_NORMAL_STATE;
}
void Hero_Shoot_4()
{
	Hero_State=HERO_SHOOTING;
	for(int i=0;i<3;++i)
	{
		plate_angle_target-=90.0*95.8;
		osDelay(300);
	}
	Hero_Order=HERO_STANDBY;
	Hero_State=HERO_NORMAL_STATE;
}

void Stuck_Process()
{
	static int16_t cnt1=0,cnt2=0;
	if((GetAuxMotorRealAngle(5)-plate_angle_target)>10*95.8)
	{
		if(cnt1++>100)
		{
			fw_printfln("Stuck! target: %f, real: %f", plate_angle_target/98.5, GetAuxMotorRealAngle(5)/98.5);
			plate_angle_target+=90.0*95.8;
			cnt1=0;
			osDelay(300);
			
		}
	}
	else
	{
		cnt1=0;
	}
	if((GetAuxMotorRealAngle(5)-plate_angle_target)<-10*98.5)
	{
		if(cnt2++>100)
		{
			fw_printfln("Stuck! target: %f, real: %f", plate_angle_target/98.5, GetAuxMotorRealAngle(5)/98.5);
			plate_angle_target-=90.0*95.8;
			cnt2=0;
			osDelay(300);
			
		}
	}
	else
	{
		cnt2=0;
	}
}