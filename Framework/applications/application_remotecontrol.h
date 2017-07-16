#ifndef FRAMEWORK_TASKS_REMOTECONTROL_H
#define FRAMEWORK_TASKS_REMOTECONTROL_H


#include "cmsis_os.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_motor.h"
#include "gpio.h"

//LASER
#define LASER_ON()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin,GPIO_PIN_SET)
#define LASER_OFF()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin,GPIO_PIN_RESET)
//摩擦轮
#define FRICTION_WHEEL_MAX_DUTY             1600
#define FRICTION_RAMP_TICK_COUNT			100
//mouse control parameters
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 3
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 3

#define LOW_FORWARD_BACK_SPEED 			60
#define LOW_LEFT_RIGHT_SPEED   			60
#define NORMAL_FORWARD_BACK_SPEED 			400
#define NORMAL_LEFT_RIGHT_SPEED   			400
#define HIGH_FORWARD_BACK_SPEED 			660
#define HIGH_LEFT_RIGHT_SPEED   			660
//遥控器拨杆状态j
#define REMOTE_SWITCH_VALUE_UP         		0x01u  
#define REMOTE_SWITCH_VALUE_DOWN			0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u

#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))   

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP)) 
//准备时间																								 
#define PREPARE_TIME_TICK_MS 250      //prapare time in ms*2
//输入模式:遥控器/键盘鼠标/停止运行
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	STOP = 2,
}InputMode_e;


typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;

typedef enum
{
	AUTO = 0,
	MANUL = 1,
}Shoot_Mode_e;

typedef enum
{
	NORMAL = 0,
	EMERGENCY = 1,
}Emergency_Flag;


//to detect the action of the switch
#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // the current switch value
	 uint8_t switch_value1;				  //  last value << 2 | value
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //keep still if no switching
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;

typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;

//摩擦轮状态枚举
typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

//拨杆动作枚举
typedef enum
{
	FROM1TO2,
	FROM1TO3,
	FROM2TO1, 
	FROM3TO1,
	FROM3TO2,
}RC_SWITCH_ACTION_e;

WorkState_e GetWorkState(void);

void SetInputMode(Remote_t *rc);

void RCProcess(RC_CtrlData_t* pRC_CtrlData);
//void RControlTask(void const * argument);
void RemoteTaskInit(void);

//调用的函数声明
Shoot_State_e GetShootState();
void MouseKeyControlProcess(Mouse_t *mouse, Key_t *key);
void RemoteControlProcess(Remote_t *rc);
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);
void SetFrictionWheelSpeed(uint16_t x);
void MouseShootControl(Mouse_t *mouse);
Emergency_Flag GetEmergencyFlag(void);
void SetEmergencyFlag(Emergency_Flag v);
void SetShootMode(Shoot_Mode_e v);
void SetShootState(Shoot_State_e v);
Shoot_Mode_e GetShootMode(void);
void Timer_1ms_lTask(void const * argument);

void WorkStateFSM(void);
void WorkStateSwitchProcess(void);

void StartBulletFrictionWheel();
void StopBulletFrictionWheel();
#endif
