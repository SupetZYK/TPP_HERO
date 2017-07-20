#include "application_remotecontrol.h"
#include "drivers_uartrc_low.h"
#include "utilities_debug.h"
#include "stdint.h"
#include "stddef.h"
#include "drivers_ramp.h"
#include "application_pidfunc.h"
#include "application_chassiscontrol.h"
#include "application_gimbalcontrol.h"
#include "cmsis_os.h"
#include "rtos_semaphore.h"
#include "utilities_tim.h"
#include "peripheral_define.h"
#include "tim.h"
#include "tasks_Hero.h"
#include "tasks_motor.h"
#include "utilities_minmax.h"
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\






//extern RampGen_t frictionRamp ;  //Ħ����б��
//extern RampGen_t LRSpeedRamp ;   //mouse�����ƶ�б��
//extern RampGen_t FBSpeedRamp  ;   //mouseǰ���ƶ�б��

extern float yawAngleTarget, pitchAngleTarget;
/////////////////////�����õ�״̬/////////////////////
//����ģʽ
InputMode_e inputmode = REMOTE_INPUT;
//����״̬
WorkState_e workState = PREPARE_STATE;  //PREPARE
//��һ�εĹ���״̬
WorkState_e lastWorkState = PREPARE_STATE;
//���ģʽ���ֶ������Զ�
Shoot_Mode_e shootMode = MANUL;
//ϵͳ״̬
Emergency_Flag emergency_Flag = NORMAL;
//Ħ����״̬
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
//ң������ದ��
static RemoteSwitch_t switch1;  
//ң�����Ҳದ��
static RemoteSwitch_t switch2;   
//���״̬
volatile Shoot_State_e shootState = NOSHOOTING;
///////////////////////////////////////////////////////



void RCProcess(RC_CtrlData_t* pRC_CtrlData){

			if(pRC_CtrlData==NULL)
				return;
			SetInputMode(&(pRC_CtrlData->rc));
			switch (inputmode)
			{
				case REMOTE_INPUT:
				{
		//			fw_printfln("in remote mode");
					SetEmergencyFlag(NORMAL);
					RemoteControlProcess(&(pRC_CtrlData->rc));
				}break;
				case KEY_MOUSE_INPUT:
				{
					//�����̿���ģʽ
					//��ʱΪ�Զ���׼ģʽ
					MouseKeyControlProcess(&(pRC_CtrlData->mouse),&(pRC_CtrlData->key));
					SetEmergencyFlag(NORMAL);
			//		SetShootMode(AUTO);
				}break;
				case REMOTE_BULLET_INPUT:
				{
					SetEmergencyFlag(NORMAL);
					BulletControlProcess(&(pRC_CtrlData->rc));  //ȡ��ģʽ
				}break;
			}
			
			
			GetRemoteSwitchAction(&switch2,pRC_CtrlData->rc.s2);
			if(switch2.switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
			{
				Hero_Order=HERO_GETBULLET;
			}
			if(switch2.switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
			{
				Hero_Order=HERO_STOP;
			}
}

extern uint64_t last_rc_time;
void Timer_1ms_lTask(void const * argument)
{
//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
//	static int countwhile = 0;
//	static int countwhile1 = 0;
//	unsigned portBASE_TYPE StackResidue; //ջʣ��
	while(1)  {       //motor control frequency 2ms
//�������
//		SuperviseTask();    
			//fw_printf("tick_2ms\r\n");
		uint64_t t=fw_getTimeMicros();
		if(t-last_rc_time>50000)
		{
			SetEmergencyFlag(EMERGENCY);
		}
		else
		{
			SetEmergencyFlag(NORMAL);
		}
		WorkStateFSM();
	  WorkStateSwitchProcess();
		osDelay(1);
	}
}


//////////////////////////////ң��������ģʽ����
extern uint8_t engineer_task_on;
float forward_kp = 1.0 ;
void RemoteControlProcess(Remote_t *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
					ChassisSpeedRef.forward_back_ref = (rc->ch1 - 1024) / 66.0 * 4000;
					ChassisSpeedRef.left_right_ref = (rc->ch0 - 1024) / 66.0 * 4000;
					ChassisSpeedRef.rotate_ref=  (rc->ch2 - 1024) /66.0*4000;
					yawAngleTarget = -ChassisSpeedRef.rotate_ref * forward_kp / 2000;
//					aux1_targetSpeed=(rc->ch3 - 1024) /66.0*3000;
//					aux2_targetSpeed=aux1_targetSpeed;
//					aux1_targetSpeed=(-(rc->ch1 - 1024) - (rc->ch2-1024) ) /66.0*5000;
//					aux2_targetSpeed=(+rc->ch1 - 1024 - (rc->ch2-1024) ) /66.0*5000;
//			if(GetShootMode() == MANUL){  
				pitchAngleTarget += (rc->ch3 - 1024)/6600.0 * (PITCHUPLIMIT-PITCHDOWNLIMIT);
//				yawAngleTarget   -= (rc->ch2 - 1024)/660.0 * (PITCHUPLIMIT-PITCHDOWNLIMIT); 
//			}
		}
		else
		{
			fw_printfln("prepare!");
		}

		
//    if(GetWorkState() == NORMAL_STATE)
//    {
//        GimbalRef.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
//        GimbalRef.yaw_angle_dynamic_ref    += (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;      	
////	      pitchAngleTarget -= (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
// //       yawAngleTarget   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT; 
//		}
	
//	/* not used to control, just as a flag */ 
//    GimbalRef.pitch_speed_ref = rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;    //speed_ref�����������ж���
//    GimbalRef.yaw_speed_ref   = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	//���-Ħ���֣����̵��״̬
	RemoteShootControl(&switch1, rc->s1);

}

void BulletControlProcess(Remote_t *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
			ChassisSpeedRef.forward_back_ref = (rc->ch1 - 1024) / 66.0 * 2000;   //ȡ��ģʽ�������ƶ�
			ChassisSpeedRef.left_right_ref = (rc->ch0 - 1024) / 66.0 * 2000;
			ChassisSpeedRef.rotate_ref=  (rc->ch2 - 1024) /66.0*1000;
			aux_motor34_position_target += (rc->ch3 - 1024)/10;
			MINMAX(aux_motor34_position_target,aux34_limit-5000,aux34_limit);
		}
		else
		{
			fw_printfln("prepare!");
		}
}

//����������ģʽ����

void MouseKeyControlProcess(Mouse_t *mouse, Key_t *key)
{
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
	if(GetWorkState()!=PREPARE_STATE)
	{
//		//����̨���豸����������̨
//		VAL_LIMIT(mouse->x, -150, 150); 
//		VAL_LIMIT(mouse->y, -150, 150); 
		pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		//yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
		//����̨���豸ֱ����������rotate
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		ChassisSpeedRef.rotate_ref = mouse->x/15.0*6000;
		yawAngleTarget = -ChassisSpeedRef.rotate_ref * forward_kp / 2000;
		//speed mode: normal speed/high speed
		if(key->v & 0x10)
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else if(key->v & 0x20)
		{
			forward_back_speed=LOW_FORWARD_BACK_SPEED;
			left_right_speed=LOW_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		//movement process
		if(key->v & 0x01)  // key: w
		{
//			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			ChassisSpeedRef.forward_back_ref = forward_back_speed/66.0 * 4000;
		}
		else if(key->v & 0x02) //key: s
		{
//			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			ChassisSpeedRef.forward_back_ref = -forward_back_speed/66.0 *4000;
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			//FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & 0x04)  // key: d
		{
//			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			ChassisSpeedRef.left_right_ref = -left_right_speed/66.0*4000;
		}
		else if(key->v & 0x08) //key: a
		{
//			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
				ChassisSpeedRef.left_right_ref = left_right_speed/66.0*4000;
		}
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			//LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}

	}
	//step2: gimbal ref calc
 /*   if(GetWorkState() == NORMAL_STATE)
    {
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		
        pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;

	}
	*/
	/* not used to control, just as a flag */ 
//    GimbalRef.pitch_speed_ref = mouse->y;    //speed_ref�����������ж���
//    GimbalRef.yaw_speed_ref   = mouse->x;
	  MouseShootControl(mouse);
	
}


// ��������ģʽ
void SetInputMode(Remote_t *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = REMOTE_BULLET_INPUT;
	}	
}

/*??????*/
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* ����״ֵ̬ */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* ȡ����ֵ����һ��ֵ */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* ���ϵ�״ֵ̬������ */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* �ϲ�����ֵ */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* �����ж� */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//����ѭ��
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
////return the state of the remote 0:no action 1:action 
//uint8_t IsRemoteBeingAction(void)
//{
//	return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
//}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/

RampGen_t frictionRamp = RAMP_GEN_DAFAULT; 
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	GetRemoteSwitchAction(sw, val);
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   //�ӹرյ�start turning
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�������ͱ��ر�
			{
				LASER_OFF();
				SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(800);
				friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(800 + (FRICTION_WHEEL_MAX_DUTY-800)*frictionRamp.Calc(&frictionRamp)); 
				SetFrictionWheelSpeed(FRICTION_WHEEL_MAX_DUTY);
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				friction_wheel_state = FRICTION_WHEEL_ON; 	
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
			{
				SetShootState(SHOOTING);
				ShootOnce();
			}
			else
			{
				SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
}

void BulletControl(RemoteSwitch_t *sw, uint8_t val) 
{
	GetRemoteSwitchAction(sw, val);
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
	{
		Hero_Order=HERO_GETBULLET;
	}
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
	{
		Hero_Order=HERO_STOP;
	}
	/*
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   //�ӹرյ�start turning
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�������ͱ��ر�
			{
				LASER_OFF();
				SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(1000);
				friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				SetFrictionWheelSpeed(FRICTION_WHEEL_MAX_DUTY);
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				friction_wheel_state = FRICTION_WHEEL_ON; 	
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
			{
				SetShootState(SHOOTING);
				ShootOnce();
			}
			else
			{
				SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
	*/
}

void MouseShootControl(Mouse_t *mouse)
{
	static int16_t closeDelayCount = 0;   //�Ҽ��ر�Ħ����3s��ʱ����
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //�ӹرյ�start turning
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
				closeDelayCount = 0;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else
			{
				//Ħ���ּ���				
				SetFrictionWheelSpeed(800 + (FRICTION_WHEEL_MAX_DUTY-800)*frictionRamp.Calc(&frictionRamp)); 
				SetFrictionWheelSpeed(FRICTION_WHEEL_MAX_DUTY);
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				friction_wheel_state = FRICTION_WHEEL_ON; 
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			
			if(closeDelayCount>50)   //�ر�Ħ����
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}			
			else if(mouse->last_press_l==0 && mouse->press_l== 1)  //������������
			{
				SetShootState(SHOOTING);		
				ShootOnce();
			}
			else
			{
				SetShootState(NOSHOOTING);				
			}					 
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
}






Shoot_State_e GetShootState()
{
	return shootState;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	friction_wheel_state = v;
}
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}

Shoot_Mode_e GetShootMode()
{
	return shootMode;
}
void SetShootMode(Shoot_Mode_e v)
{
	shootMode = v;
}


Emergency_Flag GetEmergencyFlag()
{
	return emergency_Flag;
}

void SetEmergencyFlag(Emergency_Flag v)
{
	emergency_Flag = v;
}

WorkState_e GetWorkState()
{
	return workState;
}

void RemoteTaskInit(void)
{
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
//	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
//	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
//	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
//	FBSpeedRamp.ResetCounter(&FBSpeedRamp);

//	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
//	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	aux1_targetSpeed=0;
	aux2_targetSpeed=0;

	
	SetFrictionState(FRICTION_WHEEL_OFF);
}


/**********************************************************
*����״̬�л�״̬��
**********************************************************/
static uint32_t time_tick_2ms = 0;
void WorkStateFSM(void)
{
	lastWorkState = workState;
	time_tick_2ms ++;
	switch(workState)
	{
		case PREPARE_STATE:
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//			}
//			else 
			if(time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				fw_printf("Normal state\r\n");
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//					fw_printfln("Go to STOP STATE");
//			}
//			else if(!IsRemoteBeingAction()  && GetShootState() != SHOOTING) //||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC
//			{
//				fw_printfln("����STANDBY");
//				workState = STANDBY_STATE;      
//			}
		}break;
		case STANDBY_STATE:  
		{
//			if(GetInputMode() == STOP )//|| Is_Serious_Error())
//			{
//				workState = STOP_STATE;
//				fw_printfln("Go to STOP STATE");
//			}
//			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
//			{
//				workState = NORMAL_STATE;
//			}
		}break;
		case STOP_STATE:   
		{
//			if(GetInputMode() != STOP )//&& !Is_Serious_Error())
//			{
//				workState = PREPARE_STATE;  
//				fw_printfln("Go to Prepare STATE");				
//			}
		}break;
		default:
		{
			
		}
	}	
}
void WorkStateSwitchProcess(void)
{
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		//CMControtLoopTaskInit();
		fw_printf("remote init\r\n");
		RemoteTaskInit();
	}
}


void StartBulletFrictionWheel()
{
	TIM4->CCR1 = 1300;
	TIM4->CCR2 = 1300;
	TIM4->CCR3 = 1300;
}

void StopBulletFrictionWheel()
{
	TIM4->CCR1 = 1000;
	TIM4->CCR2 = 1000;
	TIM4->CCR3 = 1000;
}

