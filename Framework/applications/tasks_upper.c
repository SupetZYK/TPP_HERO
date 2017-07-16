#include "tasks_upper.h"
#include "drivers_uartupper_user.h"

#include "drivers_flash.h"

#include "utilities_debug.h"

#include "drivers_uartupper_user.h"
#include "tasks_motor.h"
#include "string.h"
#include "drivers_uartrc_user.h"
#include "UserProtocal.h"
#include "application_pidfunc.h"
#include "drivers_canmotor_user.h"
#include "application_remotecontrol.h"
#include "drivers_imu_user.h"
#include "application_auxmotorcontrol.h"

#include "drivers_imu_low.h"
#include "tasks_Hero.h"
NaiveIOPoolDefine(upperIOPool, {0});

extern uint16_t yawAngle, pitchAngle;
int forPidDebug = 0;

extern int8_t flUpDown, frUpDown, blUpDown, brUpDown, allUpDown;
//extern float yawAngleTarget, pitchAngleTarget;
void getCtrlUartTask(void const * argument){
//	uint8_t data[10];
	while(1){
		static int cnt=0;
	zykProcessData();
	osDelay(1);
		if(cnt++>2000)
		{
			fw_printf("alive\r\n");
			cnt=0;
		}
	}
}

uint8_t print_data=0;

extern float yawAngleTarget, pitchAngleTarget;
extern int16_t YawZeroEncoderBias;
extern int16_t PitchZeroEncoderBias;
extern uint8_t GM_RUN;
extern float angles[3];
extern PID_Regulator_t CMFLSpeedPID,CMFRSpeedPID,CMBLSpeedPID,CMBRSpeedPID;
extern PID_Regulator_t yawPositionPID,yawSpeedPID,pitchPositionPID,pitchSpeedPID;
extern IMUDataTypedef imu_data;
extern float pitchRealAngle,yawRealAngle;
extern float yawRealSpeed;
extern double aux_motor34_position_target;
void zykProcessData()
{	
	//fw_printfln("ok");
		if(RX_DONE)
		{
		char data[10][15];
		//fw_printf(buf);
		/////////// GM CONTROL ////////////////
		if(strcmp(buf,"U")==0)
		{
			fw_printf("UP\r\n");
			pitchAngleTarget+=5;
		}
		else if(strcmp(buf,"D")==0)
		{
			fw_printf("DOWN\r\n");
			pitchAngleTarget-=5;
		}
		if(strcmp(buf,"L")==0)
		{
			fw_printf("LEFT\r\n");
			yawAngleTarget+=5;
		}
		else if(strcmp(buf,"R")==0)
		{
			fw_printf("RIGHT\r\n");
			yawAngleTarget-=5;
		}
		else if(strcmp(buf,"GM")==0)
		{
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			IOPool_getNextRead(GMYAWRxIOPool, 0);
			int pitch_encoder= (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			int yaw_encoder = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			float pitch_real= (pitch_encoder-PitchZeroEncoderBias)* 360 / 8192.0;
			float yaw_real= (yaw_encoder-YawZeroEncoderBias)* 360 / 8192.0;
			NORMALIZE_ANGLE180(pitch_real);
			NORMALIZE_ANGLE180(yaw_real);
			fw_printfln(">>Pitch");
			fw_printfln("Pitch real angle is %.3f",pitch_real);
			fw_printfln("Pitch encoder is %d", pitch_encoder);
			fw_printfln("Pitch target angle is %.3f",pitchAngleTarget);
			fw_printfln(">>Yaw");
			fw_printfln("Yaw real angle is %.3f",yaw_real);
			fw_printfln("Yaw encoder is %d", yaw_encoder);
			fw_printfln("Yaw target angle is %.3f",yawAngleTarget);
		}
		else if(strcmp(buf,"GM_RUN")==0)
		{
			if(GM_RUN)
			{
				GM_RUN=0;
				fw_printf("STOP\r\n");
			}
			else
			{
				GM_RUN=1;
				fw_printf("RUN\r\n");
			}
		}
		/////////// GM PID
		else if(ComProtocal(buf,"#GMYPP","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.kp=p;
			fw_printf("Yaw position P change to %f\r\n",yawPositionPID.kp);
		}
		else if(ComProtocal(buf,"#GMYPI","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.ki=p;
			fw_printf("Yaw position I change to %f\r\n",yawPositionPID.ki);
		}
		else if(ComProtocal(buf,"#GMYPD","$","@",data))
		{
			float p=atof(data[0]);
			yawPositionPID.kd=p;
			fw_printf("Yaw position D change to %f\r\n",yawPositionPID.kd);
		}
		else if(ComProtocal(buf,"#GMYSP","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.kp=p;
			fw_printf("Yaw speed P change to %f\r\n",yawSpeedPID.kp);
		}
		else if(ComProtocal(buf,"#GMYSI","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.ki=p;
			fw_printf("Yaw speed I change to %f\r\n",yawSpeedPID.ki);
		}
		else if(ComProtocal(buf,"#GMYSD","$","@",data))
		{
			float p=atof(data[0]);
			yawSpeedPID.kd=p;
			fw_printf("Yaw speed D change to %f\r\n",yawSpeedPID.kd);
		}
				/////////// GM PID ￡¨pitch￡?
		else if(ComProtocal(buf,"#GMPPP","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.kp=p;
			fw_printf("Pitch position P change to %f\r\n",pitchPositionPID.kp);
		}
		else if(ComProtocal(buf,"#GMPPI","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.ki=p;
			fw_printf("Pitch position I change to %f\r\n",pitchPositionPID.ki);
		}
		else if(ComProtocal(buf,"#GMPPD","$","@",data))
		{
			float p=atof(data[0]);
			pitchPositionPID.kd=p;
			fw_printf("Pitch position D change to %f\r\n",pitchPositionPID.kd);
		}
		else if(ComProtocal(buf,"#GMPSP","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.kp=p;
			fw_printf("Pitch speed P change to %f\r\n",pitchSpeedPID.kp);
		}
		else if(ComProtocal(buf,"#GMPSI","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.ki=p;
			fw_printf("Pitch speed I change to %f\r\n",pitchSpeedPID.ki);
		}
		else if(ComProtocal(buf,"#GMPSD","$","@",data))
		{
			float p=atof(data[0]);
			pitchSpeedPID.kd=p;
			fw_printf("Pitch speed D change to %f\r\n",pitchSpeedPID.kd);
		}
		/////////// CM INFO ////////////////
		else if(strcmp(buf,"CM")==0)
		{
			fw_printfln("CM ForwardBackSpeedRef :%f  |  CM LeftrightSpeedRef :%f  |  CM RotateSpeedRef :%f", ChassisSpeedRef.forward_back_ref,ChassisSpeedRef.left_right_ref,ChassisSpeedRef.rotate_ref);
			fw_printfln("CM Intensity FL:%f  |  FR:%f  | BL:%f  | BR:%f",CMFLSpeedPID.output,CMFRSpeedPID.output,CMBLSpeedPID.output,CMBRSpeedPID.output);
		}
		else if(strcmp(buf,"CMS")==0)
		{
			IOPool_getNextRead(CMFLRxIOPool, 0);
		float fls = (IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			IOPool_getNextRead(CMFRRxIOPool, 0);
		float frs = (IOPool_pGetReadData(CMFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			IOPool_getNextRead(CMBLRxIOPool, 0);
		float bls = (IOPool_pGetReadData(CMBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			IOPool_getNextRead(CMBRRxIOPool, 0);
		float brs = (IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			fw_printfln("speed fl: %f| fr: %f| bl: %f| br: %f",fls,frs,bls,brs);
		}
		//////AUX////////
		else if(strcmp(buf,"aux34+")==0)
		{
			aux_motor34_position_target+=500;
//			aux1_targetSpeed=-30000;
//			fw_printf("aux1 speed: %f\r\n",aux1_targetSpeed);
			fw_printf("aux34: %f\r\n",aux_motor34_position_target);
		}
		else if(strcmp(buf,"aux34-")==0)
		{
			aux_motor34_position_target-=500;
//			aux1_targetSpeed=-30000;
//			fw_printf("aux1 speed: %f\r\n",aux1_targetSpeed);
			fw_printf("aux34: %f\r\n",aux_motor34_position_target);
		}
		else if(strcmp(buf,"aux6+")==0)
		{
			getBullet_angle_target+=500;
//			aux1_targetSpeed=-30000;
//			fw_printf("aux1 speed: %f\r\n",aux1_targetSpeed);
			fw_printf("aux6: %f\r\n",getBullet_angle_target);
		}
		else if(strcmp(buf,"aux6-")==0)
		{
			getBullet_angle_target-=500;
//			aux1_targetSpeed=-30000;
//			fw_printf("aux1 speed: %f\r\n",aux1_targetSpeed);
			fw_printf("aux6: %f\r\n",getBullet_angle_target);
		}
		//////RC////////
		else if(strcmp(buf,"RC")==0)
		{
			IOPool_getNextRead(rcUartIOPool, 0);
			float testdata=0;
			fw_printf("rc0: %d | ",IOPool_pGetReadData(rcUartIOPool, 0)->rc.ch0);
			fw_printf("rc1: %d | ",IOPool_pGetReadData(rcUartIOPool, 0)->rc.ch1);
			fw_printf("rc2: %d | ",IOPool_pGetReadData(rcUartIOPool, 0)->rc.ch2);
			fw_printf("rc3: %d |  ",IOPool_pGetReadData(rcUartIOPool, 0)->rc.ch3);
			fw_printf("s1: %d |  ",IOPool_pGetReadData(rcUartIOPool, 0)->rc.s1);
			fw_printf("s2: %d\r\n",IOPool_pGetReadData(rcUartIOPool, 0)->rc.s2);
			fw_printf("s2: %d\r\n",IOPool_pGetReadData(rcUartIOPool, 0)->rc.s2);
			testdata=IOPool_pGetReadData(rcUartIOPool, 0)->rc.ch0;
			testdata=(testdata- 1024) / 660.0 * 1000;
			fw_printf("tst data: %f\r\n",testdata);
		}
		///////////MPU
		else if(strcmp(buf,"MPU")==0)
		{
			IOPool_getNextRead(IMUIOPool, 0);
			float realSpeed = -IOPool_pGetReadData(IMUIOPool, 0)->gYroZs;
			float realSpeed2=imu_data.gz/32.8;
			fw_printf("angles0 = %f | ", angles[0]);
			fw_printf("angles1 = %f | ", angles[1]);
			fw_printf("angles2 = %f | ", angles[2]);
			fw_printf("z_gyro = %f | ", realSpeed);
			fw_printf("z_gyro2= %f\r\n", realSpeed2);
			fw_printf("========================\r\n");
		}
		///////////////////UPPER
		else if(strcmp(buf,"RD1")==0)
		{
			//float realSpeed2=imu_data.gz/32.8;
			fw_printf("#DATA%.2f@%.2f@%.2f$",yawPositionPID.output,yawRealSpeed,yawRealAngle);
//			if(print_data==1)
//			{
//				print_data=0;
//			}
//			else
//			{
//				print_data=1;
//			}
		}
		else if(strcmp(buf,"RD2")==0)
		{
			//speed
			float realSpeed2=-imu_data.gx/32.8;
			fw_printf("#DATA%.2f@%.2f@%.2f$",pitchPositionPID.output,realSpeed2,pitchRealAngle);
//			if(print_data==2)
//			{
//				print_data=0;
//			}
//			else
//			{
//				print_data=2;
//			}
		}
			/////plate
		else if(strcmp(buf,"S")==0)
		{
			ShootOnce();
		}
		else if(strcmp(buf,"Plate")==0)
		{
			IOPool_getNextRead(AMPLATERxIOPool, 0);
			double AngleCurr = (IOPool_pGetReadData(AMPLATERxIOPool, 0)->angle) * 360 / 8192.0;
			fw_printfln("plate angle=%f",AngleCurr);
		}
		else if(strcmp(buf,"BF+")==0)
		{
			StartBulletFrictionWheel();
		}
		else if(strcmp(buf,"BF-")==0)
		{
			StopBulletFrictionWheel();
		}
		else if(strcmp(buf,"GET")==0)
		{
			Hero_Order=HERO_GETBULLET;
			fw_printfln("Gettting bullet!");
		}
		else if(strcmp(buf,"GET-")==0)
		{
			Hero_Order=HERO_STOP;
			fw_printfln("HERO STOP!");
		}
		strcpy(buf,"\0");
		RX_STA=0;
	}
}


void wave_task(void const * argument){
	while(1)
	{
		if(print_data==1)
		{
			float realSpeed2=-imu_data.gz/32.8;
			fw_printf("#DATA%.2f@%.2f@%.2f$",yawPositionPID.output,realSpeed2,yawRealAngle);
		}
		else if(print_data==2)
		{
			float realSpeed2=-imu_data.gy/32.8;
			fw_printf("#DATA%.2f@%.2f@%.2f$",pitchPositionPID.output,realSpeed2,pitchRealAngle);
		}
		osDelay(20);
	}
}
