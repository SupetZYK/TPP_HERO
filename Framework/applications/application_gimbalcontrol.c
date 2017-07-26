#include "application_gimbalcontrol.h"

#include "stdint.h"
#include "utilities_minmax.h"
#include "drivers_canmotor_user.h"
#include "drivers_imu_user.h"
#include "application_pidfunc.h"
#include "application_setmotor.h"
#include "drivers_imu_low.h"
//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
PID_Regulator_t yawPositionPID = PID_INIT(8.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t yawSpeedPID = PID_INIT(80.0, 0.0, 5.0, 10000.0, 10000.0, 10000.0, 4900.0);

PID_Regulator_t pitchPositionPID = PID_INIT(8, 0.00, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t pitchSpeedPID = PID_INIT(60.0, 0.0, 1.0, 10000.0, 10000.0, 10000.0, 4900.0);

//PID_Regulator_t pitchPositionPID = PID_INIT(5.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 100000.0);
//PID_Regulator_t pitchSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
//ÔÆÌ¨Æ«ÖÃ
//4ºÅ³µ590 5041
int16_t YawZeroEncoderBias=6024;
int16_t PitchZeroEncoderBias=790;
float yawRealAngle=0;
float pitchRealAngle=0;
extern IMUDataTypedef imu_data;

float yawSpeedbuf[6]={0,0,0,0,0,0};
float yawRealSpeed=0;
void setYawWithAngle(float targetAngle){
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0)){
//		static int32_t cnt=0;
//		static float last_angle=0;
		//TargetAngle
		MINMAX(targetAngle, YAWDOWNLIMIT, YAWUPLIMIT);
		//RealAngle
		IOPool_getNextRead(GMYAWRxIOPool, 0); 
		yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - YawZeroEncoderBias) * 360 / 8192.0;
		NORMALIZE_ANGLE180(yawRealAngle);
		
//		yawSpeedbuf[cnt++]=yawRealAngle-last_angle;
//		if(cnt>=6) cnt=0;
//		last_angle=yawRealAngle;
//		float tmp=0;
//		for(int i=0;i<6;i++)
//			tmp+=yawSpeedbuf[i];
//		yawRealSpeed=tmp/6*100;
		//RealSpeed
		//IOPool_getNextRead(IMUIOPool, 0);
		//float realSpeed = -IOPool_pGetReadData(IMUIOPool, 0)->gYroZs;
		yawRealSpeed=imu_data.gz/32.8;
		
		setMotorWithPositionSpeedPID(GMYAW, &yawPositionPID, &yawSpeedPID, targetAngle, yawRealAngle, yawRealSpeed);
	}
}




void setPitchWithAngle(float targetAngle){
	if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
		//TargetAngle
		MINMAX(targetAngle, PITCHDOWNLIMIT, PITCHUPLIMIT);
		//RealAngle
		IOPool_getNextRead(GMPITCHRxIOPool, 0); 
		pitchRealAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - PitchZeroEncoderBias) * 360 / 8192.0;
		NORMALIZE_ANGLE180(pitchRealAngle);
		//RealSpeed
//		IOPool_getNextRead(IMUIOPool, 0);
//		float realSpeed = -IOPool_pGetReadData(IMUIOPool, 0)->gYroXs;
		float realSpeed=-imu_data.gx/32.8;
		setMotorWithPositionSpeedPID(GMPITCH, &pitchPositionPID, &pitchSpeedPID, targetAngle, pitchRealAngle, realSpeed);
	}
}

