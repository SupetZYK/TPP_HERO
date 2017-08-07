#include "application_auxmotorcontrol.h"

#include "drivers_canmotor_user.h"

#include "application_pidfunc.h"
#include "application_setmotor.h"
#include "utilities_debug.h"
PID_Regulator_t platePositionPID = PID_INIT(7.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t plateSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t getBulletPositionPID = PID_INIT(6.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t getBulletSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t UDFLPositionPID = PID_INIT(5.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 100000.0);
PID_Regulator_t UDFLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 35000.0, 30000.0, 30000.0, 32000.0);
PID_Regulator_t UDFRPositionPID = PID_INIT(20.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 100000.0);
PID_Regulator_t UDFRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 35000.0, 30000.0, 30000.0, 32000.0);
PID_Regulator_t UDBLPositionPID = PID_INIT(20.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 100000.0);
PID_Regulator_t UDBLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);
PID_Regulator_t UDBRPositionPID = PID_INIT(20.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 100000.0);
PID_Regulator_t UDBRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);

//aux 5 motor
double plateRealAngle=0;
void setPlateWithAngle(double targetAngle){//360.0 * 12 * 2
		if(IOPool_hasNextRead(AMPLATERxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMPLATERxIOPool, 0);
		static double AngleLast = 180.0;
		double AngleCurr = (IOPool_pGetReadData(AMPLATERxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			plateRealAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			plateRealAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			plateRealAngle += AngleCurr + 360 - AngleLast;
		}else{
			plateRealAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMPLATERxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM2PLATE, &platePositionPID, &plateSpeedPID, targetAngle, plateRealAngle, realSpeed);
	}
}
//aux 6 motor
double getBulletRealAngle=0;
void setGetBulletWithAngle(double targetAngle){//360.0 * 36 * 2;
	if(IOPool_hasNextRead(AMGETBULLETRxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMGETBULLETRxIOPool, 0);
		static double AngleLast = 180.0;
		double AngleCurr = (IOPool_pGetReadData(AMGETBULLETRxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			getBulletRealAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			getBulletRealAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			getBulletRealAngle += AngleCurr + 360 - AngleLast;
		}else{
			getBulletRealAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMGETBULLETRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM2GETBULLET, &getBulletPositionPID, &getBulletSpeedPID, targetAngle, getBulletRealAngle, realSpeed);
	}
}

void setAux1WithSpeed(double targetSpeed){
	if(IOPool_hasNextRead(AMUDFLRxIOPool, 0)){
		//TargetSpeed
		//MINMAX(targetSpeed, LIMIT, LIMIT);
		//RealSpeed
		IOPool_getNextRead(AMUDFLRxIOPool, 0);
		double realSpeed = (IOPool_pGetReadData(AMUDFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
		setMotorWithSpeedPID(AM1UDFL, &UDFLSpeedPID, targetSpeed, realSpeed);
	}
}

void setAux2WithSpeed(double targetSpeed){
	if(IOPool_hasNextRead(AMUDFRRxIOPool, 0)){
		//TargetSpeed
		//MINMAX(targetSpeed, LIMIT, LIMIT);
		//RealSpeed
		IOPool_getNextRead(AMUDFRRxIOPool, 0);
		double realSpeed = (IOPool_pGetReadData(AMUDFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
		setMotorWithSpeedPID(AM1UDFR, &UDFRSpeedPID, targetSpeed, realSpeed);
	}
}

void setAux3WithSpeed(double targetSpeed){
	if(IOPool_hasNextRead(AMUDBLRxIOPool, 0)){
		//TargetSpeed
		//MINMAX(targetSpeed, LIMIT, LIMIT);
		//RealSpeed
		IOPool_getNextRead(AMUDBLRxIOPool, 0);
		double realSpeed = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
		setMotorWithSpeedPID(AM1UDBL, &UDBLSpeedPID, targetSpeed, realSpeed);
	}
}


void setAux4WithSpeed(double targetSpeed){
	if(IOPool_hasNextRead(AMUDBRRxIOPool, 0)){
		//TargetSpeed
		//MINMAX(targetSpeed, LIMIT, LIMIT);
		//RealSpeed
		IOPool_getNextRead(AMUDBRRxIOPool, 0);
		double realSpeed = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
		setMotorWithSpeedPID(AM1UDBR, &UDBRSpeedPID, targetSpeed, realSpeed);
	}
}

double aux1_realAngle=0;
void setAux1WithAngle(double angle)
{
	if(IOPool_hasNextRead(AMUDFLRxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMUDFLRxIOPool, 0);
//		static double aux1_realAngle = 0.0;
		static double AngleLast = 0.0;
		double AngleCurr = (IOPool_pGetReadData(AMUDFLRxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			aux1_realAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			aux1_realAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			aux1_realAngle += AngleCurr + 360 - AngleLast;
		}else{
			aux1_realAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMUDFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM1UDFL, &UDFLPositionPID, &UDFLSpeedPID, angle, aux1_realAngle, realSpeed);
	}
}
double aux2_realAngle=0;
void setAux2WithAngle(double angle)
{
	if(IOPool_hasNextRead(AMUDFRRxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMUDFRRxIOPool, 0);
		static double AngleLast = 180.0;
		double AngleCurr = (IOPool_pGetReadData(AMUDFRRxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			aux2_realAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			aux2_realAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			aux2_realAngle += AngleCurr + 360 - AngleLast;
		}else{
			aux2_realAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMUDFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM1UDFR, &UDFRPositionPID, &UDFRSpeedPID, angle, aux2_realAngle, realSpeed);
	}
}
double aux3_realAngle=0;
void setAux3WithAngle(double angle)
{
	if(IOPool_hasNextRead(AMUDBLRxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMUDBLRxIOPool, 0);
	
//		static double realAngle = 0.0;
		static double AngleLast = 180.0;
		double AngleCurr = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			aux3_realAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			aux3_realAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			aux3_realAngle += AngleCurr + 360 - AngleLast;
		}else{
			aux3_realAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMUDBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM1UDBL, &UDBLPositionPID, &UDBLSpeedPID, angle, aux3_realAngle, realSpeed);
	}
}

double aux4_realAngle=0;
void setAux4WithAngle(double angle)
{
	if(IOPool_hasNextRead(AMUDBRRxIOPool, 0)){
		//TargetAngle
		//double targetAngle
		//RealAngle
		IOPool_getNextRead(AMUDBRRxIOPool, 0);
//		static double realAngle = 0.0;
		static double AngleLast = 180.0;
		double AngleCurr = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->angle) * 360 / 8192.0;
		static uint8_t isInitiated=0;
		if(isInitiated==0)
		{
			aux4_realAngle=AngleCurr;
			AngleLast=AngleCurr;
			isInitiated=1;
			return;
		}
		if(AngleCurr - AngleLast > 180){
			aux4_realAngle += AngleCurr - 360 - AngleLast;
		}else if(AngleCurr - AngleLast < -180){
			aux4_realAngle += AngleCurr + 360 - AngleLast;
		}else{
			aux4_realAngle += AngleCurr - AngleLast;
		}
		AngleLast=AngleCurr;
		
		//RealSpeed
		double realSpeed = (IOPool_pGetReadData(AMUDBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)

		setMotorWithPositionSpeedPID(AM1UDBR, &UDBRPositionPID, &UDBRSpeedPID, angle, aux4_realAngle, realSpeed);
	}
}
extern double aux_motor3_zero_angle,aux_motor4_zero_angle,getBullet_zero_angle,plate_zero_angle;
double GetAuxMotorRealAngle(uint8_t aux_motor_index)
{
	switch(aux_motor_index)
	{
		case 1:
			return 0;
		case 2:
			return 0;
		case 3:
			return aux3_realAngle - aux_motor3_zero_angle;
		case 4:
			return aux4_realAngle - aux_motor4_zero_angle;
		case 5:
			return plateRealAngle - plate_zero_angle;
		case 6:
			return getBulletRealAngle-getBullet_zero_angle;
		default:
			return 0;
	}
}