#ifndef APPLICATION_AUXMOTORCONTROL_H
#define APPLICATION_AUXMOTORCONTROL_H
#include "stdint.h"

void setPlateWithAngle(double targetAngle);
void setGetBulletWithAngle(double targetAngle);
void setUpDownWithAngle(double UPFLTargetAngle, double UPFRTargetAngle, double UPBLTargetAngle, double UPBRTargetAngle);

///zyk///
void setAux1WithSpeed(double targetSpeed);
void setAux2WithSpeed(double targetSpeed);
void setAux3WithSpeed(double targetSpeed);
void setAux4WithSpeed(double targetSpeed);

void setAux1WithAngle(double angle);
void setAux2WithAngle(double angle);
void setAux3WithAngle(double angle);
void setAux4WithAngle(double angle);

double GetAuxMotorRealAngle(uint8_t aux_motor_index);
#endif
