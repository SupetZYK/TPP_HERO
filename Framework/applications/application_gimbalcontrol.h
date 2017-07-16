#ifndef APPLICATION_GIMBALCONTROL_H
#define APPLICATION_GIMBALCONTROL_H
#include "stdint.h"
extern int16_t YawZeroEncoderBias; //1075 ///1075//4906
#define YAWUPLIMIT 25
#define YAWDOWNLIMIT -25

extern int16_t PitchZeroEncoderBias; //3180 ///1075//4906
#define PITCHUPLIMIT 25
#define PITCHDOWNLIMIT -25
void setYawWithAngle(float targetAngle);
void setPitchWithAngle(float targetAngle);

void SetPitchWithAngle2006(float targetAngle);
#endif
