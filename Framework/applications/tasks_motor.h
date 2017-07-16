#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H
#include "stdint.h"


extern uint8_t GM_RUN;

typedef __packed struct
{
    float forward_back_ref;
    float left_right_ref;
    float rotate_ref;
}ChassisSpeed_Ref_t;


extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern float aux1_targetSpeed,aux2_targetSpeed;
extern double aux_motor34_position_target,getBullet_angle_target;
extern double aux34_limit,getBullet_limit;
void CMGMControlTask(void const * argument);
void AMControlTask(void const * argument);

void ShootOnce();
#endif
