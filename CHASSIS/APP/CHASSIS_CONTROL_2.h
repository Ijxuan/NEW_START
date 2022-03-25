#ifndef CHASSIS_2_H
#define CHASSIS_2_H

#include "DR16_RECIVE.h"

#include "M3508.h"
#include "my_positionPID_bate.h"
#include "MY_CHASSIS_CONTROL.h"

#if chassis_radom_mode_two ==1

void CHASSIS_CONTROUL_2(void);
#if chassis_radom_mode_two ==1

bool just_arrive_targe_speed(int targe_speed);
extern bool arrive_targe_angle;

#endif




#endif

#endif