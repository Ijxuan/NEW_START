#ifndef CHASSIS_basic_H
#define CHASSIS_basic_H

#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "MY_CHASSIS_CONTROL.h"

#if chassis_radom_mode_two ==1

void CHASSIS_CONTROUL_2(void);

//void just_arrive_targe_speed(int targe_speed);

void star_and_new(void);

extern int send_to_chassis_JUST_MOVE;//发送给底盘的数据_刚好足够动起来
extern int init_times;


#endif

#endif