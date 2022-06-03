#ifndef SHOOT_CAN_H
#define SHOOT_CAN_H
#include "main.h"

void shoot_control(void);
void driver_plate_control(void);

typedef struct
{
bool vision_shoot_is_continuous;//视觉发射指令是连续的	
bool  heat_allow;//热量允许
bool  not_in_track_end;//不在轨道末端
bool  weather_angle_error_less_than_1;//角度误差小于一
	
bool ALL_condition_satisfaction;//全部条件都满足
} allow_auto_shoot;
extern allow_auto_shoot auto_shoot_condition;

extern bool Driver_arrive;
extern int if_Driver_arrive_angle;
extern long M2006_targe_angle;
extern bool weather_error_less_than_1;
extern int auto_shoot_condition_show;//自动射击条件展现
extern bool JR_HEAT_renew;//进入枪口热量回复模式
 extern int real_shoot_1s;

#endif

