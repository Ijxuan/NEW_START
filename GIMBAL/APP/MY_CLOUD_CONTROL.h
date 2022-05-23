#ifndef CLOUD_CAN_H
#define CLOUD_CAN_H

#include "DR16_RECIVE.h"

#include "GM6020.h"
#include "my_positionPID_bate.h"
//#include "DJI_C_IMU.h"
#include "Vision.h"

#define GM6020_border_near_big 2000
#define GM6020_border_near_small 6000

#define GM6020_R_BIG 4736
//这个范围就继续增大吧
#define GM6020_R_MID 4328
//这个范围就继续减小吧
#define GM6020_R_SMALL 3891
//先右后左
#define GM6020_L_BIG 7847
//这个范围就继续增大吧
#define GM6020_L_MID 151
//这个范围就继续减小吧
#define GM6020_L_SMALL 500

void cloud_control(void);

void cloud_control_mode_choose(void);


void YAW_PID(void);

void PITCH_PID(void);
void imu_angle(void);

void scan_cloud(void);

//云台
typedef enum
{
	aoto_scan_mode = 0, //云台扫描
	vision_mode,		//云台 自瞄

} Cloud_Control_mode;
typedef struct
{

bool control_mode_NOW;
bool control_mode_LAST;

} cloud_control_mode;
extern cloud_control_mode cloud_mode;


extern		 bool scan_i_PITCH;
extern		 int scan_percent_PITCH;//0到1000,百分比
extern		 int scan_time;
extern	 float YAW_TRAGET_ANGLE_TEMP;
extern		 int scan_percent_YAW;//0到1000,百分比
extern	 float YAW_START_ANGLE;//S扫描开始时YAW轴角度
extern int arrive_targe_angle;

#endif
//MY_CLOUD_COUNTROUL

