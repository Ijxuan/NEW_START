#ifndef CHASSIS_CAN_H
#define CHASSIS_CAN_H

#include "DR16_RECIVE.h"

#include "M3508.h"
#include "my_positionPID_bate.h"
//#include "DJI_C_IMU.h"
#include "User_math.h"

void CHASSIS_CONTROUL(void);

void switch_change(void);

void Random_CHASSIS(void);

void Cruise_CHASSIS(void);
void Power_Calculate(void);


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max);
void Power_Calculate();

typedef struct 
{
    int point_one; //点1
    int point_two; //点2
}Trisection;//3等分点
typedef struct 
{
    int point_one; //点1
    int point_two; //点2
	int point_three; //点3
}Quadrisection;//四等分点Trisection
//随机运动结构体
typedef struct 
{
    uint32_t number; //随机数
    uint16_t sampling; //扫描时间
}Random_t;
//底盘的区域
typedef enum
{
	FIRST_AREA = 0,
	SECOND_AREA = 1,
	THREE_AREA=2,
	FOUR_AREA=3
} CHASSIS_AREA;
//随机运动结构体
typedef struct 
{
	Trisection Point_of_3section;
	Quadrisection Point_of_4section;
    CHASSIS_AREA CHASSIS_AREA_NOW; //此时此刻的区域
    CHASSIS_AREA CHASSIS_AREA_LAST; //上一时刻的区域
	int This_area_stay_times;//此区域停留时间
    uint16_t every_change_inteval_TIME; //间隔时间
}CH_DO_NOT_STOP_AT_ONE_AREA;
extern CH_DO_NOT_STOP_AT_ONE_AREA DO_NOT_STOP;//不要在同一区域停留过久

extern Random_t RANDOM_CHASSIS;

typedef struct
{
			
	//直接用线数的
	int16_t realValue_AB;
	
	int32_t TargerLine;
	
	int16_t lastValue_AB;
	
	int32_t Counts;
	int32_t totalLine;

	P_PID_t P_PID;
	
}Encoder_t;
extern Encoder_t Chassis_Encoder;
extern bool CHASSIS_L_MAX_new;//左右边界值是否更新
extern bool CHASSIS_R_MIN_new;
extern Ramp_Struct CHASSIS;

void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab);

#define OneLoop_LineNumber (1024*4)//编码器一圈

#define chassis_radom_mode_one 0


#define chassis_radom_mode_two 1//达到目标速度就减速




#define chassis_radom_mode_three 0

#define chassis_radom_mode_four 0


#endif

