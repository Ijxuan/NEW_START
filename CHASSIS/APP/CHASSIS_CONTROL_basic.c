#include "CHASSIS_CONTROL_basic.h"
#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
#include "CHASSIS_CONTROL_2.h"


#define HW_SWITCH_JR 2000//光电的检测距离  2000  500
#define GD_LONG 999999

bool CHASSIS_L_MAX_new=0;//左右边界值是否更新
bool CHASSIS_R_MIN_new=0;
void switch_change(void)
{
			HWswitch_L			 = HAL_GPIO_ReadPin(GPIOA,HWswitch_1_Pin);
			HWswitch_R   		 = HAL_GPIO_ReadPin(GPIOA,HWswitch_2_Pin);
			//光电变化检测  函数
	if(HWswitch_L!=HWswitch_L_last	)
	{
			if(HWswitch_L_last==1)//		0<--1
			{
				CHASSIS_L_MAX=M3508s[3].totalAngle+HW_SWITCH_JR;
//				ENCODER_L_MAX=Chassis_Encoder.totalLine+8850;
				CHASSIS_L_MAX_new=1;//边界值已更新
			}
		
	}
	if(HWswitch_R!=HWswitch_R_last	)
	{
			if(HWswitch_R_last==1)//		1-->0
			{
				CHASSIS_R_MIN=M3508s[3].totalAngle-HW_SWITCH_JR;
//				ENCODER_R_MIN=Chassis_Encoder.totalLine-8978;
				CHASSIS_R_MIN_new=1;//边界值已更新

			}
		
	}
	
ENCODER_LONG=	CHASSIS_L_MAX-CHASSIS_R_MIN;
			HWswitch_L_last		=HWswitch_L;
			HWswitch_R_last		=HWswitch_R;
	DO_NOT_STOP.Point_of_3section.point_one=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/3;
	DO_NOT_STOP.Point_of_3section.point_two=CHASSIS_L_MAX-(CHASSIS_L_MAX-CHASSIS_R_MIN)/3;
   if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_3section.point_one||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_3section.point_one)
   {
	DO_NOT_STOP.CHASSIS_AREA_NOW=FIRST_AREA;   
   }
   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_two||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_two)   
   {
		DO_NOT_STOP.CHASSIS_AREA_NOW=SECOND_AREA;   
   
   }
   
	DO_NOT_STOP.Point_of_4section.point_one=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/4;
	DO_NOT_STOP.Point_of_4section.point_two=CHASSIS_R_MIN+(CHASSIS_L_MAX-CHASSIS_R_MIN)/2;
	DO_NOT_STOP.Point_of_4section.point_three=CHASSIS_L_MAX-(CHASSIS_L_MAX-CHASSIS_R_MIN)/4;
//   if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_one||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_one)
//   {
//	DO_NOT_STOP.CHASSIS_AREA_NOW=FIRST_PERCENT;   
//   }
//   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_two||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_two)   
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=SECOND_PERCENT;   
//   
//   }
//   else if(M3508s[3].totalAngle<DO_NOT_STOP.Point_of_4section.point_three||M3508s[3].totalAngle==DO_NOT_STOP.Point_of_4section.point_three)   
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=THREE_PERCENT;   
//   }
//   else
//   {
//		DO_NOT_STOP.CHASSIS_AREA_NOW=FOUR_PERCENT;   	   
//   }
   if(DO_NOT_STOP.CHASSIS_AREA_LAST==DO_NOT_STOP.CHASSIS_AREA_NOW)
   {
	   DO_NOT_STOP.This_area_stay_times++;
   }
   else
   {
	   DO_NOT_STOP.This_area_stay_times=0;	   
   }
   DO_NOT_STOP.CHASSIS_AREA_LAST=DO_NOT_STOP.CHASSIS_AREA_NOW;
   
}

void star_and_new()
{

		if(CHASSIS_R_MIN_new==0||CHASSIS_L_MAX_new==0	)	//只有当边界值更新完了才会  真正开始巡航	
		{
					if( HWswitch_L==0)// 左光电感应到了，向右运动
					CHASSIS_trage_angle=-9990000;
					else if(HWswitch_R==0)//	右光电感应到了，向左运动
					CHASSIS_trage_angle=9990000;
	if(DR16.rc.s_left==1)//光电没检测到用的是这套PID,改变的是目标角度
	{			
	P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
	CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.5;//双环	
	}
		}

		
}



