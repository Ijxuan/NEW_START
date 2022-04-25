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
	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //两个光电传感器都正常时的初始化逻辑
{
   	if(HWswitch_L!=HWswitch_L_last)
	{
			if(HWswitch_L_last==1)//		0<--1
			{
				CHASSIS_L_MAX=M3508s[3].totalAngle+HW_SWITCH_JR;	
CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine+616;		
				CHASSIS_L_MAX_new=1;//边界值已更新
			}
	}
	if(HWswitch_R!=HWswitch_R_last)
	{
			if(HWswitch_R_last==1)//		1-->0
			{
				CHASSIS_R_MIN=M3508s[3].totalAngle-HW_SWITCH_JR;
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine-1590;
				CHASSIS_R_MIN_new=1;//边界值已更新
			}
		
	}
	ENCODER_LONG=	CHASSIS_L_MAX-CHASSIS_R_MIN;

}
else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //两个光电传感器左边好右边坏时的初始化逻辑
{
	//逻辑是左边是好的,那就只检测左边的左边界 ,减速以示区分  
	//更新左边界时,用当前值减去轨道长度
    //待测量:轨道长度        -31970到37   (用编码器测量)  
	//右传感器检测距离为-30419到-32009  1590
	//左传感器检测距离位-579到37        616    
	//得到右边界
   	if(HWswitch_L!=HWswitch_L_last)
	{
			if(HWswitch_L_last==1)//		0<--1
			{
				
				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine       +reverse_by_ENCODER;
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine-200000-reverse_by_ENCODER;//200000这个值还要改,改成轨道长度值
				CHASSIS_L_MAX_new=1;//边界值已更新
			}
		
	}
}
else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //两个光电传感器左边坏右边好时的初始化逻辑
{
	//逻辑是右边是好的,那就只检测右边的右边界 ,减速以示区分  
	//更新右边界时,用当前值减去轨道长度
    //待测量:轨道长度        -31856到78     (用编码器测量)
	//得到左边界
	if(HWswitch_R!=HWswitch_R_last)
	{
			if(HWswitch_R_last==1)////上一次是1,那就是刚刚进入传感器检测距离		1-->0
			{
				CHASSIS_L_MAX_by_ENCODER=Chassis_Encoder.totalLine+20000+reverse_by_ENCODER;//200000这个值还要改,改成轨道长度值(编码器测得)
				CHASSIS_R_MIN_by_ENCODER=Chassis_Encoder.totalLine      -reverse_by_ENCODER;

				CHASSIS_R_MIN_new=1;//边界值已更新
			}
		
	}
}
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
	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //两个光电传感器都正常时的初始化逻辑
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
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.8;//双环	//两个传感器都是好的,速度可以快一点
				}
		}
	}	
	
	else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //两个光电传感器左边好右边坏时的初始化逻辑
	{
		if(CHASSIS_L_MAX_new==0)//左边的的传感器还没检测到
		CHASSIS_trage_angle=9990000;//只有左边的传感器是好的,所以向左运动
		
				if(DR16.rc.s_left==1)//左上档位      //光电没检测到用的是这套PID,改变的是目标角度
				{			
				P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.5;//双环	//只有一个传感器都是好的,速度可以慢一点
				}
		
	}
	
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //两个光电传感器左边坏右边好时的初始化逻辑
	{
		if(CHASSIS_R_MIN_new==0)//右边的的传感器还没检测到
		CHASSIS_trage_angle=-9990000;//只有右边的传感器是好的,所以向右运动
		
				if(DR16.rc.s_left==1)//左上档位      //光电没检测到用的是这套PID,改变的是目标角度
				{			
				P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);
				CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*0.5;//双环	//只有一个传感器都是好的,速度可以慢一点
				}
		
	}
}



