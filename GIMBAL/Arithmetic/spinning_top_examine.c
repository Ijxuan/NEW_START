#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"
#include "INS_task.h"
#include "Vision.h"

/**
  ****************************(C) COPYRIGHT 2022 YL****************************
  * @file       spinning_top_examine.c/h
  * @brief      
  *             
  *            
  *             主要通过自瞄效果判断小陀螺
  *             
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     5-26-2022       JQX              1. 
  *  V2.0.0     
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 YL****************************
  */
int examine_run_times=0;
int8_t YAW_MOTION_STATE=0;//运动状态

int examine_sampling_period=20;//小陀螺检测的周期,即间隔多久进行一次采样
float examine_sampling_extent=1;//变化的角度大于多少度才算进去
float total_yaw_last_time;//上一时刻的角度
float total_yaw_change;//变化的角度

int yaw_angle_is_add=0;
int yaw_add_judge_threshold=4;//判断阈值
int vision_is_lock_in_1s=0;//一秒内是锁定状态
void S_T_examine(void)
{
	if(DR16.rc.s_left==1)
	{
	examine_run_times++;//这个档有自瞄
		if(VisionData.RawData.Armour==1)
		{
		 vision_is_lock_in_1s=0;     //识别到了装甲
		}
		else{
			vision_is_lock_in_1s++;//丢失装甲多少ms
		}
	if(vision_is_lock_in_1s<1000)
	{
if(examine_run_times%examine_sampling_period==0)
{	
	
	total_yaw_change=DJIC_IMU.total_yaw-total_yaw_last_time;

	if((DJIC_IMU.total_yaw-total_yaw_last_time)>examine_sampling_extent)
	{
yaw_angle_is_add++;	
	}
	if(yaw_angle_is_add>yaw_add_judge_threshold)
	{
	YAW_MOTION_STATE=2;
	}
total_yaw_last_time=DJIC_IMU.total_yaw;	
}

	}		


	}
	else{
	examine_run_times=0;//有自瞄
	}
	
	

	
	
}












