#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"
#include "INS_task.h"
#include "Vision.h"
//#include "stdio.h"

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
/* 
    1 未检测到大变化,在第0个周期
	
    2 检测到增大值大于阈值 开始第一个检测周期
    3 检测到增大值大于阈值 开始第二个个检测周期
    4 检测到增大值大于阈值 开始第三个检测周期

    6 检测到减小值小于阈值 开始第一个检测周期
    8 检测到减小值小于阈值 开始第二个检测周期
    10 检测到减小值小于阈值 开始第三个检测周期

    12  已经在小陀螺模式了
*/

int examine_sampling_period=30;//小陀螺检测的周期,即间隔多久进行一次采样
float examine_sampling_extent=1.2;//变化的角度大于多少度才算进去
float total_yaw_last_time;//上一时刻的角度
float total_yaw_change;//变化的角度

int yaw_angle_is_add=0;
int yaw_angle_is_small_change=0;//周期开始后,小幅度变化

int yaw_add_judge_threshold=4;//判断阈值
int vision_is_lock_in_1s=0;//一秒内是锁定状态
int over_time=9999;
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

//	if((DJIC_IMU.total_yaw-total_yaw_last_time)>examine_sampling_extent)
	if(YAW_MOTION_STATE==1)
	{
	if(total_yaw_change>examine_sampling_extent)
	{
		YAW_MOTION_STATE=2;
		over_time=examine_run_times+200;//0.2秒退出
//		printf("检测到正向大变化,开始第一个检测周期		");
	}
	if(total_yaw_change<-examine_sampling_extent)
	{
		YAW_MOTION_STATE=6;
		over_time=examine_run_times+200;//0.2秒退出
//		printf("检测到反向大变化,开始第一个检测周期		");
	}	
	}
		if(YAW_MOTION_STATE==7)
	{
		if(total_yaw_change>0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=12;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
					over_time=examine_run_times+5000;//5秒退出

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==6)
	{
		if(total_yaw_change>0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第一个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=7;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第一个周期结束	");
//			printf("进入小陀螺模式	");
//			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==3)
	{
		if(total_yaw_change<-0.3f)//小变化检测阈值
		{
			yaw_angle_is_small_change++;
//			printf("检测到第二个周期第%d个小变化	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2秒退出
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//开始第二个个检测周期
			yaw_angle_is_small_change=0;
//			printf("第二个周期结束	");
//			printf("进入小陀螺模式	");
			YAW_MOTION_STATE=12;
								over_time=examine_run_times+5000;//5秒退出

		}
//		printf("\r\n");
	}
	if(YAW_MOTION_STATE==2)
	{
		if(total_yaw_change<-0.3f)
		{
			yaw_angle_is_small_change++;
//			printf("检测到第一个周期第%d个小变化	",yaw_angle_is_small_change);
					over_time=examine_run_times+200;//0.2秒退出

		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=3;//开始第二个个检测周期
//			printf("第一个周期结束	");
			yaw_angle_is_small_change=0;
		}
//		printf("\r\n");

	}
		if(YAW_MOTION_STATE==2||YAW_MOTION_STATE==3||YAW_MOTION_STATE==6||YAW_MOTION_STATE==7)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("检测超时,退出检测模式	");
//					printf("\r\n");

		}
			
		}
				if(YAW_MOTION_STATE==12)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("在小陀螺模式超过10秒,退出小陀螺模式	");
//					printf("\r\n");

		}
			
		}
	if(yaw_angle_is_add>yaw_add_judge_threshold)
	{
		yaw_angle_is_add++;	
	}
	
total_yaw_last_time=DJIC_IMU.total_yaw;	
}

	}		


	}
	else{
	examine_run_times=0;//有自瞄
		over_time=5000;
	}
	
	

	
	
}












