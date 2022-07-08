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
  *             ��Ҫͨ������Ч���ж�С����
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
int8_t YAW_MOTION_STATE=0;//�˶�״̬
/* 
    1 δ��⵽��仯,�ڵ�0������
	
    2 ��⵽����ֵ������ֵ ��ʼ��һ���������
    3 ��⵽����ֵ������ֵ ��ʼ�ڶ������������
    4 ��⵽����ֵ������ֵ ��ʼ�������������

    6 ��⵽��СֵС����ֵ ��ʼ��һ���������
    8 ��⵽��СֵС����ֵ ��ʼ�ڶ����������
    10 ��⵽��СֵС����ֵ ��ʼ�������������

    12  �Ѿ���С����ģʽ��
*/

int examine_sampling_period=30;//С���ݼ�������,�������ý���һ�β���
float examine_sampling_extent=1.2;//�仯�ĽǶȴ��ڶ��ٶȲ����ȥ
float total_yaw_last_time;//��һʱ�̵ĽǶ�
float total_yaw_change;//�仯�ĽǶ�

int yaw_angle_is_add=0;
int yaw_angle_is_small_change=0;//���ڿ�ʼ��,С���ȱ仯

int yaw_add_judge_threshold=4;//�ж���ֵ
int vision_is_lock_in_1s=0;//һ����������״̬
int over_time=9999;
void S_T_examine(void)
{
	if(DR16.rc.s_left==1)
	{
	examine_run_times++;//�����������
		if(VisionData.RawData.Armour==1)
		{
		 vision_is_lock_in_1s=0;     //ʶ����װ��
		}
		else{
		vision_is_lock_in_1s++;//��ʧװ�׶���ms
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
		over_time=examine_run_times+200;//0.2���˳�
//		printf("��⵽�����仯,��ʼ��һ���������		");
	}
	if(total_yaw_change<-examine_sampling_extent)
	{
		YAW_MOTION_STATE=6;
		over_time=examine_run_times+200;//0.2���˳�
//		printf("��⵽�����仯,��ʼ��һ���������		");
	}	
	}
		if(YAW_MOTION_STATE==7)
	{
		if(total_yaw_change>0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=12;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
					over_time=examine_run_times+5000;//5���˳�

			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==6)
	{
		if(total_yaw_change>0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽��һ�����ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=7;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("��һ�����ڽ���	");
//			printf("����С����ģʽ	");
//			YAW_MOTION_STATE=12;
		}
//		printf("\r\n");
	}
		if(YAW_MOTION_STATE==3)
	{
		if(total_yaw_change<-0.3f)//С�仯�����ֵ
		{
			yaw_angle_is_small_change++;
//			printf("��⵽�ڶ������ڵ�%d��С�仯	",yaw_angle_is_small_change);
		over_time=examine_run_times+200;//0.2���˳�
		
		}
		if(yaw_angle_is_small_change>=3)
		{
//			YAW_MOTION_STATE=3;//��ʼ�ڶ������������
			yaw_angle_is_small_change=0;
//			printf("�ڶ������ڽ���	");
//			printf("����С����ģʽ	");
			YAW_MOTION_STATE=12;
								over_time=examine_run_times+5000;//5���˳�

		}
//		printf("\r\n");
	}
	if(YAW_MOTION_STATE==2)
	{
		if(total_yaw_change<-0.3f)
		{
			yaw_angle_is_small_change++;
//			printf("��⵽��һ�����ڵ�%d��С�仯	",yaw_angle_is_small_change);
					over_time=examine_run_times+200;//0.2���˳�

		}
		if(yaw_angle_is_small_change>=3)
		{
			YAW_MOTION_STATE=3;//��ʼ�ڶ������������
//			printf("��һ�����ڽ���	");
			yaw_angle_is_small_change=0;
		}
//		printf("\r\n");

	}
		if(YAW_MOTION_STATE==2||YAW_MOTION_STATE==3||YAW_MOTION_STATE==6||YAW_MOTION_STATE==7)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("��ⳬʱ,�˳����ģʽ	");
//					printf("\r\n");

		}
			
		}
				if(YAW_MOTION_STATE==12)
		{
		if(examine_run_times>over_time)
		{
		YAW_MOTION_STATE=1;
//			printf("��С����ģʽ����10��,�˳�С����ģʽ	");
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
	examine_run_times=0;//������
		over_time=5000;
	}
	
	

	
	
}












