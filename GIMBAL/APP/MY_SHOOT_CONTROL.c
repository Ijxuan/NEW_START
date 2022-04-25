#include "MY_SHOOT_CONTROL.h"
#include "main.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "Vision.h"


#define SHOOT_SPEED_HIGH 6700//射速 高
int shoot_speed_text=6750;
int SHOOT=0;
int SHOOT_from_V=0;
bool vision_soot_last_is_same=0;
bool SHOOT_from_v_last=1;


int DW_FREE=0;
int DW_DOWN=0;
int DW_UP=0;
long M2006_targe_angle=0;
bool Driver_arrive=1;
int if_Driver_arrive_angle=2000;
void shoot_control(void)
{
			if(DR16.rc.s_left==2)//遥控器左XIA
			{
						if(DR16.rc.ch4_DW<=-100)//拨上
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);

			}
			if(DR16.rc.ch4_DW>=200)//拨下
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);

			}
			
			}
	
	if(DR16.rc.s_left==3)
	{
//		
					if(DR16.rc.ch4_DW==0)//松手
					{
						DW_FREE++;
						DW_UP=0;DW_DOWN=0;
							M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除

					}	
if(DR16.rc.ch4_DW>=200)//拨下
			{	
			DW_DOWN++;	
		if(DW_DOWN==20)
			M2006_targe_angle+=Driver_add;//8*3=24
				if(DW_DOWN%100==0&&DW_DOWN>200)
			M2006_targe_angle+=Driver_add;//8*3=24		
				
			}

			
if(DR16.rc.ch4_DW<=-100)//拨上
			{
								DW_UP++;//没用到了

		if(DW_UP==100)
			M2006_targe_angle-=(Driver_add/3);//8*3=24


				
			}
			
			
		
		
	}
	
		if(DR16.rc.s_left==1)//遥控器控制  左中或左上
	{
		if(M3508s[1].totalAngle>(M2006_targe_angle-if_Driver_arrive_angle))
		Driver_arrive=1;

//									if(DR16.rc.ch4_DW<=-100)//拨上
//			{		
	if(DR16.rc.s_right==3||DR16.rc.s_right==2)
	{
								M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除
shoot_times_for_limit=0;
	
	}
		if(DR16.rc.s_right==2)
		{
			targe_shoot_number=JS_RC_TIMES+every_shoot_number;
		}

		if(DR16.rc.s_right==1)
		{
			shoot_times_for_limit++;
			if (shoot_times_for_limit<100)
			{
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
//						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					
				
			}
			else if(shoot_times_for_limit>=100)
			{
			shoot_times_for_limit=0;
			this_period_has_shoot=0;
				whether_shoot_in__this_period=0;
			}
				
			if(whether_shoot_in__this_period==1&&this_period_has_shoot==0)
				{
					if(JS_RC_TIMES<targe_shoot_number)
					{
				M2006_targe_angle+=Driver_add;//8*3=24  打一发
					this_period_has_shoot_number++;
					}
					this_period_has_shoot=1;
				}
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
//						M2006_targe_angle+=Driver_add;//8*3=24  打一发
					}
									if(	lose_time>200)
				{				
					M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除
				}
//					if(VisionData.RawData.Beat==0)
//								M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除

//					if(SHOOT_from_v_last==VisionData.RawData.Beat)
//						vision_soot_last_is_same=1;
//					else
//					{
//						vision_soot_last_is_same=0;
//					SHOOT_from_V=0;
//					}
					
					SHOOT_from_v_last=VisionData.RawData.Beat;
					
//					if(vision_shoot_times>0)//连续1次收到视觉的发射指令
//					{
//					SHOOT_from_V=0;
//						vision_shoot_times=0;
//					}
				}
		
				DW_FREE=0;
					SHOOT++;				



				if(DW_FREE>20)
				{
					SHOOT++;
					DW_FREE=0;
				}
			}
		
//M2006_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//遥控器给速度目标值 二选一		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//右摩擦轮速度目标值应该是正值
			
M2006_targe_speed=P_PID_bate(&Driver_ANGLE_pid,M2006_targe_angle,M3508s[1].totalAngle);//M2006_targe_speed应该大于0
	
	
send_to_2006=I_PID_Regulation(&Driver_I_PID,M2006_targe_speed,M3508s[1].realSpeed);
			
			
			

	if(DR16.rc.s_left==2||DR16.rc.s_left==0)	//失能保护
					{			
						M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除
						Driver_arrive=1;//失能,同事也抵到目标
					}
	//没有遥控器和遥控器拨到左下挡位时拨盘失能
	//在左中和左上挡位时
	
	
	
	
//		if(DR16.rc.s_left==1)//遥控器控制  左上
//SHOOT_L_speed=500;
//	if(DR16.rc.s_left==1)//遥控器控制  左中
//					{
//SHOOT_L_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//遥控器给速度目标值 二选一	
//if(DR16.rc.ch3<-600)	
			if(DR16.rc.s_left==3)//遥控器控制  左中
     	{		
			
			if (DR16.rc.s_right == 3) //中间档位 
			SHOOT_L_speed=shoot_speed_text;
			else if(DR16.rc.s_right ==1)
			SHOOT_L_speed=500;//右上
			else if(DR16.rc.s_right ==2)
			SHOOT_L_speed=0;
			
		}
		
					if(DR16.rc.s_left==1)//遥控器控制  左上
     	{				
			SHOOT_L_speed=-6800;//自瞄摩擦轮速度
		}
		
		
		
		
		SHOOT_R_speed=SHOOT_L_speed;	
		
		if(	SHOOT_R_speed<0)
			SHOOT_R_speed=-SHOOT_R_speed;//右摩擦轮速度目标值应该是 值
		if(	SHOOT_L_speed>0)
			SHOOT_L_speed=-SHOOT_L_speed;//左摩擦轮速度目标值应该是 值

send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_R_speed,M3508s[3].realSpeed);//gai//蓝线

send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_L_speed,M3508s[2].realSpeed);   //红线
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);

//	if(DR16.rc.s_left==3)//遥控器控制  左中
//					{
//SHOOT_L=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//遥控器给速度目标值 二选一		
//				SHOOT_R=SHOOT_L;					
//		if(	SHOOT_L>0)
//		SHOOT_L=-SHOOT_L;//左摩擦轮速度目标值应该是负值
//		if(	SHOOT_R<0)
//		SHOOT_R=-SHOOT_R;//右摩擦轮速度目标值应该是正值
//send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_L,M3508s[0].realSpeed);

//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);
	
//					}
	
}

void driver_plate_control(void)
{
//	if(DR16.rc.s_left==3||DR16.rc.s_left==1)//遥控器控制  左中或左上

//driver_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//遥控器给速度目标值 二选一		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//右摩擦轮速度目标值应该是正值
//send_to_driver=I_PID_Regulation(&Driver_I_PID,driver_targe_speed,M3508s[2].realSpeed);





}








