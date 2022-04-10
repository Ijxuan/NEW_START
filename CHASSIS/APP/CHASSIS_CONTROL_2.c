#include "CHASSIS_CONTROL_2.h"
#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
int speed_has_change=0;
int last_Speed=0;
int xunen_times=0;
float xunen_percent=1.5;
void CHASSIS_CONTROUL_2()
{
				stop_CH_OP_BC_LESS=0;//这个模式用不到	

				if(M3508s[3].totalAngle<(CHASSIS_R_MIN+200000))
				{
				CHASSIS_trage_angle=9900000;		
				}
			else if(M3508s[3].totalAngle>(CHASSIS_L_MAX-200000))//
			{
	CHASSIS_trage_angle=-9900000;			
			}
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
			{
				stop_CH_OP_BC_END_times=0;
								xunen_times++;

//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;
//				if(last_Speed>M3508s[3].realSpeed&&M3508s[3].realSpeed>0)//已经反向了
								if(M3508s[3].realSpeed>0)//已经反向了
{
	speed_has_change=1;
}

if(speed_has_change==0)
{
	if(	xunen_times>4)//弹簧冲能前加速的时间
	{
stop_CH_OP_BC_END=1;
//xunen_percent=0.7;		
	}
	else 
	{
//xunen_percent=1.0;
	}
}	
else
{
	stop_CH_OP_BC_END=0;
	CHASSIS_trage_angle=9900000;
}
	if(	xunen_times>2000)//说明出了意外,肯定是卡死了,不管了,直接走
	{
	stop_CH_OP_BC_END=0;
	CHASSIS_trage_angle=9900000;
	}

			}
			else if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//
//			else if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-3000))//
			{
				stop_CH_OP_BC_END_times=0;
				xunen_times++;
//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;	
//				if(M3508s[3].realSpeed<0&&M3508s[3].realSpeed>last_Speed)//速度一反还要速度下降也就是弹簧伸到最长才开始计算蓄能时间,然后启动电机
				if(M3508s[3].realSpeed<0)//只要速度一反就开始计算蓄能时间,然后启动电机

{
	
	speed_has_change=1;
}
if(speed_has_change==0)
{
	if(	xunen_times>4)//弹簧冲能前加速的时间
	{
stop_CH_OP_BC_END=1;
//xunen_percent=0.7;		
	}
	else 
	{
//xunen_percent=1.0;
	}	
		
}
else
{
	stop_CH_OP_BC_END=0;
	CHASSIS_trage_angle=-9900000;
}
	if(	xunen_times>2000)//说明出了意外,肯定是卡死了,不管了,直接走
	{
	stop_CH_OP_BC_END=0;
	CHASSIS_trage_angle=-9900000;
	}
			}
			else//不在两个柱子之间
			{
//				xunen_percent=1.5;

				stop_CH_OP_BC_END_times++;
				if(stop_CH_OP_BC_END_times>200)//避免蓄能没到时间就出了轨道边界判断
					stop_CH_OP_BC_END=0;//

				speed_has_change=0;
				xunen_times=0;
			}
		
			last_Speed=M3508s[3].realSpeed;
			
			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*xunen_percent;//双环
//			CHASSIS_trage_speed=0;
//				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
				//CHASSIS_MID-CHASSIS_R_MIN   一半行程
//				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
//				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//最多减慢百分之70
//				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//现在是中间快，两边慢
//		
	
	
	
	
}



#if chassis_radom_mode_two ==1
bool arrive_targe_angle=0;

bool just_arrive_targe_speed(int targe_speed)
{
	if(abs(M3508s[3].realSpeed)  >targe_speed||abs(M3508s[3].realSpeed)  ==targe_speed)
	{
		arrive_targe_angle=1;
		return 1;
	}
	return 0;
}

#endif








