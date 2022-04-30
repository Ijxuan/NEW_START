#include "MY_CHASSIS_CONTROL.h"
#include "CHASSIS_CONTROL_2.h"

#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
//往3508增大的方向是左边,接PA8
//往3508减小的方向是右边,接PA9
Ramp_Struct CHASSIS;

bool Random_CHASSIS_CHOOSE=1;//是否选择随机模式
bool Cruise_CHASSIS_CHOOSE=0;//是否选择巡航模式
int arrive_speed_times=0;
int disable_times=0;
int disable_targe_times=100;//失能持续时间:100  150  200 250  300
int next_disable_start_times=200;//下次失能间隔时间:200  300 400 500
int whether_change_direction=0;//重新使能时是否变向
CH_DO_NOT_STOP_AT_ONE_AREA DO_NOT_STOP;//不要在同一区域停留过久
void CHASSIS_CONTROUL(void)
{
	#if PID_CHASSIS_MOTOR
			#if 1	//底盘随机与巡航运动
					if(DR16.rc.s_left==1)//自动控制
					{
						

	if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //两个光电传感器都正常时的初始化逻辑
	{					
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//只有当边界值更新完了才会  真正开始巡航	
				{
					
				if(DR16.rc.ch4_DW<=-400)//拨上
				{
				Random_CHASSIS_CHOOSE=1;//是选择随机模式
				Cruise_CHASSIS_CHOOSE=0;
				}
				if(DR16.rc.ch4_DW>=400)//拨下
				{
				Cruise_CHASSIS_CHOOSE=1;//是选择巡航模式
				Random_CHASSIS_CHOOSE=0;	
				}  //正式比赛不需要切换巡航模式,就一直随机就好了
				
				if(Cruise_CHASSIS_CHOOSE==1)//是选择巡航模式
//				Cruise_CHASSIS();//巡航模式
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//是选择随机模式
				Random_CHASSIS();//随机模式
//				CHASSIS_trage_speed=0;//锁死//弹道测试后取消注释	
				}
	}
	else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //两个光电传感器左边好右边坏时的初始化逻辑
	{
				if(CHASSIS_L_MAX_new==1	)	//只有当左边界值更新完了才会  真正开始巡航	
				{
					
//				if(DR16.rc.ch4_DW<=-400)//拨上
//				{					
//				Random_CHASSIS_CHOOSE=1;//是选择随机模式
//				Cruise_CHASSIS_CHOOSE=0;
//				}	
//				if(DR16.rc.ch4_DW>=400)//拨下
//				{
//				Cruise_CHASSIS_CHOOSE=1;//是选择巡航模式
//				Random_CHASSIS_CHOOSE=0;		
//				} //正式比赛不需要切换巡航模式,就一直随机就好了
				
				if(Cruise_CHASSIS_CHOOSE==1)//是选择巡航模式
//				Cruise_CHASSIS();//巡航模式
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//是选择随机模式
				Random_CHASSIS();//随机模式
//				CHASSIS_trage_speed=0;//锁死//弹道测试后取消注释	
				}		
		
	}
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //两个光电传感器左边坏右边好时的初始化逻辑
	{
				if(CHASSIS_R_MIN_new==1	)	//只有当左边界值更新完了才会  真正开始巡航	
				{
					
//				if(DR16.rc.ch4_DW<=-400)//拨上
//				{					
//				Random_CHASSIS_CHOOSE=1;//是选择随机模式
//				Cruise_CHASSIS_CHOOSE=0;
//				}	
//				if(DR16.rc.ch4_DW>=400)//拨下
//				{
//				Cruise_CHASSIS_CHOOSE=1;//是选择巡航模式
//				Random_CHASSIS_CHOOSE=0;		
//				} //正式比赛不需要切换巡航模式,就一直随机就好了
				
				if(Cruise_CHASSIS_CHOOSE==1)//是选择巡航模式
//				Cruise_CHASSIS();//巡航模式
				CHASSIS_CONTROUL_2();
				if(Random_CHASSIS_CHOOSE==1)//是选择随机模式
				Random_CHASSIS();//随机模式
//				CHASSIS_trage_speed=0;//锁死//弹道测试后取消注释	
				}				
		
	}
					}
			#endif
	if(DR16.rc.s_left==3)//遥控器控制  左中间
	{
	CHASSIS_trage_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*CHASSIS_MAX_SPEED;//遥控器给速度目标值 二选一		
	}
if(0)//加上速度的斜坡
{	
CHASSIS.Current_Value=M3508s[3].realSpeed;					
CHASSIS.Target_Value=CHASSIS_trage_speed;
CHASSIS_trage_speed_temp=Ramp_Function(&CHASSIS);
					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
	
//	CHASSIS_trage_speed_temp=0;//始终锁死在轨道上//弹道测试后取消注释
					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed_temp,M3508s[3].realSpeed);
	send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;
}
if(1)//速度没有斜坡
{

					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
					P_PID_bate(&CHASSIS_MOTOR_SPEED_pid, CHASSIS_trage_speed,M3508s[3].realSpeed);
	send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result;	
}
//					Power_Calculate();
//					send_to_chassis=CHASSIS_MOTOR_SPEED_pid.result*Chassis_PowerLimit;
#endif
	

	
}




Random_t RANDOM_CHASSIS;

uint16_t Random_CHANGE_times = 400; //  PS:Random_CHANGE_times=600 那么600*3 (3ms进一次这个任务) 1.8毫秒决定一次是否变向

const uint8_t Random_Proportion = 40;      //随机概率(100-Random_Proportion)  例如30 那么随机出来的数大于30就变向  那么变向概率为70%
const uint16_t Random_CHANGE_speed = 2000;      //再次变向要达到这个速度以上

//随机模式
void Random_CHASSIS(void)
{
//    if (abs(CHASSIS_trage_speed) != 6000*Chassis_PowerLimit)
//    {
//        CHASSIS_trage_speed = 3000*Chassis_PowerLimit;//随机运动的基础速度
//    }//随机运动   初始化速度   以Random_Velocity做变向运动
    RANDOM_CHASSIS.number = Get_RandomNumbers_Range(0, 100);
//					if(M3508s[3].totalAngle>(CHASSIS_R_MIN+100000)&&M3508s[3].totalAngle<(CHASSIS_L_MAX-100000))//做实验确定多远变向 负十万到正十万

//    RANDOM_CHASSIS.sampling++;
	
	#if 1
	if(	abs(M3508s[3].realSpeed) >Random_CHANGE_speed)
	    RANDOM_CHASSIS.sampling++;
//	if(DO_NOT_STOP.This_area_stay_times>1000)//在同一区域停留超过3s,开始跑图  失能时这个值太大了,切换成自动_随机运动
//	{
//		RANDOM_CHASSIS.sampling=0;
//		arrive_speed_times=0;
//					stop_CH_OP_BC_LESS=0;

//	}
	
//		if(	arrive_targe_angle==1	)
//	    RANDOM_CHASSIS.sampling++;
	#endif
						Power_Calculate();
if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==1)  //两个光电传感器都正常时的轨道末变向逻辑
{
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+9999999))
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-9999999))//轨道边界变向 负十万到正十万
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}//只要传感器检测到了就反向
			
//						if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
//				//融合编码器,reverse_by_ENCODER是变向提前值
//			{
//				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
//			}
//						if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
//				//融合编码器,reverse_by_ENCODER是变向提前值
//			{
//				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
//			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
//			}
			
}	
else if(state_Infrared_R_is_ok==0&&state_Infrared_L_is_ok==1)  //两个光电传感器   左边好 右边坏   时的轨道末变向逻辑
{
			if(HWswitch_L==0)//轨道边界变向 负十万到正十万
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}//只要左边好的传感器检测到了就反向
			
			if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
				//融合编码器,reverse_by_ENCODER是变向提前值
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}
	
}
	else if(state_Infrared_R_is_ok==1&&state_Infrared_L_is_ok==0)  //两个光电传感器左边坏右边好时的轨道末变向逻辑
	{
	
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+9999999))
				//只要右边好的传感器检测到了就反向
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
				RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}
			
			if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
				//融合编码器,reverse_by_ENCODER是变向提前值
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
			}
			
	}


    if (RANDOM_CHASSIS.sampling >= Random_CHANGE_times)
    {
        if (RANDOM_CHASSIS.number >= Random_Proportion)//是否变向
        {
            CHASSIS_trage_speed = -CHASSIS_trage_speed;
			arrive_targe_angle=0;
			stop_CH_OP_BC_LESS=0;
			speed_change_times++;
        }
        RANDOM_CHASSIS.sampling = 0;//这个函数运行500次才会进入一次变向判断
    }
				if(CHASSIS_trage_speed>0)
			{
				CHASSIS_trage_speed=4000*Chassis_PowerLimit;
			}
			if(CHASSIS_trage_speed<0)
			{
				CHASSIS_trage_speed=-4000*Chassis_PowerLimit;
			}
	
						if(abs(CHASSIS_trage_speed)>2500)
					{
						if(abs(M3508s[3].realSpeed)>(abs(CHASSIS_trage_speed)-200))
						{
						arrive_speed_times++;		
						}
					}
					if(arrive_speed_times>250)//有规律失能
					{
						if(abs(M3508s[3].realSpeed)<(abs(CHASSIS_trage_speed)-2000))
						{
						disable_times++;	
						}
						stop_CH_OP_BC_LESS=1;
//						CHASSIS_trage_speed=0;
					}	
					if(disable_times>50)//每次失能时长:
						{
							arrive_speed_times=0;
							disable_times=0;
							stop_CH_OP_BC_LESS=0;
						}

	
}


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max)
{
	uint32_t rng_number;

	rng_number = HAL_RNG_GetRandomNumber(&hrng);
	
	return rng_number % (max - min + 1) + min;
}


void Cruise_CHASSIS(void)//		cruise	巡航
{
						
			if(HWswitch_R==0&&M3508s[3].totalAngle<(CHASSIS_R_MIN+30000))
			CHASSIS_trage_angle=9900000;
			
			
			if(HWswitch_L==0&&M3508s[3].totalAngle>(CHASSIS_L_MAX-30000))//做实验确定多远变向 负十万到正十万
			CHASSIS_trage_angle=-9900000;
			
			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result;//双环
			
				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
				//CHASSIS_MID-CHASSIS_R_MIN   一半行程
				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//最多减慢百分之70
				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//现在是中间快，两边慢
	
	
	
	
//    if (fabs(Chassis.Velocity.temp_Speed) != Cruise_Velocity)
//    {
//        Chassis.Velocity.temp_Speed = Cruise_Velocity;
//    }
}
Encoder_t Chassis_Encoder;
//获取编码器值函数
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab)
{
	
	Chassis_Encoder->realValue_AB = (short)__HAL_TIM_GET_COUNTER(htim_ab);
	
	if(Chassis_Encoder->realValue_AB - Chassis_Encoder->lastValue_AB < -3600)
	{
		Chassis_Encoder->Counts ++;
	}
	if(Chassis_Encoder->lastValue_AB - Chassis_Encoder->realValue_AB < -3600)
	{
		Chassis_Encoder->Counts --;
	}
	
	Chassis_Encoder->totalLine = Chassis_Encoder->realValue_AB + Chassis_Encoder->Counts * OneLoop_LineNumber;
	
	Chassis_Encoder->lastValue_AB = Chassis_Encoder->realValue_AB;
	M3508_3ms_change=M3508s[3].totalAngle-M3508_3ms_ago;
	ENCODER_SPEED=Chassis_Encoder->totalLine-Chassis_Encoder->TargerLine;
//	ENCODER_ADD=Chassis_Encoder->realValue_AB-Chassis_Encoder->lastValue_AB;
	if(M3508_3ms_change!=0)//在运动
	{
	encoder_fbl_k=	(ENCODER_SPEED*1.0)/M3508s[3].realSpeed;
	}
	
//	int M3508_3ms_ago_total_angle;//3毫秒以前的值
//int M3508_3ms_ago_speed;//3毫秒改变的值
//float M3508_speed_angle_kp;//角度与速度的关系
	
	M3508_speed_angle_kp=M3508s[3].realSpeed/1.0/(M3508s[3].totalAngle-M3508_3ms_ago);//2.45
	Chassis_Encoder->TargerLine=Chassis_Encoder->totalLine;
	M3508_3ms_ago=	M3508s[3].totalAngle;
    M3508_3ms_ago_speed=M3508s[3].realSpeed;
}



/***************************************
  * @brief  :底盘功率计算
  * @param  :Judge_DATA.Power,Judge_DATA.Power_Buffer,Power_MAX,Power_Buffer_MAX
****************************************/
void Power_Calculate()
{
	static uint8_t flag=0;
	static bool flag_kb=0;//保证进入狂暴模式时,不是在恢复功率

	if(ext_power_heat_data.data.chassis_power_buffer < 150)flag = 1;  
	//缓冲功率小于 50  开始限制功率
	if(flag==1 && ext_power_heat_data.data.chassis_power_buffer > 180)flag=0;
	//缓冲功率增加到  150  不再限制功率
	
	if(flag==1)Chassis_PowerLimit = 1.00f;  //限制功率 在发给电机前 乘以0.65
	else Chassis_PowerLimit =1.3500f ;//; 0.8        //不再限制功率 在发给电机前 乘以1.0
	
	if(hurt_times_ago<1000)//被击中后3s内
	{
		if(flag_kb==0)
		{
			flag=0;//保证进入狂暴模式时,不是在恢复功率
			flag_kb=1;
		}
//						Chassis_PowerLimit=KB_add_speed;//狂暴加速度

	if(ext_power_heat_data.data.chassis_power_buffer < KB_low_JB)flag = 1;  
	//缓冲功率小于 50  开始限制功率
	if(flag==1 && ext_power_heat_data.data.chassis_power_buffer > KB_high_JB)flag=0;
	//缓冲功率增加到  150  不再限制功率
	
	if(flag==1)Chassis_PowerLimit = 1.15f;  //限制功率 在发给电机前 乘以0.65
	else Chassis_PowerLimit = 1.65;         //不再限制功率 在发给电机前 乘以1.0
		
		
		Random_CHANGE_times=300;//随机变向频率
	}
	else
	{
		flag_kb=0;
				Random_CHANGE_times=700;//随机变向频率变高

	}
	
}













