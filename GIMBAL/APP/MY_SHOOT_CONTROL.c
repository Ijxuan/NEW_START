#include "MY_SHOOT_CONTROL.h"
#include "main.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "Vision.h"
#include "calibrate_task.h"


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
int if_Driver_arrive_angle=5000;

bool weather_error_less_than_1=0;
	
allow_auto_shoot auto_shoot_condition;
int auto_shoot_condition_show;//自动射击条件展现

bool JR_HEAT_renew=0;//进入枪口热量回复模式
void shoot_control(void)
{
	if(in_END==0&&in_MID==1)
	{
		auto_shoot_condition.not_in_track_end=1;//不在轨道末端
	}
	else
	{
		auto_shoot_condition.not_in_track_end=0;//不在轨道末端		
	}
	
	if(vision_shoot_times>1)
	{
		auto_shoot_condition.vision_shoot_is_continuous=1;//视觉发射指令是连续的
	}
	else
	{
		auto_shoot_condition.vision_shoot_is_continuous=0;//视觉发射指令是连续的		
	}
	
	if(VISION_Yaw_IMU_Angle_pid.Error>-1.0f||VISION_Yaw_IMU_Angle_pid.Error<1.0f)
	{
	    auto_shoot_condition.weather_angle_error_less_than_1=1;	//角度误差小于一
	}
	else
	{
	    auto_shoot_condition.weather_angle_error_less_than_1=0;	//角度误差小于一		
	}
	
	if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<200)//热量没超过200
	{
		auto_shoot_condition.heat_allow=1;//热量允许
	}
	else
	{
		auto_shoot_condition.heat_allow=0;//热量允许		
	}
	
	
	if(auto_shoot_condition.heat_allow==1/*热量允许*/
	 &&auto_shoot_condition.weather_angle_error_less_than_1==1	/*角度误差小于一*/ 
	 &&auto_shoot_condition.vision_shoot_is_continuous==1/*视觉发射指令是连续的*/
//	 &&auto_shoot_condition.not_in_track_end==1/*不在轨道末端*/
	  )
	{
	auto_shoot_condition.ALL_condition_satisfaction=1;	//所有条件全部满足
	}
	else
	{
	auto_shoot_condition.ALL_condition_satisfaction=0;			
	}
	auto_shoot_condition_show=0;
	if(auto_shoot_condition.heat_allow==1)
		auto_shoot_condition_show+=1;
	if(auto_shoot_condition.weather_angle_error_less_than_1==1)
		auto_shoot_condition_show+=10;
	if(auto_shoot_condition.vision_shoot_is_continuous==1)
		auto_shoot_condition_show+=100;	
	if(auto_shoot_condition.not_in_track_end==1)
		auto_shoot_condition_show+=1000;
	if(auto_shoot_condition.ALL_condition_satisfaction==1)
		auto_shoot_condition_show+=10000;
	
			if(DR16.rc.s_left==2)//遥控器左下 激光瞄准控制
			{	
				ch4_DW_total+=DR16.rc.ch4_DW;

						if(DR16.rc.ch4_DW<=-100)//拨上
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);

			}
			if(DR16.rc.ch4_DW>=200)//拨下
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);
//					cali_sensor[0].cali_cmd = 1;//陀螺仪校准
			}
			 
			}
			if(VisionData.RawData.Beat==0)//
				{
		SHOOT_STOP_time++;
				}
			else if(VisionData.RawData.Beat==1)//
				{
		SHOOT_STOP_time=0;
				}
	
	if(DR16.rc.s_left==3)
	{
			if(DR16.rc.s_right==1||DR16.rc.s_right==3)//左中手动档,右中和右上
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
			if(JR_HEAT_renew==1&&ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<100)//热量恢复到100以下
			{
			JR_HEAT_renew=0;
			}
				if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<200&&JR_HEAT_renew==0)//热量没超过200,不在热量恢复模式下  //注意是枪管1的热量
				{
							if(DW_DOWN==20)
			M2006_targe_angle+=(Driver_add);//8*3=24
						if(DW_DOWN%50==0&&DW_DOWN>200)//理论一秒20发
			M2006_targe_angle+=Driver_add;//8*3=24			
				
				
				}
				else if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat>=200)
				{
				JR_HEAT_renew=1;//进入枪口热量恢复模式
				
				}
				if(JR_HEAT_renew==1)
				{
									if(DW_DOWN%100==0&&DW_DOWN>200)//理论一秒10发
			M2006_targe_angle+=Driver_add;//8*3=24		
				}
//		if(DW_DOWN==20)
//			M2006_targe_angle+=(Driver_add/4);//8*3=24
//				if(DW_DOWN%100==0&&DW_DOWN>200)
//			M2006_targe_angle+=Driver_add;//8*3=24		
				
			}

			
if(DR16.rc.ch4_DW<=-100)//拨上
			{
								DW_UP++;//没用到了

//		if(DW_UP==100)
//			M2006_targe_angle+=(Driver_add/10);//8*3=24
									if(DW_DOWN%50==0&&DW_DOWN>200)//理论一秒20发
			M2006_targe_angle+=Driver_add;//8*3=24		

				
			}
		}
			
		
		
	}
	
		if(DR16.rc.s_left==1)//遥控器控制  左中或左上
	{
		if(M3508s[1].totalAngle>(M2006_targe_angle*0.8-if_Driver_arrive_angle))
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
#if 0
			if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<200)//热量没超过200
			{
			if (shoot_times_for_limit<50)
			{
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
//						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					
				
			}
			else if(shoot_times_for_limit>=50)
			{
			shoot_times_for_limit=0;
			this_period_has_shoot=0;
				whether_shoot_in__this_period=0;
				M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除 防连发

			}
		}
			else if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat>=200)//热量超过200
		{
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
			else if(shoot_times_for_limit>=110)
			{
			shoot_times_for_limit=0;
			this_period_has_shoot=0;
				whether_shoot_in__this_period=0;
				M2006_targe_angle=M3508s[1].totalAngle;////拨盘误差消除 防连发

			}
		}
		if(whether_shoot_in__this_period==1&&this_period_has_shoot==0)//判断一下:这个周期是否发射(如果是) 并且 这个周期是否已经发射(如果没有)
				{
					if(JS_RC_TIMES<targe_shoot_number)//一次打多少 还有子弹发射数允许
					{
						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						this_period_has_shoot_number++;
						this_period_has_shoot=1;
					}
				}
					if(VisionData.RawData.Beat==1)
					{
						SHOOT_from_V++;
					}
		
							if(Armour_lose_time>40)//40ms没检测到装甲板
				{				
					M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除
				}
					if(SHOOT_STOP_time>200)//200ms没检测到开火指令
				{				
					M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除
				}

#endif
#if 0
						if (shoot_times_for_limit<200)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//击打标志位为1并且连续收到4帧
					{
						SHOOT_from_V++;

						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//连续收到10次停火指令 清除拨盘目标角度累计
					}
	
			}
			else if(shoot_times_for_limit>=200)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除 半秒清除累计目标值一次 防止连发
			}
#endif
#if 0//不在末端
						if (shoot_times_for_limit<200)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//击打标志位为1并且连续收到4帧
					{
						SHOOT_from_V++;
						if(	in_MID==1)//不在末端
						{
						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						whether_shoot_in__this_period=1;
						}
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//连续收到10次停火指令 清除拨盘目标角度累计
					}
	
			}
			else if(shoot_times_for_limit>=200)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除 半秒清除累计目标值一次 防止连发
			}
#endif
#if 1//不在末端+误差小于1度
						if (shoot_times_for_limit<333)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>0)//击打标志位为1并且连续收到4帧
					{
						SHOOT_from_V++;
						if(VISION_Yaw_IMU_Angle_pid.Error>1.5||VISION_Yaw_IMU_Angle_pid.Error<-1.5)
						{
						weather_error_less_than_1=0;	
						}
						else//误差绝对值小于1
						{
						weather_error_less_than_1=1;	
						}
						if(disable_for_test_CHASSIS==0)
						{
						if(in_MID==1&&weather_error_less_than_1==1&&whether_shoot_in__this_period==0)//不在末端in_MID==1&&
						{
						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						whether_shoot_in__this_period=1;
						}
					    }
						else if(disable_for_test_CHASSIS==1)
						{
						if(weather_error_less_than_1==1&&whether_shoot_in__this_period==0)//不在末端in_MID==1&&
						{
						M2006_targe_angle+=Driver_add;//8*3=24  打一发
						whether_shoot_in__this_period=1;
						}	
						}
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>100)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//连续收到10次停火指令 清除拨盘目标角度累计
					}
	
			}
			else if(shoot_times_for_limit>=333)
			{
				whether_shoot_in__this_period=0;
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除 半秒清除累计目标值一次 防止连发
			}
#endif
#if 0
						if (shoot_times_for_limit<500)
			{
					if(VisionData.RawData.Beat==1&&vision_shoot_times>2)//击打标志位为1并且连续收到4帧
					{
						SHOOT_from_V++;
						if(Driver_arrive==1)
						{
						M2006_targe_angle+=Driver_add*0.8;//8*3=24  打一发
								Driver_arrive=0;
						}
						whether_shoot_in__this_period=1;
						VisionData.RawData.Beat=0;
					}
					if(SHOOT_STOP_time>10)
					{
			M2006_targe_angle=M3508s[1].totalAngle;//连续收到10次停火指令 清除拨盘目标角度累计
					}
	
			}
			else if(shoot_times_for_limit>=500)
			{
			shoot_times_for_limit=0;
			M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除 半秒清除累计目标值一次 防止连发
			}
#endif
					SHOOT_from_v_last=VisionData.RawData.Beat;


				
//					if(VisionData.RawData.Beat==0)
//								M2006_targe_angle=M3508s[1].totalAngle;//拨盘误差消除

//					if(SHOOT_from_v_last==VisionData.RawData.Beat)
//						vision_soot_last_is_same=1;
//					else
//					{
//						vision_soot_last_is_same=0;
//					SHOOT_from_V=0;
//					}
					
					
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








