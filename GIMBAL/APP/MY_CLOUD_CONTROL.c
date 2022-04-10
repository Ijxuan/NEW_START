#include "MY_CLOUD_CONTROL.h"


void cloud_control(void)
{
	
//								if(DR16.rc.s_left==1)//YAW轴控制挡位
//							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW轴遥控器控制
//							if(DR16.rc.ch4_DW<=-400)//拨上
//							yaw_trage_angle=yaw_trage_angle2;//陀螺仪角速度最大为140
//							/*
//							所以目标阶跃要大于140/0.03   4666
//							//还要考虑29000的输出多久才能到140这个最大速度
//							//先给10000的阶跃吧
//							现在是800
//*/
////							if(DR16.rc.ch4_DW>=400)//拨下
////							yaw_trage_angle=0;
//							}
///*					else
//					从-10000到30000，最开始20000是上升加速段，然后是10000的220，最后是10000的减速		
//					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

//*/
//	
//
 scan_cloud();

							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	
//							PITCH_trage_angle=0;//保持水平位置
							imu_angle();//去仿真做测试！！！  已做
//									PITCH_trage_angle=28;

							//陀螺仪零漂限幅
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}
bool YAW_TARGE_ANGLE_ADD=1;
int arrive_targe_angle=0;
int TEXT_targe_SPEED=0;

void YAW_PID()
{
	
									if(DR16.rc.s_left==1)//YAW轴控制挡位
							{
	if(DJIC_IMU.total_yaw>CLOUD_enable_imu+70.0)	
	{
		
//TEXT_targe_SPEED		
//		arrive_targe_angle++;
//		if(arrive_targe_angle>20)
//		{
cloud_text_add=0;
			arrive_targe_angle=0;
//		}
	}	
	
	if(DJIC_IMU.total_yaw<CLOUD_enable_imu-70.0)	
	{
		
		
//		arrive_targe_angle++;
//		if(arrive_targe_angle>20)
//		{
cloud_text_add=1;
			arrive_targe_angle=0;
//	}
	}	
	
	if(cloud_text_add==1)//增加
	{
//yaw_trage_speed=TEXT_targe_SPEED;
	}	
	if(cloud_text_add==0)//增加
	{
//yaw_trage_speed=-TEXT_targe_SPEED;
	}	
	
//if(cloud_enable==0)
//{
//}
							}	
		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //失能保护
		{
CLOUD_enable_moto=GM6020s[0].totalAngle;
CLOUD_enable_imu=DJIC_IMU.total_yaw;
		}
//						#if PID_MOTOR//YAW轴电机角度
//					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

//					yaw_trage_speed=Yaw_Angle_pid.result;//阶跃


//							yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;//左手推速度
//					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);//用电机做速度闭环
//	
//			                   send_to_yaw=Yaw_Speed_pid.result;

//					#endif
//							
					#if PID_YAW_IMU//YAW轴陀螺仪
									if(DR16.rc.s_left==3)//YAW轴控制挡位
							{
							yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW轴遥控器控制
							CH0_TOTAL_in_con+=	DR16.rc.ch0;
								if(DR16.rc.ch0!=0)
								dr16_controul_times++;
							}
									if(DR16.rc.s_left==1)//YAW轴控制挡位
							{
								if(VisionData.RawData.Armour==1)
							yaw_trage_angle=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;//YAW轴遥控器控制
								else
								yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW轴遥控器控制

							}							
//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//调试过程中这个值要不断更新
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
							
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
//	
//	
	
}

void PITCH_PID()
{
//	#if PID_PITCH_MOTOR      //PITCH轴电机角度
//if(PITCH_trage_angle>7125)
//	PITCH_trage_angle=7125;
//if(PITCH_trage_angle<6435)
//	PITCH_trage_angle=6435;
//					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
////陀螺仪的速度值会有小数
//					PITCH_trage_speed=PITCH_Angle_pid.result;//外环的结果给内环  二选一
////					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
//				
//				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
//				send_to_pitch=PITCH_Speed_pid.result;
//#endif	
//	
//	#if PID_PITCH_IMU		//PITCH轴陀螺仪
							if(DR16.rc.s_left==3)//PITCH轴控制挡位
							{
							PITCH_trage_angle+=(DR16.rc.ch1*1.0/660.0)*0.4;//遥控器给速度目标值 二选一

//							if(DR16.rc.ch4_DW<=-400)//拨上
//							PITCH_trage_angle=PITCH_MAX_angle-10;
//							if(DR16.rc.ch4_DW>=400)//拨下
//							PITCH_trage_angle=PITCH_MIN_angle+10;
							
							}
														if(DR16.rc.s_left==1)//PITCH轴控制挡位
							{
								if(VisionData.RawData.Armour==1)
							PITCH_trage_angle=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;//YAW轴遥控器控制
								else
								PITCH_trage_angle+=(DR16.rc.ch0/660.0)*0.4;//YAW轴遥控器控制
							
							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//先去做实验


//#endif
//	
}



void imu_angle()
{
//	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7990-GM6020s[3].totalAngle)/8196.0*360.0;
//	PITCH_MIN_angle=DJIC_IMU.total_pitch+(7450-GM6020s[3].totalAngle)/8196.0*360.0;
	
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7950-GM6020s[3].totalAngle)/8191.0*360.0;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(6850-GM6020s[3].totalAngle)/8191.0*360.0;
			allow_angle=	PITCH_MAX_angle-PITCH_MIN_angle;

}
//2022-3-27:
//
//上边界-下边界

//
//使劲抬头:陀螺仪值为23    6020值为8026
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为-31      6020值为6756
//2022-3-18:
//
//上边界-下边界

//
//使劲抬头:陀螺仪值为-1.1    6020值为4008
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为-44      6020值为3028

//
//3070-3950
//3100-3930
//8017-7044
//8000-7400
//上边界-下边界

//
//使劲抬头:陀螺仪值为6.5    6020值为8022
//从
//   速度为负           
//到
//使劲低头:陀螺仪值为50      6020值为7020


//扫描函数
//V1.0
//功能描述：
//匀速扫描
//拨到左中右中档位开始扫描运动
//拨到左中右下，停止，并清楚时间累加值
//t=0时，PITCH轴运动到中间，然后向下运动
//完成，但是只有YAW轴是丝滑开启，PITCH轴时默认行程中间开启扫描,PITCH与YAW轴速度绑定
//V1.1
//PITCH与YAW轴速度解除绑定,可以分别设置


		 bool scan_i_PITCH=1;
		 int scan_percent_PITCH=500;//0到1000,百分比
		 int scan_time=0;

		 int scan_percent_YAW=500;//0到1000,百分比
	 float YAW_START_ANGLE;//S扫描开始时YAW轴角度

void scan_cloud(void)
{
//	if(DR16.rc.s_left==3)//控制挡位-扫描
			if(DR16.rc.s_left==1)//控制挡位-扫描
	{
//		if(DR16.rc.s_right==3)//控制挡位-扫描开始
		if(VisionData.RawData.Armour==0&&lose_time>2000)//控制挡位-扫描开始
		{
		 int scan_speed_PITCH=1;//PITCH轴扫描速度,最小为1
		int scan_speed_YWA=4;//YAW轴扫描速度,最小为1


		static bool scan_i_YAW=0;
			scan_time++;
if(scan_time%scan_speed_PITCH==0)//是扫描速度的整数倍
{
	if(scan_percent_PITCH>0&&scan_percent_PITCH<1000)
	{
		if(scan_i_PITCH==0)//0增大
		{
		scan_percent_PITCH++;
		}
		else//1减小
		{
		scan_percent_PITCH--;
		}
	}
	else if(scan_percent_PITCH<1)//超出下边界,切换到增大模式
	{
		scan_i_PITCH=0;
		scan_percent_PITCH=1;
	}
	else//超出上边界,切换到减小模式
	{
		scan_i_PITCH=1;
				scan_percent_PITCH=999;

	}
	
}
if(scan_time%scan_speed_YWA==0)//是扫描速度的整数倍
{
	if(scan_percent_YAW>0&&scan_percent_YAW<1000)
	{
		if(scan_i_YAW==0)//0增大
		{
		scan_percent_YAW++;
		}
		else//1减小
		{
		scan_percent_YAW--;
		}
	}
	else if(scan_percent_YAW<1)//超出下边界,切换到增大模式
	{
		scan_i_YAW=0;
		scan_percent_YAW=1;
	}
	else//超出上边界,切换到减小模式
	{
		scan_i_YAW=1;
				scan_percent_YAW=999;
	}
	
}	
			
//			PITCH_trage_angle=PITCH_MIN_angle+(allow_angle-22)*(scan_percent_PITCH/1000.0);//PITCH
//yaw_trage_angle=YAW_START_ANGLE+400*(scan_percent_YAW/1000.0);//YAW轴转一圈多一点
		}
		else //控制挡位-扫描结束
		{
			scan_time=0;
							if(lose_time>4000)//控制挡位-扫描开始
YAW_START_ANGLE=DJIC_IMU.total_yaw;//丝滑开始扫描
							
scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*1000	;	
scan_percent_YAW=0;			
			
			
			
		}
			

		
		
		
		
	}
	else
	{
		scan_time=0;
		YAW_START_ANGLE=DJIC_IMU.total_yaw;//丝滑开始扫描
scan_percent_YAW=0;
	}
		
	

	
	
}

