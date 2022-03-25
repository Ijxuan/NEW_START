#include "MY_CLOUD_CONTROL.h"
#include "FPS_Calculate.h"

#include "User_math.h"

Ramp_Struct *EM_Ramp;
void cloud_control(void)
{
	
								if(DR16.rc.s_left==1)//YAW轴控制挡位
							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW轴遥控器控制
//								PITCH_trage_angle+=(DR16.rc.ch1/660.0)*5;//pitch轴遥控器控制
								
//							if(DR16.rc.ch4_DW<=-400)//拨上
//							yaw_trage_angle=yaw_trage_angle2;//陀螺仪角速度最大为140
							/*
							所以目标阶跃要大于140/0.03   4666
							//还要考虑29000的输出多久才能到140这个最大速度
							//先给10000的阶跃吧
							现在是800
*/
//							if(DR16.rc.ch4_DW>=400)//拨下
//							yaw_trage_angle=0;
							}
/*					else
					从-10000到30000，最开始20000是上升加速段，然后是10000的220，最后是10000的减速		
					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

*/
	
							
							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	

						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

void YAW_PID()
{
	//							PITCH_trage_angle=0;//保持水平位置


	
	
	
	
	
						#if PID_MOTOR//YAW轴电机角度
					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

					yaw_trage_speed=Yaw_Angle_pid.result;//阶跃


					//		yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;
					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);
					#endif
#if PID_YAW_EM==1	  //是否开启YAW轴的自瞄代码  1开启

	
				if(DR16.rc.s_right==2)	//是否陀螺仪
				{		
#endif	
					#if PID_IMU//YAW轴陀螺仪

//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//调试过程中这个值要不断更新
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
#if PID_YAW_EM==1	  //是否开启YAW轴的自瞄代码  1开启
					
				}
				else//视觉控制
				{
				if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
										{
											yaw_trage_angle=DJIC_IMU.total_yaw;
//											EM_Ramp->Target_Value=VisionData.RawData.Yaw_Angle;
//											EM_Ramp->Current_Value=DJIC_IMU.total_yaw;
//											EM_Ramp->Absolute_Max=50;
yaw_trage_angle2=+VisionData.RawData.Yaw_Angle;
P_PID_bate(&Yaw_EM_Angle_pid, yaw_trage_angle2,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle

					yaw_trage_speed=Yaw_EM_Angle_pid.result;//外环的结果给内环  二选一
					
P_PID_bate(&Yaw_EM_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
					send_to_yaw=Yaw_EM_Speed_pid.result;
										}
				else
				{
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
					
				}
				
				}
#endif	

	
	
}

void PITCH_PID()
{
												if(DR16.rc.s_right==3)	//是否陀螺仪
				{		
							if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
							{
								PITCH_trage_angle-=(Vision_RawData_Pitch_Angle/10.0);
//							PITCH_trage_angle_2=;
							}
							else
								PITCH_trage_angle =DJIC_IMU.total_pitch;
				}			
//							else
//								PITCH_trage_angle =PITCH_trage_angle_2;
							imu_angle();//去仿真做测试！！！  已做
							//陀螺仪零漂限幅
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
	
	
	
	#if PID_PITCH_MOTOR      //PITCH轴电机角度
if(PITCH_trage_angle>7125)
	PITCH_trage_angle=7125;
if(PITCH_trage_angle<6435)
	PITCH_trage_angle=6435;
					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
					PITCH_trage_speed=PITCH_Angle_pid.result;//外环的结果给内环  二选一
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
				send_to_pitch=PITCH_Speed_pid.result;
#endif	
	

#if PID_PITCH_EM==1	  //是否开启PITCH轴的自瞄代码  1开启
	
					if(DR16.rc.s_right==2)	//是否陀螺仪
				{		
#endif
					
	#if PID_PITCH_IMU		//PITCH轴陀螺仪
//							if(DR16.rc.s_left==3)//PITCH轴控制挡位
//							{
//							PITCH_trage_angle+=(DR16.rc.ch3*1.0/660.0)*1;//遥控器给速度目标值 二选一

//							if(DR16.rc.ch4_DW<=-400)//拨上
//							PITCH_trage_angle=PITCH_MAX_angle-1;
//							if(DR16.rc.ch4_DW>=400)//拨下
//							PITCH_trage_angle=PITCH_MIN_angle+1;
//							
//							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//先去做实验
#endif

#if PID_PITCH_EM==1	//是否开启PITCH轴的自瞄代码  1开启
				}
				else//视觉控制
				{
								if(FPS_ALL.Vision.FPS>20&&VisionData.RawData.Armour == 1)	
										{
					
//P_PID_bate(&PITCH_EM_Angle_pid,PITCH_trage_angle ,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle

//					PITCH_trage_speed=PITCH_EM_Angle_pid.result;//外环的结果给内环  二选一
//					
//P_PID_bate(&PITCH_EM_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
//					
//					send_to_pitch=PITCH_EM_Speed_pid.result;
					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//先去做实验
										}
				else
				{
					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//陀螺仪的速度值会有小数
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//外环的结果给内环  二选一
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//遥控器给速度目标值 二选一
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//先去做实验
					
				}
										
										
				}
#endif	

}


