#include "MY_CLOUD_CONTROL.h"
#include "bsp_buzzer.h"
#include "Vision_Control.h"
#include "spinning_top_examine.h"
#include "DR16_RECIVE.h"

cloud_control_mode cloud_mode;
float YAW_TRAGET_ANGLE_TEMP;
float PITCH_TRAGET_ANGLE_TEMP;
bool simulation_target_yaw_is_stop=0;
int pitch_motor_tagert=5000;

 bool our_outpost_is_live=1;
CHASSIS_KEY key_message;

int in_end_times=0;
#if USE_PITCH_BC==1
int pitch_bc=0;

#endif
void cloud_control(void)
{
		if(key_message.game_state_progress==3)
		key_message.our_outpost_is_live=1;//����ʱ�׶�ȷ��ǰ��ս���
	
	if(key_message.game_state_progress==4)
	{//�����׶�Ϊ�����оͻ��ж�ǰ��վ�Ƿ���
	
	 if(ext_game_robot_HP.data.blue_outpost_HP>0 )
 {
	key_message.blue_outpost_is_live=1; 
 }
 else//ǰ���ж�
 {
	key_message.blue_outpost_is_live=0; 
 }
 if(ext_game_robot_HP.data.red_outpost_HP>0 )
 {
	key_message.red_outpost_is_live=1; 
 }
 else//ǰ���ж�
 {
	key_message.red_outpost_is_live=0; 
 } 
if(ext_game_robot_state.data.robot_id== 7)//�Լ��Ǻ�ɫ
{
	if(	key_message.red_outpost_is_live==1)
    key_message.our_outpost_is_live=1;
	else
    key_message.our_outpost_is_live=0;
		
}
if(ext_game_robot_state.data.robot_id== 107)//�Լ���ɫ
{
	if(	key_message.blue_outpost_is_live==1)
    key_message.our_outpost_is_live=1;
	else
    key_message.our_outpost_is_live=0;
		
}
	}
	
	
	
	
//	
//								if(DR16.rc.s_left==1)//YAW����Ƶ�λ
//							{
//							yaw_trage_angle+=(DR16.rc.ch0/660.0)/-3;//YAW��ң��������
//							if(DR16.rc.ch4_DW<=-400)//����
//							yaw_trage_angle=(CLOUD_enable_imu+200.0);//�����ǽ��ٶ����Ϊ140?
//							if(DR16.rc.ch4_DW>=400)//����
//							yaw_trage_angle=(CLOUD_enable_imu-200.0);
//							
//							/*
//							����Ŀ���ԾҪ����140/0.03   4666
//							//��Ҫ����29000�������ò��ܵ�140�������ٶ�
//							//�ȸ�10000�Ľ�Ծ��
//							������800
//							*/
//							

//							}
///*					else
//					��-10000��30000���ʼ20000���������ٶΣ�Ȼ����10000��220�������10000�ļ���		
//					yaw_trage_angle+=(DR16.rc.ch3*1.0/660.0)*20;

//*/
//	
//


cloud_control_mode_choose();
 scan_cloud();
// if(DR16.rc.s_left!=3)
//{
//simulation_target_yaw=	DJIC_IMU.total_yaw;
//}
if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//��������
{
							if(DR16.rc.ch4_DW<=-400)//����
							simulation_target_yaw_is_stop=1;//�����ǽ��ٶ����Ϊ140?
							if(DR16.rc.ch4_DW>=400)//����
							simulation_target_yaw_is_stop=0;//�����ǽ��ٶ����Ϊ140?
	}
 if(DR16.rc.s_left==3)
{
	if(DR16.rc.s_right==1)//��������
	{
		if(simulation_target_yaw_is_stop==0)
	simulation_target_yaw+= 0.036f;   // 18��/1000����
//		else
//		{
//		simulation_target_yaw=
//		}
	}
	else
	{
		simulation_target_yaw=	DJIC_IMU.total_yaw;
	}
	
}
else
{
		simulation_target_yaw=	DJIC_IMU.total_yaw;

}

							YAW_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	
//							PITCH_trage_angle=0;//����ˮƽλ��
							imu_angle();//ȥ���������ԣ�����  ����
//									PITCH_trage_angle=28;

							//��������Ư�޷�
						if(PITCH_trage_angle>PITCH_MAX_angle||PITCH_trage_angle==PITCH_MAX_angle)
													PITCH_trage_angle=PITCH_MAX_angle;
						if(PITCH_trage_angle<PITCH_MIN_angle||PITCH_trage_angle==PITCH_MIN_angle)
													PITCH_trage_angle=PITCH_MIN_angle;	
						
							PITCH_PID();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}


void cloud_control_mode_choose(void)
{
	if(DR16.rc.s_left == 1)//����ʱ
	{
//		if(VisionData.RawData.Armour==1)

		if(Armour_lose_time>=3)//��ʧ3֡װ��
		{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);			
			
		}
		else
		{
			cloud_mode.control_mode_NOW=vision_mode;
			cloud_mode.control_mode_LAST=aoto_scan_mode;
			P_PID_Parameter_Clear(&Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&Yaw_IMU_Speed_pid);			
		}

		
	}
//	else if(DR16.rc.s_left == 3&&DR16.rc.s_right == 3)
//	{
////				cloud_mode.control_mode_NOW=vision_mode;

//	}
	#if YAW_TEXT==0

	else
	{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);	
			}

#endif		

#if YAW_TEXT==1
			else{
if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//��������
{
			cloud_mode.control_mode_NOW=vision_mode;
			cloud_mode.control_mode_LAST=aoto_scan_mode;
}
else
{
			cloud_mode.control_mode_NOW=aoto_scan_mode;
			cloud_mode.control_mode_LAST=vision_mode;
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Angle_pid);
			P_PID_Parameter_Clear(&VISION_Yaw_IMU_Speed_pid);
}
				}

#endif		
	
	
}


bool YAW_TARGE_ANGLE_ADD=1;
int arrive_targe_angle=0;
int TEXT_targe_SPEED=400;

void YAW_PID()
{
	
//									if(DR16.rc.s_left==1)//YAW����Ƶ�λ
//							{
//	if(DJIC_IMU.total_yaw>(CLOUD_enable_imu+360.0))	
//	{
//		
////TEXT_targe_SPEED		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=0;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(DJIC_IMU.total_yaw<(CLOUD_enable_imu-360.0))	
//	{
//		
//		
////		arrive_targe_angle++;
////		if(arrive_targe_angle>20)
////		{
//cloud_text_add=1;
////			arrive_targe_angle=0;
////		}
//	}	
//	
//	if(cloud_text_add==1)//����
//	{
//yaw_trage_speed=TEXT_targe_SPEED;
//	}	
//	if(cloud_text_add==0)//�Ƕȼ�С,�ٶ�Ϊ��
//	{
//yaw_trage_speed=-TEXT_targe_SPEED;
//	}	
//	
////if(cloud_enable==0)
////{
////}
//							}	
		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //ʧ�ܱ���
		{
CLOUD_enable_moto=GM6020s[0].totalAngle;
CLOUD_enable_imu=DJIC_IMU.total_yaw;
		}
//						#if PID_MOTOR//YAW�����Ƕ�
//					P_PID_bate(&Yaw_Angle_pid, yaw_trage_angle,GM6020s[0].totalAngle);//GM6020s[EMID].totalAngle readAngle

//					yaw_trage_speed=Yaw_Angle_pid.result;//��Ծ


//							yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*22;//�������ٶ�
//					P_PID_bate(&Yaw_Speed_pid, yaw_trage_speed,GM6020s[0].readSpeed);//�õ�����ٶȱջ�
//	
//			                   send_to_yaw=Yaw_Speed_pid.result;

//					#endif
//							
					#if PID_YAW_IMU//YAW��������
									if(DR16.rc.s_left==3)//YAW����Ƶ�λ
							{
									#if YAW_TEXT==0

								if(cloud_mode.control_mode_NOW==aoto_scan_mode)//ɨ��PID
								{
							yaw_trage_angle-=(DR16.rc.ch0/660.0)/2.0;//YAW��ң��������
								YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw;
								PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch;
								}
//								if(cloud_mode.control_mode_NOW==vision_mode)//ɨ��PID
//								{
//								yaw_trage_angle=simulation_target_yaw;
//								}
//							CH0_TOTAL_in_con+=	DR16.rc.ch0;
//								if(DR16.rc.ch0!=0)
//								dr16_controul_times++;
								#endif
										#if YAW_TEXT==1
								if(DR16.rc.s_left==3&&DR16.rc.s_right==1)//��������
								{
								yaw_trage_angle=simulation_target_yaw;
								}
								else
								{
								yaw_trage_angle-=(DR16.rc.ch0/660.0)/2.0;//YAW��ң��������

								}
								#endif

							}
									if(DR16.rc.s_left==1)//YAW����Ƶ�λ
							{
//								if(VisionData.RawData.Armour==1)
//							yaw_trage_angle-=Vision_RawData_Yaw_Angle;//YAW��ң��������
								
				if(cloud_mode.control_mode_NOW==vision_mode)//�Ӿ�PID
				{
//					yaw_trage_angle=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;//YAW��ң��������
					yaw_trage_angle=YAW_TRAGET_ANGLE_TEMP;//YAW���Ӿ�����
													

				}			
//								else
//								yaw_trage_angle-=(DR16.rc.ch0/660.0)/10.0;//YAW��ң��������

							}							
//					Yaw_IMU_Angle_pid.Kp=-YAW_IMU_Kp;//���Թ��������ֵҪ���ϸ���
							
				if(cloud_mode.control_mode_NOW==aoto_scan_mode)//ɨ��PID
				{
					P_PID_bate(&Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
							
					P_PID_bate(&Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);
					
		                   send_to_yaw=Yaw_IMU_Speed_pid.result;
				}
				
				if(cloud_mode.control_mode_NOW==vision_mode)//����PID
				{
					P_PID_bate(&VISION_Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
		

					yaw_trage_speed=VISION_Yaw_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
//					yaw_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
							
					P_PID_bate(&VISION_Yaw_IMU_Speed_pid, yaw_trage_speed,DJIC_IMU.Gyro_z);	
		                   send_to_yaw=VISION_Yaw_IMU_Speed_pid.result;
				
					if(YAW_MOTION_STATE==12)
					{
				Vision_Control_Cloud();
						
//										P_PID_bate(&VISION_Yaw_IMU_Angle_pid, yaw_trage_angle,DJIC_IMU.total_yaw);//GM6020s[EMID].totalAngle readAngle
	
					}
				
				}
				
				
//					send_to_pitch=(DR16.rc.ch3*1.0/660.0)*29000;
					#endif
//	
//	
	
}

void PITCH_PID()
{
//	#if PID_PITCH_MOTOR      //PITCH�����Ƕ�
//if(PITCH_trage_angle>7125)
//	PITCH_trage_angle=7125;
//if(PITCH_trage_angle<6435)
//	PITCH_trage_angle=6435;
//					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle,GM6020s[1].totalAngle);//GM6020s[EMID].totalAngle readAngle
////�����ǵ��ٶ�ֵ����С��
//					PITCH_trage_speed=PITCH_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ
////					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
//				
//				P_PID_bate(&PITCH_Speed_pid, PITCH_trage_speed,GM6020s[1].readSpeed);
//				send_to_pitch=PITCH_Speed_pid.result;
//#endif	
//	
//	#if PID_PITCH_IMU		//PITCH��������
							if(DR16.rc.s_left==3)//PITCH����Ƶ�λ
							{
							PITCH_trage_angle+=(DR16.rc.ch1*1.0/660.0)*0.4;//ң�������ٶ�Ŀ��ֵ ��ѡһ
                            PITCH_trage_angle_motor+=(DR16.rc.ch1*1.0/660.0)*1.0;
 ////							if(DR16.rc.ch4_DW<=-400)//����
////							PITCH_trage_angle=PITCH_MAX_angle-10;
////							if(DR16.rc.ch4_DW>=400)//����
////							PITCH_trage_angle=PITCH_MIN_angle+10;
//							
							}
														if(DR16.rc.s_left==1)//PITCH����Ƶ�λ
							{
//								if(VisionData.RawData.Armour==1)
//							PITCH_trage_angle=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;//YAW��ң��������
												if(cloud_mode.control_mode_NOW==vision_mode)//�Ӿ�PID
								PITCH_trage_angle=PITCH_TRAGET_ANGLE_TEMP;
//								else
//								PITCH_trage_angle+=(DR16.rc.ch1/660.0)*0.4;//YAW��ң��������
							PITCH_trage_angle_motor=PITCH_TRAGET_ANGLE_TEMP;
							}

					P_PID_bate(&PITCH_IMU_Angle_pid, PITCH_trage_angle,DJIC_IMU.total_pitch);//GM6020s[EMID].totalAngle readAngle
//�����ǵ��ٶ�ֵ����С��
							if(PITCH_trage_angle_motor>5080)
PITCH_trage_angle_motor=5080;
							if(PITCH_trage_angle_motor<3900)
PITCH_trage_angle_motor=3900;
					P_PID_bate(&PITCH_Angle_pid, PITCH_trage_angle_motor,GM6020s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle

					#if USE_MOTOR_angle==1
					PITCH_trage_speed=PITCH_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ


#endif		
												#if USE_MOTOR_angle==0
					PITCH_trage_speed=PITCH_IMU_Angle_pid.result;//�⻷�Ľ�����ڻ�  ��ѡһ


#endif		
//					PITCH_trage_speed=(DR16.rc.ch3*1.0/660.0)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
				
				P_PID_bate(&PITCH_IMU_Speed_pid, PITCH_trage_speed,DJIC_IMU.Gyro_y);
				send_to_pitch=PITCH_IMU_Speed_pid.result;//��ȥ��ʵ��

#if USE_PITCH_BC==1
							if(PITCH_trage_angle_motor<4492&&PITCH_trage_angle_motor>3899)
send_to_pitch=PITCH_IMU_Speed_pid.result+
							(PITCH_trage_angle_motor*PITCH_trage_angle_motor*PITCH_trage_angle_motor*-2.5419
							+PITCH_trage_angle_motor*PITCH_trage_angle_motor*62.982f
							-PITCH_trage_angle_motor*745.77+768.52);
/*y = 1.9764x2 - 342.62x + 146.26    R2 = 0.967
*/
#endif
//#endif
//	
}



void imu_angle()
{
//	PITCH_MAX_angle=DJIC_IMU.total_pitch+(7990-GM6020s[3].totalAngle)/8196.0*360.0;
//	PITCH_MIN_angle=DJIC_IMU.total_pitch+(7450-GM6020s[3].totalAngle)/8196.0*360.0;
	
	PITCH_MAX_angle=DJIC_IMU.total_pitch+(5080-GM6020s[3].totalAngle)/8191.0f*360.0f;
	PITCH_MIN_angle=DJIC_IMU.total_pitch+(3900-GM6020s[3].totalAngle)/8191.0f*360.0f;//3900
			allow_angle=	PITCH_MAX_angle-PITCH_MIN_angle;

}

//2022-4-14:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ15    6020ֵΪ4408      5170
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-39.5      6020ֵΪ3154    3820

//2022-3-27:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ23    6020ֵΪ8026
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-31      6020ֵΪ6756
//2022-3-18:
//
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ-1.1    6020ֵΪ4008
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ-44      6020ֵΪ3028

//
//3070-3950
//3100-3930
//8017-7044
//8000-7400
//�ϱ߽�-�±߽�

//
//ʹ��̧ͷ:������ֵΪ6.5    6020ֵΪ8022
//��
//   �ٶ�Ϊ��           
//��
//ʹ����ͷ:������ֵΪ50      6020ֵΪ7020


//ɨ�躯��
//V1.0
//����������
//����ɨ��
//�����������е�λ��ʼɨ���˶�
//�����������£�ֹͣ�������ʱ���ۼ�ֵ
//t=0ʱ��PITCH���˶����м䣬Ȼ�������˶�
//��ɣ�����ֻ��YAW����˿��������PITCH��ʱĬ���г��м俪��ɨ��,PITCH��YAW���ٶȰ�
//V1.1
//PITCH��YAW���ٶȽ����,���Էֱ�����


		 bool scan_i_PITCH=1;
		 int scan_percent_PITCH=500;//0��1000,�ٷֱ�
		 int scan_time=0;

		 int scan_percent_YAW=500;//0��1000,�ٷֱ�
	 float YAW_START_ANGLE;//Sɨ�迪ʼʱYAW��Ƕ�

void scan_cloud(void)
{
//	if(DR16.rc.s_left==3)//���Ƶ�λ-ɨ��
			if(DR16.rc.s_left==1)//���Ƶ�λ-ɨ��
	{
//		if(DR16.rc.s_right==3)//���Ƶ�λ-ɨ�迪ʼ
		if(Armour_lose_time>500)//�Ӿ�0.5��û����װ�װ�-ɨ�迪ʼ
		{
		 int scan_speed_PITCH=1;//PITCH��ɨ���ٶ�,��СΪ1
		int scan_speed_YWA=10;//YAW��ɨ���ٶ�,��СΪ1


		static bool scan_i_YAW=0;
			scan_time++;
if(scan_time%scan_speed_PITCH==0)//��ɨ���ٶȵ�������
{
	if(scan_percent_PITCH>0&&scan_percent_PITCH<500)
	{
		if(scan_i_PITCH==0)//0����
		{
		scan_percent_PITCH++;
		}
		else//1��С
		{
		scan_percent_PITCH--;
		}
	}
	else if(scan_percent_PITCH<1)//�����±߽�,�л�������ģʽ
	{
		scan_i_PITCH=0;
		scan_percent_PITCH=1;
	}
	else//�����ϱ߽�,�л�����Сģʽ
	{
		scan_i_PITCH=1;
				scan_percent_PITCH=499;

	}
	
}
if(scan_time%scan_speed_YWA==0)//��ɨ���ٶȵ�������  scan_percent_YAW��0��1000֮�䲨��
{
	if(scan_percent_YAW>0&&scan_percent_YAW<1000)
	{
		if(scan_i_YAW==0)//0����
		{
		scan_percent_YAW++;
		}
		else//1��С
		{
		scan_percent_YAW--;
		}
	}
	else if(scan_percent_YAW<1)//�����±߽�,�л�������ģʽ
	{
		scan_i_YAW=0;
		scan_percent_YAW=1;
	}
	else//�����ϱ߽�,�л�����Сģʽ
	{
		scan_i_YAW=1;
				scan_percent_YAW=999;
	}
	
}
#if 1
if	(in_END_L==1)
{
		in_end_times++;
	if(in_end_times<5000)//5����
	{
	if(GM6020s[0].readAngle>3842&&GM6020s[0].readAngle<4900)//4700-4900
	{
				scan_i_YAW=1;//����ģʽ
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>2687&&GM6020s[0].readAngle<=3842) //3700-4000
	{
				scan_i_YAW=0;//��Сģʽ
							Buzzer.mode = One_times;

	}
}
//	if(GM6020s[0].readAngle>=4330&&GM6020s[0].readAngle<=4700)//4000-4700
//	{
//					scan_i_YAW=0;//����ģʽ
//		YAW_START_ANGLE=DJIC_IMU.total_yaw+400;
//		scan_percent_YAW=0;
//				Buzzer.mode = One_times;

//		
//	}
//	if(GM6020s[0].readAngle>=4000&&GM6020s[0].readAngle<=4329)//4000-4329
//	{
//				scan_i_YAW=1;//��Сģʽ
//		YAW_START_ANGLE=DJIC_IMU.total_yaw-500;
//		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

//	}
}
if	(in_END_R==1)
{		
	in_end_times++;
	if(in_end_times<5000)//5����
	{
		if(GM6020s[0].readAngle>0&&GM6020s[0].readAngle<1200)//500-1200
	{
				scan_i_YAW=1;//����ģʽ
							Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>7000&&GM6020s[0].readAngle<8191) //7000-7700
	{
				scan_i_YAW=0;//��Сģʽ
							Buzzer.mode = One_times;

	}
}
	if(in_END_R==0&&in_END_L==0)
	{//�������ұ߽�
	in_end_times=0;
	}
//	if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<=1000)//0-500
//	{
//					scan_i_YAW=0;//����ģʽ
//		YAW_START_ANGLE=DJIC_IMU.total_yaw+500;
//		scan_percent_YAW=0;
//		Buzzer.mode = One_times;

// //  7888
//	}
//	if(GM6020s[0].readAngle>=7000&&GM6020s[0].readAngle<=8191)//4000-4329
//	{
//				scan_i_YAW=1;//��Сģʽ
//		YAW_START_ANGLE=DJIC_IMU.total_yaw-400;
//		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

//	}
}
#endif	
#if 0
if	(in_END_R==1)
{
	if(GM6020s[0].readAngle>4700&&GM6020s[0].readAngle<4900)//4700-4900
	{
				scan_i_YAW=0;//����ģʽ
	}
	if(GM6020s[0].readAngle>3700&&GM6020s[0].readAngle<4000) //3700-4000
	{
				scan_i_YAW=1;//��Сģʽ
	}
	if(GM6020s[0].readAngle>=4330&&GM6020s[0].readAngle<=4700)//4000-4700
	{
					scan_i_YAW=0;//����ģʽ
		YAW_START_ANGLE=DJIC_IMU.total_yaw+400;
		scan_percent_YAW=0;
				Buzzer.mode = One_times;

		
	}
	if(GM6020s[0].readAngle>=4000&&GM6020s[0].readAngle<=4329)//4000-4329
	{
				scan_i_YAW=1;//��Сģʽ
		YAW_START_ANGLE=DJIC_IMU.total_yaw-500-720;
		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

	}
}
if	(in_END_L==1)
{		

	if(GM6020s[0].readAngle>500&&GM6020s[0].readAngle<1200)//500-1200
	{
				scan_i_YAW=0;//����ģʽ
	}
	if(GM6020s[0].readAngle>7000&&GM6020s[0].readAngle<7700) //7000-7700
	{
				scan_i_YAW=1;//��Сģʽ
	}
	if(GM6020s[0].readAngle>=0&&GM6020s[0].readAngle<=500)//0-500
	{
					scan_i_YAW=0;//����ģʽ
		YAW_START_ANGLE=DJIC_IMU.total_yaw+500;
		scan_percent_YAW=0;
//		Buzzer.mode = One_times;

	}
	if(GM6020s[0].readAngle>=7700&&GM6020s[0].readAngle<=8191)//4000-4329
	{
				scan_i_YAW=1;//��Сģʽ
		YAW_START_ANGLE=DJIC_IMU.total_yaw-500-720;
		scan_percent_YAW=999;
//				Buzzer.mode = One_times;

	}
}
#endif			
PITCH_trage_angle=PITCH_MIN_angle+(allow_angle)*0.65f*(scan_percent_PITCH/500.0f);//PITCH
yaw_trage_angle=YAW_START_ANGLE+720*(scan_percent_YAW/1000.0f);//YAW��תһȦ��һ��
								YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw;
								PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch;
		}
		else //�Ӿ�����װ�װ�-ɨ�����
		{
			scan_time=0;
		if(Armour_lose_time>400)//���Ƶ�λ-ɨ�迪ʼ
YAW_START_ANGLE=DJIC_IMU.total_yaw;//˿����ʼɨ��
scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*500	;	
scan_percent_YAW=0;	
		
		}

	}
	else//����ɨ�赵λ
	{
		scan_time=0;
		YAW_START_ANGLE=DJIC_IMU.total_yaw;//˿����ʼɨ��
		scan_percent_PITCH=	(DJIC_IMU.total_pitch-PITCH_MIN_angle)/allow_angle*1000	;	
scan_percent_YAW=0;
	}
		
	

	
	
}

