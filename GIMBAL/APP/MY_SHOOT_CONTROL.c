#include "MY_SHOOT_CONTROL.h"
#include "main.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"


#define SHOOT_SPEED_HIGH 6700//���� ��

int SHOOT=0;

int DW_FREE=0;
int DW_DOWN=0;
int DW_UP=0;
long M2006_targe_angle=0;
bool Driver_arrive=1;
int if_Driver_arrive_angle=2000;
void shoot_control(void)
{
			if(DR16.rc.s_left==2)//ң������XIA
			{
						if(DR16.rc.ch4_DW<=-100)//����
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_RESET);

			}
			if(DR16.rc.ch4_DW>=200)//����
			{		
            HAL_GPIO_WritePin(RED_GPIO_Port,RED_Pin,GPIO_PIN_SET);

			}
			
			}
	
	
	
		if(DR16.rc.s_left==3||DR16.rc.s_left==1)//ң��������  ���л�����
	{
		if(M3508s[1].totalAngle>(M2006_targe_angle-if_Driver_arrive_angle))
		Driver_arrive=1;
		
					if(DR16.rc.ch4_DW==0)//����
					{
						DW_FREE++;
						DW_UP=0;
							M2006_targe_angle=M3508s[1].totalAngle;//�����������

					}
									if(DR16.rc.ch4_DW<=-100)//����
			{		
				DW_UP++;//û�õ���
				if(DW_UP==20)
			M2006_targe_angle+=Driver_add;//8*3=24
				if(DW_UP%100==0&&DW_UP>800)
				M2006_targe_angle+=Driver_add*3;//8*3=24

				DW_FREE=0;
					SHOOT++;				

			}
//				if(DR16.rc.ch4_DW<=-100)//����
//			{		
//				DW_UP++;//û�õ���
//				if(DW_UP==20||DW_UP>800)
//				if(Driver_arrive==1)
//				{
//			M2006_targe_angle+=Driver_add;//8*3=24
//					Driver_arrive=0;
//				}
//				DW_FREE=0;
//					SHOOT++;				
////				if(DW_FREE>100)
////				{
////									yaw_trage_angle-=36;
////				}
//			}
			if(DR16.rc.ch4_DW>=200)//����
			{		
//				DW_FREE++;
				if(DW_FREE>20)
				{
					DW_DOWN=0;
					
				M2006_targe_angle-=Driver_add;//8*3=24
						SHOOT++;
					DW_FREE=0;

				}
			}
		
//M2006_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
M2006_targe_speed=P_PID_bate(&Driver_ANGLE_pid,M2006_targe_angle,M3508s[1].totalAngle);//M2006_targe_speedӦ�ô���0
	
	
send_to_2006=I_PID_Regulation(&Driver_I_PID,M2006_targe_speed,M3508s[1].realSpeed);
			
			
			

	}	
	if(DR16.rc.s_left==2||DR16.rc.s_left==0)	//ʧ�ܱ���
					{			
						M2006_targe_angle=M3508s[1].totalAngle;//�����������
						Driver_arrive=1;//ʧ��,ͬ��Ҳ�ֵ�Ŀ��
					}
	//û��ң������ң�����������µ�λʱ����ʧ��
	//�����к����ϵ�λʱ
	
	
	
	
//		if(DR16.rc.s_left==1)//ң��������  ����
//SHOOT_L_speed=500;
//	if(DR16.rc.s_left==1)//ң��������  ����
//					{
//SHOOT_L_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//ң�������ٶ�Ŀ��ֵ ��ѡһ	
//if(DR16.rc.ch3<-600)	
					
//	if (DR16.rc.s_right == 3) //�Ƿ���λ��  �м䵵λ 

//			SHOOT_L_speed=7000;
//		else 
			if(DR16.rc.s_right ==1)
							SHOOT_L_speed=500;//����

else if(DR16.rc.s_right ==2)
				SHOOT_L_speed=0;

//						}
				SHOOT_R_speed=SHOOT_L_speed;					
		if(	SHOOT_L_speed<0)
		SHOOT_L_speed=-SHOOT_L_speed;//��Ħ�����ٶ�Ŀ��ֵӦ���Ǹ�ֵ
		if(	SHOOT_R_speed>0)
		SHOOT_R_speed=-SHOOT_R_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_R_speed,M3508s[3].realSpeed);//gai

send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_L_speed,M3508s[2].realSpeed);
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);

//	if(DR16.rc.s_left==3)//ң��������  ����
//					{
//SHOOT_L=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//				SHOOT_R=SHOOT_L;					
//		if(	SHOOT_L>0)
//		SHOOT_L=-SHOOT_L;//��Ħ�����ٶ�Ŀ��ֵӦ���Ǹ�ֵ
//		if(	SHOOT_R<0)
//		SHOOT_R=-SHOOT_R;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
//send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_L,M3508s[0].realSpeed);

//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);
	
//					}
	
}

void driver_plate_control(void)
{
//	if(DR16.rc.s_left==3||DR16.rc.s_left==1)//ң��������  ���л�����

//driver_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//ң�������ٶ�Ŀ��ֵ ��ѡһ		
//		if(	driver_targe_speed<0)
//		driver_targe_speed=-driver_targe_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
//send_to_driver=I_PID_Regulation(&Driver_I_PID,driver_targe_speed,M3508s[2].realSpeed);





}








