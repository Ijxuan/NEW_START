#include "CHASSIS_CONTROL_2.h"
#include "math.h"
#include "rng.h"
#include "RM_JudgeSystem.h"
#include "COM_TO_PC.h"

int speed_has_change=0;
int last_Speed=0;
int xunen_times=0;
float xunen_percent=1.5;
int test_speed=5500;
void CHASSIS_CONTROUL_2()
{
				stop_CH_OP_BC_LESS=0;//���ģʽ�ò���	

//				if(M3508s[3].totalAngle<(CHASSIS_R_MIN+200000))
//				{
//				CHASSIS_trage_angle=9900000;		
//				}
//			else if(M3508s[3].totalAngle>(CHASSIS_L_MAX-200000))//
//			{
//	CHASSIS_trage_angle=-9900000;			
//			}
//			if(M3508s[3].totalAngle<(ENCODER_R_MIN+2000))
				if(Chassis_Encoder.totalLine<(CHASSIS_R_MIN_by_ENCODER+reverse_by_ENCODER))
			{

				stop_CH_OP_BC_END_times=0;
								xunen_times++;
						CHASSIS_trage_speed_last=test_speed;

//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;
//				if(last_Speed>M3508s[3].realSpeed&&M3508s[3].realSpeed>0)//�Ѿ�������
								if(M3508s[3].realSpeed>0)//�Ѿ�������
{
	speed_has_change=1;
}

if(speed_has_change==0)
{
	
					if(Chassis_Encoder.totalLine>(CHASSIS_R_MIN_by_ENCODER+8000))
				{
					
								CHASSIS_trage_speed=-test_speed;
				stop_CH_OP_BC_END=0;
	C_T_P.C_motion.data='4';//�Զ�_ײ��ʧ�ܽ��

				}
				else
				{
	C_T_P.C_motion.data='3';//�Զ�_ײ��ʧ�ܴ���
					
						CHASSIS_trage_speed=-test_speed;
			stop_CH_OP_BC_END=1;

				}


}	
else
{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=9900000;
				CHASSIS_trage_speed=test_speed*Chassis_PowerLimit;
	C_T_P.C_motion.data='4';//�Զ�_ײ��ʧ�ܽ��

}
	if(	xunen_times>500)//˵����������,�϶��ǿ�����,������,ֱ����
	{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=9900000;
						CHASSIS_trage_speed=test_speed*Chassis_PowerLimit;

	}
			}
			
else if(Chassis_Encoder.totalLine>(CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER))
			{
				
								CHASSIS_trage_speed_last=-test_speed;

				stop_CH_OP_BC_END_times=0;
				xunen_times++;
//							stop_chassic_output=1;	
//							  	  HAL_Delay(3000);
//							stop_chassic_output=0;	
//				if(M3508s[3].realSpeed<0&&M3508s[3].realSpeed>last_Speed)//�ٶ�һ����Ҫ�ٶ��½�Ҳ���ǵ����쵽��ſ�ʼ��������ʱ��,Ȼ���������
				if(M3508s[3].realSpeed<0)//ֻҪ�ٶ�һ���Ϳ�ʼ��������ʱ��,Ȼ���������

{
	
	speed_has_change=1;
}
if(speed_has_change==0)
{
	
					if(Chassis_Encoder.totalLine<(CHASSIS_L_MAX_by_ENCODER-8000))
				{
					
	stop_CH_OP_BC_END=0;
					CHASSIS_trage_speed=test_speed*Chassis_PowerLimit;
//				CHASSIS_trage_speed=4000;
	C_T_P.C_motion.data='4';//�Զ�_ײ��ʧ�ܽ��

				}
				else
				{
	C_T_P.C_motion.data='3';//�Զ�_ײ��ʧ�ܴ���
					
				CHASSIS_trage_speed=test_speed;
			stop_CH_OP_BC_END=1;

				}
//	if(	xunen_times>4)//���ɳ���ǰ���ٵ�ʱ��
//	{
//xunen_percent=0.7;		
//	}
//	else 
//	{
////xunen_percent=1.0;
//	}	
		
}
else
{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=-9900000;
					CHASSIS_trage_speed=-test_speed*Chassis_PowerLimit;
	C_T_P.C_motion.data='4';//�Զ�_ײ��ʧ�ܽ��

}
	if(	xunen_times>500)//˵����������,�϶��ǿ�����,������,ֱ����
	{
	stop_CH_OP_BC_END=0;
//	CHASSIS_trage_angle=-9900000;
						CHASSIS_trage_speed=-test_speed*Chassis_PowerLimit;

	}
			}
			else//������������֮��
			{
//				xunen_percent=1.5;
	C_T_P.C_motion.data='0';//�Զ�_ײ��ʧ�ܽ��

				stop_CH_OP_BC_END_times++;
				if(stop_CH_OP_BC_END_times>500)//��������û��ʱ��ͳ��˹���߽��ж�
				{
					stop_CH_OP_BC_END=0;//
					CHASSIS_trage_speed=CHASSIS_trage_speed_last;
				}
				speed_has_change=0;
				xunen_times=0;
			}
		
			last_Speed=M3508s[3].realSpeed;
			
//			P_PID_bate(&CHASSIS_MOTOR_ANGLE_pid, CHASSIS_trage_angle,M3508s[3].totalAngle);//GM6020s[EMID].totalAngle readAngle
//			CHASSIS_trage_speed=CHASSIS_MOTOR_ANGLE_pid.result*xunen_percent;//˫��
//			CHASSIS_trage_speed=0;
//				CHASSIS_MID=(CHASSIS_R_MIN+CHASSIS_L_MAX)/2;
				//CHASSIS_MID-CHASSIS_R_MIN   һ���г�
//				DEBUFF=abs(M3508s[3].totalAngle-CHASSIS_MID)/(CHASSIS_MID-CHASSIS_R_MIN);
//				speed_change=DEBUFF*CHASSIS_trage_speed*0.7;		//�������ٷ�֮70
//				CHASSIS_trage_speed=CHASSIS_trage_speed-speed_change;//�������м�죬������
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


void XNQ_TEXT(void)
{
if(Chassis_Encoder.totalLine<CHASSIS_R_XNQ_by_ENCODER)
{
								CHASSIS_trage_speed=-test_speed;
}
if(Chassis_Encoder.totalLine>CHASSIS_L_XNQ_by_ENCODER)
{
								CHASSIS_trage_speed=test_speed;
}



}

void XNQ_CHANGE(void)
{

//CHASSIS_R_XNQ_by_ENCODER=Chassis_Encoder.totalLine+100;

	

}



