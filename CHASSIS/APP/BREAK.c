#include "BREAK.h"
#include "main.h"

BREAK_e break_basic;

long M2006_targe_angle = 0;
int M2006_targe_speed = 0;
int send_to_break = 0;

int M2006_init_times = 0;            //��ʼ�� ��ʱ
int M2006_init_change_angle = 0;     //��ʼ�� �Ƕȱ仯��
int M2006_init_totalAngle_last = 0;  //��ʼ�� ��һʱ�̽Ƕ�ֵ


bool start_use_break=0;//ʹ��ɲ��,���Ǳ�־λ


/*
P_PID_t BREAK_ANGLE_pid;//ɲ��PID
P_PID_t BREAK_SPEED_pid;
*/
void break_init(void) {
  M2006_init_times++;

  if (M2006_init_times % 50 == 0)  //ÿ150ms���һ��
  {
    M2006_init_change_angle =
        M3508s[BREAK_ID].totalAngle - M2006_init_totalAngle_last;
    M2006_init_totalAngle_last = M3508s[BREAK_ID].totalAngle;
  }

  
  
  if (DR16.rc.s_left == 1)  //���ϵ�/�Զ��� ��һ��Ҫ�ȳ�ʼ��ɲ��
  {
    if (break_is_raedy == 0)  //ɲ��û��׼����
      if (send_to_chassis == 0 && abs(M3508s[3].realSpeed) < 10)  //�����Ѿ�ʧ��
      {
      }
  } 
  
  
  else if (DR16.rc.s_left == 3 &&
             DR16.rc.s_right == 2)  //�ֶ� ��ʼ��ɲ��,��ʱ Ħ�����ٶ�Ϊ0
  {
	  /* �ֶ�������ʼ�� (�����ж�ң����)
    if (DR16.rc.ch4_DW >= 200)  //����
    {
      if (break_basic.STATE == 0)  //δ��ʼ��
      {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //��ʼ��ʼ������0
      }
    }
	  */
	        if (break_basic.STATE == 0)  //δ��ʼ��
      {
		  if(start_use_break==1)
		  {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //��ʼ��ʼ������0
			  
		  }
      }
    else if (break_basic.STATE == 1)  //ɲ�����ֵû��׼����.���ڳ�ʼ�����ֵ
    {
      BREAK_SPEED_pid.Max_result = 2000;
      BREAK_SPEED_pid.Min_result = -2000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle + 999;
      if (M2006_init_times > 200)  //��200*3=600ms ������
      {
        if (abs(M2006_init_change_angle) < 200)  //�Ƕȱ仯С��200
        {
          //				if(abs(M3508s[BREAK_ID].realCurrent)>1500)//�������ֵ�ж�
          break_basic.BREAK_MAX = M3508s[BREAK_ID].totalAngle;  //���ֵ�ҵ���
          break_basic.STATE = 2;  //ȥ��ʼ����Сֵ
          M2006_init_times = 0;
        }
      }
    }
	else if (break_basic.STATE == 2)  //���ڳ�ʼ����Сֵ
    {
      BREAK_SPEED_pid.Max_result = 2000;
      BREAK_SPEED_pid.Min_result = -2000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle - 999;
      if (M2006_init_times > 200)  //��200*3=600ms ������
      {
        if (abs(M2006_init_change_angle) < 200)  //�Ƕȱ仯С��200
        {
          break_basic.BREAK_MIN = M3508s[BREAK_ID].totalAngle;  //��Сֵ�ҵ���
          break_basic.BREAK_MID =(break_basic.BREAK_MIN + break_basic.BREAK_MAX) /2;  //�м�ֵ�ҵ���
			M2006_targe_angle=break_basic.BREAK_MID;
			BREAK_SPEED_pid.Max_result = 9000;
      BREAK_SPEED_pid.Min_result = -9000;
          break_basic.STATE = 3;  //��ʼ��ȫ�����
        }
      }
    }
  }

  if (DR16.rc.s_left == 2)  //ʧ��
  {
    M2006_init_times = 0;  //��ʼ����ʱ����
    if (break_basic.STATE == 2 ||
        break_basic.STATE == 1)  //��ʼ����һ���ʧ����,ֱ�Ӵ�ͷ��ʼ��ʼ��
    {
      break_basic.STATE = 0;
    }
  }
}


void break_control(void) {
	if (DR16.rc.s_left  == 3 &&
		DR16.rc.s_right == 2) 
{
		if(DR16.rc.ch4_DW>300)//���µ�һ��
		{
	start_use_break=1;	
		}
		else
		{
	start_use_break=0;	
		}
}
	
break_init();

	if(DR16.rc.s_left == 3)//���е�λ
	{
if(break_basic.STATE == 3)  //��ʼ��ȫ�����
{	
	if(start_use_break==0)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;	
				stop_CH_OP_BC_BREAK=0;	
		stop_CH_OP_BC_BREAK_times=0;//ǿ��ֹͣɲ������
				if(DR16.rc.ch3==0)//����������
	{
	break_FX=-1;
	}
	
	}
	
	if(start_use_break==1)//�ֶ����Ҳ���
	{
		if(stop_CH_OP_BC_BREAK_times==0)//time������ܿ�ʼ��һ��ɲ�� һ��ɲ�������������ֵ����
	{	stop_CH_OP_BC_BREAK_times=1;	
		stop_CH_OP_BC_BREAK=1;//��ʼʧ��
	
	}	
	
	
	
	if(stop_CH_OP_BC_BREAK_times!=0)
	{
	stop_CH_OP_BC_BREAK_times++;
	}


	if(stop_CH_OP_BC_BREAK_times==45)
	{
	break_FX=-break_FX;//ʹ��ǰ��Ŀ���ٶ�ȡ��
	}
			if(stop_CH_OP_BC_BREAK_times>50)//����300ms,ȡ������ʧ��,
	{
				stop_CH_OP_BC_BREAK=0;		
	}
	
				if(stop_CH_OP_BC_BREAK_times<50&&stop_CH_OP_BC_BREAK_times>0)//
				{
	if(	M3508s[3].realSpeed>1000)
	{
	M2006_targe_angle=	break_basic.BREAK_MAX+200;
	}
	if(	M3508s[3].realSpeed<-1000)
	{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;
	}//�Զ�����ɲ������					
				}
			if(stop_CH_OP_BC_BREAK_times>200)//����500ms,ɲ������
	{
	M2006_targe_angle=	break_basic.BREAK_MID;
	stop_CH_OP_BC_BREAK=0;		
	}
		
	
	if(stop_CH_OP_BC_BREAK_times>2000)//����500*3ms��ŻῪʼ��һ��ɲ��
	{
	stop_CH_OP_BC_BREAK_times=0;
	}
	
	}
	/*�����ж�ң������ֵ
	if(DR16.rc.ch4_DW>=-100&&DR16.rc.ch4_DW<=100)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;	
stop_CH_OP_BC_BREAK_times=0;
				stop_CH_OP_BC_BREAK=0;
		
	}
	else if(DR16.rc.ch4_DW<-100)//����
	{
	M2006_targe_angle=	break_basic.BREAK_MAX+200;
		stop_CH_OP_BC_BREAK=1;
		break_FX=1;
		stop_CH_OP_BC_BREAK_times++;
		if(stop_CH_OP_BC_BREAK_times>50)//����300ms,ȡ������ʧ��,
		{
		stop_CH_OP_BC_BREAK=0;
		}
		
		if(stop_CH_OP_BC_BREAK_times>250)//����500ms,ɲ������
		{
		M2006_targe_angle=	break_basic.BREAK_MID;	
					stop_CH_OP_BC_BREAK=0;

		}
	}
	else if(DR16.rc.ch4_DW>100)//����
	{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;
				stop_CH_OP_BC_BREAK=1;
		stop_CH_OP_BC_BREAK_times++;
				break_FX=-1;

				if(stop_CH_OP_BC_BREAK_times>50)//����300ms,ȡ������ʧ��,
		{
					stop_CH_OP_BC_BREAK=0;		
		}
				if(stop_CH_OP_BC_BREAK_times>250)//����500ms,ɲ������
		{
		M2006_targe_angle=	break_basic.BREAK_MID;
		stop_CH_OP_BC_BREAK=0;		
		}
	}
	if(DR16.rc.ch3==0)
	{
	break_FX=-1;
	}
	*/
}	
	}

		if(DR16.rc.s_left == 3&&DR16.rc.s_right == 4)
	{
if(break_basic.STATE == 3)  //��ʼ��ȫ�����
{	
	if(DR16.rc.ch4_DW>=-100&&DR16.rc.ch4_DW<=200)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;		
	}
	else if(DR16.rc.ch4_DW>=200)
	{
		if(M3508s[3].realSpeed>0)
		{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;

		}
	}
	
}	
	}
	
M2006_targe_speed=P_PID_bate(&BREAK_ANGLE_pid,M2006_targe_angle,M3508s[BREAK_ID].totalAngle);//M2006_targe_speedӦ�ô���0
	
	
send_to_break=P_PID_bate(&BREAK_SPEED_pid,M2006_targe_speed,M3508s[BREAK_ID].realSpeed);
	
	
//	M2006_targe_angle_text+=DR16.rc.ch1/10;
//	M2006_targe_speed=P_PID_bate(&BREAK_ANGLE_pid,M2006_targe_angle_text,M3508s[BREAK_ID].totalAngle);//M2006_targe_speedӦ�ô���0

//send_to_break_text=P_PID_bate(&BREAK_SPEED_pid,M2006_targe_speed,M3508s[BREAK_ID].realSpeed);
//	
//	if(send_to_break_text>4000)//���Բ���ʱ����ײ��е��λ����ת�յ��  ����һ��
//	{send_to_break_text=4000;}
//		if(send_to_break_text<-4000)
//	{send_to_break_text=-4000;}
	
}



