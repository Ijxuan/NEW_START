#include "COM_TO_PC.h"
#include "usart.h"
#include "FPS_Calculate.h"
#include "CHASSIS_CONTROL_basic.h"


uint8_t send_data[33];//��λ������,���շ���
uint16_t send_begin=0;//��һ�� ���ݴ���һλ��ʼ���
uint16_t send_total_bit=0;//һ��Ҫ���Ͷ���λ
uint16_t send_err=0;//���ʹ���
COM_TO_PC_e C_T_P;//ͨ�����ڷ��͸�������λ��


	int XNQ_L_BMQ_BFB=0;//����ǽ��,ͨ��������,�ٷֱ���ʽ����0-999

	int XNQ_R_BMQ_BFB=0;//����ǽ��,ͨ��������,�ٷֱ���ʽ����0-999

void com_to_pc_INIT(void)//Э���ʼ��
{
C_T_P.YKQ.begin='a';
C_T_P.YKQ.end='b';
C_T_P.YKQ.distance_next_send_ms=1050;
	
C_T_P.C_model.begin='c';
	C_T_P.C_model.end='d';
C_T_P.C_model.distance_next_send_ms=1100;	
	C_T_P.C_model.Burst_send=0;
	
C_T_P.C_motion.begin='e';
C_T_P.C_motion.end='f';	
C_T_P.C_motion.distance_next_send_ms=1150;	
	C_T_P.C_motion.Burst_send=0;
	C_T_P.C_motion.data='0';
	
C_T_P.BMQ.begin='g';
C_T_P.BMQ.end='h';	
	C_T_P.BMQ.distance_next_send_ms=1020;
	
		
C_T_P.XNQ_L.begin='i';
C_T_P.XNQ_L.end='j';	
	C_T_P.XNQ_L.distance_next_send_ms=980;
		
C_T_P.XNQ_R.begin='k';
C_T_P.XNQ_R.end='l';	
	C_T_P.XNQ_R.distance_next_send_ms=960;
	
	C_T_P.CH_TOTAL.begin='A';
	C_T_P.CH_TOTAL.end='B';
	C_T_P.CH_TOTAL.distance_next_send_ms=900;
	
}
int i_times=0;
//���ݸ���
void com_to_pc_DATA_update(void)
{
//		if(FPS_ALL.YKQ.FPS>=50)
//			{
//			C_T_P.YKQ.data='2';
//			}
//		else if(20<FPS_ALL.YKQ.FPS&&FPS_ALL.YKQ.FPS<50)
//			{
//			C_T_P.YKQ.data='1';
//			}
//		else if(FPS_ALL.YKQ.FPS<=20)
//			{
//			C_T_P.YKQ.data='0';
//			}
	
//C_T_P.CH_TOTAL.data_1=	C_T_P.YKQ.data;
//C_T_P.CH_TOTAL.data_2=	C_T_P.C_model.data;	
//C_T_P.CH_TOTAL.data_3=	C_T_P.C_motion.data;


}

void com_to_pc_control(void)
{
send_begin=0;//��һ�� ���ݴ���һλ��ʼ���
send_total_bit=0;//һ��Ҫ���Ͷ���λ

	if(C_T_P.C_motion.Burst_send==1)//ͻ������
	{
	com_to_pc_C_motion();
		C_T_P.C_motion.Burst_send=0;
	}

	if(C_T_P.C_model.Burst_send==1)//ͻ������
	{
	com_to_pc_C_model();
		C_T_P.C_model.Burst_send=0;
	}
	
//	if(C_T_P.YKQ.distance_next_send_ms<=0)//��һ�η���ʱ���ѵ�
//	{//
//		com_to_pc_YKQ();
//		C_T_P.YKQ.distance_next_send_ms=1000;//Ĭ��1000ms����һ��	
//	}
//	
//	if(C_T_P.C_model.distance_next_send_ms<=1100)//��һ�ε���ģʽ����ʱ���ѵ�
//	{
//	com_to_pc_C_model();
//	C_T_P.C_model.distance_next_send_ms=1000;//Ĭ��1000ms����һ��	
//	}
//	
//	if(C_T_P.C_motion.distance_next_send_ms<=1100)//��һ���˶�״̬����ʱ���ѵ�
//	{
//	com_to_pc_C_motion();
//	C_T_P.C_motion.distance_next_send_ms=1000;//Ĭ��1000ms����һ��	
//	}

	if(C_T_P.CH_TOTAL.distance_next_send_ms<=100)//��һ�ε����ܷ��ͷ���ʱ���ѵ�
	{
		com_to_pc_DATA_update();
//	com_to_pc_CH_TOTAL();
	C_T_P.CH_TOTAL.distance_next_send_ms=100;//Ĭ��1000ms����һ��	
	}	
C_T_P.CH_TOTAL.distance_next_send_ms-=100;
	
	
	if(C_T_P.BMQ.distance_next_send_ms<=100)//��һ�α���������ʱ���ѵ�
	{	
		BMQ_data_update();
		com_to_pc_BMQ();
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//ֻ�е��߽�ֵ�������˲Ż�  ������ʼѲ��	
				{

	
	XNQ_L_data_update();
	com_to_pc_XNQ_L();
	XNQ_R_data_update();
	com_to_pc_XNQ_R();
				}
	C_T_P.BMQ.distance_next_send_ms=100;//Ĭ��1000ms����һ��	
	}
	C_T_P.BMQ.distance_next_send_ms-=100;
	
	if(send_total_bit!=0)//Ҫ���͵����ݲ�Ϊ��    
{	
	if(send_total_bit>33)
	send_err++;//�������
	
	HAL_UART_Transmit_DMA(&huart6,&send_data[0],send_total_bit);
}

}

void com_to_pc_C_model(void)
{
	send_data[send_begin]=C_T_P.C_model.begin;
		send_data[send_begin+1]=C_T_P.C_model.data;
		send_data[send_begin+2]=C_T_P.C_model.end;
		send_total_bit+=3;
		send_begin+=3;

}
void com_to_pc_YKQ(void)
{
	send_data[send_begin]=C_T_P.YKQ.begin;
		send_data[send_begin+1]=C_T_P.YKQ.data;
		send_data[send_begin+2]=C_T_P.YKQ.end;
		send_total_bit+=3;
		send_begin+=3;
}

void com_to_pc_C_motion(void)
{
	send_data[send_begin]=C_T_P.C_motion.begin;
		send_data[send_begin+1]=C_T_P.C_motion.data;
		send_data[send_begin+2]=C_T_P.C_motion.end;
		send_total_bit+=3;
		send_begin+=3;
}

void com_to_pc_CH_TOTAL(void)
{
	send_data[send_begin]=C_T_P.CH_TOTAL.begin;
		send_data[send_begin+1]=C_T_P.CH_TOTAL.data_1;
		send_data[send_begin+2]=C_T_P.CH_TOTAL.data_2;
		send_data[send_begin+3]=C_T_P.CH_TOTAL.data_3;
		send_data[send_begin+4]=C_T_P.CH_TOTAL.end;	
		send_total_bit+=5;
		send_begin+=5;
}

void BMQ_data_update(void)//���������ݸ��º���
{
	int ZGWZ_BMQ_BFB=0;//�ڹ�λ��,ͨ��������,�ٷֱ���ʽ����0-999
	/*...*/
	if(ZGWZ_BMQ_BFB<0)
		ZGWZ_BMQ_BFB=0;
	if(ZGWZ_BMQ_BFB>999)
		ZGWZ_BMQ_BFB=999;	
	
	ZGWZ_BMQ_BFB=(Chassis_Encoder.totalLine-CHASSIS_R_MIN_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);
//	C_T_P.BMQ.data_1=ZGWZ_BMQ_BFB/100+'0';
//	C_T_P.BMQ.data_2=(ZGWZ_BMQ_BFB%100)/10+'0';
//	C_T_P.BMQ.data_3=ZGWZ_BMQ_BFB%10+'0';
	
		i_times++;
		if(i_times<0)
		i_times=1;
	if(i_times>999)
		i_times=1;
//	if(i_times==50)
//		i_times=53;
	
		C_T_P.BMQ.data_1=i_times/100+'0';
	C_T_P.BMQ.data_2=(i_times%100)/10+'0';
	C_T_P.BMQ.data_3=i_times%10+'0';
}

void XNQ_L_data_update(void)//����ǽ�����ݸ��º���
{
	/*...*/
	if(XNQ_L_BMQ_BFB<0)
		XNQ_L_BMQ_BFB=0;
	if(XNQ_L_BMQ_BFB>999)
		XNQ_L_BMQ_BFB=999;	
	/*�������ǽλ�úú�����*/
	XNQ_L_BMQ_BFB=(reverse_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);
	C_T_P.XNQ_L.data_1=XNQ_L_BMQ_BFB/100+'0';
	C_T_P.XNQ_L.data_2=(XNQ_L_BMQ_BFB%100)/10+'0';
	C_T_P.XNQ_L.data_3=XNQ_L_BMQ_BFB%10+'0';
	
}

void XNQ_R_data_update(void)//����ǽ�����ݸ��º���
{
	/*...*/
	if(XNQ_R_BMQ_BFB<0)
		XNQ_R_BMQ_BFB=0;
	if(XNQ_R_BMQ_BFB>999)
		XNQ_R_BMQ_BFB=999;	
	/*�ұ�����ǽλ�úú�����*/
	XNQ_R_BMQ_BFB=((CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER)-CHASSIS_R_MIN_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);
	C_T_P.XNQ_R.data_1=XNQ_R_BMQ_BFB/100+'0';
	C_T_P.XNQ_R.data_2=(XNQ_R_BMQ_BFB%100)/10+'0';
	C_T_P.XNQ_R.data_3=XNQ_R_BMQ_BFB%10+'0';
	
}

void com_to_pc_BMQ(void)//���������Ϳ��ƺ���
{
		send_data[send_begin]=C_T_P.BMQ.begin;
		send_data[send_begin+1]=C_T_P.BMQ.data_1;
		send_data[send_begin+2]=C_T_P.BMQ.data_2;
		send_data[send_begin+3]=C_T_P.BMQ.data_3;
		send_data[send_begin+4]=C_T_P.BMQ.end;
		send_data[send_begin+5]='V';
		send_data[send_begin+6]='V';	

		send_data[send_begin+7]='V';	
		send_data[send_begin+8]='V';	
		send_data[send_begin+9]='V';	
		send_data[send_begin+10]='V';	
		send_data[send_begin+11]='V';	
		send_data[send_begin+12]='V';	
		send_data[send_begin+13]='V';	
		send_data[send_begin+14]='V';	
		send_data[send_begin+15]='V';	
			send_data[send_begin+16]='V';
		send_data[send_begin+17]='V';	

		send_data[send_begin+18]='V';	
		send_data[send_begin+19]='V';	
		send_data[send_begin+20]='V';	
		send_data[send_begin+21]='V';	
		send_data[send_begin+22]='V';	
		send_data[send_begin+23]='V';	
		send_data[send_begin+24]='V';	
		send_data[send_begin+25]='V';	
		send_data[send_begin+26]='V';
		send_data[send_begin+27]='V';	
		send_data[send_begin+28]='V';	
		send_data[send_begin+29]='V';	
		send_data[send_begin+30]='V';	
		send_data[send_begin+31]='V';
		
		send_total_bit+=5;
		send_begin+=5;
}

void com_to_pc_XNQ_L(void)//����ǽ���Ϳ��ƺ���
{
		send_data[send_begin]=C_T_P.XNQ_L.begin;
		send_data[send_begin+1]=C_T_P.XNQ_L.data_1;
		send_data[send_begin+2]=C_T_P.XNQ_L.data_2;
		send_data[send_begin+3]=C_T_P.XNQ_L.data_3;
		send_data[send_begin+4]=C_T_P.XNQ_L.end;	
		send_total_bit+=5;
		send_begin+=5;
}

void com_to_pc_XNQ_R(void)//����ǽ�ҷ��Ϳ��ƺ���
{
		send_data[send_begin]=C_T_P.XNQ_R.begin;
		send_data[send_begin+1]=C_T_P.XNQ_R.data_1;
		send_data[send_begin+2]=C_T_P.XNQ_R.data_2;
		send_data[send_begin+3]=C_T_P.XNQ_R.data_3;
		send_data[send_begin+4]=C_T_P.XNQ_R.end;	
		send_total_bit+=5;
		send_begin+=5;
}



