#include "COM_TO_PC.h"
#include "usart.h"
#include "FPS_Calculate.h"
#include "CHASSIS_CONTROL_basic.h"


uint8_t send_data[20];//上位机发送,最终发送
uint16_t send_begin=0;//下一个 数据从哪一位开始填充
uint16_t send_total_bit=0;//一共要发送多少位
uint16_t send_err=0;//发送错误
COM_TO_PC_e C_T_P;//通过串口发送给电脑上位机

void com_to_pc_INIT(void)//协议初始化
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
	
	C_T_P.CH_TOTAL.begin='A';
	C_T_P.CH_TOTAL.end='B';
	C_T_P.CH_TOTAL.distance_next_send_ms=900;
}

//数据更新
void com_to_pc_DATA_update(void)
{
		if(FPS_ALL.YKQ.FPS>=50)
			{
			C_T_P.YKQ.data='2';
			}
		else if(20<FPS_ALL.YKQ.FPS&&FPS_ALL.YKQ.FPS<50)
			{
			C_T_P.YKQ.data='1';
			}
		else if(FPS_ALL.YKQ.FPS<=20)
			{
			C_T_P.YKQ.data='0';
			}
C_T_P.CH_TOTAL.data_1=	C_T_P.YKQ.data;
C_T_P.CH_TOTAL.data_2=	C_T_P.C_model.data;	
C_T_P.CH_TOTAL.data_3=	C_T_P.C_motion.data;
		

}

void com_to_pc_control(void)
{
send_begin=0;//下一个 数据从哪一位开始填充
send_total_bit=0;//一共要发送多少位

	if(C_T_P.C_motion.Burst_send==1)//突发传输
	{
	com_to_pc_C_motion();
		C_T_P.C_motion.Burst_send=0;
	}

	if(C_T_P.C_model.Burst_send==1)//突发传输
	{
	com_to_pc_C_model();
		C_T_P.C_model.Burst_send=0;
	}
	
//	if(C_T_P.YKQ.distance_next_send_ms<=0)//下一次发送时机已到
//	{//
//		com_to_pc_YKQ();
//		C_T_P.YKQ.distance_next_send_ms=1000;//默认1000ms发送一次	
//	}
//	
//	if(C_T_P.C_model.distance_next_send_ms<=1100)//下一次底盘模式发送时机已到
//	{
//	com_to_pc_C_model();
//	C_T_P.C_model.distance_next_send_ms=1000;//默认1000ms发送一次	
//	}
//	
//	if(C_T_P.C_motion.distance_next_send_ms<=1100)//下一次运动状态发送时机已到
//	{
//	com_to_pc_C_motion();
//	C_T_P.C_motion.distance_next_send_ms=1000;//默认1000ms发送一次	
//	}

	if(C_T_P.CH_TOTAL.distance_next_send_ms<=100)//下一次底盘总发送发送时机已到
	{
		com_to_pc_DATA_update();
	com_to_pc_CH_TOTAL();
	C_T_P.CH_TOTAL.distance_next_send_ms=100;//默认1000ms发送一次	
	}	
C_T_P.CH_TOTAL.distance_next_send_ms-=100;
	if(C_T_P.BMQ.distance_next_send_ms<=100)//下一次编码器发送时机已到
	{
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//只有当边界值更新完了才会  真正开始巡航	
				{
	BMQ_data_update();
//		com_to_pc_BMQ();
				}

	C_T_P.BMQ.distance_next_send_ms=100;//默认1000ms发送一次	
	}
				C_T_P.CH_TOTAL.distance_next_send_ms-=100;
	
	if(send_total_bit!=0)//要发送的数据不为空    
{	
	if(send_total_bit>20)
	send_err++;//数组溢出
	
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

void BMQ_data_update(void)//编码器数据更新函数
{
	int ZGWZ_BMQ_BFB=0;//在轨位置,通过编码器,百分比形式发送0-999
	/*...*/
	ZGWZ_BMQ_BFB=(Chassis_Encoder.totalLine-CHASSIS_R_MIN_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);

}
void com_to_pc_BMQ(void)//编码器发送控制函数
{
		send_data[send_begin]=C_T_P.CH_TOTAL.begin;
		send_data[send_begin+1]=C_T_P.CH_TOTAL.data_1;
		send_data[send_begin+2]=C_T_P.CH_TOTAL.data_2;
		send_data[send_begin+3]=C_T_P.CH_TOTAL.data_3;
		send_data[send_begin+4]=C_T_P.CH_TOTAL.end;	
		send_total_bit+=5;
		send_begin+=5;
}






