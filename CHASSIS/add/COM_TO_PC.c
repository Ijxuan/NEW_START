#include "COM_TO_PC.h"
#include "usart.h"
#include "FPS_Calculate.h"
#include "CHASSIS_CONTROL_basic.h"


uint8_t send_data[33];//上位机发送,最终发送
uint16_t send_begin=0;//下一个 数据从哪一位开始填充
uint16_t send_total_bit=0;//一共要发送多少位
uint16_t send_err=0;//发送错误
COM_TO_PC_e C_T_P;//通过串口发送给电脑上位机


	int XNQ_L_BMQ_BFB=0;//虚拟墙左,通过编码器,百分比形式发送0-999

	int XNQ_R_BMQ_BFB=0;//虚拟墙右,通过编码器,百分比形式发送0-999

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
//数据更新
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
//	com_to_pc_CH_TOTAL();
	C_T_P.CH_TOTAL.distance_next_send_ms=100;//默认1000ms发送一次	
	}	
C_T_P.CH_TOTAL.distance_next_send_ms-=100;
	
	
	if(C_T_P.BMQ.distance_next_send_ms<=100)//下一次编码器发送时机已到
	{	
		BMQ_data_update();
		com_to_pc_BMQ();
				if(CHASSIS_R_MIN_new==1&&CHASSIS_L_MAX_new==1	)	//只有当边界值更新完了才会  真正开始巡航	
				{

	
	XNQ_L_data_update();
	com_to_pc_XNQ_L();
	XNQ_R_data_update();
	com_to_pc_XNQ_R();
				}
	C_T_P.BMQ.distance_next_send_ms=100;//默认1000ms发送一次	
	}
	C_T_P.BMQ.distance_next_send_ms-=100;
	
	if(send_total_bit!=0)//要发送的数据不为空    
{	
	if(send_total_bit>33)
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

void XNQ_L_data_update(void)//虚拟墙左数据更新函数
{
	/*...*/
	if(XNQ_L_BMQ_BFB<0)
		XNQ_L_BMQ_BFB=0;
	if(XNQ_L_BMQ_BFB>999)
		XNQ_L_BMQ_BFB=999;	
	/*左边虚拟墙位置好好想想*/
	XNQ_L_BMQ_BFB=(reverse_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);
	C_T_P.XNQ_L.data_1=XNQ_L_BMQ_BFB/100+'0';
	C_T_P.XNQ_L.data_2=(XNQ_L_BMQ_BFB%100)/10+'0';
	C_T_P.XNQ_L.data_3=XNQ_L_BMQ_BFB%10+'0';
	
}

void XNQ_R_data_update(void)//虚拟墙右数据更新函数
{
	/*...*/
	if(XNQ_R_BMQ_BFB<0)
		XNQ_R_BMQ_BFB=0;
	if(XNQ_R_BMQ_BFB>999)
		XNQ_R_BMQ_BFB=999;	
	/*右边虚拟墙位置好好想想*/
	XNQ_R_BMQ_BFB=((CHASSIS_L_MAX_by_ENCODER-reverse_by_ENCODER)-CHASSIS_R_MIN_by_ENCODER)*1000/(CHASSIS_L_MAX_by_ENCODER-CHASSIS_R_MIN_by_ENCODER);
	C_T_P.XNQ_R.data_1=XNQ_R_BMQ_BFB/100+'0';
	C_T_P.XNQ_R.data_2=(XNQ_R_BMQ_BFB%100)/10+'0';
	C_T_P.XNQ_R.data_3=XNQ_R_BMQ_BFB%10+'0';
	
}

void com_to_pc_BMQ(void)//编码器发送控制函数
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

void com_to_pc_XNQ_L(void)//虚拟墙左发送控制函数
{
		send_data[send_begin]=C_T_P.XNQ_L.begin;
		send_data[send_begin+1]=C_T_P.XNQ_L.data_1;
		send_data[send_begin+2]=C_T_P.XNQ_L.data_2;
		send_data[send_begin+3]=C_T_P.XNQ_L.data_3;
		send_data[send_begin+4]=C_T_P.XNQ_L.end;	
		send_total_bit+=5;
		send_begin+=5;
}

void com_to_pc_XNQ_R(void)//虚拟墙右发送控制函数
{
		send_data[send_begin]=C_T_P.XNQ_R.begin;
		send_data[send_begin+1]=C_T_P.XNQ_R.data_1;
		send_data[send_begin+2]=C_T_P.XNQ_R.data_2;
		send_data[send_begin+3]=C_T_P.XNQ_R.data_3;
		send_data[send_begin+4]=C_T_P.XNQ_R.end;	
		send_total_bit+=5;
		send_begin+=5;
}



