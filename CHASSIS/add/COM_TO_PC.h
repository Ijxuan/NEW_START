#ifndef COM_TO_PC
#define COM_TO_PC

#include "main.h"






typedef struct
{
   char begin;//帧头
   char data;//数据位
   char end;//帧尾	
	int16_t	distance_next_send_ms;//距离下一次发送还有多少ms
	int8_t Remaining_times;//剩余的发送次数
}YKQ_e;

typedef struct
{
   char begin;//帧头
   char data;//数据位
   char end;//帧尾	
	int16_t	distance_next_send_ms;//距离下一次发送还有多少ms
	int8_t Remaining_times;//剩余的发送次数
		int8_t Burst_send;//突发传输

}CHASSIS_MODEL_e;

typedef struct
{
   char begin;//帧头
   char data;//数据位
   char end;//帧尾	
	int16_t	distance_next_send_ms;//距离下一次发送还有多少ms
	int8_t Remaining_times;//剩余的发送次数
	int8_t Burst_send;//突发传输
}CHASSIS_MOTION_STATE_e;

typedef struct
{
   char begin;//帧头
   char data_1;//数据位1
   char data_2;//数据位2
   char data_3;//数据位3
   char end;//帧尾	
	int16_t	distance_next_send_ms;//距离下一次发送还有多少ms
}BMQ_e;

typedef struct
{
   char begin;//帧头
   char data_1;//数据位1
   char data_2;//数据位2
   char data_3;//数据位3
   char end;//帧尾	
	int16_t	distance_next_send_ms;//距离下一次发送还有多少ms
}CH_TOTAL_e;
typedef struct
{
YKQ_e YKQ;//遥控器
CHASSIS_MODEL_e C_model;//底盘模式
CHASSIS_MOTION_STATE_e C_motion;	//底盘运动状态
CH_TOTAL_e CH_TOTAL;//遥控器+底盘模式+底盘运动状态
BMQ_e BMQ;	//编码器
BMQ_e XNQ_L;	//左边界虚拟墙
BMQ_e XNQ_R;	//右边界虚拟墙
}COM_TO_PC_e;

extern COM_TO_PC_e C_T_P;
extern 	int XNQ_L_BMQ_BFB;//虚拟墙左,通过编码器,百分比形式发送0-999

extern 	int XNQ_R_BMQ_BFB;//虚拟墙右,通过编码器,百分比形式发送0-999

void com_to_pc_INIT(void);


void com_to_pc_C_model(void);

void com_to_pc_control(void);
void com_to_pc_YKQ(void);
void com_to_pc_C_motion(void);
void com_to_pc_CH_TOTAL(void);

void com_to_pc_BMQ(void);//编码器发送控制函数

void BMQ_data_update(void);//编码器数据更新函数

void XNQ_L_data_update(void);//虚拟墙左数据更新函数
void XNQ_R_data_update(void);//虚拟墙右数据更新函数

void com_to_pc_XNQ_L(void);//虚拟墙左发送控制函数
void com_to_pc_XNQ_R(void);//虚拟墙右发送控制函数

#endif