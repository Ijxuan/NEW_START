#ifndef __DR16RECIVE_H
#define __DR16RECIVE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


void usart1_dr16_init(void);
void DR16_Process(uint8_t *pData);
void DR_16hander(UART_HandleTypeDef *huart);
void NM_swj(void);
void NM_swj2(void);

#define DR16BufferNumber 22
#define DR16BufferTruthNumber 18
#define DR16BufferLastNumber 4
//extern UART_HandleTypeDef huart1;

extern uint8_t DR16Buffer[DR16BufferNumber];
extern uint8_t JSBuffer[8];

#define DR16_GroundInit \
{ \
{0,0,0,0,0,0,0}, \
{0,0,0,0,0}, \
{0}, \
0, \
0, \
&DR16_Process, \
} \


typedef struct
{
	struct {
		int16_t ch0;//yaw
		int16_t ch1;//pitch
		int16_t ch2;//left_right
		int16_t ch3;//forward_back
		uint8_t s_left;
		uint8_t s_right;
		int16_t ch4_DW; //拨轮
	}rc;//遥控器接收到的原始值。

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t keyLeft;
		uint8_t keyRight;

	}mouse;

	union {//union联合体用法
		uint16_t key_code;
		struct { //位域的使用
			bool press_W : 1;
			bool press_S : 1;
			bool press_A : 1;
			bool press_D : 1;

			bool press_Shift : 1;
			bool press_Ctrl : 1;
			bool press_Q : 1;
			bool press_E : 1;

			bool press_R : 1;
			bool press_F : 1;
			bool press_G : 1;
			bool press_Z : 1;

			bool press_X : 1;
			bool press_C : 1;
			bool press_V : 1;
			bool press_B : 1;
		};
	}keyBoard;



	uint16_t infoUpdateFrame;	//帧率
	uint8_t offLineFlag;		//设备离线标志

	/*指针函数*/
//	void(*DR16_ReInit)(void);
	void(*DR16_Process)(uint8_t *pData);
//	void(*DR16_Handler)(UART_HandleTypeDef *huart);

} DR16_t;
extern DR16_t DR16;


/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[7];
		__packed struct
		{
		  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
      uint8_t shooter_id;//发射机构 ID
      uint8_t bullet_freq;//子弹射频 单位 Hz
      float bullet_speed;//子弹射速 单位 m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;
extern ext_shoot_data_t ext_shoot_data;

/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[1];
		__packed struct
		{
		  uint8_t armor_id : 4;/*bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
                             五个装甲片，其他血量变化类型，该变量数值为 0。*/
      uint8_t hurt_type : 4;/*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                              0x1 模块掉线扣血；
                              0x2 超射速扣血；
                              0x3 超枪口热量扣血；
                              0x4 超底盘功率扣血；
                              0x5 装甲撞击扣血*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;
extern ext_robot_hurt_t ext_robot_hurt;


/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#endif
