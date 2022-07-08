#include "IVC.h"
#include "main.h"
#include "usart.h"
#include "RM_JudgeSystem.h"

/*. ID 说明
机器人 ID：1，英雄(红)；
2，工程(红)；
3/4/5，步兵(红)；
6，空中(红)；
7，哨兵(红)；
9，雷达站（红）；
101，英雄(蓝)；
102，工程(蓝)；
103/104/105，步兵(蓝)；
106，空中(蓝)；
107，哨兵(蓝)； 
109，雷达站（蓝）。

客户端 ID：0x0101 为英雄操作手客户端(红)；0x0102，工程操作手客户端((红)；0x0103/0x0104/0x0105，
步兵操作手客户端(红)；0x0106，空中操作手客户端((红)； 0x0165，英雄操作手客户端(蓝)；0x0166，工
程操作手客户端(蓝)；0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；0x016A，空中操作手客户端
(蓝)*/

/*
交互数据包括一个统一的数据段头结构。数据段包含了内容 ID，发送者以及接收者的 ID 和内容数据段，
整个交互数据的包总共长最大为 128 个字节，减去 frame_header头,cmd_id 和 frame_tail尾 共 9 个字节以及
数据段头结构的 6 个字节，故而发送的内容数据段最大为 113。整个交互数据 0x0301 的字节限制如下表
所示，其中数据量包括 frame-header,cmd_id,frame_tail 以及数据段头结构的字节数量。
每个机器人交互数据与自定义控制器数据上下行合计带宽不超过 5000 Byte。
上下行发送频率分别不超过30Hz。
*/
/*
命令码 数据段长度 功能说明
0x0301     n       机器人间交互数据，发送方触发发送，上限 10Hz
0x0302     n       自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
*/
#if I_am_sender==1
/*0字节开始 大小2 数据的内容 ID      0x0200~0x02FF           
可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

2字节开始 大小2 发送者的 ID 需要校验发送者的 ID 正确性
4字节开始 大小2 接收者的 ID 需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
6字节开始 大小n 数据段      n 需要小于 113*/

IVC_text_8_long IVC_SEND_DATA_2;
uint16_t text_send_data;
void IVC_SEND(void)
{
		static uint8_t temp[128];	

		static uint8_t seq = 0;

		FrameHeader send_frame_header;

	send_frame_header.SOF = START_ID;
	send_frame_header.DataLength = 8;//测试数据长度8
	send_frame_header.Seq = seq++;
	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
	send_frame_header.CmdID = 0x0301;/* 车间通信和客户端都是0x0301 */
	
	uint8_t header_len = sizeof(send_frame_header);

		memcpy((void*)temp, &send_frame_header, header_len);//frame header


	
IVC_SEND_DATA_2.data.data_cmd_id=DATA_TEXT_ID;
IVC_SEND_DATA_2.data.sender_ID=IVC_SEND_ID;
IVC_SEND_DATA_2.data.receiver_ID=IVC_receiv_ID;

text_send_data++;
IVC_SEND_DATA_2.data.data_text_1=	text_send_data;
IVC_SEND_DATA_2.data.data_text_2=	text_send_data+1;

memcpy((void*)(temp+header_len), IVC_SEND_DATA_2.data.dataBuff, 8);//data
	* (__packed short *)(&temp[header_len + 8]) = Get_CRC16_Check_Sum(temp,header_len + 8,0xffff);//CRC16

	if(text_send_data>10)
	{
	text_send_data=0;
	}
		uint16_t total_len = header_len + 8 + 2;//header_len + _data_len + CRC16_len

		for (uint8_t i = 0; i < total_len; i++)
	{
		while ((USART3->SR & 0X40) == 0);
		USART3->DR = temp[i];
	}
	
}


#endif
IVC_text_8_long IVC_RC_DATA_2;

#if I_am_receiver==1




#endif








/*


void referee_Classdef::send_toReferee(uint16_t _cmd_id, uint8_t* _data, uint16_t _data_len, receiverType_e _receiver)
{
	static uint8_t temp[128];	
	static uint8_t seq = 0;
	
	FrameHeader send_frame_header;

	send_frame_header.SOF = START_ID;
	send_frame_header.DataLength = _data_len;
	send_frame_header.Seq = seq++;
	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
	send_frame_header.CmdID = _cmd_id;
	
	uint8_t header_len = sizeof(send_frame_header);
	
	memcpy((void*)temp, &send_frame_header, header_len);//frame header
	memcpy((void*)(temp+header_len), _data, _data_len);//data
	* (__packed short *)(&temp[header_len + _data_len]) = Get_CRC16_Check_Sum(temp,header_len + _data_len,0xffff);//CRC16
	
	
	uint8_t send_cnt = 3;//传输次数
	uint16_t total_len = header_len + _data_len + 2;//header_len + _data_len + CRC16_len
	
	while(send_cnt != 0)
	{
		uint32_t now_time = Get_SystemTick()/1000;	//ms
		if(now_time > next_send_time)
		{
			HAL_UART_Transmit_DMA(refereeUart,temp,total_len);
			
			// 计算下一次允许传输的时间 
			next_send_time = now_time + float(total_len) / 3720 * 1000;
			
			if(_receiver == OtherRobot)
				send_cnt = 0;
			else
				send_cnt--;
		}	
		vTaskDelay(5);
	}
	
}
*/

/*
2. 接口协议说明
表 2-1 通信协议格式
frame_header (5-byte)    cmd_id (2-byte)    data (n-byte)    frame_tail (2-byte，CRC16，整包校验)

表 2-2 frame_header 格式
SOF     data_length  seq      CRC8
1-byte  2-byte      1-byte   1-byte

表 2-3 帧头详细定义
域            偏移位置   大小（字节）   详细描述
SOF               0        1               数据帧起始字节，固定值为 0xA5
data_length       1        2            数据帧中 data 的长度
seq               3        1            包序号
CRC8              4        1            帧头 CRC8 校验
*/




