#ifndef IVC_H
#define IVC_H
#include "stdint.h"

#define I_am_sender 1
#define I_am_receiver 0


#define DATA_TEXT_ID 0x0201   //测试数据ID是0x0201
#define IVC_SEND_ID 109   //发送ID是雷达站（蓝）。
#define IVC_receiv_ID 107//接收ID是哨兵(蓝)。receive

#define START_ID	0XA5



/*裁判系统数据结构*/
typedef __packed struct { 
  uint8_t SOF;
  uint16_t DataLength;
  uint8_t Seq;
  uint8_t CRC8;
  uint16_t CmdID;
} FrameHeader;

#if I_am_sender==1

void IVC_SEND(void);

/* ID: 0x0201      Byte: 8    车间通信测试 */
typedef struct
{
	union
	{
		uint8_t dataBuff[8];
		__packed struct
		{
			uint16_t data_cmd_id;//数据的内容 ID
			uint16_t sender_ID;//发送者的 ID
			uint16_t receiver_ID;//接受者的 ID
			uint8_t data_text_1;//测试数据1
			uint8_t data_text_2;//测试数据2
		};
	}data;
	uint8_t InfoUpdataFlag;
}IVC_text_8_long;



#endif

#if I_am_recive==1

/* ID: 0x0201      Byte: 8    车间通信测试 */
typedef struct
{
	union
	{
		uint8_t dataBuff[8];
		__packed struct
		{
			uint16_t data_cmd_id;//数据的内容 ID
			uint16_t sender_ID;//发送者的 ID
			uint16_t receiver_ID;//接受者的 ID
			uint8_t data_text_1;//测试数据1
			uint8_t data_text_2;//测试数据2
		};
	}data;
	uint8_t InfoUpdataFlag;
}IVC_text_8_long;




#endif

extern IVC_text_8_long IVC_RC_DATA_2;

#endif
