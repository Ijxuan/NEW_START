#ifndef IVC_H
#define IVC_H
#include "stdint.h"

#define I_am_sender 1
#define I_am_receiver 0


#define DATA_TEXT_ID 0x0201   //��������ID��0x0201
#define IVC_SEND_ID 109   //����ID���״�վ��������
#define IVC_receiv_ID 107//����ID���ڱ�(��)��receive

#define START_ID	0XA5



/*����ϵͳ���ݽṹ*/
typedef __packed struct { 
  uint8_t SOF;
  uint16_t DataLength;
  uint8_t Seq;
  uint8_t CRC8;
  uint16_t CmdID;
} FrameHeader;

#if I_am_sender==1

void IVC_SEND(void);

/* ID: 0x0201      Byte: 8    ����ͨ�Ų��� */
typedef struct
{
	union
	{
		uint8_t dataBuff[8];
		__packed struct
		{
			uint16_t data_cmd_id;//���ݵ����� ID
			uint16_t sender_ID;//�����ߵ� ID
			uint16_t receiver_ID;//�����ߵ� ID
			uint8_t data_text_1;//��������1
			uint8_t data_text_2;//��������2
		};
	}data;
	uint8_t InfoUpdataFlag;
}IVC_text_8_long;



#endif

#if I_am_recive==1

/* ID: 0x0201      Byte: 8    ����ͨ�Ų��� */
typedef struct
{
	union
	{
		uint8_t dataBuff[8];
		__packed struct
		{
			uint16_t data_cmd_id;//���ݵ����� ID
			uint16_t sender_ID;//�����ߵ� ID
			uint16_t receiver_ID;//�����ߵ� ID
			uint8_t data_text_1;//��������1
			uint8_t data_text_2;//��������2
		};
	}data;
	uint8_t InfoUpdataFlag;
}IVC_text_8_long;




#endif

extern IVC_text_8_long IVC_RC_DATA_2;

#endif
