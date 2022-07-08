#include "IVC.h"
#include "main.h"
#include "usart.h"
#include "RM_JudgeSystem.h"

/*. ID ˵��
������ ID��1��Ӣ��(��)��
2������(��)��
3/4/5������(��)��
6������(��)��
7���ڱ�(��)��
9���״�վ���죩��
101��Ӣ��(��)��
102������(��)��
103/104/105������(��)��
106������(��)��
107���ڱ�(��)�� 
109���״�վ��������

�ͻ��� ID��0x0101 ΪӢ�۲����ֿͻ���(��)��0x0102�����̲����ֿͻ���((��)��0x0103/0x0104/0x0105��
���������ֿͻ���(��)��0x0106�����в����ֿͻ���((��)�� 0x0165��Ӣ�۲����ֿͻ���(��)��0x0166����
�̲����ֿͻ���(��)��0x0167/0x0168/0x0169�����������ֿͻ��˲���(��)��0x016A�����в����ֿͻ���
(��)*/

/*
�������ݰ���һ��ͳһ�����ݶ�ͷ�ṹ�����ݶΰ��������� ID���������Լ������ߵ� ID ���������ݶΣ�
�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ���ȥ frame_headerͷ,cmd_id �� frame_tailβ �� 9 ���ֽ��Լ�
���ݶ�ͷ�ṹ�� 6 ���ֽڣ��ʶ����͵��������ݶ����Ϊ 113�������������� 0x0301 ���ֽ��������±�
��ʾ���������������� frame-header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ���ֽ�������
ÿ�������˽����������Զ�����������������кϼƴ������� 5000 Byte��
�����з���Ƶ�ʷֱ𲻳���30Hz��
*/
/*
������ ���ݶγ��� ����˵��
0x0301     n       �����˼佻�����ݣ����ͷ��������ͣ����� 10Hz
0x0302     n       �Զ���������������ݽӿڣ�ͨ���ͻ��˴������ͣ����� 30Hz
*/
#if I_am_sender==1
/*0�ֽڿ�ʼ ��С2 ���ݵ����� ID      0x0200~0x02FF           
���������� ID ��ѡȡ������ ID �����ɲ������Զ���

2�ֽڿ�ʼ ��С2 �����ߵ� ID ��ҪУ�鷢���ߵ� ID ��ȷ��
4�ֽڿ�ʼ ��С2 �����ߵ� ID ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID
6�ֽڿ�ʼ ��Сn ���ݶ�      n ��ҪС�� 113*/

IVC_text_8_long IVC_SEND_DATA_2;
uint16_t text_send_data;
void IVC_SEND(void)
{
		static uint8_t temp[128];	

		static uint8_t seq = 0;

		FrameHeader send_frame_header;

	send_frame_header.SOF = START_ID;
	send_frame_header.DataLength = 8;//�������ݳ���8
	send_frame_header.Seq = seq++;
	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
	send_frame_header.CmdID = 0x0301;/* ����ͨ�źͿͻ��˶���0x0301 */
	
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
	
	
	uint8_t send_cnt = 3;//�������
	uint16_t total_len = header_len + _data_len + 2;//header_len + _data_len + CRC16_len
	
	while(send_cnt != 0)
	{
		uint32_t now_time = Get_SystemTick()/1000;	//ms
		if(now_time > next_send_time)
		{
			HAL_UART_Transmit_DMA(refereeUart,temp,total_len);
			
			// ������һ���������ʱ�� 
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
2. �ӿ�Э��˵��
�� 2-1 ͨ��Э���ʽ
frame_header (5-byte)    cmd_id (2-byte)    data (n-byte)    frame_tail (2-byte��CRC16������У��)

�� 2-2 frame_header ��ʽ
SOF     data_length  seq      CRC8
1-byte  2-byte      1-byte   1-byte

�� 2-3 ֡ͷ��ϸ����
��            ƫ��λ��   ��С���ֽڣ�   ��ϸ����
SOF               0        1               ����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
data_length       1        2            ����֡�� data �ĳ���
seq               3        1            �����
CRC8              4        1            ֡ͷ CRC8 У��
*/




