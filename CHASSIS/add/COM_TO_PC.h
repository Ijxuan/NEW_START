#ifndef COM_TO_PC
#define COM_TO_PC

#include "main.h"






typedef struct
{
   char begin;//֡ͷ
   char data;//����λ
   char end;//֡β	
	int16_t	distance_next_send_ms;//������һ�η��ͻ��ж���ms
	int8_t Remaining_times;//ʣ��ķ��ʹ���
}YKQ_e;

typedef struct
{
   char begin;//֡ͷ
   char data;//����λ
   char end;//֡β	
	int16_t	distance_next_send_ms;//������һ�η��ͻ��ж���ms
	int8_t Remaining_times;//ʣ��ķ��ʹ���
		int8_t Burst_send;//ͻ������

}CHASSIS_MODEL_e;

typedef struct
{
   char begin;//֡ͷ
   char data;//����λ
   char end;//֡β	
	int16_t	distance_next_send_ms;//������һ�η��ͻ��ж���ms
	int8_t Remaining_times;//ʣ��ķ��ʹ���
	int8_t Burst_send;//ͻ������
}CHASSIS_MOTION_STATE_e;

typedef struct
{
   char begin;//֡ͷ
   char data_1;//����λ1
   char data_2;//����λ2
   char data_3;//����λ3
   char end;//֡β	
	int16_t	distance_next_send_ms;//������һ�η��ͻ��ж���ms
}BMQ_e;

typedef struct
{
   char begin;//֡ͷ
   char data_1;//����λ1
   char data_2;//����λ2
   char data_3;//����λ3
   char end;//֡β	
	int16_t	distance_next_send_ms;//������һ�η��ͻ��ж���ms
}CH_TOTAL_e;
typedef struct
{
YKQ_e YKQ;//ң����
CHASSIS_MODEL_e C_model;//����ģʽ
CHASSIS_MOTION_STATE_e C_motion;	//�����˶�״̬
CH_TOTAL_e CH_TOTAL;//ң����+����ģʽ+�����˶�״̬
BMQ_e BMQ;	//������
BMQ_e XNQ_L;	//��߽�����ǽ
BMQ_e XNQ_R;	//�ұ߽�����ǽ
}COM_TO_PC_e;

extern COM_TO_PC_e C_T_P;
extern 	int XNQ_L_BMQ_BFB;//����ǽ��,ͨ��������,�ٷֱ���ʽ����0-999

extern 	int XNQ_R_BMQ_BFB;//����ǽ��,ͨ��������,�ٷֱ���ʽ����0-999

void com_to_pc_INIT(void);


void com_to_pc_C_model(void);

void com_to_pc_control(void);
void com_to_pc_YKQ(void);
void com_to_pc_C_motion(void);
void com_to_pc_CH_TOTAL(void);

void com_to_pc_BMQ(void);//���������Ϳ��ƺ���

void BMQ_data_update(void);//���������ݸ��º���

void XNQ_L_data_update(void);//����ǽ�����ݸ��º���
void XNQ_R_data_update(void);//����ǽ�����ݸ��º���

void com_to_pc_XNQ_L(void);//����ǽ���Ϳ��ƺ���
void com_to_pc_XNQ_R(void);//����ǽ�ҷ��Ϳ��ƺ���

#endif