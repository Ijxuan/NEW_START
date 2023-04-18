#include "CAN2_SEND.h"
bool send_YAW_60020;
uint8_t GM_69020_SEND_all[9];
void CAN_2_SEND_YAW_6020()
{
	if(send_YAW_60020==1)
	{
			CAN_SendData(&hcan2,CAN_ID_STD,YAW_6020_SEND_ID,&GM_69020_SEND_all[0]);
	send_YAW_60020=0;//发送标志位清零
	}
}







