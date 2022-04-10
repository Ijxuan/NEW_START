#include "CAN2_SEND.h"

uint8_t js_SEND_all[33];  // 4x8=32

void JS_send_SHOOT_control()
{
	for(int i=0;i<7;i++ )//0到6位有效,一共7位
{
	js_SEND_all[i]=ext_shoot_data.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_SHOOT_ID,&js_SEND_all[0]);

}
void JS_send_HURT_control()
{

	js_SEND_all[0]=ext_robot_hurt.data.dataBuff[0];

	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_HURT_ID,&js_SEND_all[0]);

}

void JS_send_STATUS_control()
{

	for(int i=0;i<27;i++ )//0到26位有效,一共27位
{
	js_SEND_all[i]=ext_game_robot_state.data.dataBuff[i];
}
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_ONE,&js_SEND_all[0]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_TWO,&js_SEND_all[8]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_THREE,&js_SEND_all[16]);
osDelay(1);
	CAN_SendData(&hcan2,CAN_ID_STD,JS_SEND_STATUS_ID_FOUR,&js_SEND_all[24]);
osDelay(1);


}
