#ifndef CAN2_H
#define CAN2_H

#include "main.h"
#include "user_can.h"		//二选一
//#include "BSP_CAN.h"		//二选一

#define YAW_6020_SEND_ID 0x132

//#include "DR16_RECIVE.h"

//#include "GM6020.h"
//#include "my_positionPID_bate.h"
////#include "DJI_C_IMU.h"

extern bool send_YAW_60020;
extern uint8_t GM_69020_SEND_all[9];


//void cloud_control(void);
//void YAW_PID(void);

//void PITCH_PID(void);
//void imu_angle(void);
void CAN_2_SEND_YAW_6020(void);

#endif






