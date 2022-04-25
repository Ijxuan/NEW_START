#ifndef SENSOR_H
#define SENSOR_H

//#include "PID_Position.h"
//#include "PID_Increment.h"
#include <stdio.h>
//#include "CRC.h"
#include "usart.h"
//#include "Robots_Detection.h"
//#include "User_typedef.h"
#include "CRC.h" 

#pragma anon_unions


#define SENSOR_BuffSize (9 + 2) //视觉接收数据缓冲区长度
extern uint8_t SENSOR_L_DataBuff[SENSOR_BuffSize];
extern uint8_t SENSOR_R_DataBuff[SENSOR_BuffSize];

typedef struct
{
	struct
	{
		union
		{
			struct
			{
				uint8_t Start_Tag_one; //帧头
				uint8_t Start_Tag_two; //帧头
				
				uint8_t DIST_L; //帧头
				uint8_t DIST_H; //帧头
				
				uint8_t APM_L; //帧头
				uint8_t APM_H; //帧头

				uint8_t TEMP_L; //帧头
				uint8_t TEMP_H; //帧头
				
				uint8_t ZHECK_SUM; //帧尾


			};
			uint8_t SENSOR_RawData[9];
		};

		int DIST;				//Yaw轴的角度
		int APM;				//Pitch轴的角度
		int TEMP;					//深度
	
	} RawData; //视觉的协议 接收一帧的数据结构体

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//数据错误标志位
	uint8_t DataUpdate_Flag;				//数据更新标志位
	


} sensor_t;

void Sensor_R_DataReceive(uint8_t *data);
void SENSOR_Handler(UART_HandleTypeDef *huart);

void Sensor_L_DataReceive(uint8_t *data);
void SENSOL_Handler(UART_HandleTypeDef *huart);

extern sensor_t Sensor_L;
extern sensor_t Sensor_R;
#endif