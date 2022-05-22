/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//extern uint32_t time_every100us;
extern int time_every1s;
extern uint32_t time_every100ms;

//extern	uint32_t task2_times;
//extern uint32_t task3_times;
//extern uint32_t pendsv_times;
//extern uint32_t sysclk_to_pendsv;
//extern int guance1;
//extern int guance2;

#define HWswitch_1_Pin GPIO_PIN_8
#define HWswitch_2_Pin GPIO_PIN_9

extern int mydelay_100us;
extern uint32_t time7_times;//定时器7中断进入次数

extern uint32_t task_init_times;
extern uint32_t task_can_times;
extern uint32_t task_can2_times;
extern uint32_t task_debug_times;
extern uint32_t task_controul_times;//总控制任务运行次数
extern int every_1s_times;//方便计算

extern float yaw_trage_angle;
extern float yaw_trage_angle2;
extern float yaw_trage_speed;
extern float PITCH_trage_angle;
extern float PITCH_trage_angle_2;

extern int PITCH_trage_speed;

extern int send_to_yaw;
extern int send_to_pitch;

extern float PITCH_IMU_Kp;//这个值要取反
extern float PITCH_MAX_angle;
extern float PITCH_MIN_angle;
extern int send_to_chassis;//发送给底盘的数据
extern int CHASSIS_trage_speed;//底盘的目标速度
extern int CHASSIS_trage_speed_last;//上一次底盘的目标速度

extern int CHASSIS_trage_angle;//底盘的目标角度
extern int send_to_SHOOT_R;//发送给右发射摩擦轮的数据
extern int send_to_SHOOT_L;//发送给左发射摩擦轮的数据
extern int send_to_driver;//发送给拨盘电机的数据

extern int CHASSIS_MAX_SPEED;//底盘最大速度

extern int M_3508_error;

extern int SHOOT_L;//左摩擦轮的目标速度
extern int SHOOT_R;//右摩擦轮的目标速度

extern bool HWswitch_R;
extern bool HWswitch_L;

extern bool HWswitch_R_last;
extern bool HWswitch_L_last;

extern int CHASSIS_L_MAX;
extern int CHASSIS_R_MIN;
extern int speed_change;
extern int CHASSIS_MID;
extern float DEBUFF;
extern int ENCODER_L_MAX;
extern int ENCODER_R_MIN;
extern int ENCODER_M_MID;
extern int ENCODER_LONG;//编码器通过光电更新的轨道长度
extern int M3508_3ms_ago;//3毫秒以前的值
extern int M3508_3ms_change;//3毫秒改变的值
extern short int speed_change_times;
extern int ENCODER_ADD;//
extern float encoder_fbl_k;//分辨率的比例关系
extern int ENCODER_SPEED;//编码器这次的值减上次的值得到加速度
extern int M3508_3ms_ago_total_angle;//3毫秒以前的值
extern int M3508_3ms_ago_speed;//3毫秒改变的值
extern float M3508_speed_angle_kp;//角度与速度的关系
extern int ENCODER_ARRIVE_MAX;//编码器抵达的最大值
extern int ENCODER_ARRIVE_MIN;//编码器抵达的最小值

extern int32_t CHASSIS_L_MAX_by_ENCODER;//变向用的左边界值
extern int32_t CHASSIS_R_MIN_by_ENCODER;//变向用的右边界值
extern int reverse_by_ENCODER; //变向提前值

extern int driver_targe_speed;//拨盘的目标速度


extern int uart_8_times;

extern int time_3_times;

extern float Vision_RawData_Yaw_Angle;
extern float Vision_RawData_Pitch_Angle;
extern bool send_to_C;
extern bool send_to_C_JS_SHOOT;
extern bool send_to_C_JS_HURT;//裁判系统_伤害数据_是否发送给C板
extern bool send_to_C_JS_STATUS;//裁判系统_状态数据_是否发送给C板
extern bool send_to_C_JS_HEAT;//裁判系统_热量数据_是否发送给C板

extern bool send_to_C_IN_END;//是否 处于轨道尽头_状态数据_是否发送给C板
extern bool send_to_C_IN_MID;// 处于轨道中间_状态数据_是否发送给C板  1秒一次


extern bool in_MID;//处在轨道中间段
extern bool in_END;//处在轨道尽头
extern bool last_in_MID;//刚刚处在轨道中间段
extern bool last_in_END;//刚刚处在轨道尽头
extern int send_to_C_STATUS_times;//因为状态数据分4段发送,所以计数保险一点

extern int JS_SEND_times;

extern int send_to_C_times;

extern int CH_TOTAL;

extern float Chassis_PowerLimit;
extern bool stop_chassic_output;
extern bool stop_CH_OP_BC_END;
extern int stop_CH_OP_BC_END_times;
extern bool stop_CH_OP_BC_LESS;
extern int stop_CH_OP_BC_LESS_times;
extern int CHASSIS_trage_speed_temp;


extern int uart_3_times;

extern int M3508_acceleration_cycle;//加速度计算周期(每次计算的间隔)(ms)
extern int M3508_last_speed;//上一时刻的速度,用于加速度计算
extern float M3508_acceleration; //加速度

extern int hurt_times_ago;//多久前被击打
extern float KB_add_speed;//狂暴模式加速
extern float KB_low_JB;//狂暴模式缓冲功率下限
extern float KB_high_JB;//狂暴模式缓冲功率回复目标

extern int usart_7_times;

extern bool disable_for_test;//为了调试

extern bool state_Infrared_R_is_ok;//右边红外传感器运转正常
extern bool state_Infrared_L_is_ok;//左边红外传感器运转正常
typedef struct
{
	bool step_1;
	bool step_2;
	bool step_3;
	bool step_4;
	bool step_5;

	
	
}JC;
extern int ch4_DW_total;

extern int send_to_chassis_special;//发送给底盘的数据
extern bool use_special_send;
extern int ENCODER_CHANGE;//BMQ改变值
extern int restart_times;//重启时间

extern int CAN2_SEND_TASK_times;//任务运行时间
extern int place_SEND_times;//底盘位置发送次数

extern bool break_is_raedy;//刹车已经装备好了

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//控制任务的参数
//堆栈大小
#define RobotCtrl_Size 512
//优先级
#define RobotCtrl_Priority osPriorityRealtime

#define Sensor_Size 128

#define Sensor_Priority osPriorityHigh

//任务句柄
//extern TaskHandle_t RobotCtrl_Handle;
////控制任务入口
//extern void Robot_Control(void const * argument);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int my_ms_delay1(uint32_t ms_Delay);
void my_ms_delay2(uint32_t ms_Delay2);
uint32_t MY_GetTick(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encoder_A_Pin GPIO_PIN_0
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_1
#define Encoder_B_GPIO_Port GPIOA
#define Judge_TX_Pin GPIO_PIN_8
#define Judge_TX_GPIO_Port GPIOD
#define Judge_RX_Pin GPIO_PIN_9
#define Judge_RX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
