/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DR16_RECIVE.h"
#include "can.h"
#include "BSP_CAN.h"
#include "FreeRTOS.h"
#include "DJI_C_IMU.h"
#include "Vision.h"
#include "timer.h"
#include "RM_JudgeSystem.h"
#include "M3508.h"
#include "Sensor.h"

//#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define pai 3.1415926535897932384626433832795f
float lun_1_quan_m=pai*0.06;
int last_time_s=0;
uint32_t last_time_100_ms=0;

int last_time_totalangle=0;
int last_time_totalangle_100_ms=0;

float speed_every_1s=0;
float speed_every_100_ms=0;

int angle_1_s=0;
int angle_100_ms=0;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
CAN_Rx_TypeDef CAN1_Rx_Structure;
CAN_Rx_TypeDef CAN2_Rx_Structure;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	zdcsjc/**/++;
	#if 0
//		BaseType_t CAN1_pxHigherPriorityTaskWoken;
//			uint32_t ID;

//	                __HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
					/*can1接收不用队列*/
//					HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,
//					&CAN_RxMessage.CAN_RxHeader/**/,
//					CAN_RxMessage.CAN_RxMessage);
//	__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

					/*can1接收不用队列*/
//	if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	
////	BaseType_t CAN2_pxHigherPriorityTaskWoken;
//		if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
//	{
//			if(CAN1_Queue != pdFALSE)
//		{

//			HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_Rx_Structure.CAN_RxMessage,CAN1_Rx_Structure.CAN_RxMessageData);
//				
////			xQueueSendFromISR(CAN1_Queue,&CAN1_Rx_Structure,&CAN1_pxHigherPriorityTaskWoken);
//		
//		}	
//	}
//		//出队的状态变量
//	BaseType_t ExitQueue_Status;
//  /* Infinite loop */
//	  		//死等队列有消息
//		ExitQueue_Status = xQueueReceive(CAN1_Queue, &CAN1_Rx_Structure, portMAX_DELAY);
//		//出队成功
//				__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	
//	if(hcanx == &hcan1)
//	{
		
//		if(CAN1_Queue != pdFALSE)
//		{

//			HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_Rx_Structure.CAN_RxMessage,CAN1_Rx_Structure.CAN_RxMessageData);
//				
//			xQueueSendFromISR(CAN1_Queue,&CAN1_Rx_Structure,&CAN1_pxHigherPriorityTaskWoken);
//		
//			__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
//		}	

//	}
#endif
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
//					__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
	time_3_times++;
		FreeRTOSRunTimeTicks++;
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
DR_16hander(&huart1);

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	uart_3_times++;
	JudgeSystem_Handler(&huart3);

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
time7_times++;//定时器7中断进入次数
//现在是1ms进入一次中断
// time_every100us++;
 time_every1s=time7_times/1000;
	time_every100ms=time7_times/100;
	if(time_every1s!=last_time_s)//每秒刷新
	{
		angle_1_s=M3508s[3].totalAngle-last_time_totalangle;
		speed_every_1s=(angle_1_s/8191)/14*lun_1_quan_m;//这一秒走过的路程 变成圈数 再经过减速比 最后根据轮径
		last_time_totalangle=M3508s[3].totalAngle;
	}
		if(time_every100ms!=last_time_100_ms)//每0.1秒刷新
	{
		angle_100_ms=M3508s[3].totalAngle-last_time_totalangle_100_ms;
//		speed_every_100_ms=(angle_100_ms/8191.0)/19.0*lun_1_quan_m;//这一秒走过的路程 变成圈数 再经过减速比 最后根据轮径
		speed_every_100_ms=angle_100_ms*lun_1_quan_m/8191.0/14.0*10;//这100m秒走过的路程 变成圈数 再经过减速比 最后根据轮径

		last_time_totalangle_100_ms=M3508s[3].totalAngle;
	}
	every_1s_times=task_controul_times/(time7_times/1000);

//	every1s_task2times=task2_times/time_every1s;
//	every1s_task3times=task3_times/time_every1s;
//	every1s_sys_to_pendtimes= sysclk_to_pendsv   /time_every1s;
	
	
//			 	  guance1=time_every100us- mydelay_100us;
		last_time_100_ms=time_every100ms;

last_time_s=time_every1s;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
		task_can2_times++;

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
	usart_7_times++;
SENSOR_Handler(&huart7);
  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */
	uart_8_times++;
//	Vision_Handler(&huart8);
//SENSOL_Handler(&huart8);
  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcanx)
{

	BaseType_t CAN1_pxHigherPriorityTaskWoken;
	BaseType_t CAN2_pxHigherPriorityTaskWoken;
	if(hcanx == &hcan1)
	{
		
		if(CAN1_Queue != pdFALSE)
		{

			HAL_CAN_GetRxMessage(
				&hcan1,
			CAN_RX_FIFO0,
				&CAN1_Rx_Structure.CAN_RxMessage,
			CAN1_Rx_Structure.CAN_RxMessageData
									);
				
			xQueueSendFromISR(CAN1_Queue,
							&CAN1_Rx_Structure,
							&CAN1_pxHigherPriorityTaskWoken);
//							HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,
//					&CAN_RxMessage.CAN_RxHeader/**/,
//					CAN_RxMessage.CAN_RxMessage);
			__HAL_CAN_CLEAR_FLAG(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}	

	}
//						__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		portYIELD_FROM_ISR(CAN1_pxHigherPriorityTaskWoken);

	if(hcanx == &hcan2)
	{
//		if(CAN2_Queue != pdFALSE)
//		{
			HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN2_Rx_Structure.CAN_RxMessage,CAN2_Rx_Structure.CAN_RxMessageData);
		
			xQueueSendFromISR(CAN2_Queue,&CAN2_Rx_Structure,&CAN2_pxHigherPriorityTaskWoken);
			
			__HAL_CAN_CLEAR_FLAG(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
			
//		}
		
//					/*DJIC_IMU*/
//			//IMU_Euler
//			if(CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Angle)
//			{
//				DJI_C_Euler_getInfo(CAN2_Rx_Structure);
//			}
//			//Gyro
//			if(CAN2_Rx_Structure.CAN_RxMessage.StdId == DJI_C_Gyro)
//			{
//				DJI_C_Gyro_getInfo(CAN2_Rx_Structure);
//			}
//			Updata_Hand_Euler_Gyro_Data();
	}
	
//		portYIELD_FROM_ISR(CAN1_pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(CAN2_pxHigherPriorityTaskWoken);
	
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
