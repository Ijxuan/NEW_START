/**
 * @file CRC.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DRIVER_CRC_H 
#define __DRIVER_CRC_H 

#include "stm32f4xx.h"

/*CRC8校验*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);
/*CRC16添加校验位*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
/*CRC16校验*/
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

#endif	// __DRIVER_CRC_H
/*-----------------------------------file of end------------------------------*/


