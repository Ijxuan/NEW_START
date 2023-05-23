#ifndef  WS2812_H
#define WS2812_H

#include "stdint.h"
#include "tim.h"

//#define WS2812_new

// WS2812�Ƶ�����
#define WS2812_NUM 8

// PWMƵ��Ϊ800KHz
#define PWM_FREQUENCY 800000

// DMA��������ݳ���
#define DMA_BUFFER_SIZE (WS2812_NUM * 24)

// ��ɫ����
#define COLOR_MASK 0x7

extern uint8_t ws2812_data[];

void WS2812_Update(void);

#define WS2812_old

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);

void ws2812_example(void);
void ws2812_init(uint8_t led_nums);
extern void ws2812_blue(uint8_t led_nums);
extern void ws2812_red(uint8_t led_nums);
extern void ws2812_green(uint8_t led_nums);



#endif
