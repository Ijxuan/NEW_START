#include "WS2812.h"
#include "tim.h"
						
#include "DJI_IMU.h"
#include "DR16_RECIVE.h"

#ifdef WS2812_new



// WS2812灯的颜色数据
uint8_t ws2812_data[WS2812_NUM * 3];

// DMA传输的缓存
uint16_t dma_buffer[DMA_BUFFER_SIZE];

// 更新WS2812灯的颜色
void WS2812_Update(void)
{
    uint32_t i, j;
    uint8_t data;

    // 将颜色数据转换为DMA传输的数据
    for (i = 0; i < WS2812_NUM * 3; i++)
    {
        data = ws2812_data[i];
        for (j = 0; j < 8; j++)
        {
            if (data & 0x80)
            {
                dma_buffer[i * 8 + j] = 28; // 1的高电平持续时间为28个时钟周期
            }
            else
            {
                dma_buffer[i * 8 + j] = 14; // 0的高电平持续时间为14个时钟周期
            }
            data <<= 1;
        }
    }

    // 启动DMA传输
    DMA_HandleTypeDef hdma_tim8_up;
    hdma_tim8_up.Instance = DMA1_Stream1;
    HAL_DMA_Start_IT(&hdma_tim8_up, (uint32_t)dma_buffer, (uint32_t)&TIM8->CCR2, DMA_BUFFER_SIZE);
}


#endif

#ifdef WS2812_old

#define ONE_PULSE        (149)                           //1 码
#define ZERO_PULSE       (74)                           //0 码
#define RESET_PULSE      (80)                           //80 ，复位信号
#define LED_NUMS         (11)                            //led 数量
#define LED_DATA_LEN     (24)                           //led 长度，单个需要24bits
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812灯条需要的数组长度

uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };

void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0;i < 8;i++)
    {
        //
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;

    }
}
/*ws2812 初始化*/
void ws2812_init(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data)); 
}
/*全亮蓝灯*/
int rgb_r;
int rgb_g;
int rgb_b;

void ws2812_blue(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
//	for(uint8_t i = 0; i < led_nums; i++)
//	{
		ws2812_set_RGB(rgb_r, rgb_g, rgb_b, 4);
	if(RGB_SEND.RGB_CH1>=90)//超电电量判断
	{
		ws2812_set_RGB(0, 0xf, 0, 5);//绿
	}
	else if(RGB_SEND.RGB_CH1>=70&&RGB_SEND.RGB_CH1<90)
	{
		ws2812_set_RGB(0xf, 0xf, 0, 5);	//黄
	}
	else if(RGB_SEND.RGB_CH1>50&&RGB_SEND.RGB_CH1<70)
	{
		ws2812_set_RGB(0xf, 0, 0, 5);	//红色的RGB值
	}
	else
	{
		ws2812_set_RGB(0,0,0xf,5);	//蓝色的RGB值	
	}
if(ext_shoot_data.data.bullet_speed==0)//发射数据离线
{
		ws2812_set_RGB(0,0,0xf,1);	//蓝色的RGB值	
}	
else{
	if(ext_shoot_data.data.bullet_speed>=MAX_SPEE_SHOOT)
	{
		ws2812_set_RGB(0xf, 0, 0, 1);	//红色的RGB值	
	}
	if(ext_shoot_data.data.bullet_speed>(MAX_SPEE_SHOOT-1))
	{
		ws2812_set_RGB(0xf, 0xf, 0, 1);	//黄
	}
	else
	{
		ws2812_set_RGB(0, 0xf, 0, 1);//绿
	}
}
if(ext_power_heat_data_rc_times_100ms==0)//热量指示灯
{
		ws2812_set_RGB(0,0,0xf,2);	//蓝色的RGB值	
}
else
{
if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<(MAX_HOT_SHOOT/3.0))
{
		ws2812_set_RGB(0, 0xf, 0, 2);//绿
}
else if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<(MAX_HOT_SHOOT/3.0*2))
{
		ws2812_set_RGB(0xf, 0xf, 0, 2);	//黄
}
else
{
		ws2812_set_RGB(0xf, 0, 0, 2);	//红色的RGB值	
}
}

//	}//ws2812_blue
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮红灯*/
void ws2812_red(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x22, 0x00, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮绿灯*/
void ws2812_green(uint8_t led_nums)
{
	uint16_t num_data;
	num_data = 80 + led_nums * 24;
	for(uint8_t i = 0; i < led_nums; i++)
	{
		ws2812_set_RGB(0x00, 0x22, 0x00, i);
	}
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

void ws2812_example(void)
{		
	  ws2812_set_RGB(0x00, 0x00, 0x22, 0);
    ws2812_set_RGB(0x00, 0x00, 0x22, 1);
    ws2812_set_RGB(0x00, 0x00, 0x22, 2);
    ws2812_set_RGB(0x00, 0x00, 0x22, 3);
    ws2812_set_RGB(0x00, 0x00, 0x22, 6);
    ws2812_set_RGB(0x00, 0x00, 0x22, 7);
		ws2812_set_RGB(0x00, 0x00, 0x22, 8);
    ws2812_set_RGB(0x00, 0x00, 0x22, 10);
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(344)); //344 = 80 + 24 * LED_NUMS(11)

    HAL_Delay(500);
}


#endif


int now_speed_0_100;

void get_rgb_value()
{

//# 定义颜色常量

int black_r = 0, black_g = 0, black_b = 0; // 黑色的RGB值

int green_r = 0, green_g = 255, green_b = 0; // 绿色的RGB值
int yellow_r = 255, yellow_g = 255, yellow_b = 0; // 黄色的RGB值
int red_r = 255, red_g = 0, red_b = 0; // 红色的RGB值
int steps = 51; // 变换步数
//int i;
// 输出变换过程中的RGB值
//for (i = 0; i <= steps; i++) {
	if(now_speed_0_100<50)
	{
    rgb_r = (int)(black_r + (green_r - black_r) * ((float)now_speed_0_100 / steps));
    rgb_g = (int)(black_g + (green_g - black_g) * ((float)now_speed_0_100 / steps));
    rgb_b = (int)(black_b + (green_b - black_b) * ((float)now_speed_0_100 / steps));	
	}
	else if(now_speed_0_100>=50&&now_speed_0_100<=100)
	{
    rgb_r = (int)(green_r + (red_r - green_r) * ((float)(now_speed_0_100-50) / steps));
    rgb_g = (int)(green_g + (red_g - green_g) * ((float)(now_speed_0_100-50) / steps));
    rgb_b = (int)(green_b + (red_b - green_b) * ((float)(now_speed_0_100-50) / steps));	
	}
	rgb_r=rgb_r*now_speed_0_100/100.0;
	rgb_g=rgb_g*now_speed_0_100/100.0;
	rgb_b=rgb_b*now_speed_0_100/100.0;
	if(rgb_r>255){rgb_r=255;}
	if(rgb_g>255){rgb_g=255;}
	if(rgb_b>255){rgb_b=255;}
	if(rgb_r<0){rgb_r=0;}
	if(rgb_g<0){rgb_g=0;}
	if(rgb_b<0){rgb_b=0;}	
//}
}


void ws2812_cell(void)
{
	uint16_t num_data;
	num_data = 80 + 8 * 24;
//	for(uint8_t i = 0; i < led_nums; i++)
//	{
		ws2812_set_RGB(rgb_r, rgb_g, rgb_b, 4);
//	}//ws2812_blue
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}



