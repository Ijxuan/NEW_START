#include "WS2812.h"
#include "tim.h"
						
#include "DJI_IMU.h"
#include "DR16_RECIVE.h"

#ifdef WS2812_new



// WS2812�Ƶ���ɫ����
uint8_t ws2812_data[WS2812_NUM * 3];

// DMA����Ļ���
uint16_t dma_buffer[DMA_BUFFER_SIZE];

// ����WS2812�Ƶ���ɫ
void WS2812_Update(void)
{
    uint32_t i, j;
    uint8_t data;

    // ����ɫ����ת��ΪDMA���������
    for (i = 0; i < WS2812_NUM * 3; i++)
    {
        data = ws2812_data[i];
        for (j = 0; j < 8; j++)
        {
            if (data & 0x80)
            {
                dma_buffer[i * 8 + j] = 28; // 1�ĸߵ�ƽ����ʱ��Ϊ28��ʱ������
            }
            else
            {
                dma_buffer[i * 8 + j] = 14; // 0�ĸߵ�ƽ����ʱ��Ϊ14��ʱ������
            }
            data <<= 1;
        }
    }

    // ����DMA����
    DMA_HandleTypeDef hdma_tim8_up;
    hdma_tim8_up.Instance = DMA1_Stream1;
    HAL_DMA_Start_IT(&hdma_tim8_up, (uint32_t)dma_buffer, (uint32_t)&TIM8->CCR2, DMA_BUFFER_SIZE);
}


#endif

#ifdef WS2812_old

#define ONE_PULSE        (149)                           //1 ��
#define ZERO_PULSE       (74)                           //0 ��
#define RESET_PULSE      (80)                           //80 ����λ�ź�
#define LED_NUMS         (11)                            //led ����
#define LED_DATA_LEN     (24)                           //led ���ȣ�������Ҫ24bits
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812������Ҫ�����鳤��

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
/*ws2812 ��ʼ��*/
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
/*ȫ������*/
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
	if(RGB_SEND.RGB_CH1>=90)//��������ж�
	{
		ws2812_set_RGB(0, 0xf, 0, 5);//��
	}
	else if(RGB_SEND.RGB_CH1>=70&&RGB_SEND.RGB_CH1<90)
	{
		ws2812_set_RGB(0xf, 0xf, 0, 5);	//��
	}
	else if(RGB_SEND.RGB_CH1>50&&RGB_SEND.RGB_CH1<70)
	{
		ws2812_set_RGB(0xf, 0, 0, 5);	//��ɫ��RGBֵ
	}
	else
	{
		ws2812_set_RGB(0,0,0xf,5);	//��ɫ��RGBֵ	
	}
if(ext_shoot_data.data.bullet_speed==0)//������������
{
		ws2812_set_RGB(0,0,0xf,1);	//��ɫ��RGBֵ	
}	
else{
	if(ext_shoot_data.data.bullet_speed>=MAX_SPEE_SHOOT)
	{
		ws2812_set_RGB(0xf, 0, 0, 1);	//��ɫ��RGBֵ	
	}
	if(ext_shoot_data.data.bullet_speed>(MAX_SPEE_SHOOT-1))
	{
		ws2812_set_RGB(0xf, 0xf, 0, 1);	//��
	}
	else
	{
		ws2812_set_RGB(0, 0xf, 0, 1);//��
	}
}
if(ext_power_heat_data_rc_times_100ms==0)//����ָʾ��
{
		ws2812_set_RGB(0,0,0xf,2);	//��ɫ��RGBֵ	
}
else
{
if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<(MAX_HOT_SHOOT/3.0))
{
		ws2812_set_RGB(0, 0xf, 0, 2);//��
}
else if(ext_power_heat_data.data.shooter_id1_17mm_cooling_heat<(MAX_HOT_SHOOT/3.0*2))
{
		ws2812_set_RGB(0xf, 0xf, 0, 2);	//��
}
else
{
		ws2812_set_RGB(0xf, 0, 0, 2);	//��ɫ��RGBֵ	
}
}

//	}//ws2812_blue
	 HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*ȫ�����*/
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
/*ȫ���̵�*/
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

//# ������ɫ����

int black_r = 0, black_g = 0, black_b = 0; // ��ɫ��RGBֵ

int green_r = 0, green_g = 255, green_b = 0; // ��ɫ��RGBֵ
int yellow_r = 255, yellow_g = 255, yellow_b = 0; // ��ɫ��RGBֵ
int red_r = 255, red_g = 0, red_b = 0; // ��ɫ��RGBֵ
int steps = 51; // �任����
//int i;
// ����任�����е�RGBֵ
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



