#ifndef CHASSIS_basic_H
#define CHASSIS_basic_H

#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "MY_CHASSIS_CONTROL.h"

#if chassis_radom_mode_two ==1

void CHASSIS_CONTROUL_2(void);

//void just_arrive_targe_speed(int targe_speed);

void star_and_new(void);

typedef struct
{
int game_state_progress;/*0：未开始比赛；
? 1：准备阶段；
? 2：自检阶段；
? 3：5s 倒计时；
? 4：对战中；
? 5：比赛结算中
	*/
int this_progress_remain_time;/*当前阶段剩余时间，单位 */
	
bool red_outpost_is_live;
bool blue_outpost_is_live;	
bool our_outpost_is_live;
} CHASSIS_KEY;

extern CHASSIS_KEY key_message;

extern int send_to_chassis_JUST_MOVE;//发送给底盘的数据_刚好足够动起来
extern int init_times;


#endif

#endif