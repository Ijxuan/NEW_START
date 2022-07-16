#include "BREAK.h"
#include "main.h"

BREAK_e break_basic;

long M2006_targe_angle = 0;
int M2006_targe_speed = 0;
int send_to_break = 0;

int M2006_init_times = 0;            //初始化 计时
int M2006_init_change_angle = 0;     //初始化 角度变化量
int M2006_init_totalAngle_last = 0;  //初始化 上一时刻角度值


bool start_use_break=0;//使用刹车,这是标志位


/*
P_PID_t BREAK_ANGLE_pid;//刹车PID
P_PID_t BREAK_SPEED_pid;
*/
void break_init(void) {
  M2006_init_times++;

  if (M2006_init_times % 50 == 0)  //每150ms检测一次
  {
    M2006_init_change_angle =
        M3508s[BREAK_ID].totalAngle - M2006_init_totalAngle_last;
    M2006_init_totalAngle_last = M3508s[BREAK_ID].totalAngle;
  }

  
  
  if (DR16.rc.s_left == 1)  //左上挡/自动挡 就一定要先初始化刹车
  {
    if (break_is_raedy == 0)  //刹车没有准备好
      if (send_to_chassis == 0 && abs(M3508s[3].realSpeed) < 10)  //底盘已经失能
      {
      }
  } 
  
  
  else if (DR16.rc.s_left == 3 &&
             DR16.rc.s_right == 2)  //手动 初始化刹车,此时 摩擦轮速度为0
  {
	  /* 手动开启初始化 (单独判断遥控器)
    if (DR16.rc.ch4_DW >= 200)  //拨下
    {
      if (break_basic.STATE == 0)  //未初始化
      {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //开始初始化就清0
      }
    }
	  */
	        if (break_basic.STATE == 0)  //未初始化
      {
		  if(start_use_break==1)
		  {
        break_basic.STATE = 1;
        M2006_init_times = 0;  //开始初始化就清0
			  
		  }
      }
    else if (break_basic.STATE == 1)  //刹车最大值没有准备好.正在初始化最大值
    {
      BREAK_SPEED_pid.Max_result = 2000;
      BREAK_SPEED_pid.Min_result = -2000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle + 999;
      if (M2006_init_times > 200)  //给200*3=600ms 用来起步
      {
        if (abs(M2006_init_change_angle) < 200)  //角度变化小于200
        {
          //				if(abs(M3508s[BREAK_ID].realCurrent)>1500)//加入电流值判断
          break_basic.BREAK_MAX = M3508s[BREAK_ID].totalAngle;  //最大值找到了
          break_basic.STATE = 2;  //去初始化最小值
          M2006_init_times = 0;
        }
      }
    }
	else if (break_basic.STATE == 2)  //正在初始化最小值
    {
      BREAK_SPEED_pid.Max_result = 2000;
      BREAK_SPEED_pid.Min_result = -2000;
      M2006_targe_angle = M3508s[BREAK_ID].totalAngle - 999;
      if (M2006_init_times > 200)  //给200*3=600ms 用来起步
      {
        if (abs(M2006_init_change_angle) < 200)  //角度变化小于200
        {
          break_basic.BREAK_MIN = M3508s[BREAK_ID].totalAngle;  //最小值找到了
          break_basic.BREAK_MID =(break_basic.BREAK_MIN + break_basic.BREAK_MAX) /2;  //中间值找到了
			M2006_targe_angle=break_basic.BREAK_MID;
			BREAK_SPEED_pid.Max_result = 9000;
      BREAK_SPEED_pid.Min_result = -9000;
          break_basic.STATE = 3;  //初始化全部完成
        }
      }
    }
  }

  if (DR16.rc.s_left == 2)  //失能
  {
    M2006_init_times = 0;  //初始化计时清零
    if (break_basic.STATE == 2 ||
        break_basic.STATE == 1)  //初始化到一半就失能了,直接从头开始初始化
    {
      break_basic.STATE = 0;
    }
  }
}


void break_control(void) {
	if (DR16.rc.s_left  == 3 &&
		DR16.rc.s_right == 2) 
{
		if(DR16.rc.ch4_DW>300)//拨下到一半
		{
	start_use_break=1;	
		}
		else
		{
	start_use_break=0;	
		}
}
	
break_init();

	if(DR16.rc.s_left == 3)//左中挡位
	{
if(break_basic.STATE == 3)  //初始化全部完成
{	
	if(start_use_break==0)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;	
				stop_CH_OP_BC_BREAK=0;	
		stop_CH_OP_BC_BREAK_times=0;//强制停止刹车流程
				if(DR16.rc.ch3==0)//彻底松手了
	{
	break_FX=-1;
	}
	
	}
	
	if(start_use_break==1)//手动挡且拨下
	{
		if(stop_CH_OP_BC_BREAK_times==0)//time清零才能开始下一次刹车 一次刹车的周期由这个值控制
	{	stop_CH_OP_BC_BREAK_times=1;	
		stop_CH_OP_BC_BREAK=1;//开始失能
	
	}	
	
	
	
	if(stop_CH_OP_BC_BREAK_times!=0)
	{
	stop_CH_OP_BC_BREAK_times++;
	}


	if(stop_CH_OP_BC_BREAK_times==45)
	{
	break_FX=-break_FX;//使能前将目标速度取反
	}
			if(stop_CH_OP_BC_BREAK_times>50)//超过300ms,取消底盘失能,
	{
				stop_CH_OP_BC_BREAK=0;		
	}
	
				if(stop_CH_OP_BC_BREAK_times<50&&stop_CH_OP_BC_BREAK_times>0)//
				{
	if(	M3508s[3].realSpeed>1000)
	{
	M2006_targe_angle=	break_basic.BREAK_MAX+200;
	}
	if(	M3508s[3].realSpeed<-1000)
	{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;
	}//自动决定刹车方向					
				}
			if(stop_CH_OP_BC_BREAK_times>200)//超过500ms,刹车回中
	{
	M2006_targe_angle=	break_basic.BREAK_MID;
	stop_CH_OP_BC_BREAK=0;		
	}
		
	
	if(stop_CH_OP_BC_BREAK_times>2000)//过了500*3ms后才会开始下一次刹车
	{
	stop_CH_OP_BC_BREAK_times=0;
	}
	
	}
	/*单纯判度遥控器的值
	if(DR16.rc.ch4_DW>=-100&&DR16.rc.ch4_DW<=100)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;	
stop_CH_OP_BC_BREAK_times=0;
				stop_CH_OP_BC_BREAK=0;
		
	}
	else if(DR16.rc.ch4_DW<-100)//拨上
	{
	M2006_targe_angle=	break_basic.BREAK_MAX+200;
		stop_CH_OP_BC_BREAK=1;
		break_FX=1;
		stop_CH_OP_BC_BREAK_times++;
		if(stop_CH_OP_BC_BREAK_times>50)//超过300ms,取消底盘失能,
		{
		stop_CH_OP_BC_BREAK=0;
		}
		
		if(stop_CH_OP_BC_BREAK_times>250)//超过500ms,刹车回中
		{
		M2006_targe_angle=	break_basic.BREAK_MID;	
					stop_CH_OP_BC_BREAK=0;

		}
	}
	else if(DR16.rc.ch4_DW>100)//拨下
	{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;
				stop_CH_OP_BC_BREAK=1;
		stop_CH_OP_BC_BREAK_times++;
				break_FX=-1;

				if(stop_CH_OP_BC_BREAK_times>50)//超过300ms,取消底盘失能,
		{
					stop_CH_OP_BC_BREAK=0;		
		}
				if(stop_CH_OP_BC_BREAK_times>250)//超过500ms,刹车回中
		{
		M2006_targe_angle=	break_basic.BREAK_MID;
		stop_CH_OP_BC_BREAK=0;		
		}
	}
	if(DR16.rc.ch3==0)
	{
	break_FX=-1;
	}
	*/
}	
	}

		if(DR16.rc.s_left == 3&&DR16.rc.s_right == 4)
	{
if(break_basic.STATE == 3)  //初始化全部完成
{	
	if(DR16.rc.ch4_DW>=-100&&DR16.rc.ch4_DW<=200)
	{
	M2006_targe_angle=	break_basic.BREAK_MID;		
	}
	else if(DR16.rc.ch4_DW>=200)
	{
		if(M3508s[3].realSpeed>0)
		{
	M2006_targe_angle=	break_basic.BREAK_MIN-200;

		}
	}
	
}	
	}
	
M2006_targe_speed=P_PID_bate(&BREAK_ANGLE_pid,M2006_targe_angle,M3508s[BREAK_ID].totalAngle);//M2006_targe_speed应该大于0
	
	
send_to_break=P_PID_bate(&BREAK_SPEED_pid,M2006_targe_speed,M3508s[BREAK_ID].realSpeed);
	
	
//	M2006_targe_angle_text+=DR16.rc.ch1/10;
//	M2006_targe_speed=P_PID_bate(&BREAK_ANGLE_pid,M2006_targe_angle_text,M3508s[BREAK_ID].totalAngle);//M2006_targe_speed应该大于0

//send_to_break_text=P_PID_bate(&BREAK_SPEED_pid,M2006_targe_speed,M3508s[BREAK_ID].realSpeed);
//	
//	if(send_to_break_text>4000)//测试参数时避免撞机械限位，堵转烧电机  限制一下
//	{send_to_break_text=4000;}
//		if(send_to_break_text<-4000)
//	{send_to_break_text=-4000;}
	
}



