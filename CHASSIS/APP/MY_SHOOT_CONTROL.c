#include "MY_SHOOT_CONTROL.h"
#include "main.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "M3508.h"

void shoot_control(void)
{
	
	//send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);

	if(DR16.rc.s_left==3)//遥控器控制  左中
					{
//SHOOT_L=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//遥控器给速度目标值 二选一		
				
if(DR16.rc.ch3<-600)
	SHOOT_L=6700;
else if(DR16.rc.ch3>600)
			SHOOT_L=700;
else			
			SHOOT_L=0;

SHOOT_R=SHOOT_L;		
		if(	SHOOT_L>0)
		SHOOT_L=-SHOOT_L;//左摩擦轮速度目标值应该是负值
		if(	SHOOT_R<0)
		SHOOT_R=-SHOOT_R;//右摩擦轮速度目标值应该是正值
send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_L,M3508s[0].realSpeed);

send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_R,M3508s[1].realSpeed);
	
					}
	else
	{
		send_to_SHOOT_R=0;
		send_to_SHOOT_L=0;
	}	
}

void driver_plate_control(void)
{
	if(DR16.rc.s_left==3||DR16.rc.s_left==1)//遥控器控制  左中或左上

//driver_targe_speed=(DR16.rc.ch1*1.0/660.0)*(-1)*10000;//遥控器给速度目标值 二选一		
		if(	driver_targe_speed<0)
		driver_targe_speed=-driver_targe_speed;//右摩擦轮速度目标值应该是正值
send_to_driver=I_PID_Regulation(&Driver_I_PID,driver_targe_speed,M3508s[2].realSpeed);





}








