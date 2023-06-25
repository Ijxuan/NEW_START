#include "keyBoard_to_vjoy.h"
#include "DR16_RECIVE.h"
#include "User_math.h"

/*主要完成一个功能：
将键盘键位映射成一个虚拟摇杆

需要能够缓启动  缓失能

能够将虚拟摇杆的值在上位机中绘制曲线观看
*/
keyBoard_PRESS keyBoard_W; // W键的结构体
keyBoard_PRESS keyBoard_S; // S键的结构体
keyBoard_PRESS keyBoard_A; // A键的结构体
keyBoard_PRESS keyBoard_D; // D键的结构体

keyBoard_PRESS keyBoard_R; // A键的结构体
keyBoard_PRESS keyBoard_F; // D键的结构体

keyBoard_PRESS keyBoard_Q; // Q键的结构体
keyBoard_PRESS keyBoard_E; // E键的结构体
keyBoard_PRESS keyBoard_B; // B键的结构体
keyBoard_PRESS keyBoard_G; // G键的结构体

keyBoard_PRESS keyBoard_shift; // shift键的结构体
keyBoard_PRESS keyBoard_ctrl; // ctrl键的结构体

keyBoard_PRESS mouse_Left; // 鼠标左键键的结构体
keyBoard_PRESS mouse_Right; // 鼠标右键键的结构体

vjoy vjoy_TEST;			// 虚拟摇杆测试
Ramp_Struct vjoy_ch_WS; // 键盘转摇杆 斜坡函数
Ramp_Struct vjoy_ch_AD; // 键盘转摇杆 斜坡函数

float rate_add = 0.01;
float rate_decrease = 0.01;

void keyBoard_WASD()
{
	// DR16.keyBoard.press_W;
	if (DR16.keyBoard.press_W == 1)
	{
		keyBoard_W.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_W.Press_TIMES = 0;
		keyBoard_W.Press_static = No_Press; // 松开
	}
	if (keyBoard_W.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_W.Press_static = Click_Press; // 单击

		if (keyBoard_W.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_W.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_W.Press_static_last_time == Click_Press &&
		keyBoard_W.Press_static_last_time == Click_Press)
	{
		keyBoard_W.Click_Press_wait_use = 1; // 单击次数，单击增加一，使用减少一
	}
	//////////

	if (DR16.keyBoard.press_S == 1)
	{
		keyBoard_S.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_S.Press_TIMES = 0;
		keyBoard_S.Press_static = No_Press; // 松开
	}
	if (keyBoard_S.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_S.Press_static = Click_Press; // 单击

		if (keyBoard_S.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_S.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_S.Press_static_last_time == Click_Press &&
		keyBoard_S.Press_static == No_Press)
	{
		keyBoard_S.Click_Press_wait_use = 1; // 单击次数，单击增加一，使用减少一
	}
	//////////

	if (DR16.keyBoard.press_A == 1)
	{
		keyBoard_A.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_A.Press_TIMES = 0;
		keyBoard_A.Press_static = No_Press; // 松开
	}
	if (keyBoard_A.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_A.Press_static = Click_Press; // 单击

		if (keyBoard_A.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_A.Press_static = Long_Press; // 长按
		}
	}
	//////////

	if (DR16.keyBoard.press_D == 1)
	{
		keyBoard_D.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_D.Press_TIMES = 0;
		keyBoard_D.Press_static = No_Press; // 松开
	}
	if (keyBoard_D.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_D.Press_static = Click_Press; // 单击

		if (keyBoard_D.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_D.Press_static = Long_Press; // 长按
		}
	}
	//////////

	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	if (keyBoard_W.Press_static == No_Press && keyBoard_S.Press_static == No_Press) // ws都松开
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 0.05; // 快速归零 2秒
	}

	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static != Long_Press) // 仅W长按
	{
		vjoy_ch_WS.Target_Value = 100;
		vjoy_ch_WS.Rate = 0.02;
	}
	else if (keyBoard_W.Press_static != Long_Press && keyBoard_S.Press_static == Long_Press) // 仅S长按
	{
		vjoy_ch_WS.Target_Value = -100;
		vjoy_ch_WS.Rate = 0.02;
	}
	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static == Long_Press) // WS双长按 算误触 归零
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 1;
	}

	vjoy_ch_WS.Absolute_Max = 101;
	vjoy_ch_WS.Current_Value = vjoy_TEST.ch_WS;

	vjoy_TEST.ch_WS = Ramp_Function(&vjoy_ch_WS);

	keyBoard_W.Press_static_last_time = keyBoard_W.Press_static;
	keyBoard_S.Press_static_last_time = keyBoard_S.Press_static;

	/////////////////////////////////////////////////////////////////

	if (keyBoard_A.Press_static == No_Press && keyBoard_D.Press_static == No_Press) // AD都松开
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 0.5; // 快速归零 2秒
	}

	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static != Long_Press) // 仅A长按
	{
		vjoy_ch_AD.Target_Value = -100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static != Long_Press && keyBoard_D.Press_static == Long_Press) // 仅D长按
	{
		vjoy_ch_AD.Target_Value = 100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static == Long_Press) // AD双长按 算误触 归零
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 1;
	}

	vjoy_ch_AD.Absolute_Max = 101;
	vjoy_ch_AD.Current_Value = vjoy_TEST.ch_AD;

	vjoy_TEST.ch_AD = Ramp_Function(&vjoy_ch_AD);
}



void keyBoard_RF()
{

	if (DR16.keyBoard.press_R == 1)
	{
		keyBoard_R.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_R.Press_TIMES = 0;
		keyBoard_R.Press_static = No_Press; // 松开
	}
	if (keyBoard_R.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_R.Press_static = Click_Press; // 单击

//		if (keyBoard_R.Press_TIMES > TIME_KeyMouse_LongPress)
//		{
//			keyBoard_R.Press_static = Long_Press; // 长按
//		}
	}

	if (keyBoard_R.Press_static_last_time == Click_Press &&
		keyBoard_R.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_R.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_R.Press_static_last_time=	keyBoard_R.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_F == 1)
	{
		keyBoard_F.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_F.Press_TIMES = 0;
		keyBoard_F.Press_static = No_Press; // 松开
	}
	if (keyBoard_F.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_F.Press_static = Click_Press; // 单击

//		if (keyBoard_F.Press_TIMES > TIME_KeyMouse_LongPress)
//		{
//			keyBoard_F.Press_static = Long_Press; // 长按
//		}
	}

	if (keyBoard_F.Press_static_last_time == Click_Press &&
		keyBoard_F.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_F.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_F.Press_static_last_time=	keyBoard_F.Press_static;
}

void keyBoard_QE()
{

	if (DR16.keyBoard.press_Q == 1)
	{
		keyBoard_Q.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_Q.Press_TIMES = 0;
		keyBoard_Q.Press_static = No_Press; // 松开
	}
	if (keyBoard_Q.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_Q.Press_static = Click_Press; // 单击

		if (keyBoard_Q.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_Q.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_Q.Press_static_last_time == Click_Press &&
		keyBoard_Q.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_Q.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_Q.Press_static_last_time=	keyBoard_Q.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_E == 1)
	{
		keyBoard_E.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_E.Press_TIMES = 0;
		keyBoard_E.Press_static = No_Press; // 松开
	}
	if (keyBoard_E.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_E.Press_static = Click_Press; // 单击

		if (keyBoard_E.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_E.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_E.Press_static_last_time == Click_Press &&
		keyBoard_E.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_E.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_E.Press_static_last_time=	keyBoard_E.Press_static;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_B == 1)
	{
		keyBoard_B.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_B.Press_TIMES = 0;
		keyBoard_B.Press_static = No_Press; // 松开
	}
	if (keyBoard_B.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_B.Press_static = Click_Press; // 单击

		if (keyBoard_B.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_B.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_B.Press_static_last_time == Click_Press &&
		keyBoard_B.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_B.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_B.Press_static_last_time=	keyBoard_B.Press_static;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_G == 1)
	{
		keyBoard_G.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_G.Press_TIMES = 0;
		keyBoard_G.Press_static = No_Press; // 松开
	}
	if (keyBoard_G.Press_TIMES > 1)//特殊判断
	{
		keyBoard_G.Press_static = Click_Press; // 单击

		if (keyBoard_G.Press_TIMES > 20)
		{
			keyBoard_G.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_G.Press_static_last_time == Click_Press &&
		keyBoard_G.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_G.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_G.Press_static_last_time=	keyBoard_G.Press_static;	
}

void keyBoard_shift_ctrl()
{

	if (DR16.keyBoard.press_Ctrl == 1)
	{
		keyBoard_ctrl.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_ctrl.Press_TIMES = 0;
		keyBoard_ctrl.Press_static = No_Press; // 松开
	}
	if (keyBoard_ctrl.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_ctrl.Press_static = Click_Press; // 单击

		if (keyBoard_ctrl.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_ctrl.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_ctrl.Press_static_last_time == Click_Press &&
		keyBoard_ctrl.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_ctrl.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_ctrl.Press_static_last_time=	keyBoard_ctrl.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_Shift == 1)
	{
		keyBoard_shift.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_shift.Press_TIMES = 0;
		keyBoard_shift.Press_static = No_Press; // 松开
	}
	if (keyBoard_shift.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_shift.Press_static = Click_Press; // 单击

		if (keyBoard_shift.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_shift.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_shift.Press_static_last_time == Click_Press &&
		keyBoard_shift.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		keyBoard_shift.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		keyBoard_shift.Press_static_last_time=	keyBoard_shift.Press_static;
}

void mouse_Left_Right()
{

	if (DR16.mouse.keyLeft == 1)
	{
		mouse_Left.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		mouse_Left.Press_TIMES = 0;
		mouse_Left.Press_static = No_Press; // 松开
	}
	if (mouse_Left.Press_TIMES > TIME_KeyMouse_Press)
	{
		mouse_Left.Press_static = Click_Press; // 单击

		if (mouse_Left.Press_TIMES > 100)
		{
			mouse_Left.Press_static = Long_Press; // 长按
		}
	}

	if (mouse_Left.Press_static_last_time == No_Press &&
		mouse_Left.Press_static == Click_Press)//上一刻还是松开，这一刻单击，算一次点击，但是长按不算
	{
		mouse_Left.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		mouse_Left.Press_static_last_time=	mouse_Left.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.mouse.keyRight == 1)
	{
		mouse_Right.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		mouse_Right.Press_TIMES = 0;
		mouse_Right.Press_static = No_Press; // 松开
	}
	if (mouse_Right.Press_TIMES > TIME_KeyMouse_Press)
	{
		mouse_Right.Press_static = Click_Press; // 单击

		if (mouse_Right.Press_TIMES > 20)
		{
			mouse_Right.Press_static = Long_Press; // 长按
		}
	}

	if (mouse_Right.Press_static_last_time == Click_Press &&
		mouse_Right.Press_static == No_Press)//上一刻还是单击，这一刻松开，算一次点击，但是长按不算
	{
		mouse_Right.Click_Press_wait_use += 1; // 单击次数，单击增加一，使用减少一
	}
		mouse_Right.Press_static_last_time=	mouse_Right.Press_static;
}

