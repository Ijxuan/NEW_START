#include "keyBoard_to_vjoy.h"
#include "DR16_RECIVE.h"
#include "User_math.h"

/*��Ҫ���һ�����ܣ�
�����̼�λӳ���һ������ҡ��

��Ҫ�ܹ�������  ��ʧ��

�ܹ�������ҡ�˵�ֵ����λ���л������߹ۿ�
*/
keyBoard_PRESS keyBoard_W; // W���Ľṹ��
keyBoard_PRESS keyBoard_S; // S���Ľṹ��
keyBoard_PRESS keyBoard_A; // A���Ľṹ��
keyBoard_PRESS keyBoard_D; // D���Ľṹ��

keyBoard_PRESS keyBoard_R; // A���Ľṹ��
keyBoard_PRESS keyBoard_F; // D���Ľṹ��

keyBoard_PRESS keyBoard_Q; // Q���Ľṹ��
keyBoard_PRESS keyBoard_E; // E���Ľṹ��
keyBoard_PRESS keyBoard_B; // B���Ľṹ��
keyBoard_PRESS keyBoard_G; // G���Ľṹ��

keyBoard_PRESS keyBoard_shift; // shift���Ľṹ��
keyBoard_PRESS keyBoard_ctrl; // ctrl���Ľṹ��

keyBoard_PRESS mouse_Left; // ���������Ľṹ��
keyBoard_PRESS mouse_Right; // ����Ҽ����Ľṹ��

vjoy vjoy_TEST;			// ����ҡ�˲���
Ramp_Struct vjoy_ch_WS; // ����תҡ�� б�º���
Ramp_Struct vjoy_ch_AD; // ����תҡ�� б�º���

float rate_add = 0.01;
float rate_decrease = 0.01;

void keyBoard_WASD()
{
	// DR16.keyBoard.press_W;
	if (DR16.keyBoard.press_W == 1)
	{
		keyBoard_W.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_W.Press_TIMES = 0;
		keyBoard_W.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_W.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_W.Press_static = Click_Press; // ����

		if (keyBoard_W.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_W.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_W.Press_static_last_time == Click_Press &&
		keyBoard_W.Press_static_last_time == Click_Press)
	{
		keyBoard_W.Click_Press_wait_use = 1; // ������������������һ��ʹ�ü���һ
	}
	//////////

	if (DR16.keyBoard.press_S == 1)
	{
		keyBoard_S.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_S.Press_TIMES = 0;
		keyBoard_S.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_S.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_S.Press_static = Click_Press; // ����

		if (keyBoard_S.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_S.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_S.Press_static_last_time == Click_Press &&
		keyBoard_S.Press_static == No_Press)
	{
		keyBoard_S.Click_Press_wait_use = 1; // ������������������һ��ʹ�ü���һ
	}
	//////////

	if (DR16.keyBoard.press_A == 1)
	{
		keyBoard_A.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_A.Press_TIMES = 0;
		keyBoard_A.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_A.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_A.Press_static = Click_Press; // ����

		if (keyBoard_A.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_A.Press_static = Long_Press; // ����
		}
	}
	//////////

	if (DR16.keyBoard.press_D == 1)
	{
		keyBoard_D.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_D.Press_TIMES = 0;
		keyBoard_D.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_D.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_D.Press_static = Click_Press; // ����

		if (keyBoard_D.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_D.Press_static = Long_Press; // ����
		}
	}
	//////////

	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	if (keyBoard_W.Press_static == No_Press && keyBoard_S.Press_static == No_Press) // ws���ɿ�
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 0.05; // ���ٹ��� 2��
	}

	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static != Long_Press) // ��W����
	{
		vjoy_ch_WS.Target_Value = 100;
		vjoy_ch_WS.Rate = 0.02;
	}
	else if (keyBoard_W.Press_static != Long_Press && keyBoard_S.Press_static == Long_Press) // ��S����
	{
		vjoy_ch_WS.Target_Value = -100;
		vjoy_ch_WS.Rate = 0.02;
	}
	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static == Long_Press) // WS˫���� ���� ����
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

	if (keyBoard_A.Press_static == No_Press && keyBoard_D.Press_static == No_Press) // AD���ɿ�
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 0.5; // ���ٹ��� 2��
	}

	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static != Long_Press) // ��A����
	{
		vjoy_ch_AD.Target_Value = -100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static != Long_Press && keyBoard_D.Press_static == Long_Press) // ��D����
	{
		vjoy_ch_AD.Target_Value = 100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static == Long_Press) // AD˫���� ���� ����
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
		keyBoard_R.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_R.Press_TIMES = 0;
		keyBoard_R.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_R.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_R.Press_static = Click_Press; // ����

//		if (keyBoard_R.Press_TIMES > TIME_KeyMouse_LongPress)
//		{
//			keyBoard_R.Press_static = Long_Press; // ����
//		}
	}

	if (keyBoard_R.Press_static_last_time == Click_Press &&
		keyBoard_R.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_R.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_R.Press_static_last_time=	keyBoard_R.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_F == 1)
	{
		keyBoard_F.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_F.Press_TIMES = 0;
		keyBoard_F.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_F.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_F.Press_static = Click_Press; // ����

//		if (keyBoard_F.Press_TIMES > TIME_KeyMouse_LongPress)
//		{
//			keyBoard_F.Press_static = Long_Press; // ����
//		}
	}

	if (keyBoard_F.Press_static_last_time == Click_Press &&
		keyBoard_F.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_F.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_F.Press_static_last_time=	keyBoard_F.Press_static;
}

void keyBoard_QE()
{

	if (DR16.keyBoard.press_Q == 1)
	{
		keyBoard_Q.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_Q.Press_TIMES = 0;
		keyBoard_Q.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_Q.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_Q.Press_static = Click_Press; // ����

		if (keyBoard_Q.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_Q.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_Q.Press_static_last_time == Click_Press &&
		keyBoard_Q.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_Q.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_Q.Press_static_last_time=	keyBoard_Q.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_E == 1)
	{
		keyBoard_E.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_E.Press_TIMES = 0;
		keyBoard_E.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_E.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_E.Press_static = Click_Press; // ����

		if (keyBoard_E.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_E.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_E.Press_static_last_time == Click_Press &&
		keyBoard_E.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_E.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_E.Press_static_last_time=	keyBoard_E.Press_static;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_B == 1)
	{
		keyBoard_B.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_B.Press_TIMES = 0;
		keyBoard_B.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_B.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_B.Press_static = Click_Press; // ����

		if (keyBoard_B.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_B.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_B.Press_static_last_time == Click_Press &&
		keyBoard_B.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_B.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_B.Press_static_last_time=	keyBoard_B.Press_static;
	
	///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_G == 1)
	{
		keyBoard_G.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_G.Press_TIMES = 0;
		keyBoard_G.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_G.Press_TIMES > 1)//�����ж�
	{
		keyBoard_G.Press_static = Click_Press; // ����

		if (keyBoard_G.Press_TIMES > 20)
		{
			keyBoard_G.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_G.Press_static_last_time == Click_Press &&
		keyBoard_G.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_G.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_G.Press_static_last_time=	keyBoard_G.Press_static;	
}

void keyBoard_shift_ctrl()
{

	if (DR16.keyBoard.press_Ctrl == 1)
	{
		keyBoard_ctrl.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_ctrl.Press_TIMES = 0;
		keyBoard_ctrl.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_ctrl.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_ctrl.Press_static = Click_Press; // ����

		if (keyBoard_ctrl.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_ctrl.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_ctrl.Press_static_last_time == Click_Press &&
		keyBoard_ctrl.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_ctrl.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_ctrl.Press_static_last_time=	keyBoard_ctrl.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.keyBoard.press_Shift == 1)
	{
		keyBoard_shift.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_shift.Press_TIMES = 0;
		keyBoard_shift.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_shift.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_shift.Press_static = Click_Press; // ����

		if (keyBoard_shift.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_shift.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_shift.Press_static_last_time == Click_Press &&
		keyBoard_shift.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		keyBoard_shift.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		keyBoard_shift.Press_static_last_time=	keyBoard_shift.Press_static;
}

void mouse_Left_Right()
{

	if (DR16.mouse.keyLeft == 1)
	{
		mouse_Left.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		mouse_Left.Press_TIMES = 0;
		mouse_Left.Press_static = No_Press; // �ɿ�
	}
	if (mouse_Left.Press_TIMES > TIME_KeyMouse_Press)
	{
		mouse_Left.Press_static = Click_Press; // ����

		if (mouse_Left.Press_TIMES > 100)
		{
			mouse_Left.Press_static = Long_Press; // ����
		}
	}

	if (mouse_Left.Press_static_last_time == No_Press &&
		mouse_Left.Press_static == Click_Press)//��һ�̻����ɿ�����һ�̵�������һ�ε�������ǳ�������
	{
		mouse_Left.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		mouse_Left.Press_static_last_time=	mouse_Left.Press_static;
///////////////////////////////////////////////////////////////////////////////////////////
	if (DR16.mouse.keyRight == 1)
	{
		mouse_Right.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		mouse_Right.Press_TIMES = 0;
		mouse_Right.Press_static = No_Press; // �ɿ�
	}
	if (mouse_Right.Press_TIMES > TIME_KeyMouse_Press)
	{
		mouse_Right.Press_static = Click_Press; // ����

		if (mouse_Right.Press_TIMES > 20)
		{
			mouse_Right.Press_static = Long_Press; // ����
		}
	}

	if (mouse_Right.Press_static_last_time == Click_Press &&
		mouse_Right.Press_static == No_Press)//��һ�̻��ǵ�������һ���ɿ�����һ�ε�������ǳ�������
	{
		mouse_Right.Click_Press_wait_use += 1; // ������������������һ��ʹ�ü���һ
	}
		mouse_Right.Press_static_last_time=	mouse_Right.Press_static;
}

