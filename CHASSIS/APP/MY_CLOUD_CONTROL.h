#ifndef CLOUD_CAN_H
#define CLOUD_CAN_H

#include "DR16_RECIVE.h"

#include "GM6020.h"
#include "my_positionPID_bate.h"
#include "DJI_C_IMU.h"

#include "Vision.h"
#include "User_math.h"

extern Ramp_Struct *EM_Ramp;


void cloud_control(void);
void YAW_PID(void);

void PITCH_PID(void);

#endif
//MY_CLOUD_COUNTROUL

