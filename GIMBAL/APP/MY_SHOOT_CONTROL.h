#ifndef SHOOT_CAN_H
#define SHOOT_CAN_H
#include "main.h"

void shoot_control(void);
void driver_plate_control(void);
extern bool Driver_arrive;
extern int if_Driver_arrive_angle;
extern long M2006_targe_angle;

#endif

