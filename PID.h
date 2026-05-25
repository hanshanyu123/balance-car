#ifndef __PID_H
#define __PID_H

#include "main.h"

#define BALANCE_TARGET 1.0f

void PID_Init(void);
void PID_Reset(void);
void Speed_Reset(void);
void Turn_Reset(void);

void Balance_PID(float pitch, float gyro_y, float target);
void Speed_PI_Control(float left_speed, float right_speed);
void Turn_Control(float left_speed, float right_speed);

int Get_Balance_Output(void);
float Get_Speed_Angle_Offset(void);
int Get_Turn_Output(void);
int Get_Total_Output(void);

void Set_Balance_Kp(float kp);
void Set_Balance_Ki(float ki);
void Set_Balance_Kd(float kd);
void Set_Speed_Target(float target);
void Set_Speed_Kp(float kp);
void Set_Speed_Ki(float ki);
void Set_Speed_Kd(float kd);
void Set_Turn_Target(float target);
void Set_Turn_Kp(float kp);
void Set_Turn_Ki(float ki);
void Set_Turn_Kd(float kd);

float Get_Balance_Kp(void);
float Get_Balance_Ki(void);
float Get_Balance_Kd(void);
float Get_Speed_Target(void);
float Get_Speed_Kp(void);
float Get_Speed_Ki(void);
float Get_Speed_Kd(void);
float Get_Turn_Target(void);
float Get_Turn_Kp(void);
float Get_Turn_Ki(void);
float Get_Turn_Kd(void);

float Get_Balance_P_Term(void);
float Get_Balance_I_Term(void);
float Get_Balance_D_Term(void);

#endif
