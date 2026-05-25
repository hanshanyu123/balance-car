#ifndef __TASK4_H
#define __TASK4_H

#include "main.h"

typedef enum {
    TASK4_STATE_IDLE = 0,
    TASK4_STATE_READY,
    TASK4_STATE_TURN_AC,
    TASK4_STATE_STRAIGHT_AC,
    TASK4_STATE_LINE_CB,
    TASK4_STATE_TURN_BD,
    TASK4_STATE_STRAIGHT_BD,
    TASK4_STATE_LINE_DA,
    TASK4_STATE_FINISHED,
    TASK4_STATE_FAULT
} Task4_State;

typedef enum {
    TASK4_POINT_NONE = 0,
    TASK4_POINT_C,
    TASK4_POINT_B,
    TASK4_POINT_D,
    TASK4_POINT_A
} Task4_Point;

void Task4_Init(void);
void Task4_Task(void);
void Task4_RequestStart(uint8_t loop_target);
void Task4_RequestFault(void);
void Task4_Reset(void);
void Task4_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid);
Task4_State Task4_GetState(void);
uint8_t Task4_IsStarted(void);
uint8_t Task4_GetLoopCount(void);
uint8_t Task4_GetLoopTarget(void);
float Task4_GetTargetSpeed(void);
float Task4_GetTargetTurn(void);
Task4_Point Task4_GetPointEvent(void);
void Task4_ClearPointEvent(void);
uint8_t Task4_GetSensorBits(void);
int16_t Task4_GetSensorError(void);
uint8_t Task4_GetSensorValid(void);

#endif
