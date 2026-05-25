#ifndef __TASK3_H
#define __TASK3_H

#include "main.h"

typedef enum {
    TASK3_STATE_IDLE = 0,
    TASK3_STATE_READY,
    TASK3_STATE_STRAIGHT_AB,
    TASK3_STATE_LINE_BC,
    TASK3_STATE_STRAIGHT_CD,
    TASK3_STATE_LINE_DA,
    TASK3_STATE_FINISHED,
    TASK3_STATE_FAULT
} Task3_State;

typedef enum {
    TASK3_POINT_NONE = 0,
    TASK3_POINT_B,
    TASK3_POINT_C,
    TASK3_POINT_D,
    TASK3_POINT_A
} Task3_Point;

void Task3_Init(void);
void Task3_Task(void);
void Task3_RequestStart(void);
void Task3_RequestFault(void);
void Task3_Reset(void);
void Task3_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid);
Task3_State Task3_GetState(void);
uint8_t Task3_IsStarted(void);
float Task3_GetTargetSpeed(void);
float Task3_GetTargetTurn(void);
Task3_Point Task3_GetPointEvent(void);
void Task3_ClearPointEvent(void);
uint8_t Task3_GetSensorBits(void);
int16_t Task3_GetSensorError(void);
uint8_t Task3_GetSensorValid(void);

#endif
