#ifndef __TASK2_H
#define __TASK2_H

#include "main.h"

typedef enum {
    TASK2_STATE_IDLE = 0,
    TASK2_STATE_READY,
    TASK2_STATE_RUNNING,
    TASK2_STATE_STOPPING,
    TASK2_STATE_FINISHED,
    TASK2_STATE_FAULT
} Task2_State;

void Task2_Init(void);
void Task2_Task(void);
void Task2_RequestStart(void);
void Task2_RequestFault(void);
void Task2_Reset(void);
void Task2_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid);
Task2_State Task2_GetState(void);
uint8_t Task2_IsStarted(void);
uint8_t Task2_IsStopRequested(void);
void Task2_ClearStopRequest(void);
uint32_t Task2_GetStateTick(void);
float Task2_GetTargetSpeed(void);
uint8_t Task2_GetSensorBits(void);
int16_t Task2_GetSensorError(void);
uint8_t Task2_GetSensorValid(void);

#endif
