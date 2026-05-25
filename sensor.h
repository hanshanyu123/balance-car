#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"

void Sensor_Init(void);
void Sensor_JC(void);

uint8_t Sensor_GetBits(void);
int16_t Sensor_GetError(void);
uint8_t Sensor_IsValid(void);
uint8_t Sensor_GetHighLevelCount(void);

#endif
