#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include "main.h"
#include <stdint.h>

extern char BlueSerial_RxPacket[100];
extern uint8_t BlueSerial_RxFlag;

void BlueSerial_Init(void);
void BlueSerial_Task(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);
void BlueSerial_SendPitchTask(float pitch);
uint8_t BlueSerial_IsTaskControlEnabled(void);
uint8_t BlueSerial_IsBalanceOnlyActive(void);
uint8_t BlueSerial_GetActiveTask(void);
float BlueSerial_GetTargetSpeed(void);
float BlueSerial_GetTargetTurn(void);

#endif
