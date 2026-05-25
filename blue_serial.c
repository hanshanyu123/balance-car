#include "blue_serial.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#define BLUE_SERIAL_TASK_NONE      0U
#define BLUE_SERIAL_TASK_MIN       1U
#define BLUE_SERIAL_TASK_MAX       5U

char BlueSerial_RxPacket[100];
uint8_t BlueSerial_RxFlag;

static uint8_t BlueSerial_RxByte;
static uint8_t BlueSerial_RxState = 0U;
static uint8_t BlueSerial_RxIndex = 0U;
static float BlueSerial_TargetSpeed = 0.0f;
static float BlueSerial_TargetTurn = 0.0f;
static uint8_t BlueSerial_ActiveTask = BLUE_SERIAL_TASK_NONE;
static uint32_t BlueSerial_PitchSendTick = 0U;

static uint8_t BlueSerial_TaskIsBusy(void)
{
    if (BlueSerial_ActiveTask != BLUE_SERIAL_TASK_NONE) return 1U;

    if ((Task2_GetState() != TASK2_STATE_IDLE) &&
        (Task2_GetState() != TASK2_STATE_FINISHED) &&
        (Task2_GetState() != TASK2_STATE_FAULT)) return 1U;

    if ((Task3_GetState() != TASK3_STATE_IDLE) &&
        (Task3_GetState() != TASK3_STATE_FINISHED) &&
        (Task3_GetState() != TASK3_STATE_FAULT)) return 1U;

    if ((Task4_GetState() != TASK4_STATE_IDLE) &&
        (Task4_GetState() != TASK4_STATE_FINISHED) &&
        (Task4_GetState() != TASK4_STATE_FAULT)) return 1U;

    return 0U;
}

static void BlueSerial_UpdateActiveTask(void)
{
    switch (BlueSerial_ActiveTask) {
        case 2U:
            if (Task2_GetState() == TASK2_STATE_FINISHED) BlueSerial_ActiveTask = 1U;
            else if (Task2_GetState() == TASK2_STATE_FAULT) BlueSerial_ActiveTask = BLUE_SERIAL_TASK_NONE;
            break;
        case 3U:
            if (Task3_GetState() == TASK3_STATE_FINISHED) BlueSerial_ActiveTask = 1U;
            else if (Task3_GetState() == TASK3_STATE_FAULT) BlueSerial_ActiveTask = BLUE_SERIAL_TASK_NONE;
            break;
        case 4U:
        case 5U:
            if (Task4_GetState() == TASK4_STATE_FINISHED) BlueSerial_ActiveTask = 1U;
            else if (Task4_GetState() == TASK4_STATE_FAULT) BlueSerial_ActiveTask = BLUE_SERIAL_TASK_NONE;
            break;
        default: break;
    }
}

static void BlueSerial_StartTask(uint8_t task)
{
    if ((task < BLUE_SERIAL_TASK_MIN) || (task > BLUE_SERIAL_TASK_MAX)) {
        BlueSerial_Printf("TASK,ERR,%u\r\n", task);
        return;
    }

    if (BlueSerial_TaskIsBusy() != 0U) {
        BlueSerial_Printf("TASK,BUSY,%u\r\n", BlueSerial_ActiveTask);
        return;
    }

    switch (task) {
        case 1U: BlueSerial_ActiveTask = 1U; BlueSerial_TargetSpeed = 0.0f; BlueSerial_TargetTurn = 0.0f; break;
        case 2U: Task2_RequestStart(); BlueSerial_ActiveTask = 2U; break;
        case 3U: Task3_RequestStart(); BlueSerial_ActiveTask = 3U; break;
        case 4U: Task4_RequestStart(1U); BlueSerial_ActiveTask = 4U; break;
        case 5U: Task4_RequestStart(4U); BlueSerial_ActiveTask = 5U; break;
        default: break;
    }

    BlueSerial_Printf("TASK,OK,%u\r\n", task);
}

void BlueSerial_Init(void)
{
    BlueSerial_RxPacket[0] = '\0';
    BlueSerial_RxFlag = 0U;
    BlueSerial_RxByte = 0U;
    BlueSerial_RxState = 0U;
    BlueSerial_RxIndex = 0U;
    BlueSerial_TargetSpeed = 0.0f;
    BlueSerial_TargetTurn = 0.0f;
    BlueSerial_ActiveTask = BLUE_SERIAL_TASK_NONE;
    BlueSerial_PitchSendTick = system_ms;

    /* Enable UART1 RX interrupt for Bluetooth */
    DL_UART_Main_enableInterrupt(UART_1_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

void BlueSerial_SendByte(uint8_t Byte)
{
    DL_UART_Main_transmitData(UART_1_INST, Byte);
    while (!(DL_UART_Main_getRawInterruptStatus(UART_1_INST, DL_UART_MAIN_INTERRUPT_TX)));
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) BlueSerial_SendByte(Array[i]);
}

void BlueSerial_SendString(char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++) BlueSerial_SendByte((uint8_t)String[i]);
}

static uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1U;
    while (Y--) Result *= X;
    return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        BlueSerial_SendByte((uint8_t)(Number / BlueSerial_Pow(10U, Length - i - 1U) % 10U + '0'));
}

void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    BlueSerial_SendString(String);
}

void BlueSerial_SendPitchTask(float pitch)
{
    int32_t pitch_x100;
    int32_t pitch_abs;

    if ((system_ms - BlueSerial_PitchSendTick) < 100U) return;
    BlueSerial_PitchSendTick = system_ms;

    if (pitch >= 0.0f) pitch_x100 = (int32_t)(pitch * 100.0f + 0.5f);
    else              pitch_x100 = (int32_t)(pitch * 100.0f - 0.5f);

    if (pitch_x100 < 0) {
        pitch_abs = -pitch_x100;
        BlueSerial_Printf("P,-%ld.%02ld\r\n", pitch_abs / 100L, pitch_abs % 100L);
    } else {
        BlueSerial_Printf("P,%ld.%02ld\r\n", pitch_x100 / 100L, pitch_x100 % 100L);
    }
}

void BlueSerial_Task(void)
{
    char Buffer[100];
    char *Tag;

    if (BlueSerial_RxFlag == 0U) {
        BlueSerial_UpdateActiveTask();
        return;
    }

    BlueSerial_UpdateActiveTask();
    strcpy(Buffer, BlueSerial_RxPacket);
    Tag = strtok(Buffer, ",");

    if (Tag != NULL) {
        if (strcmp(Tag, "task") == 0) {
            char *TaskId = strtok(NULL, ",");
            if (TaskId != NULL) BlueSerial_StartTask((uint8_t)atoi(TaskId));
            else BlueSerial_Printf("TASK,ERR\r\n");
        } else if (strcmp(Tag, "slider") == 0) {
            char *Name = strtok(NULL, ",");
            char *Value = strtok(NULL, ",");
            if ((Name != NULL) && (Value != NULL) &&
                ((strcmp(Name, "task") == 0) || (strcmp(Name, "Task") == 0))) {
                BlueSerial_StartTask((uint8_t)atoi(Value));
            }
        } else if (strcmp(Tag, "joystick") == 0) {
            char *LH_Text = strtok(NULL, ",");
            char *LV_Text = strtok(NULL, ",");
            char *RH_Text = strtok(NULL, ",");
            char *RV_Text = strtok(NULL, ",");
            if ((LH_Text == NULL) || (LV_Text == NULL) || (RH_Text == NULL) || (RV_Text == NULL)) {
                BlueSerial_RxFlag = 0U; return;
            }
            BlueSerial_TargetSpeed = atoi(LV_Text) / 21.0f;
            BlueSerial_TargetTurn  = atoi(RH_Text) / 21.0f;
        }
    }

    BlueSerial_RxFlag = 0U;
}

uint8_t BlueSerial_IsTaskControlEnabled(void)
{
    BlueSerial_UpdateActiveTask();
    return (BlueSerial_ActiveTask != BLUE_SERIAL_TASK_NONE) ? 1U : 0U;
}

uint8_t BlueSerial_IsBalanceOnlyActive(void)
{
    return (BlueSerial_ActiveTask == 1U) ? 1U : 0U;
}

uint8_t BlueSerial_GetActiveTask(void)
{
    BlueSerial_UpdateActiveTask();
    return BlueSerial_ActiveTask;
}

float BlueSerial_GetTargetSpeed(void) { return BlueSerial_TargetSpeed; }
float BlueSerial_GetTargetTurn(void)  { return BlueSerial_TargetTurn; }

/* UART1 interrupt handler for Bluetooth RX */
void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            BlueSerial_RxByte = DL_UART_Main_receiveData(UART_1_INST);

            if (BlueSerial_RxState == 0U) {
                if ((BlueSerial_RxByte == '[') && (BlueSerial_RxFlag == 0U)) {
                    BlueSerial_RxState = 1U;
                    BlueSerial_RxIndex = 0U;
                }
            } else if (BlueSerial_RxState == 1U) {
                if (BlueSerial_RxByte == ']') {
                    BlueSerial_RxState = 0U;
                    BlueSerial_RxPacket[BlueSerial_RxIndex] = '\0';
                    BlueSerial_RxFlag = 1U;
                } else {
                    if (BlueSerial_RxIndex < (sizeof(BlueSerial_RxPacket) - 1U)) {
                        BlueSerial_RxPacket[BlueSerial_RxIndex] = (char)BlueSerial_RxByte;
                        BlueSerial_RxIndex++;
                    } else {
                        BlueSerial_RxState = 0U;
                        BlueSerial_RxIndex = 0U;
                    }
                }
            }
            break;
        default:
            break;
    }
}
