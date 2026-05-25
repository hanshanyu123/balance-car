#include "main.h"
#include "tasks.h"
#include "task2.h"
#include "task3.h"
#include "task4.h"
#include "sensor.h"
#include "blue_serial.h"
#include "alert.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor.h"
#include "PID.h"
#include <math.h>

/*===========================================================================
 * System tick (1ms)
 *===========================================================================*/
volatile uint32_t system_ms = 0;

/* SysTick_Handler: increment system tick every 1ms */
void SysTick_Handler(void)
{
    system_ms++;
}

/* Busy-wait delay using system_ms */
void delay_ms(uint32_t ms)
{
    uint32_t start = system_ms;
    while ((system_ms - start) < ms) { }
}

/*===========================================================================
 * printf redirect to UART0 (debug backchannel, 115200)
 *===========================================================================*/
int fputc(int ch, FILE *f)
{
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t)ch);
    /* Wait for TX FIFO empty */
    while (!(DL_UART_Main_getRawInterruptStatus(UART_0_INST,
           DL_UART_MAIN_INTERRUPT_TX))) { }
    return ch;
}

/*===========================================================================
 * Safety state machine
 *===========================================================================*/
typedef enum {
    SAFETY_RUNNING = 0,
    SAFETY_FAULT
} Safety_State;

#define CONTROL_PERIOD_SEC        0.005f
#define FALL_ANGLE_LIMIT_DEG      40.0f
#define GYRO_ABS_LIMIT_DPS        MPU6050_GYRO_VALID_LIMIT_DPS
#define CONTROL_OVERRUN_LIMIT     3U

MPU6050_Data mpu_data;
Attitude_Data attitude;

volatile Safety_State safety_state = SAFETY_FAULT;
volatile uint8_t control_tick_count = 0;
static uint8_t speed_loop_counter = 0;
static uint8_t control_overrun_count = 0;

/*===========================================================================
 * Forward declarations
 *===========================================================================*/
void Control_Task(void);
static uint8_t Float_Is_Valid(float value);
static uint8_t IMU_Data_Is_Valid(const MPU6050_Data *data, float pitch, float gyro_y);
static void Safety_Stop(void);
static void Safety_Trip(void);
static uint8_t Task2_Control_Enable(void);
static uint8_t Task3_Control_Enable(void);
static uint8_t Task4_Control_Enable(void);

/*===========================================================================
 * Helper functions
 *===========================================================================*/
static uint8_t Float_Is_Valid(float value)
{
    return (value == value) && (value < 1.0e30f) && (value > -1.0e30f);
}

static uint8_t Task2_Control_Enable(void)
{
    Task2_State state = Task2_GetState();
    return ((state == TASK2_STATE_RUNNING) || (state == TASK2_STATE_STOPPING)) ? 1U : 0U;
}

static uint8_t Task3_Control_Enable(void)
{
    Task3_State state = Task3_GetState();
    return ((state == TASK3_STATE_STRAIGHT_AB) || (state == TASK3_STATE_LINE_BC) ||
            (state == TASK3_STATE_STRAIGHT_CD) || (state == TASK3_STATE_LINE_DA)) ? 1U : 0U;
}

static uint8_t Task4_Control_Enable(void)
{
    Task4_State state = Task4_GetState();
    return ((state == TASK4_STATE_TURN_AC) || (state == TASK4_STATE_STRAIGHT_AC) ||
            (state == TASK4_STATE_LINE_CB) || (state == TASK4_STATE_TURN_BD) ||
            (state == TASK4_STATE_STRAIGHT_BD) || (state == TASK4_STATE_LINE_DA)) ? 1U : 0U;
}

static uint8_t IMU_Data_Is_Valid(const MPU6050_Data *data, float pitch, float gyro_y)
{
    if(!Float_Is_Valid(pitch) || !Float_Is_Valid(gyro_y)) return 0;
    if(!Float_Is_Valid(data->ax) || !Float_Is_Valid(data->ay) || !Float_Is_Valid(data->az)) return 0;
    if(!Float_Is_Valid(data->gx) || !Float_Is_Valid(data->gy) || !Float_Is_Valid(data->gz)) return 0;
    if(fabsf(data->gx) > GYRO_ABS_LIMIT_DPS) return 0;
    if(fabsf(data->gy) > GYRO_ABS_LIMIT_DPS) return 0;
    if(fabsf(data->gz) > GYRO_ABS_LIMIT_DPS) return 0;
    return 1;
}

static void Safety_Stop(void)
{
    Set_Motor_Speed(0, 0);
    PID_Reset();
    speed_loop_counter = 0;
    (void)Read_Encoder(0);
    (void)Read_Encoder(1);
}

static void Safety_Trip(void)
{
    safety_state = SAFETY_FAULT;
    Task2_RequestFault();
    Task3_RequestFault();
    Task4_RequestFault();
    Safety_Stop();
}

/*===========================================================================
 * TIMA0 interrupt handler (5ms control tick)
 *===========================================================================*/
void TIMA0_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            if (control_tick_count < 255U) control_tick_count++;
            break;
        default:
            break;
    }
}

/*===========================================================================
 * Control_Task: called every 5ms from main loop
 *===========================================================================*/
void Control_Task(void)
{
    float gyro_y;
    float angle_error;
    float speed_offset;
    float target_with_offset;
    int balance_out;
    int turn_out;
    int left_out;
    int right_out;
    uint8_t task4_active, task3_active, task2_active;
    static float left_v = 0.0f;
    static float right_v = 0.0f;

    if (safety_state == SAFETY_FAULT) { Safety_Stop(); return; }

    if (BlueSerial_IsTaskControlEnabled() == 0U) { Safety_Stop(); return; }

    if (MPU6050_Read_Data(&mpu_data) != MPU6050_OK) { Safety_Trip(); return; }

    MPU6050_Calculate_Attitude(&mpu_data, &attitude, CONTROL_PERIOD_SEC);
    gyro_y = MPU6050_Get_Gyro_Pitch(&mpu_data);
    angle_error = attitude.pitch - BALANCE_TARGET;

    if (!IMU_Data_Is_Valid(&mpu_data, attitude.pitch, gyro_y)) { Safety_Stop(); return; }

    if (fabsf(angle_error) > FALL_ANGLE_LIMIT_DEG) { Safety_Trip(); return; }

    task4_active = Task4_Control_Enable();
    task3_active = Task3_Control_Enable();
    task2_active = Task2_Control_Enable();

    speed_loop_counter++;
    if (speed_loop_counter >= 2U) {
        speed_loop_counter = 0U;

        left_v = Read_Encoder(0) / 44.0f / 0.01f / 9.27666f;
        right_v = Read_Encoder(1) / 44.0f / 0.01f / 9.27666f;

        if (task4_active != 0U) {
            Set_Speed_Target(Task4_GetTargetSpeed());
            Set_Turn_Target(Task4_GetTargetTurn());
        } else if (task3_active != 0U) {
            Set_Speed_Target(Task3_GetTargetSpeed());
            Set_Turn_Target(Task3_GetTargetTurn());
        } else if (task2_active != 0U) {
            Set_Speed_Target(Task2_GetTargetSpeed());
            Set_Turn_Target(0.0f);
        } else {
            Set_Speed_Target(0.0f);
            Set_Turn_Target(0.0f);
        }

        Speed_PI_Control(left_v, right_v);
        Turn_Control(left_v, right_v);
    }

    speed_offset = Get_Speed_Angle_Offset();
    target_with_offset = BALANCE_TARGET + speed_offset;

    Balance_PID(attitude.pitch, gyro_y, target_with_offset);

    balance_out = Get_Balance_Output();
    if ((task4_active != 0U) || (task3_active != 0U) || (task2_active != 0U)) {
        turn_out = Get_Turn_Output();
    } else {
        turn_out = 0;
    }
    left_out = balance_out + turn_out / 2;
    right_out = balance_out - turn_out / 2;
    Set_Motor_Speed(left_out, right_out);
}

/*===========================================================================
 * Error Handler
 *===========================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    /* SysConfig-generated init: power, GPIO, SYSCTL, PWM, QEI, TIMER, UARTs */
    SYSCFG_DL_init();

    /* Configure SysTick for 1ms interrupt (32MHz / 1000 = 32000) */
    SysTick_Config(CPUCLK_FREQ / 1000U);

    /* Module inits */
    Motor_Init();
    Encoder_Init();
    PID_Init();
    Tasks_Init();
    BlueSerial_Init();
    Alert_Init();

    if (MPU6050_Init() != MPU6050_OK) {
        safety_state = SAFETY_FAULT;
        Safety_Stop();
    } else {
        safety_state = SAFETY_RUNNING;
    }

    if (safety_state == SAFETY_FAULT) {
        printf("MPU6050 Init Failed!\n");
    } else {
        printf("Balance Car Init OK!\n");
    }

    __disable_irq();
    control_tick_count = 0;
    control_overrun_count = 0;
    __enable_irq();

    /* Enable TIMA0 interrupt for 5ms control tick */
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* Main loop */
    while (1) {
        Alert_Task();

        if (control_tick_count > 0) {
            uint8_t pending_ticks;

            __disable_irq();
            pending_ticks = control_tick_count;
            control_tick_count = 0;
            __enable_irq();

            if (pending_ticks > 1U) {
                if (control_overrun_count < CONTROL_OVERRUN_LIMIT) {
                    control_overrun_count++;
                }
                if (control_overrun_count >= CONTROL_OVERRUN_LIMIT) {
                    Safety_Trip();
                    continue;
                }
            } else {
                control_overrun_count = 0;
            }

            Control_Task();
            continue;
        }

        /* Idle tasks */
        BlueSerial_Task();
        BlueSerial_SendPitchTask(attitude.pitch);
        Tasks_Task();
    }
}
