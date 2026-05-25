#include "task2.h"

#define TASK2_READY_DELAY_MS        1000U
#define TASK2_RUN_SPEED_TARGET      -1.66f
#define TASK2_SLOW_SPEED_TARGET     -0.8f
#define TASK2_SLOW_TIME_MS          700U
#define TASK2_STOP_HOLD_TIME_MS     400U
#define TASK2_LINE_DETECT_COUNT     2U

static Task2_State task2_state = TASK2_STATE_IDLE;
static uint8_t task2_start_request = 0U;
static uint8_t task2_fault_request = 0U;
static uint8_t task2_started = 0U;
static uint8_t task2_stop_request = 0U;
static uint32_t task2_state_tick = 0U;
static uint8_t task2_sensor_bits = 0U;
static int16_t task2_sensor_error = 0;
static uint8_t task2_sensor_valid = 0U;
static uint8_t task2_line_detect_count = 0U;
static uint8_t task2_stop_detect_armed = 0U;
static float task2_target_speed = 0.0f;

static void Task2_SetState(Task2_State new_state)
{
    task2_state = new_state;
    task2_state_tick = system_ms;
}

void Task2_Init(void)
{
    task2_start_request = 0U;
    task2_fault_request = 0U;
    task2_started = 0U;
    task2_stop_request = 0U;
    task2_sensor_bits = 0U;
    task2_sensor_error = 0;
    task2_sensor_valid = 0U;
    task2_line_detect_count = 0U;
    task2_stop_detect_armed = 0U;
    task2_target_speed = 0.0f;
    Task2_SetState(TASK2_STATE_IDLE);
}

void Task2_RequestStart(void)
{
    if ((task2_state == TASK2_STATE_IDLE) || (task2_state == TASK2_STATE_FINISHED))
        task2_start_request = 1U;
}

void Task2_RequestFault(void) { task2_fault_request = 1U; }

void Task2_Reset(void) { Task2_Init(); }

void Task2_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid)
{
    task2_sensor_bits = bits;
    task2_sensor_error = error;
    task2_sensor_valid = valid;
}

void Task2_Task(void)
{
    uint32_t state_time_ms;

    if (task2_fault_request != 0U) {
        task2_fault_request = 0U;
        task2_started = 0U;
        task2_target_speed = 0.0f;
        Task2_SetState(TASK2_STATE_FAULT);
    }

    switch (task2_state) {
        case TASK2_STATE_IDLE:
            if (task2_start_request != 0U) {
                task2_start_request = 0U;
                task2_stop_request = 0U;
                task2_started = 0U;
                task2_line_detect_count = 0U;
                task2_stop_detect_armed = 0U;
                task2_target_speed = 0.0f;
                Task2_SetState(TASK2_STATE_READY);
            }
            break;

        case TASK2_STATE_READY:
            if ((system_ms - task2_state_tick) >= TASK2_READY_DELAY_MS) {
                task2_line_detect_count = 0U;
                task2_stop_detect_armed = 0U;
                task2_started = 1U;
                task2_target_speed = TASK2_RUN_SPEED_TARGET;
                Task2_SetState(TASK2_STATE_RUNNING);
            }
            break;

        case TASK2_STATE_RUNNING:
            if (task2_stop_detect_armed == 0U) {
                if (task2_sensor_valid == 0U) task2_stop_detect_armed = 1U;
                break;
            }
            if (task2_sensor_valid != 0U) {
                if (task2_line_detect_count < TASK2_LINE_DETECT_COUNT) task2_line_detect_count++;
            } else {
                task2_line_detect_count = 0U;
            }
            if (task2_line_detect_count >= TASK2_LINE_DETECT_COUNT) {
                task2_stop_request = 1U;
                Task2_SetState(TASK2_STATE_STOPPING);
            }
            break;

        case TASK2_STATE_STOPPING:
            state_time_ms = system_ms - task2_state_tick;
            if (state_time_ms < TASK2_SLOW_TIME_MS) {
                task2_target_speed = TASK2_SLOW_SPEED_TARGET;
            } else if (state_time_ms < (TASK2_SLOW_TIME_MS + TASK2_STOP_HOLD_TIME_MS)) {
                task2_target_speed = 0.0f;
            } else {
                task2_started = 0U;
                task2_target_speed = 0.0f;
                Task2_SetState(TASK2_STATE_FINISHED);
            }
            break;

        case TASK2_STATE_FINISHED: task2_target_speed = 0.0f; break;
        case TASK2_STATE_FAULT:    task2_target_speed = 0.0f; break;
        default: break;
    }
}

Task2_State Task2_GetState(void) { return task2_state; }
uint8_t Task2_IsStarted(void) { return task2_started; }
uint8_t Task2_IsStopRequested(void) { return task2_stop_request; }
void Task2_ClearStopRequest(void) { task2_stop_request = 0U; }
uint32_t Task2_GetStateTick(void) { return task2_state_tick; }
float Task2_GetTargetSpeed(void) { return task2_target_speed; }
uint8_t Task2_GetSensorBits(void) { return task2_sensor_bits; }
int16_t Task2_GetSensorError(void) { return task2_sensor_error; }
uint8_t Task2_GetSensorValid(void) { return task2_sensor_valid; }
