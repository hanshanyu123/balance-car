#include "task3.h"
#include "PID.h"

#define TASK3_READY_DELAY_MS          1500U
#define TASK3_STRAIGHT_IGNORE_MS      500U
#define TASK3_LINE_DETECT_COUNT       2U
#define TASK3_LINE_CLEAR_COUNT        2U
#define TASK3_LINE_LOST_COUNT         5U
#define TASK3_LINE_LOST_SAMPLE_MS     10U
#define TASK3_LINE_MIN_RUN_MS         2000U
#define TASK3_AB_START_SPEED_TIME_MS  1500U
#define TASK3_CD_START_SPEED_TIME_MS  400U

#define TASK3_AB_START_SPEED_TARGET   (-0.8f)
#define TASK3_CD_START_SPEED_TARGET   (-0.8f)
#define TASK3_STRAIGHT_SPEED_TARGET   (-2.3f)
#define TASK3_LINE_SPEED_TARGET       (-2.3f)
#define TASK3_LINE_TURN_K             (0.012f)
#define TASK3_CD_TURN_BIAS            (-0.1f)

static Task3_State task3_state = TASK3_STATE_IDLE;
static uint8_t task3_start_request = 0U;
static uint8_t task3_fault_request = 0U;
static uint8_t task3_started = 0U;
static uint32_t task3_state_tick = 0U;
static uint32_t task3_line_lost_sample_tick = 0U;
static uint8_t task3_sensor_bits = 0U;
static int16_t task3_sensor_error = 0;
static uint8_t task3_sensor_valid = 0U;
static uint8_t task3_line_detect_count = 0U;
static uint8_t task3_line_clear_count = 0U;
static uint8_t task3_line_lost_count = 0U;
static uint8_t task3_line_search_enable = 0U;
static float task3_target_speed = 0.0f;
static float task3_target_turn = 0.0f;
static Task3_Point task3_point_event = TASK3_POINT_NONE;

static void Task3_SetState(Task3_State new_state)
{
    task3_state = new_state;
    task3_state_tick = system_ms;
}

static void Task3_SetPointEvent(Task3_Point point) { task3_point_event = point; }

static void Task3_ResetLineLost(void)
{
    task3_line_lost_count = 0U;
    task3_line_lost_sample_tick = system_ms;
}

static uint8_t Task3_LineDetected(void)
{
    if (task3_sensor_valid != 0U) {
        if (task3_line_detect_count < TASK3_LINE_DETECT_COUNT) task3_line_detect_count++;
    } else {
        task3_line_detect_count = 0U;
    }
    return (task3_line_detect_count >= TASK3_LINE_DETECT_COUNT) ? 1U : 0U;
}

static uint8_t Task3_LineCleared(void)
{
    if (task3_sensor_valid == 0U) {
        if (task3_line_clear_count < TASK3_LINE_CLEAR_COUNT) task3_line_clear_count++;
    } else {
        task3_line_clear_count = 0U;
    }
    return (task3_line_clear_count >= TASK3_LINE_CLEAR_COUNT) ? 1U : 0U;
}

static uint8_t Task3_LineLost(void)
{
    if (task3_sensor_valid != 0U) { Task3_ResetLineLost(); return 0U; }
    if ((system_ms - task3_line_lost_sample_tick) >= TASK3_LINE_LOST_SAMPLE_MS) {
        task3_line_lost_sample_tick = system_ms;
        if (task3_line_lost_count < TASK3_LINE_LOST_COUNT) task3_line_lost_count++;
    }
    return (task3_line_lost_count >= TASK3_LINE_LOST_COUNT) ? 1U : 0U;
}

void Task3_Init(void)
{
    task3_start_request = 0U;
    task3_fault_request = 0U;
    task3_started = 0U;
    task3_line_lost_sample_tick = system_ms;
    task3_sensor_bits = 0U;
    task3_sensor_error = 0;
    task3_sensor_valid = 0U;
    task3_line_detect_count = 0U;
    task3_line_clear_count = 0U;
    Task3_ResetLineLost();
    task3_line_search_enable = 0U;
    task3_target_speed = 0.0f;
    task3_target_turn = 0.0f;
    task3_point_event = TASK3_POINT_NONE;
    Task3_SetState(TASK3_STATE_IDLE);
}

void Task3_RequestStart(void)
{
    if ((task3_state == TASK3_STATE_IDLE) || (task3_state == TASK3_STATE_FINISHED))
        task3_start_request = 1U;
}

void Task3_RequestFault(void) { task3_fault_request = 1U; }
void Task3_Reset(void) { Task3_Init(); }

void Task3_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid)
{
    task3_sensor_bits = bits;
    task3_sensor_error = error;
    task3_sensor_valid = valid;
}

void Task3_Task(void)
{
    if (task3_fault_request != 0U) {
        task3_fault_request = 0U;
        task3_started = 0U;
        task3_target_speed = 0.0f;
        task3_target_turn = 0.0f;
        Task3_SetState(TASK3_STATE_FAULT);
    }

    switch (task3_state) {
        case TASK3_STATE_IDLE:
            task3_target_speed = 0.0f;
            task3_target_turn = 0.0f;
            if (task3_start_request != 0U) {
                task3_start_request = 0U;
                task3_started = 0U;
                task3_line_detect_count = 0U;
                task3_line_clear_count = 0U;
                Task3_ResetLineLost();
                task3_line_search_enable = 0U;
                task3_point_event = TASK3_POINT_NONE;
                Turn_Reset();
                Task3_SetState(TASK3_STATE_READY);
            }
            break;

        case TASK3_STATE_READY:
            task3_target_speed = 0.0f;
            task3_target_turn = 0.0f;
            if ((system_ms - task3_state_tick) >= TASK3_READY_DELAY_MS) {
                task3_started = 1U;
                task3_line_detect_count = 0U;
                task3_line_clear_count = 0U;
                Task3_ResetLineLost();
                task3_line_search_enable = 0U;
                Turn_Reset();
                Task3_SetState(TASK3_STATE_STRAIGHT_AB);
            }
            break;

        case TASK3_STATE_STRAIGHT_AB:
            task3_target_speed = ((system_ms - task3_state_tick) < TASK3_AB_START_SPEED_TIME_MS)
                ? TASK3_AB_START_SPEED_TARGET : TASK3_STRAIGHT_SPEED_TARGET;
            task3_target_turn = 0.0f;
            if (((system_ms - task3_state_tick) >= TASK3_STRAIGHT_IGNORE_MS) &&
                (task3_line_search_enable == 0U) && (Task3_LineCleared() != 0U)) {
                task3_line_search_enable = 1U;
                task3_line_detect_count = 0U;
            }
            if ((task3_line_search_enable != 0U) && (Task3_LineDetected() != 0U)) {
                task3_line_detect_count = 0U; task3_line_clear_count = 0U;
                Task3_ResetLineLost(); task3_line_search_enable = 0U;
                Task3_SetPointEvent(TASK3_POINT_B); Turn_Reset();
                Task3_SetState(TASK3_STATE_LINE_BC);
            }
            break;

        case TASK3_STATE_LINE_BC:
            task3_target_speed = TASK3_LINE_SPEED_TARGET;
            task3_target_turn = (task3_sensor_valid != 0U) ? (float)task3_sensor_error * TASK3_LINE_TURN_K : 0.0f;
            if (((system_ms - task3_state_tick) >= TASK3_LINE_MIN_RUN_MS) && (Task3_LineLost() != 0U)) {
                task3_target_speed = TASK3_STRAIGHT_SPEED_TARGET; task3_target_turn = 0.0f;
                task3_line_detect_count = 0U; task3_line_clear_count = 0U;
                Task3_ResetLineLost(); task3_line_search_enable = 0U;
                Task3_SetPointEvent(TASK3_POINT_C); Turn_Reset();
                Task3_SetState(TASK3_STATE_STRAIGHT_CD);
            }
            break;

        case TASK3_STATE_STRAIGHT_CD:
            task3_target_speed = ((system_ms - task3_state_tick) < TASK3_CD_START_SPEED_TIME_MS)
                ? TASK3_CD_START_SPEED_TARGET : TASK3_STRAIGHT_SPEED_TARGET;
            task3_target_turn = TASK3_CD_TURN_BIAS;
            if (((system_ms - task3_state_tick) >= TASK3_STRAIGHT_IGNORE_MS) &&
                (task3_line_search_enable == 0U) && (Task3_LineCleared() != 0U)) {
                task3_line_search_enable = 1U; task3_line_detect_count = 0U;
            }
            if ((task3_line_search_enable != 0U) && (Task3_LineDetected() != 0U)) {
                task3_line_detect_count = 0U; task3_line_clear_count = 0U;
                Task3_ResetLineLost(); task3_line_search_enable = 0U;
                Task3_SetPointEvent(TASK3_POINT_D); Turn_Reset();
                Task3_SetState(TASK3_STATE_LINE_DA);
            }
            break;

        case TASK3_STATE_LINE_DA:
            task3_target_speed = TASK3_LINE_SPEED_TARGET;
            task3_target_turn = (task3_sensor_valid != 0U) ? (float)task3_sensor_error * TASK3_LINE_TURN_K : 0.0f;
            if (((system_ms - task3_state_tick) >= TASK3_LINE_MIN_RUN_MS) && (Task3_LineLost() != 0U)) {
                task3_started = 0U; task3_target_speed = 0.0f; task3_target_turn = 0.0f;
                Task3_SetPointEvent(TASK3_POINT_A); Turn_Reset();
                Task3_SetState(TASK3_STATE_FINISHED);
            }
            break;

        case TASK3_STATE_FINISHED:
        case TASK3_STATE_FAULT:
            task3_started = 0U; task3_target_speed = 0.0f; task3_target_turn = 0.0f;
            break;
        default: break;
    }
}

Task3_State Task3_GetState(void) { return task3_state; }
uint8_t Task3_IsStarted(void) { return task3_started; }
float Task3_GetTargetSpeed(void) { return task3_target_speed; }
float Task3_GetTargetTurn(void) { return task3_target_turn; }
Task3_Point Task3_GetPointEvent(void) { return task3_point_event; }
void Task3_ClearPointEvent(void) { task3_point_event = TASK3_POINT_NONE; }
uint8_t Task3_GetSensorBits(void) { return task3_sensor_bits; }
int16_t Task3_GetSensorError(void) { return task3_sensor_error; }
uint8_t Task3_GetSensorValid(void) { return task3_sensor_valid; }
