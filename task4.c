#include "task4.h"
#include "PID.h"

#define TASK4_READY_DELAY_MS                  888U
#define TASK4_STRAIGHT_IGNORE_MS              1500U
#define TASK4_LINE_MIN_RUN_MS                 4000U
#define TASK4_LINE_LOST_SAMPLE_MS             10U

#define TASK4_LINE_DETECT_COUNT               2U
#define TASK4_LINE_CLEAR_COUNT                2U
#define TASK4_LINE_LOST_COUNT                 5U

#define TASK4_LINE_INVERT_TIME_MS             1333U
#define TASK4_LINE_BOOST_TIME_MS              1234U
#define TASK4_LINE_ENTRY_SLOW_TIME_MS         1234U
#define TASK4_LINE_BOOST_GAIN                 1.6f
#define TASK4_LINE_ENTRY_SLOW_RATIO           0.81f
#define TASK4_LINE_POST_INVERT_WEIGHT_TIME_MS 1500U
#define TASK4_LINE_POST_INVERT_WEIGHT_RATIO   0.8f
#define TASK4_LINE_NON_INVERT_WEIGHT_RATIO    1.2345f
#define TASK4_LINE_SPEED_TARGET               (-2.5f)
#define TASK4_LINE_TURN_K                     (0.0666f)

#define TASK4_EXIT_TURN_SPEED_TARGET          TASK4_LINE_SPEED_TARGET
#define TASK4_AC_EXIT_TURN_TIME_MS            1100U
#define TASK4_BD_EXIT_TURN_TIME_MS            950U
#define TASK4_AC_EXIT_AFTER_FIRST_LOOP_RATIO  1.00f
#define TASK4_AC_EXIT_TURN_TARGET             (-12.f)
#define TASK4_BD_EXIT_TURN_TARGET             (12.0f)

#define TASK4_AC_STATIC_TURN_TIME_MS          950U
#define TASK4_BD_STATIC_TURN_TIME_MS          850U
#define TASK4_AC_STATIC_TURN_TARGET           (-12.f)
#define TASK4_BD_STATIC_TURN_TARGET           (10.0f)

#define TASK4_AC_SPEED_STAGE1_TIME_MS         1234U
#define TASK4_AC_SPEED_STAGE2_TIME_MS         2000U
#define TASK4_AC_SPEED_STAGE1_TARGET          (-1.0f)
#define TASK4_AC_SPEED_STAGE2_TARGET          (-1.6f)
#define TASK4_AC_SPEED_STAGE3_TARGET          (-2.5f)

#define TASK4_STRAIGHT_SPEED_TARGET           (-2.5f)
#define TASK4_AC_STRAIGHT_BIAS                (0.050f)
#define TASK4_BD_STRAIGHT_BIAS                (-0.055f)

#define TASK4_INVERT_NONE                     0U
#define TASK4_INVERT_LEFT_4                   1U
#define TASK4_INVERT_RIGHT_4                  2U

static Task4_State task4_state = TASK4_STATE_IDLE;
static uint8_t task4_start_request = 0U;
static uint8_t task4_fault_request = 0U;
static uint8_t task4_started = 0U;
static uint8_t task4_loop_target = 1U;
static uint8_t task4_loop_count = 0U;
static uint32_t task4_state_tick = 0U;
static uint32_t task4_line_lost_sample_tick = 0U;
static uint8_t task4_sensor_bits = 0U;
static int16_t task4_sensor_error = 0;
static uint8_t task4_sensor_valid = 0U;
static uint8_t task4_line_detect_count = 0U;
static uint8_t task4_line_clear_count = 0U;
static uint8_t task4_line_lost_count = 0U;
static uint8_t task4_line_search_enable = 0U;
static uint8_t task4_exit_turn_enable = 0U;
static float task4_target_speed = 0.0f;
static float task4_target_turn = 0.0f;
static Task4_Point task4_point_event = TASK4_POINT_NONE;

#define TICK() system_ms

static void Task4_SetState(Task4_State new_state)
{
    task4_state = new_state;
    task4_state_tick = TICK();
}

static void Task4_SetPointEvent(Task4_Point point) { task4_point_event = point; }

static void Task4_ResetLineLost(void)
{
    task4_line_lost_count = 0U;
    task4_line_lost_sample_tick = TICK();
}

static uint8_t Task4_LineDetected(void)
{
    if (task4_sensor_valid != 0U) { if (task4_line_detect_count < TASK4_LINE_DETECT_COUNT) task4_line_detect_count++; }
    else { task4_line_detect_count = 0U; }
    return (task4_line_detect_count >= TASK4_LINE_DETECT_COUNT) ? 1U : 0U;
}

static uint8_t Task4_LineCleared(void)
{
    if (task4_sensor_valid == 0U) { if (task4_line_clear_count < TASK4_LINE_CLEAR_COUNT) task4_line_clear_count++; }
    else { task4_line_clear_count = 0U; }
    return (task4_line_clear_count >= TASK4_LINE_CLEAR_COUNT) ? 1U : 0U;
}

static uint8_t Task4_LineLost(void)
{
    if (task4_sensor_valid != 0U) { Task4_ResetLineLost(); return 0U; }
    if ((TICK() - task4_line_lost_sample_tick) >= TASK4_LINE_LOST_SAMPLE_MS) {
        task4_line_lost_sample_tick = TICK();
        if (task4_line_lost_count < TASK4_LINE_LOST_COUNT) task4_line_lost_count++;
    }
    return (task4_line_lost_count >= TASK4_LINE_LOST_COUNT) ? 1U : 0U;
}

static int16_t Task4_ScaleLineWeight(int16_t weight, float ratio)
{
    float scaled = (float)weight * ratio;
    return (scaled >= 0.0f) ? (int16_t)(scaled + 0.5f) : (int16_t)(scaled - 0.5f);
}

static void Task4_ResetLineFlags(void)
{
    task4_line_detect_count = 0U;
    task4_line_clear_count = 0U;
    Task4_ResetLineLost();
    task4_line_search_enable = 0U;
}

static int16_t Task4_GetInvertedSensorError(uint8_t invert_side)
{
    int16_t error = 0;
    uint8_t boost_left = (invert_side == TASK4_INVERT_RIGHT_4) ? 1U : 0U;
    uint8_t boost_right = (invert_side == TASK4_INVERT_LEFT_4) ? 1U : 0U;

    if ((task4_sensor_bits & 128U) != 0U) error += (invert_side == TASK4_INVERT_LEFT_4) ? -1000 : ((boost_left != 0U) ? Task4_ScaleLineWeight(1000, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 1000);
    if ((task4_sensor_bits & 64U) != 0U)  error += (invert_side == TASK4_INVERT_LEFT_4) ? -500  : ((boost_left != 0U) ? Task4_ScaleLineWeight(500, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 500);
    if ((task4_sensor_bits & 32U) != 0U)  error += (invert_side == TASK4_INVERT_LEFT_4) ? -300  : ((boost_left != 0U) ? Task4_ScaleLineWeight(300, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 300);
    if ((task4_sensor_bits & 16U) != 0U)  error += (invert_side == TASK4_INVERT_LEFT_4) ? -100  : ((boost_left != 0U) ? Task4_ScaleLineWeight(100, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 100);
    if ((task4_sensor_bits & 8U) != 0U)   error += (invert_side == TASK4_INVERT_RIGHT_4) ? 100  : ((boost_right != 0U) ? Task4_ScaleLineWeight(-100, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -100);
    if ((task4_sensor_bits & 4U) != 0U)   error += (invert_side == TASK4_INVERT_RIGHT_4) ? 300  : ((boost_right != 0U) ? Task4_ScaleLineWeight(-300, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -300);
    if ((task4_sensor_bits & 2U) != 0U)   error += (invert_side == TASK4_INVERT_RIGHT_4) ? 500  : ((boost_right != 0U) ? Task4_ScaleLineWeight(-500, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -500);
    if ((task4_sensor_bits & 1U) != 0U)   error += (invert_side == TASK4_INVERT_RIGHT_4) ? 1000 : ((boost_right != 0U) ? Task4_ScaleLineWeight(-1000, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -1000);
    return error;
}

static int16_t Task4_GetPostInvertSensorError(uint8_t invert_side)
{
    int16_t error = 0;
    uint8_t weaken_left = (invert_side == TASK4_INVERT_LEFT_4) ? 1U : 0U;
    uint8_t weaken_right = (invert_side == TASK4_INVERT_RIGHT_4) ? 1U : 0U;
    uint8_t boost_left = (invert_side == TASK4_INVERT_RIGHT_4) ? 1U : 0U;
    uint8_t boost_right = (invert_side == TASK4_INVERT_LEFT_4) ? 1U : 0U;

    if ((task4_sensor_bits & 128U) != 0U) error += (weaken_left != 0U) ? Task4_ScaleLineWeight(999, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_left != 0U) ? Task4_ScaleLineWeight(999, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 999);
    if ((task4_sensor_bits & 64U) != 0U)  error += (weaken_left != 0U) ? Task4_ScaleLineWeight(888, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_left != 0U) ? Task4_ScaleLineWeight(888, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 888);
    if ((task4_sensor_bits & 32U) != 0U)  error += (weaken_left != 0U) ? Task4_ScaleLineWeight(666, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_left != 0U) ? Task4_ScaleLineWeight(666, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 666);
    if ((task4_sensor_bits & 16U) != 0U)  error += (weaken_left != 0U) ? Task4_ScaleLineWeight(333, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_left != 0U) ? Task4_ScaleLineWeight(333, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : 333);
    if ((task4_sensor_bits & 8U) != 0U)   error += (weaken_right != 0U) ? Task4_ScaleLineWeight(-333, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_right != 0U) ? Task4_ScaleLineWeight(-333, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -333);
    if ((task4_sensor_bits & 4U) != 0U)   error += (weaken_right != 0U) ? Task4_ScaleLineWeight(-666, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_right != 0U) ? Task4_ScaleLineWeight(-666, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -666);
    if ((task4_sensor_bits & 2U) != 0U)   error += (weaken_right != 0U) ? Task4_ScaleLineWeight(-888, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_right != 0U) ? Task4_ScaleLineWeight(-888, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -888);
    if ((task4_sensor_bits & 1U) != 0U)   error += (weaken_right != 0U) ? Task4_ScaleLineWeight(-999, TASK4_LINE_POST_INVERT_WEIGHT_RATIO) : ((boost_right != 0U) ? Task4_ScaleLineWeight(-999, TASK4_LINE_NON_INVERT_WEIGHT_RATIO) : -999);
    return error;
}

static int16_t Task4_GetLineError(uint8_t invert_side)
{
    uint32_t line_time_ms = TICK() - task4_state_tick;
    if (line_time_ms < TASK4_LINE_INVERT_TIME_MS) return Task4_GetInvertedSensorError(invert_side);
    if (line_time_ms < (TASK4_LINE_INVERT_TIME_MS + TASK4_LINE_POST_INVERT_WEIGHT_TIME_MS)) return Task4_GetPostInvertSensorError(invert_side);
    return task4_sensor_error;
}

static float Task4_GetLineTurnK(void)
{
    return ((TICK() - task4_state_tick) < TASK4_LINE_BOOST_TIME_MS)
        ? TASK4_LINE_TURN_K * TASK4_LINE_BOOST_GAIN : TASK4_LINE_TURN_K;
}

static float Task4_GetLineSpeedTarget(void)
{
    return ((TICK() - task4_state_tick) < TASK4_LINE_ENTRY_SLOW_TIME_MS)
        ? TASK4_LINE_SPEED_TARGET * TASK4_LINE_ENTRY_SLOW_RATIO : TASK4_LINE_SPEED_TARGET;
}

static uint32_t Task4_GetAcExitTurnTimeMs(void)
{
    return (task4_loop_count >= 1U)
        ? (uint32_t)(((float)TASK4_AC_EXIT_TURN_TIME_MS * TASK4_AC_EXIT_AFTER_FIRST_LOOP_RATIO) + 0.5f)
        : TASK4_AC_EXIT_TURN_TIME_MS;
}

void Task4_Init(void)
{
    task4_start_request = 0U; task4_fault_request = 0U; task4_started = 0U;
    task4_loop_target = 1U; task4_loop_count = 0U;
    task4_line_lost_sample_tick = TICK();
    task4_sensor_bits = 0U; task4_sensor_error = 0; task4_sensor_valid = 0U;
    Task4_ResetLineFlags(); task4_exit_turn_enable = 0U;
    task4_target_speed = 0.0f; task4_target_turn = 0.0f; task4_point_event = TASK4_POINT_NONE;
    Task4_SetState(TASK4_STATE_IDLE);
}

void Task4_RequestStart(uint8_t loop_target)
{
    if (loop_target == 0U) loop_target = 1U;
    if ((task4_state == TASK4_STATE_IDLE) || (task4_state == TASK4_STATE_FINISHED)) {
        task4_loop_target = loop_target;
        task4_start_request = 1U;
    }
}

void Task4_RequestFault(void) { task4_fault_request = 1U; }
void Task4_Reset(void) { Task4_Init(); }

void Task4_UpdateSensor(uint8_t bits, int16_t error, uint8_t valid)
{
    task4_sensor_bits = bits; task4_sensor_error = error; task4_sensor_valid = valid;
}

void Task4_Task(void)
{
    if (task4_fault_request != 0U) {
        task4_fault_request = 0U; task4_started = 0U; task4_exit_turn_enable = 0U;
        task4_target_speed = 0.0f; task4_target_turn = 0.0f;
        Task4_SetState(TASK4_STATE_FAULT);
    }

    switch (task4_state) {
        case TASK4_STATE_IDLE:
            task4_target_speed = 0.0f; task4_target_turn = 0.0f;
            if (task4_start_request != 0U) {
                task4_start_request = 0U; task4_started = 0U; task4_loop_count = 0U;
                Task4_ResetLineFlags(); task4_exit_turn_enable = 0U; task4_point_event = TASK4_POINT_NONE;
                Turn_Reset(); Task4_SetState(TASK4_STATE_READY);
            }
            break;

        case TASK4_STATE_READY:
            task4_target_speed = 0.0f; task4_target_turn = 0.0f;
            if ((TICK() - task4_state_tick) >= TASK4_READY_DELAY_MS) {
                task4_started = 1U; Task4_ResetLineFlags(); task4_exit_turn_enable = 0U;
                Turn_Reset(); Task4_SetState(TASK4_STATE_TURN_AC);
            }
            break;

        case TASK4_STATE_TURN_AC:
            if (task4_exit_turn_enable != 0U) {
                task4_target_speed = TASK4_EXIT_TURN_SPEED_TARGET; task4_target_turn = TASK4_AC_EXIT_TURN_TARGET;
                if ((TICK() - task4_state_tick) >= Task4_GetAcExitTurnTimeMs()) {
                    task4_exit_turn_enable = 0U; Task4_ResetLineFlags(); Turn_Reset();
                    Task4_SetState(TASK4_STATE_STRAIGHT_AC);
                }
            } else {
                task4_target_speed = 0.0f; task4_target_turn = TASK4_AC_STATIC_TURN_TARGET;
                if ((TICK() - task4_state_tick) >= TASK4_AC_STATIC_TURN_TIME_MS) {
                    Task4_ResetLineFlags(); Turn_Reset();
                    Task4_SetState(TASK4_STATE_STRAIGHT_AC);
                }
            }
            break;

        case TASK4_STATE_STRAIGHT_AC: {
            uint32_t el = TICK() - task4_state_tick;
            if (task4_loop_count == 0U) {
                if (el < TASK4_AC_SPEED_STAGE1_TIME_MS) task4_target_speed = TASK4_AC_SPEED_STAGE1_TARGET;
                else if (el < TASK4_AC_SPEED_STAGE2_TIME_MS) task4_target_speed = TASK4_AC_SPEED_STAGE2_TARGET;
                else task4_target_speed = TASK4_AC_SPEED_STAGE3_TARGET;
            } else {
                task4_target_speed = TASK4_LINE_SPEED_TARGET;
            }
            task4_target_turn = TASK4_AC_STRAIGHT_BIAS;
            if ((el >= TASK4_STRAIGHT_IGNORE_MS) && (task4_line_search_enable == 0U) && (Task4_LineCleared() != 0U)) {
                task4_line_search_enable = 1U; task4_line_detect_count = 0U;
            }
            if ((task4_line_search_enable != 0U) && (Task4_LineDetected() != 0U)) {
                Task4_ResetLineFlags(); Task4_SetPointEvent(TASK4_POINT_C); Turn_Reset();
                Task4_SetState(TASK4_STATE_LINE_CB);
            }
            break;
        }

        case TASK4_STATE_LINE_CB:
            task4_target_speed = Task4_GetLineSpeedTarget();
            task4_target_turn = (task4_sensor_valid != 0U)
                ? (float)Task4_GetLineError(TASK4_INVERT_RIGHT_4) * Task4_GetLineTurnK() : 0.0f;
            if (((TICK() - task4_state_tick) >= TASK4_LINE_MIN_RUN_MS) && (Task4_LineLost() != 0U)) {
                task4_target_speed = TASK4_EXIT_TURN_SPEED_TARGET; task4_target_turn = TASK4_BD_EXIT_TURN_TARGET;
                Task4_ResetLineFlags(); Task4_SetPointEvent(TASK4_POINT_B); task4_exit_turn_enable = 1U;
                Turn_Reset(); Task4_SetState(TASK4_STATE_TURN_BD);
            }
            break;

        case TASK4_STATE_TURN_BD:
            if (task4_exit_turn_enable != 0U) {
                task4_target_speed = TASK4_EXIT_TURN_SPEED_TARGET; task4_target_turn = TASK4_BD_EXIT_TURN_TARGET;
                if ((TICK() - task4_state_tick) >= TASK4_BD_EXIT_TURN_TIME_MS) {
                    task4_exit_turn_enable = 0U; Task4_ResetLineFlags(); Turn_Reset();
                    Task4_SetState(TASK4_STATE_STRAIGHT_BD);
                }
            } else {
                task4_target_speed = 0.0f; task4_target_turn = TASK4_BD_STATIC_TURN_TARGET;
                if ((TICK() - task4_state_tick) >= TASK4_BD_STATIC_TURN_TIME_MS) {
                    Task4_ResetLineFlags(); Turn_Reset();
                    Task4_SetState(TASK4_STATE_STRAIGHT_BD);
                }
            }
            break;

        case TASK4_STATE_STRAIGHT_BD:
            task4_target_speed = TASK4_LINE_SPEED_TARGET; task4_target_turn = TASK4_BD_STRAIGHT_BIAS;
            if (((TICK() - task4_state_tick) >= TASK4_STRAIGHT_IGNORE_MS) && (task4_line_search_enable == 0U) && (Task4_LineCleared() != 0U)) {
                task4_line_search_enable = 1U; task4_line_detect_count = 0U;
            }
            if ((task4_line_search_enable != 0U) && (Task4_LineDetected() != 0U)) {
                Task4_ResetLineFlags(); Task4_SetPointEvent(TASK4_POINT_D); Turn_Reset();
                Task4_SetState(TASK4_STATE_LINE_DA);
            }
            break;

        case TASK4_STATE_LINE_DA:
            task4_target_speed = Task4_GetLineSpeedTarget();
            task4_target_turn = (task4_sensor_valid != 0U)
                ? (float)Task4_GetLineError(TASK4_INVERT_LEFT_4) * Task4_GetLineTurnK() : 0.0f;
            if (((TICK() - task4_state_tick) >= TASK4_LINE_MIN_RUN_MS) && (Task4_LineLost() != 0U)) {
                Task4_SetPointEvent(TASK4_POINT_A); task4_loop_count++;
                if (task4_loop_count >= task4_loop_target) {
                    task4_started = 0U; task4_exit_turn_enable = 0U;
                    task4_target_speed = 0.0f; task4_target_turn = 0.0f;
                    Turn_Reset(); Task4_SetState(TASK4_STATE_FINISHED);
                } else {
                    task4_target_speed = TASK4_EXIT_TURN_SPEED_TARGET; task4_target_turn = TASK4_AC_EXIT_TURN_TARGET;
                    Task4_ResetLineFlags(); task4_exit_turn_enable = 1U; Turn_Reset();
                    Task4_SetState(TASK4_STATE_TURN_AC);
                }
            }
            break;

        case TASK4_STATE_FINISHED:
        case TASK4_STATE_FAULT:
            task4_started = 0U; task4_exit_turn_enable = 0U;
            task4_target_speed = 0.0f; task4_target_turn = 0.0f;
            break;
        default: break;
    }
}

Task4_State Task4_GetState(void) { return task4_state; }
uint8_t Task4_IsStarted(void) { return task4_started; }
uint8_t Task4_GetLoopCount(void) { return task4_loop_count; }
uint8_t Task4_GetLoopTarget(void) { return task4_loop_target; }
float Task4_GetTargetSpeed(void) { return task4_target_speed; }
float Task4_GetTargetTurn(void) { return task4_target_turn; }
Task4_Point Task4_GetPointEvent(void) { return task4_point_event; }
void Task4_ClearPointEvent(void) { task4_point_event = TASK4_POINT_NONE; }
uint8_t Task4_GetSensorBits(void) { return task4_sensor_bits; }
int16_t Task4_GetSensorError(void) { return task4_sensor_error; }
uint8_t Task4_GetSensorValid(void) { return task4_sensor_valid; }
