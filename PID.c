#include "PID.h"
#include <math.h>

static float balance_kp = 600.0f;
static float balance_ki = 0.0f;
static float balance_kd = -9.1f;

static float speed_kp = 2.0f;
static float speed_ki = 0.01f;
static float speed_kd = 0.0f;

static float turn_kp = 500.0f;
static float turn_ki = 5.0f;
static float turn_kd = 0.0f;

#define BALANCE_INTEGRAL_LIMIT         1000.0f
#define BALANCE_INTEGRAL_ACTIVE_ERROR  10.0f
#define MAX_ANGLE_OFFSET               20.0f
#define SPEED_FILTER_ALPHA             0.3f
#define PWM_MAX                        6666
#define TURN_OUTPUT_LIMIT              2000
#define TURN_INTEGRAL_LIMIT            20.0f

static int balance_output = 0;
static float balance_p_term = 0.0f;
static float balance_i_term = 0.0f;
static float balance_d_term = 0.0f;
static float balance_integral = 0.0f;

static float speed_output_angle = 0.0f;
static float speed_integral = 0.0f;
static float filtered_speed = 0.0f;
static float speed_d_term = 0.0f;
static float last_speed_error = 0.0f;
static float speed_target = 0.0f;

static int turn_output = 0;
static float turn_target = 0.0f;
static float turn_integral = 0.0f;
static float last_turn_actual = 0.0f;

void Speed_Reset(void)
{
    speed_output_angle = 0.0f;
    speed_integral = 0.0f;
    filtered_speed = 0.0f;
    speed_d_term = 0.0f;
    last_speed_error = 0.0f;
    speed_target = 0.0f;
}

void Turn_Reset(void)
{
    turn_output = 0;
    turn_target = 0.0f;
    turn_integral = 0.0f;
    last_turn_actual = 0.0f;
}

void PID_Reset(void)
{
    balance_output = 0;
    balance_p_term = 0.0f;
    balance_i_term = 0.0f;
    balance_d_term = 0.0f;
    balance_integral = 0.0f;

    Speed_Reset();
    Turn_Reset();
}

void PID_Init(void)
{
    PID_Reset();
}

void Balance_PID(float pitch, float gyro_y, float target)
{
    float error;
    float raw_output;

    error = target - pitch;

    balance_p_term = balance_kp * error;

    if (fabsf(error) <= BALANCE_INTEGRAL_ACTIVE_ERROR) {
        balance_integral += error;
        if (balance_integral > BALANCE_INTEGRAL_LIMIT) {
            balance_integral = BALANCE_INTEGRAL_LIMIT;
        }
        if (balance_integral < -BALANCE_INTEGRAL_LIMIT) {
            balance_integral = -BALANCE_INTEGRAL_LIMIT;
        }
    } else {
        balance_integral = 0.0f;
    }

    balance_i_term = balance_ki * balance_integral;
    balance_d_term = balance_kd * gyro_y;

    raw_output = balance_p_term + balance_i_term + balance_d_term;
    balance_output = (int)roundf(raw_output);

    if (balance_output > PWM_MAX) {
        balance_output = PWM_MAX;
    }
    if (balance_output < -PWM_MAX) {
        balance_output = -PWM_MAX;
    }
}

void Speed_PI_Control(float left_speed, float right_speed)
{
    float speed_total;
    float speed_error;
    float speed_delta;

    speed_total = (left_speed + right_speed) * 0.5f;
    filtered_speed = filtered_speed + SPEED_FILTER_ALPHA * (speed_total - filtered_speed);

    speed_error = filtered_speed - speed_target;
    speed_integral += speed_error;

    if (speed_integral > 10000.0f) {
        speed_integral = 10000.0f;
    }
    if (speed_integral < -10000.0f) {
        speed_integral = -10000.0f;
    }

    speed_delta = speed_error - last_speed_error;
    speed_d_term = speed_kd * speed_delta;
    last_speed_error = speed_error;

    speed_output_angle = speed_kp * speed_error + speed_ki * speed_integral + speed_d_term;

    if (speed_output_angle > MAX_ANGLE_OFFSET) {
        speed_output_angle = MAX_ANGLE_OFFSET;
    }
    if (speed_output_angle < -MAX_ANGLE_OFFSET) {
        speed_output_angle = -MAX_ANGLE_OFFSET;
    }
}

void Turn_Control(float left_speed, float right_speed)
{
    float turn_actual;
    float turn_error;
    float turn_d;
    float raw_output;

    turn_actual = left_speed - right_speed;
    turn_error = turn_target - turn_actual;

    if (turn_ki != 0.0f) {
        turn_integral += turn_error;
        if (turn_integral > TURN_INTEGRAL_LIMIT) {
            turn_integral = TURN_INTEGRAL_LIMIT;
        }
        if (turn_integral < -TURN_INTEGRAL_LIMIT) {
            turn_integral = -TURN_INTEGRAL_LIMIT;
        }
    } else {
        turn_integral = 0.0f;
    }

    turn_d = turn_actual - last_turn_actual;
    raw_output = turn_kp * turn_error + turn_ki * turn_integral - turn_kd * turn_d;
    turn_output = (int)roundf(raw_output);

    if (turn_output > TURN_OUTPUT_LIMIT) {
        turn_output = TURN_OUTPUT_LIMIT;
    }
    if (turn_output < -TURN_OUTPUT_LIMIT) {
        turn_output = -TURN_OUTPUT_LIMIT;
    }

    last_turn_actual = turn_actual;
}

int Get_Balance_Output(void) { return balance_output; }
float Get_Speed_Angle_Offset(void) { return speed_output_angle; }
int Get_Turn_Output(void) { return turn_output; }
int Get_Total_Output(void) { return balance_output; }

void Set_Balance_Kp(float kp) { balance_kp = kp; }
void Set_Balance_Ki(float ki) { balance_ki = ki; }
void Set_Balance_Kd(float kd) { balance_kd = kd; }
void Set_Speed_Target(float target) { speed_target = target; }
void Set_Speed_Kp(float kp) { speed_kp = kp; }
void Set_Speed_Ki(float ki) { speed_ki = ki; }
void Set_Speed_Kd(float kd) { speed_kd = kd; }
void Set_Turn_Target(float target) { turn_target = target; }
void Set_Turn_Kp(float kp) { turn_kp = kp; }
void Set_Turn_Ki(float ki) { turn_ki = ki; }
void Set_Turn_Kd(float kd) { turn_kd = kd; }

float Get_Balance_Kp(void) { return balance_kp; }
float Get_Balance_Ki(void) { return balance_ki; }
float Get_Balance_Kd(void) { return balance_kd; }
float Get_Speed_Target(void) { return speed_target; }
float Get_Speed_Kp(void) { return speed_kp; }
float Get_Speed_Ki(void) { return speed_ki; }
float Get_Speed_Kd(void) { return speed_kd; }
float Get_Turn_Target(void) { return turn_target; }
float Get_Turn_Kp(void) { return turn_kp; }
float Get_Turn_Ki(void) { return turn_ki; }
float Get_Turn_Kd(void) { return turn_kd; }
float Get_Balance_P_Term(void) { return balance_p_term; }
float Get_Balance_I_Term(void) { return balance_i_term; }
float Get_Balance_D_Term(void) { return balance_d_term; }
