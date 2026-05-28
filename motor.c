#include "motor.h"
#include "main.h"

#define PWM_MAX          6666
#define DEAD_ZONE_LEFT   300
#define DEAD_ZONE_RIGHT  300

/* Motor direction GPIO initialization */
static void Motor_Dir_GPIO_Init(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM31); /* PB14 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM43); /* PB17 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM27); /* PB10 */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM45); /* PB19 */

    DL_GPIO_clearPins(MOTOR_DIR_L1_PORT, MOTOR_DIR_L1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_R1_PORT, MOTOR_DIR_R1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_L1_PORT, MOTOR_DIR_L1_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_R1_PORT, MOTOR_DIR_R1_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_PIN);
}

void Motor_Init(void)
{
    Motor_Dir_GPIO_Init();
    Set_Motor_Speed(0, 0);
}

void Set_Motor_Speed(int left_speed, int right_speed)
{
    int left_pwm = 0;
    int right_pwm = 0;

    if (left_speed > 0) {
        DL_GPIO_setPins(MOTOR_DIR_L1_PORT, MOTOR_DIR_L1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_PIN);
        left_pwm = left_speed + DEAD_ZONE_LEFT;
    } else if (left_speed < 0) {
        DL_GPIO_clearPins(MOTOR_DIR_L1_PORT, MOTOR_DIR_L1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_PIN);
        left_pwm = -left_speed + DEAD_ZONE_LEFT;
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_L1_PORT, MOTOR_DIR_L1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_L2_PORT, MOTOR_DIR_L2_PIN);
        left_pwm = 0;
    }

    if (right_speed > 0) {
        DL_GPIO_setPins(MOTOR_DIR_R1_PORT, MOTOR_DIR_R1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_PIN);
        right_pwm = right_speed + DEAD_ZONE_RIGHT;
    } else if (right_speed < 0) {
        DL_GPIO_clearPins(MOTOR_DIR_R1_PORT, MOTOR_DIR_R1_PIN);
        DL_GPIO_setPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_PIN);
        right_pwm = -right_speed + DEAD_ZONE_RIGHT;
    } else {
        DL_GPIO_clearPins(MOTOR_DIR_R1_PORT, MOTOR_DIR_R1_PIN);
        DL_GPIO_clearPins(MOTOR_DIR_R2_PORT, MOTOR_DIR_R2_PIN);
        right_pwm = 0;
    }

    if (left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, left_pwm, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, right_pwm, DL_TIMER_CC_1_INDEX);
}
