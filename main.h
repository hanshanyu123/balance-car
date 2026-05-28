#ifndef __MAIN_H
#define __MAIN_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* CPU frequency from SysConfig (defined in ti_msp_dl_config.h) */
#ifndef CPUCLK_FREQ
#define CPUCLK_FREQ          32000000U
#endif

/*======== System tick (ms) ========*/
extern volatile uint32_t system_ms;
void delay_ms(uint32_t ms);

/*======== Motor Direction GPIO ========*/
#define MOTOR_DIR_L1_PORT    GPIOB
#define MOTOR_DIR_L1_PIN     DL_GPIO_PIN_14
#define MOTOR_DIR_L2_PORT    GPIOB
#define MOTOR_DIR_L2_PIN     DL_GPIO_PIN_17
#define MOTOR_DIR_R1_PORT    GPIOB
#define MOTOR_DIR_R1_PIN     DL_GPIO_PIN_10
#define MOTOR_DIR_R2_PORT    GPIOB
#define MOTOR_DIR_R2_PIN     DL_GPIO_PIN_19

/*======== Right Encoder ========*/
/* Right encoder uses hardware QEI on TIMG0 (PA12/PA13), no GPIO defines needed */

/*======== I2C (Software) for MPU6050 ========*/
#define IIC_SCL_PORT         GPIOB
#define IIC_SCL_PIN          DL_GPIO_PIN_2
#define IIC_SDA_PORT         GPIOB
#define IIC_SDA_PIN          DL_GPIO_PIN_3

/*======== Tracking Sensors (8路) ========*/
#define LS1_PORT             GPIOA
#define LS1_PIN              DL_GPIO_PIN_15
#define LS2_PORT             GPIOA
#define LS2_PIN              DL_GPIO_PIN_17
#define LS3_PORT             GPIOB
#define LS3_PIN              DL_GPIO_PIN_5
#define LS4_PORT             GPIOB
#define LS4_PIN              DL_GPIO_PIN_13
#define LS5_PORT             GPIOB
#define LS5_PIN              DL_GPIO_PIN_8
#define LS6_PORT             GPIOB
#define LS6_PIN              DL_GPIO_PIN_9
#define LS7_PORT             GPIOB
#define LS7_PIN              DL_GPIO_PIN_1
#define LS8_PORT             GPIOA
#define LS8_PIN              DL_GPIO_PIN_28

/*======== Task Key ========*/
#define TASK_KEY_PORT        GPIOB
#define TASK_KEY_PIN         DL_GPIO_PIN_21

/*======== Beeper ========*/
#define BEEP_PORT            GPIOB
#define BEEP_PIN             DL_GPIO_PIN_4

/*======== Status LED (LaunchPad LED1, PA0 with J4) ========*/
#define LED_PORT             GPIOA
#define LED_PIN              DL_GPIO_PIN_0

/*======== GPIO read helper macro ========*/
#define GPIO_READ(port, pin)      ((DL_GPIO_readPins((port), (pin)) != 0) ? 1U : 0U)
#define GPIO_WRITE(port, pin, on) do { \
    if (on) DL_GPIO_setPins(port, pin); \
    else    DL_GPIO_clearPins(port, pin); \
} while(0)

void Error_Handler(void);

#endif /* __MAIN_H */
