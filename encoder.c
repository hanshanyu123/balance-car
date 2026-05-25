#include "encoder.h"
#include "main.h"

/* Left encoder: QEI on TIMG8 */
/* Right encoder: software GPIO interrupt on PA12 */
static volatile int16_t right_encoder_count = 0;

void Encoder_Init(void)
{
    right_encoder_count = 0;

    /* Configure PA12 (IOMUX_PINCM34) as input with interrupt for right encoder */
    DL_GPIO_initDigitalInput(IOMUX_PINCM34);
    DL_GPIO_setLowerPinsPolarity(GPIOA, DL_GPIO_PIN_12_EDGE_RISE_FALL);
    DL_GPIO_enableInterrupt(GPIOA, RIGHT_ENC_PIN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* Read initial encoder values to clear QEI counter */
    (void)DL_TimerG_getTimerCount(QEI_0_INST);
}

int Read_Encoder(uint8_t side)
{
    int count = 0;

    if (side == 0) {
        /* Left encoder: TIMG8 QEI */
        count = (int16_t)DL_TimerG_getTimerCount(QEI_0_INST);
        DL_TimerG_setTimerCount(QEI_0_INST, 0);
    } else {
        /* Right encoder: software interrupt counter */
        __disable_irq();
        count = right_encoder_count;
        right_encoder_count = 0;
        __enable_irq();
    }

    return count;
}

/* GPIOA interrupt handler for right encoder */
void GPIOA_IRQHandler(void)
{
    if (DL_GPIO_getEnabledInterruptStatus(GPIOA, RIGHT_ENC_PIN) & RIGHT_ENC_PIN) {
        right_encoder_count++;
        DL_GPIO_clearInterruptStatus(GPIOA, RIGHT_ENC_PIN);
    }
}
