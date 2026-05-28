#include "encoder.h"
#include "main.h"

/* QEI_1 (Right Encoder) — manually configured, not via SysConfig.
 * MSPM0G3507 SysConfig only supports 1 QEI instance.
 * TIMG0 hardware supports QEI natively, configured here directly. */
#define QEI_1_INST              TIMG0
#define QEI_1_PHA_IOMUX         (IOMUX_PINCM34)   /* PA12 */
#define QEI_1_PHA_FUNC          IOMUX_PINCM34_PF_TIMG0_CCP0
#define QEI_1_PHB_IOMUX         (IOMUX_PINCM35)   /* PA13 */
#define QEI_1_PHB_FUNC          IOMUX_PINCM35_PF_TIMG0_CCP1

static void QEI_1_Init(void)
{
    DL_TimerG_reset(QEI_1_INST);
    DL_TimerG_enablePower(QEI_1_INST);
    delay_cycles(16);

    DL_GPIO_initPeripheralInputFunction(QEI_1_PHA_IOMUX, QEI_1_PHA_FUNC);
    DL_GPIO_initPeripheralInputFunction(QEI_1_PHB_IOMUX, QEI_1_PHB_FUNC);

    static const DL_TimerG_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U
    };
    DL_TimerG_setClockConfig(QEI_1_INST, (DL_TimerG_ClockConfig *)&clkCfg);

    DL_TimerG_configQEI(QEI_1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_1_INST, 65535);
    DL_TimerG_enableClock(QEI_1_INST);
    DL_TimerG_startCounter(QEI_1_INST);
}

void Encoder_Init(void)
{
    (void)DL_TimerG_getTimerCount(QEI_0_INST);
    QEI_1_Init();
    (void)DL_TimerG_getTimerCount(QEI_1_INST);
}

int Read_Encoder(uint8_t side)
{
    int count = 0;

    if (side == 0) {
        count = (int16_t)DL_TimerG_getTimerCount(QEI_0_INST);
        DL_TimerG_setTimerCount(QEI_0_INST, 0);
    } else {
        count = -(int16_t)DL_TimerG_getTimerCount(QEI_1_INST);
        DL_TimerG_setTimerCount(QEI_1_INST, 0);
    }

    return count;
}
