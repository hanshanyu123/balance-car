#include "alert.h"
#include "main.h"

#define ALERT_BEEP_ACTIVE_LEVEL     0U
#define ALERT_BEEP_INACTIVE_LEVEL   1U
#define ALERT_LED_ACTIVE_LEVEL      0U
#define ALERT_LED_INACTIVE_LEVEL    1U

typedef struct {
    uint8_t pulse_count;
    uint16_t on_ms;
    uint16_t off_ms;
    uint8_t repeat;
} Alert_Pattern;

static Alert_Pattern alert_pattern = {0U, 0U, 0U, 0U};
static uint32_t alert_phase_tick = 0U;
static uint8_t alert_active = 0U;
static uint8_t alert_phase_on = 0U;
static uint8_t alert_pulses_done = 0U;

static void Alert_SetOutput(uint8_t on)
{
    GPIO_WRITE(BEEP_PORT, BEEP_PIN, !on);
    GPIO_WRITE(LED_PORT, LED_PIN, !on);
}

static Alert_Pattern Alert_GetPattern(Alert_Event event)
{
    Alert_Pattern pattern;

    switch (event) {
        case ALERT_EVENT_POINT:
        default:
            pattern.pulse_count = 1U;
            pattern.on_ms = 180U;
            pattern.off_ms = 120U;
            pattern.repeat = 0U;
            break;
    }

    return pattern;
}

void Alert_Init(void)
{
    alert_active = 0U;
    alert_phase_on = 0U;
    alert_pulses_done = 0U;
    Alert_SetOutput(0U);
}

void Alert_Request(Alert_Event event)
{
    alert_pattern = Alert_GetPattern(event);
    alert_active = 1U;
    alert_phase_on = 1U;
    alert_pulses_done = 0U;
    alert_phase_tick = system_ms;
    Alert_SetOutput(1U);
}

void Alert_Task(void)
{
    uint32_t now;
    uint32_t period_ms;

    if (alert_active == 0U) return;

    now = system_ms;
    period_ms = (alert_phase_on != 0U) ? alert_pattern.on_ms : alert_pattern.off_ms;
    if ((now - alert_phase_tick) < period_ms) return;

    alert_phase_tick = now;

    if (alert_phase_on != 0U) {
        alert_phase_on = 0U;
        alert_pulses_done++;
        Alert_SetOutput(0U);
        return;
    }

    if (alert_pulses_done >= alert_pattern.pulse_count) {
        if (alert_pattern.repeat == 0U) {
            alert_active = 0U;
            Alert_SetOutput(0U);
            return;
        }
        alert_pulses_done = 0U;
    }

    alert_phase_on = 1U;
    Alert_SetOutput(1U);
}
