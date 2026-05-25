#include "sensor.h"
#include "main.h"

static uint8_t sensor_bits = 0U;
static int16_t sensor_err = 0;
static uint8_t sensor_ok = 0U;
static uint8_t sensor_high_level_count = 0U;

void Sensor_Init(void)
{
    sensor_bits = 0U;
    sensor_err = 0;
    sensor_ok = 0U;
    sensor_high_level_count = 0U;
}

void Sensor_JC(void)
{
    sensor_bits = 0U;
    sensor_err = 0;
    sensor_high_level_count = 0U;

    if (GPIO_READ(LS1_PORT, LS1_PIN)) { sensor_high_level_count++; } else { sensor_bits += 128U; sensor_err += 999; }
    if (GPIO_READ(LS2_PORT, LS2_PIN)) { sensor_high_level_count++; } else { sensor_bits += 64U;  sensor_err += 888; }
    if (GPIO_READ(LS3_PORT, LS3_PIN)) { sensor_high_level_count++; } else { sensor_bits += 32U;  sensor_err += 666; }
    if (GPIO_READ(LS4_PORT, LS4_PIN)) { sensor_high_level_count++; } else { sensor_bits += 16U;  sensor_err += 333; }
    if (GPIO_READ(LS5_PORT, LS5_PIN)) { sensor_high_level_count++; } else { sensor_bits += 8U;   sensor_err -= 333; }
    if (GPIO_READ(LS6_PORT, LS6_PIN)) { sensor_high_level_count++; } else { sensor_bits += 4U;   sensor_err -= 666; }
    if (GPIO_READ(LS7_PORT, LS7_PIN)) { sensor_high_level_count++; } else { sensor_bits += 2U;   sensor_err -= 888; }
    if (GPIO_READ(LS8_PORT, LS8_PIN)) { sensor_high_level_count++; } else { sensor_bits += 1U;   sensor_err -= 999; }

    sensor_ok = (sensor_bits != 0U) ? 1U : 0U;
}

uint8_t Sensor_GetBits(void) { return sensor_bits; }
int16_t Sensor_GetError(void) { return sensor_err; }
uint8_t Sensor_IsValid(void) { return sensor_ok; }
uint8_t Sensor_GetHighLevelCount(void) { return sensor_high_level_count; }
