#include "iic.h"
#include "main.h"

/* Software I2C GPIO macros using DL_GPIO */
#define IIC_SCL_HIGH  DL_GPIO_setPins(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SCL_LOW   DL_GPIO_clearPins(IIC_SCL_PORT, IIC_SCL_PIN)
#define IIC_SDA_HIGH  DL_GPIO_setPins(IIC_SDA_PORT, IIC_SDA_PIN)
#define IIC_SDA_LOW   DL_GPIO_clearPins(IIC_SDA_PORT, IIC_SDA_PIN)
#define READ_SDA      (DL_GPIO_readPins(IIC_SDA_PORT, IIC_SDA_PIN) ? 1U : 0U)

static void delay_us(uint32_t us) {
    uint32_t count = us * (CPUCLK_FREQ / 1000000 / 4);
    volatile uint32_t i = 0;
    while (count--) { i++; }
}

void IIC_Init(void)
{
    /* Configure SCL and SDA as open-drain output with pull-up and input enable */
    DL_GPIO_initDigitalOutputFeatures(IOMUX_PINCM15,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_initDigitalOutputFeatures(IOMUX_PINCM16,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    /* Enable input so SDA can be read back */
    IOMUX->SECCFG.PINCM[IOMUX_PINCM15] |= IOMUX_PINCM_INENA_ENABLE;
    IOMUX->SECCFG.PINCM[IOMUX_PINCM16] |= IOMUX_PINCM_INENA_ENABLE;
    DL_GPIO_enableOutput(IIC_SCL_PORT, IIC_SCL_PIN);
    DL_GPIO_enableOutput(IIC_SDA_PORT, IIC_SDA_PIN);

    IIC_SCL_HIGH;
    IIC_SDA_HIGH;
}

void IIC_Start(void)
{
    IIC_SDA_HIGH;
    IIC_SCL_HIGH;
    delay_us(4);
    IIC_SDA_LOW;
    delay_us(4);
    IIC_SCL_LOW;
}

void IIC_Stop(void)
{
    IIC_SCL_LOW;
    IIC_SDA_LOW;
    delay_us(4);
    IIC_SCL_HIGH;
    IIC_SDA_HIGH;
    delay_us(4);
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ack = 0;

    IIC_SDA_HIGH;
    delay_us(2);
    IIC_SCL_HIGH;
    delay_us(2);

    if (READ_SDA) {
        ack = 1;
    }

    IIC_SCL_LOW;
    delay_us(2);
    return ack;
}

void IIC_Ack(void)
{
    IIC_SCL_LOW;
    IIC_SDA_LOW;
    delay_us(2);
    IIC_SCL_HIGH;
    delay_us(2);
    IIC_SCL_LOW;
}

void IIC_NAck(void)
{
    IIC_SCL_LOW;
    IIC_SDA_HIGH;
    delay_us(2);
    IIC_SCL_HIGH;
    delay_us(2);
    IIC_SCL_LOW;
}

void IIC_Send_Byte(uint8_t data)
{
    uint8_t i;
    IIC_SCL_LOW;

    for (i = 0; i < 8; i++) {
        if (data & 0x80) {
            IIC_SDA_HIGH;
        } else {
            IIC_SDA_LOW;
        }
        data <<= 1;
        delay_us(2);
        IIC_SCL_HIGH;
        delay_us(2);
        IIC_SCL_LOW;
        delay_us(2);
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, data = 0;

    IIC_SDA_HIGH;

    for (i = 0; i < 8; i++) {
        IIC_SCL_LOW;
        delay_us(2);
        IIC_SCL_HIGH;
        data <<= 1;
        if (READ_SDA) {
            data |= 0x01;
        }
        delay_us(1);
    }

    if (ack) {
        IIC_Ack();
    } else {
        IIC_NAck();
    }

    return data;
}
