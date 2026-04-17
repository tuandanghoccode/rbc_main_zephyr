/*
 * pca9685_zephyr.c
 * Port PCA9685 driver tu HAL sang Zephyr I2C API
 */
#include "pca9685.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

static void PCA9685_Write8(const struct i2c_dt_spec *i2c, uint8_t addr,
                            uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    /* addr trong HAL la 8-bit (7-bit << 1), trong Zephyr i2c_dt_spec da co addr 7-bit */
    (void)addr; /* Dung addr trong i2c_dt_spec */
    i2c_write_dt(i2c, buf, sizeof(buf));
}

static uint8_t PCA9685_Read8(const struct i2c_dt_spec *i2c, uint8_t addr,
                               uint8_t reg)
{
    uint8_t result = 0;
    (void)addr;
    i2c_write_read_dt(i2c, &reg, 1, &result, 1);
    return result;
}

void PCA9685_Init_Z(const struct i2c_dt_spec *i2c, uint8_t addr)
{
    PCA9685_Write8(i2c, addr, 0x00, MODE1_RESTART);
    k_msleep(10);
}

void PCA9685_SetPWMFreq_Z(const struct i2c_dt_spec *i2c, uint8_t addr,
                            float freq)
{
    freq *= 0.9f;
    float prescaleval = FREQUENCY_OSCILLATOR / (freq * 4096.0f) - 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    uint8_t oldmode = PCA9685_Read8(i2c, addr, 0x00);
    uint8_t sleepmode = oldmode | MODE1_SLEEP;
    PCA9685_Write8(i2c, addr, 0x00, sleepmode);
    PCA9685_Write8(i2c, addr, 0xFE, prescale);
    PCA9685_Write8(i2c, addr, 0x00, oldmode);
    k_msleep(1);
    PCA9685_Write8(i2c, addr, 0x00, oldmode | MODE1_RESTART);
}

void PCA9685_SetPWM_Z(const struct i2c_dt_spec *i2c, uint8_t addr,
                       uint8_t num, uint16_t on, uint16_t off)
{
    PCA9685_Write8(i2c, addr, 0x06 + 4 * num, on & 0xFF);
    PCA9685_Write8(i2c, addr, 0x07 + 4 * num, on >> 8);
    PCA9685_Write8(i2c, addr, 0x08 + 4 * num, off & 0xFF);
    PCA9685_Write8(i2c, addr, 0x09 + 4 * num, off >> 8);
}
