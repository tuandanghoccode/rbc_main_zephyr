/*
 * bno055_zephyr.c
 * HAL binding cho BNO055 su dung Zephyr I2C API.
 * Tra ve int de caller biet loi I2C.
 */
#include "bno055_zephyr.h"
#include <zephyr/kernel.h>

static const struct i2c_dt_spec *_bno055_i2c = NULL;

void bno055_assignI2C(const struct i2c_dt_spec *i2c_dev)
{
    _bno055_i2c = i2c_dev;
}

void bno055_delay(int ms)
{
    k_msleep(ms);
}

/* Tra ve 0 neu thanh cong, am neu loi I2C */
void bno055_writeData(uint8_t reg, uint8_t data)
{
    if (!_bno055_i2c) return;
    uint8_t buf[2] = {reg, data};
    (void)i2c_write_dt(_bno055_i2c, buf, sizeof(buf));
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!_bno055_i2c) return;
    (void)i2c_write_read_dt(_bno055_i2c, &reg, 1, data, len);
}
