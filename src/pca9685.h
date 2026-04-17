/*
 * pca9685.h - Ported to Zephyr I2C API
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include <zephyr/drivers/i2c.h>

/* PCA9685 I2C address (7-bit): 0x40. Trong HAL la 0x80 (shift left 1) */
#define PCA9685_I2C_ADDRESS_1  0x80  /* Giu de tuong thich, Zephyr se >> 1 */
#define FREQUENCY_OSCILLATOR   25000000

#define MODE1_RESTART  0x80
#define MODE1_SLEEP    0x10
#define MODE1_AI       0x20
#define MODE1_EXTCLK   0x40
#define MODE2_OUTDRV   0x04
#define PRESCALE_MIN   3
#define PRESCALE_MAX   255

/* Ham Zephyr (su dung i2c_dt_spec) */
void PCA9685_Init_Z(const struct i2c_dt_spec *i2c, uint8_t addr);
void PCA9685_SetPWMFreq_Z(const struct i2c_dt_spec *i2c, uint8_t addr, float freq);
void PCA9685_SetPWM_Z(const struct i2c_dt_spec *i2c, uint8_t addr,
                       uint8_t num, uint16_t on, uint16_t off);

#endif /* PCA9685_H_ */
