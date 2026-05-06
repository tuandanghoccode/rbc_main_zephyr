/*
 * pca9685_zephyr.c
 * Port PCA9685 driver tu HAL sang Zephyr I2C API
 *
 * Fix: Init them MODE1_AI + MODE2_OUTDRV, sua SetPWMFreq sequence
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
    /* Reset PCA9685 */
    PCA9685_Write8(i2c, addr, 0x00, MODE1_RESTART);
    k_msleep(10);

    /* MODE1: Auto-Increment ON (bat buoc de ghi nhieu register lien tiep)
     * Gia tri: 0x20 = AI bit. Khong set SLEEP -> chip bat dau chay */
    PCA9685_Write8(i2c, addr, 0x00, MODE1_AI);
    k_msleep(1);

    /* MODE2: Totem-pole output (OUTDRV=1) - can thiet cho servo
     * Neu khong set -> output la open-drain, servo khong nhan duoc tin hieu */
    PCA9685_Write8(i2c, addr, 0x01, MODE2_OUTDRV);
}

void PCA9685_SetPWMFreq_Z(const struct i2c_dt_spec *i2c, uint8_t addr,
                            float freq)
{
    /* Khong dung 0.9 compensation - SERVOMIN/SERVOMAX da calibrate cho 50Hz thuc */
    float prescaleval = FREQUENCY_OSCILLATOR / (freq * 4096.0f) - 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    /* Prescale chi co the ghi khi SLEEP=1 */
    uint8_t oldmode = PCA9685_Read8(i2c, addr, 0x00);

    /* Buoc 1: Vao SLEEP mode (giu AI neu dang bat) */
    uint8_t sleepmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    PCA9685_Write8(i2c, addr, 0x00, sleepmode);

    /* Buoc 2: Ghi prescale (chi hop le khi SLEEP=1) */
    PCA9685_Write8(i2c, addr, 0xFE, prescale);

    /* Buoc 3: Wake up - clear SLEEP, giu AI */
    uint8_t newmode = (oldmode & ~MODE1_SLEEP) | MODE1_AI;
    PCA9685_Write8(i2c, addr, 0x00, newmode);
    k_msleep(5);  /* doi oscillator on dinh (datasheet: 500us min) */

    /* Buoc 4: RESTART de bat dau PWM output */
    PCA9685_Write8(i2c, addr, 0x00, newmode | MODE1_RESTART);
}

void PCA9685_SetPWM_Z(const struct i2c_dt_spec *i2c, uint8_t addr,
                       uint8_t num, uint16_t on, uint16_t off)
{
    PCA9685_Write8(i2c, addr, 0x06 + 4 * num, on & 0xFF);
    PCA9685_Write8(i2c, addr, 0x07 + 4 * num, on >> 8);
    PCA9685_Write8(i2c, addr, 0x08 + 4 * num, off & 0xFF);
    PCA9685_Write8(i2c, addr, 0x09 + 4 * num, off >> 8);
}
