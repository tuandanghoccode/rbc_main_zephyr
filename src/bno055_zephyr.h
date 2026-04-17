/*
 * bno055_zephyr.h
 * Port lop HAL binding sang Zephyr I2C API.
 * Cac ham duoc khai bao trong bno055.h la non-static, nen day chi chua
 * include guards va khai bao extern. Dinh nghia thuc te trong bno055_zephyr.c
 */
#ifndef BNO055_ZEPHYR_H
#define BNO055_ZEPHYR_H

#include <zephyr/drivers/i2c.h>
#include "bno055.h"

/* Gan con tro I2C truoc khi goi bat ky ham bno055 nao */
void bno055_assignI2C(const struct i2c_dt_spec *i2c_dev);

#endif /* BNO055_ZEPHYR_H */
