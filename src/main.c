/*
 * main.c - RBC2026 Mainboard - Zephyr OS Port
 * Port tu STM32 HAL: RBC2026_MAINBOARD_OK
 *
 * Peripheral map:
 *   UART3 (PD8/PD9)  - HC12 thu khong day, async
 *   CAN1  (PD0/PD1)  - Gui lenh toc do motor, ID=0x123
 *   I2C1  (PB6/PB7)  - BNO055 IMU (Euler)
 *   I2C2  (PB10/B11) - PCA9685 servo driver
 *   ADC1  (PA0)      - Analog MUX x4 kenh (s0-s3 tren PA1-PA4)
 *   Timer 100Hz      - Vong dieu khien chinh (IMU + tinh motor + CAN TX)
 *   Timer 5Hz        - Watchdog ket noi HC12
 *   GPIO             - LED, output relay, input sensor
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "bno055_zephyr.h"
#include "bno055.h"
#include "pca9685.h"

/* ================================================================
 *  Device Tree specs
 * ================================================================ */
#define UART3_NODE  DT_NODELABEL(usart3)
static const struct device *uart3_dev = DEVICE_DT_GET(UART3_NODE);

#define CAN1_NODE   DT_NODELABEL(can1)
static const struct device *can1_dev = DEVICE_DT_GET(CAN1_NODE);

static const struct i2c_dt_spec bno_i2c = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = BNO055_I2C_ADDR,
};

static const struct i2c_dt_spec pca_i2c = {
    .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = PCA9685_I2C_ADDRESS_1 >> 1
};

#define ADC1_NODE   DT_NODELABEL(adc1)
static const struct device *adc1_dev = DEVICE_DT_GET(ADC1_NODE);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct gpio_dt_spec mux_s[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(s0), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s3), gpios),
};

/* ================================================================
 *  Defines
 * ================================================================ */
#define SERVOMIN  125
#define SERVOMAX  490

/* ================================================================
 *  Bien toan cuc
 * ================================================================ */
typedef struct {
    int goc_ht[2];
    int goc_trc[2];
    int delta_goc[2];
    int goc_tong[2];
    int goc_offset[2];
    int goc_ok[2];
} IMU_t;

static IMU_t BNO055_data;

/* HC12 */
static volatile uint8_t rx_hc12[5];
static volatile uint8_t id_hc12    = 0;
static volatile uint8_t check_hc12 = 0;
static volatile uint8_t pre_check  = 0;
static volatile uint8_t flag_ok    = 0;
static volatile uint8_t pre_bt3    = 0;

/* Dieu khien */
static volatile float X = 0, Y = 0;
static float pii  = 3.141592f;
static float pi4  = 0.7854f;
static float pi34 = 2.3562f;
static float rad_dh = 0, add_rad = 0;
static float goc_bu = 0;
static int   goc_quay = 0;
static uint8_t db = 9;

/* CAN */
static uint8_t TxData[8] = {100, 100, 100, 100, 100, 100, 100, 100};

/* ADC */
static volatile float adc_list[4];
static volatile uint8_t mux_channel = 0;

/* UART double-buffer */
/* UART RX - interrupt-driven, xu ly tung byte trong IRQ callback */
static uint8_t uart3_rx_buf[1]; /* Placeholder, khong dung trong IRQ mode */

/* ================================================================
 *  Ham tien ich
 * ================================================================ */
static uint8_t Chuan_hoa(float x)
{
    if (x >= 255.0f) x = 255.0f;
    if (x <= 0.0f)   x = 0.0f;
    return (uint8_t)roundf(x);
}

static uint16_t AngleToPWM(uint8_t angle)
{
    if (angle > 180) angle = 180;
    return (uint16_t)((float)SERVOMIN +
                      ((float)angle * (SERVOMAX - SERVOMIN) / 180.0f));
}

static void SetServoAngle(uint8_t num, float angle)
{
    uint16_t pwm = AngleToPWM((uint8_t)angle);
    PCA9685_SetPWM_Z(&pca_i2c, PCA9685_I2C_ADDRESS_1, num, 0, pwm);
}

static void MUX_Channel(uint8_t ch)
{
    gpio_pin_set_dt(&mux_s[0], (ch >> 0) & 1);
    gpio_pin_set_dt(&mux_s[1], (ch >> 1) & 1);
    gpio_pin_set_dt(&mux_s[2], (ch >> 2) & 1);
    gpio_pin_set_dt(&mux_s[3], (ch >> 3) & 1);
}

/* ================================================================
 *  Doc IMU
 * ================================================================ */
static void Read_IMU(void)
{
    bno055_vector_t d = bno055_getVectorEuler();

    BNO055_data.goc_ht[0] = (int)d.x;
    BNO055_data.delta_goc[0] = BNO055_data.goc_ht[0] - BNO055_data.goc_trc[0];
    if (BNO055_data.delta_goc[0] >  180) BNO055_data.delta_goc[0] -= 360;
    if (BNO055_data.delta_goc[0] < -180) BNO055_data.delta_goc[0] += 360;
    BNO055_data.goc_tong[0] += BNO055_data.delta_goc[0];
    BNO055_data.goc_trc[0]   = BNO055_data.goc_ht[0];

    BNO055_data.goc_ht[1] = (int)d.z;
    BNO055_data.delta_goc[1] = BNO055_data.goc_ht[1] - BNO055_data.goc_trc[1];
    if (BNO055_data.delta_goc[1] >  180) BNO055_data.delta_goc[1] -= 360;
    if (BNO055_data.delta_goc[1] < -180) BNO055_data.delta_goc[1] += 360;
    BNO055_data.goc_tong[1] += BNO055_data.delta_goc[1];
    BNO055_data.goc_trc[1]   = BNO055_data.goc_ht[1];
}

/* ================================================================
 *  Vong dieu khien 100Hz
 * ================================================================ */
static void control_loop_100Hz(void)
{
    Read_IMU();

    /* Button: rx_hc12[3] = du.button[0] (UP/DN/Triangle/Circle...)
     *         rx_hc12[2] = du.button[1] (L1/R1/L2/R2) */
    if (rx_hc12[3] == 8) {          /* UP (button[0] bit) */
        goc_bu -= 0.5f;
    } else if (rx_hc12[3] == 4) {   /* DN */
        goc_bu += 0.5f;
    } else {
        if      (rx_hc12[3] == 16)  add_rad = 0;
        else if (rx_hc12[3] == 32)  add_rad = 2 * pi4;
        else if (rx_hc12[3] == 64)  add_rad = pii;
        else if (rx_hc12[3] == 128) add_rad = -2 * pi4;
        pre_bt3 = rx_hc12[2];
        rad_dh += (add_rad - rad_dh) * 0.1f;

        goc_quay = BNO055_data.goc_tong[0] + (int)goc_bu
                   - (int)(rad_dh / pii * 180.0f);
        if (goc_quay >  20) goc_quay =  20;
        if (goc_quay < -20) goc_quay = -20;

        float goc_rad = ((goc_quay + BNO055_data.goc_tong[0] + goc_bu)
                         * pii) / 180.0f;

        TxData[2] = Chuan_hoa(X * sinf(goc_rad + pi34) + Y * cosf(goc_rad + pi34) + 100) + goc_quay;
        TxData[3] = Chuan_hoa(X * sinf(goc_rad - pi34) + Y * cosf(goc_rad - pi34) + 100) + goc_quay;
        TxData[4] = Chuan_hoa(X * sinf(goc_rad - pi4)  + Y * cosf(goc_rad - pi4)  + 100) + goc_quay;
        TxData[5] = Chuan_hoa(X * sinf(goc_rad + pi4)  + Y * cosf(goc_rad + pi4)  + 100) + goc_quay;
    }

    TxData[6] = db;

    struct can_frame frame = { .id = 0x123, .dlc = 8 };
    memcpy(frame.data, TxData, 8);
    int r = can_send(can1_dev, &frame, K_NO_WAIT, NULL, NULL);
    (void)r;
}

/* ================================================================
 *  Watchdog HC12 - 5Hz
 * ================================================================ */
static void watchdog_5Hz(void)
{
    if (check_hc12 != pre_check) {
        flag_ok = 0;
        gpio_pin_toggle_dt(&led0);
    } else {
        flag_ok = 1;
        X = 0;
        Y = 0;
    }
    pre_check = check_hc12;
}

/* ================================================================
 *  k_timer
 * ================================================================ */
static struct k_timer tim1_timer;
static struct k_timer tim8_timer;

static void tim1_work_handler(struct k_work *work);
static K_WORK_DEFINE(tim1_work, tim1_work_handler);
static void tim1_work_handler(struct k_work *work) { control_loop_100Hz(); }
static void tim1_expiry(struct k_timer *t)          { k_work_submit(&tim1_work); }

static void tim8_work_handler(struct k_work *work);
static K_WORK_DEFINE(tim8_work, tim8_work_handler);
static void tim8_work_handler(struct k_work *work) { watchdog_5Hz(); }
static void tim8_expiry(struct k_timer *t)          { k_work_submit(&tim8_work); }

/* ================================================================
 *  UART3 IRQ callback - HC12 (interrupt-driven, khong can DMA)
 * ================================================================ */
static void uart3_irq_cb(const struct device *dev, void *user_data)
{
    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t byte;
        int n = uart_fifo_read(dev, &byte, 1);
        if (n != 1) break;

        if (byte == 255) {
            /* Sync byte: reset index, bat dau frame moi */
            id_hc12 = 0;
        } else {
            if (id_hc12 < 5) {
                rx_hc12[id_hc12++] = byte;
            }
            if (id_hc12 >= 5) {
                id_hc12 = 0;
                check_hc12 = rx_hc12[4]; /* cnt - checkbyte */

                /* Packet sau sync byte: [val_x][val_y][btn1][btn0][cnt]
                 * val_x: 0-255, center=100
                 * val_y: 0-255, center=100 */
                if (flag_ok == 0) {
                    X =  (float)rx_hc12[0] - 100.0f;
                    Y = -(float)rx_hc12[1] + 100.0f;
                } else {
                    X = 0;
                    Y = 0;
                }
            }
        }
    }
}

/* ================================================================
 *  ADC MUX
 * ================================================================ */
static struct adc_channel_cfg adc_ch_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .differential     = 0,
};

static int16_t adc_raw_buf;
static struct adc_sequence adc_seq = {
    .channels    = BIT(0),
    .buffer      = &adc_raw_buf,
    .buffer_size = sizeof(adc_raw_buf),
    .resolution  = 12,
};

static void ADC_Read_MUX(void)
{
    MUX_Channel(mux_channel);
    k_usleep(100);
    if (adc_read(adc1_dev, &adc_seq) == 0) {
        adc_list[mux_channel] = (float)adc_raw_buf;
    }
    mux_channel = (mux_channel + 1) % 4;
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    /* GPIO LED - bat buoc dau tien */
    if (!device_is_ready(led0.port)) {
        while (1) {}
    }
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

    /* Blink 3 lan = board da boot */
    for (int i = 0; i < 3; i++) {
        gpio_pin_set_dt(&led0, 1);
        k_msleep(100);
        gpio_pin_set_dt(&led0, 0);
        k_msleep(100);
    }

    /* GPIO MUX */
    for (int i = 0; i < 4; i++) {
        gpio_pin_configure_dt(&mux_s[i], GPIO_OUTPUT_INACTIVE);
    }

    /* UART3 HC12 - interrupt driven */
    if (device_is_ready(uart3_dev)) {
        uart_irq_callback_user_data_set(uart3_dev, uart3_irq_cb, NULL);
        uart_irq_rx_enable(uart3_dev);
    }

    /* CAN1 */
    if (device_is_ready(can1_dev)) {
        const struct can_filter f = { .flags = 0, .id = 0, .mask = 0 };
        can_add_rx_filter(can1_dev, NULL, NULL, &f);
        can_start(can1_dev);
    }

    /* I2C1 BNO055 */
    if (device_is_ready(bno_i2c.bus)) {
        bno055_assignI2C(&bno_i2c);
        bno055_setup();
        bno055_setOperationModeNDOF();
    }

    /* I2C2 PCA9685 */
    if (device_is_ready(pca_i2c.bus)) {
        PCA9685_Init_Z(&pca_i2c, PCA9685_I2C_ADDRESS_1);
        PCA9685_SetPWMFreq_Z(&pca_i2c, PCA9685_I2C_ADDRESS_1, 50.0f);
    }

    /* ADC1 */
    if (device_is_ready(adc1_dev)) {
        adc_channel_setup(adc1_dev, &adc_ch_cfg);
    }

    TxData[0] = 100; TxData[1] = 100;
    k_msleep(1000);

    /* Timers */
    k_timer_init(&tim1_timer, tim1_expiry, NULL);
    k_timer_start(&tim1_timer, K_MSEC(10), K_MSEC(10));

    k_timer_init(&tim8_timer, tim8_expiry, NULL);
    k_timer_start(&tim8_timer, K_MSEC(200), K_MSEC(200));

    /* Main loop - blink de debug:
     *   Nhanh 100ms = dang nhan HC12
     *   Cham 500ms  = khong co tin hieu HC12
     */
    while (1) {
        gpio_pin_toggle_dt(&led0);
        /* flag_ok=0 khi dang nhan (check_hc12 thay doi lien tuc) */
        k_msleep(flag_ok == 0 ? 100 : 500);
        ADC_Read_MUX();
    }

    return 0;
}