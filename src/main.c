/*
 * main.c - RBC2026 Mainboard - Zephyr OS Port (Optimized)
 * Port tu STM32 HAL: RBC2026_MAINBOARD_OK
 *
 * Peripheral map:
 *   UART3 (PD8/PD9)  - HC12 thu khong day (IRQ-driven)
 *   CAN1  (PD0/PD1)  - Gui lenh toc do motor, ID=0x123, 500kbps
 *   I2C1  (PB6/PB7)  - BNO055 IMU 400kHz (Euler, ~200us/read)
 *   I2C2  (PB10/B11) - PCA9685 servo driver 400kHz
 *   ADC1  (PA0)      - Analog MUX x4 kenh (s0-s3 tren PA1-PA4)
 *
 * Thread architecture (do tre thap):
 *   ctrl_thread      prio=2, 100Hz: k_sem <- timer ISR -> no queue overhead
 *   watchdog_thread  prio=6,   5Hz: kiem tra ket noi HC12
 *   adc_thread       prio=8,  50Hz: doc ADC MUX, khong anh huong ctrl
 *   main (idle)      prio=14       : LED blink
 *
 * Data architecture:
 *   inp (Ctrl_Input_t)  - tat ca input: HC12 (atomic), IMU, ADC
 *   out (Ctrl_Output_t) - tat ca output: CAN frame, trang thai dieu khien
 *   De debug: chi can Watch "inp" va "out" la thay het
 *
 * ISR safety:
 *   inp.hc12.x10, .y10: atomic_t  - an toan chia se ISR <-> ctrl_thread
 *   inp.hc12.seq: atomic_t        - tang moi packet, watchdog so sanh
 */

#include <math.h>
#include <stdio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bno055.h"
#include "bno055_zephyr.h"
#include "pca9685.h"

/* ================================================================
 *  Device Tree specs
 * ================================================================ */
#define UART3_NODE DT_NODELABEL(usart3)
static const struct device *uart3_dev = DEVICE_DT_GET(UART3_NODE);

#define UART2_NODE DT_NODELABEL(usart2)
static const struct device *uart2_dev = DEVICE_DT_GET(UART2_NODE);

#define CAN1_NODE DT_NODELABEL(can1)
static const struct device *can1_dev = DEVICE_DT_GET(CAN1_NODE);

static const struct i2c_dt_spec bno_i2c = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = BNO055_I2C_ADDR,
};

static const struct i2c_dt_spec pca_i2c = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c2)),
    .addr = PCA9685_I2C_ADDRESS_1 >> 1};

#define ADC1_NODE DT_NODELABEL(adc1)
static const struct device *adc1_dev = DEVICE_DT_GET(ADC1_NODE);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
/* LED2 (PC15): nhay moi khi nhan duoc 1 packet HC12 tu tay dieu khien */
static const struct gpio_dt_spec led2 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led2), gpios);

static const struct gpio_dt_spec mux_s[] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(s0), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s1), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s2), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(s3), gpios),
};

/* ================================================================
 *  Defines
 * ================================================================ */
#define SERVOMIN 125
#define SERVOMAX 490

/* ================================================================
 *  Thread stacks & handles
 * ================================================================ */
#define CTRL_STACK_SIZE 2048
#define WATCHDOG_STACK_SIZE 512
#define ADC_STACK_SIZE 512
#define TELEM_STACK_SIZE 1024

#define CTRL_THREAD_PRIO 2
#define WATCHDOG_THREAD_PRIO 6
#define ADC_THREAD_PRIO 8
#define TELEM_THREAD_PRIO 10

K_THREAD_STACK_DEFINE(ctrl_stack, CTRL_STACK_SIZE);
K_THREAD_STACK_DEFINE(watchdog_stack, WATCHDOG_STACK_SIZE);
K_THREAD_STACK_DEFINE(adc_stack, ADC_STACK_SIZE);
K_THREAD_STACK_DEFINE(telem_stack, TELEM_STACK_SIZE);

static struct k_thread ctrl_thread_data;
static struct k_thread watchdog_thread_data;
static struct k_thread adc_thread_data;
static struct k_thread telem_thread_data;

/* ================================================================
 *  Semaphores - timer ISR wake threads (do tre thap nhat)
 * ================================================================ */
K_SEM_DEFINE(ctrl_sem, 0, 1);
K_SEM_DEFINE(watchdog_sem, 0, 1);
K_SEM_DEFINE(adc_sem, 0, 1);
K_SEM_DEFINE(telem_sem, 0, 1);

/* ================================================================
 *  Timers - chi goi k_sem_give (ISR-safe, khong queue overhead)
 * ================================================================ */
static struct k_timer ctrl_timer;
static struct k_timer watchdog_timer;
static struct k_timer adc_timer;
static struct k_timer telem_timer;

static void ctrl_timer_expiry(struct k_timer *t) {
  k_sem_give(&ctrl_sem); /* wake ctrl_thread ngay lap tuc */
}

static void watchdog_timer_expiry(struct k_timer *t) {
  k_sem_give(&watchdog_sem);
}

static void adc_timer_expiry(struct k_timer *t) { k_sem_give(&adc_sem); }

static void telem_timer_expiry(struct k_timer *t) {
  k_sem_give(&telem_sem); /* 10Hz telemetry wake */
}

/* ================================================================
 *  INPUT struct - tat ca du lieu dau vao gom lai 1 cho
 *  De debug: Watch "inp" la thay het input
 * ================================================================ */
typedef struct {
  /* --- HC12 wireless (atomic vi ISR ghi, thread doc) --- */
  struct {
    atomic_t x10;  /* X * 10, range ~ -1000..1000 */
    atomic_t y10;  /* Y * 10 */
    atomic_t btn3; /* button byte (rx_buf[2] = button[1]) */
    atomic_t btn2; /* button byte (rx_buf[3] = button[0]) */
    atomic_t seq;  /* tang moi packet nhan duoc */
    atomic_t conn; /* 1=connected, 0=mat ket noi */
  } hc12;

  /* --- IMU BNO055 (chi ctrl_thread doc/ghi) --- */
  struct {
    int goc_ht[2];
    int goc_trc[2];
    int delta_goc[2];
    int goc_tong[2];
  } imu;

  /* --- ADC MUX (adc_thread ghi, cac thread khac doc) --- */
  struct {
    volatile float val[4]; /* 4 kenh MUX */
    uint8_t ch;            /* kenh hien tai dang doc */
  } adc;
} Ctrl_Input_t;

static Ctrl_Input_t inp = {
    .hc12 =
        {
            .x10 = ATOMIC_INIT(0),
            .y10 = ATOMIC_INIT(0),
            .btn3 = ATOMIC_INIT(0),
            .btn2 = ATOMIC_INIT(0),
            .seq = ATOMIC_INIT(0),
            .conn = ATOMIC_INIT(0),
        },
    /* imu, adc zero-init tu dong */
};

/* ================================================================
 *  OUTPUT struct - tat ca du lieu dau ra + trang thai dieu khien
 *  De debug: Watch "out" la thay het output
 * ================================================================ */
static const float pii = 3.141592f;
static const float pi4 = 0.7854f;

typedef struct {
  /* --- CAN frame gui xuong PID boards --- */
  struct can_frame can_tx;

  /* --- Trang thai dieu khien (chi ctrl_thread ghi) --- */
  float rad_dh;  /* huong di chuyen hien tai (rad, lerpd) */
  float add_rad; /* huong di chuyen dich (rad) */
  float goc_bu;  /* bu goc */
  int goc_quay;  /* goc quay hien tai (clamped +-20) */
  uint8_t db;    /* he so nhan toc do, mac dinh 9 */

  /* --- Van toc hinh thang: gia tri hien tai (float de muot) --- */
  float cur_spd[4]; /* data[2..5] hien tai, khoi tao 100 */
} Ctrl_Output_t;

static Ctrl_Output_t out = {
    .can_tx =
        {
            .id = 0x123,
            .dlc = 8,
            .data = {100, 100, 100, 100, 100, 100, 100, 100},
        },
    .rad_dh = 0.0f,
    .add_rad = 0.0f,
    .goc_bu = 0.0f,
    .goc_quay = 0,
    .db = 15,
    .cur_spd = {100.0f, 100.0f, 100.0f, 100.0f},
};

/* ================================================================
 *  Ham tien ich
 * ================================================================ */

/* Toc do ramp: so don vi thay doi toi da moi cycle (100Hz = 10ms)
 * RAMP_STEP = 3 -> tu 100 den 200 mat 100/3 = ~33 cycles = 330ms
 * RAMP_STEP = 5 -> tu 100 den 200 mat 100/5 = 20 cycles = 200ms
 * Dieu chinh tuy y: tang = tang toc nhanh hon, giam = muot hon */
#define RAMP_STEP 2.7f

static inline float ramp_toward(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target)
      current = target;
  } else if (current > target) {
    current -= step;
    if (current < target)
      current = target;
  }
  return current;
}

static inline uint8_t f_clamp_u8(float x) {
  if (x >= 255.0f)
    return 255;
  if (x <= 0.0f)
    return 0;
  return (uint8_t)x;
}

static inline uint16_t AngleToPWM(uint8_t angle) {
  if (angle > 180)
    angle = 180;
  return (uint16_t)(SERVOMIN +
                    ((uint32_t)angle * (SERVOMAX - SERVOMIN) / 180U));
}

static inline void MUX_Channel(uint8_t ch) {
  gpio_pin_set_dt(&mux_s[0], (ch >> 0) & 1);
  gpio_pin_set_dt(&mux_s[1], (ch >> 1) & 1);
  gpio_pin_set_dt(&mux_s[2], (ch >> 2) & 1);
  gpio_pin_set_dt(&mux_s[3], (ch >> 3) & 1);
}

/* ================================================================
 *  Doc IMU - goi trong ctrl_thread (I2C1 @ 400kHz, ~200us)
 * ================================================================ */
static bool imu_ok = false; /* true khi BNO055 init thanh cong */

static void Read_IMU(void) {
  static bool first_read = true;
  bno055_vector_t d = bno055_getVectorEuler();

  if (first_read) {
    /* Lan doc dau: seed goc_trc de delta = 0, tranh goc_tong nhay */
    inp.imu.goc_trc[0] = (int)d.x;
    inp.imu.goc_trc[1] = (int)d.z;
    first_read = false;
    return;
  }

  inp.imu.goc_ht[0] = (int)d.x;
  inp.imu.delta_goc[0] = inp.imu.goc_ht[0] - inp.imu.goc_trc[0];
  if (inp.imu.delta_goc[0] > 180)
    inp.imu.delta_goc[0] -= 360;
  if (inp.imu.delta_goc[0] < -180)
    inp.imu.delta_goc[0] += 360;
  inp.imu.goc_tong[0] += inp.imu.delta_goc[0];
  inp.imu.goc_trc[0] = inp.imu.goc_ht[0];

  inp.imu.goc_ht[1] = (int)d.z;
  inp.imu.delta_goc[1] = inp.imu.goc_ht[1] - inp.imu.goc_trc[1];
  if (inp.imu.delta_goc[1] > 180)
    inp.imu.delta_goc[1] -= 360;
  if (inp.imu.delta_goc[1] < -180)
    inp.imu.delta_goc[1] += 360;
  inp.imu.goc_tong[1] += inp.imu.delta_goc[1];
  inp.imu.goc_trc[1] = inp.imu.goc_ht[1];
}

/* ================================================================
 *  ctrl_thread - 100Hz, priority 2 (cao nhat userspace)
 *  Wake: k_sem_give tu timer ISR, KHONG qua work queue
 * ================================================================ */
static void ctrl_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    /* Block cho den khi timer ISR keu - do tre ~10us */
    k_sem_take(&ctrl_sem, K_FOREVER);

    if (imu_ok) {
      Read_IMU();
    }

    /* Doc trang thai phim tu atomic (ISR da ghi) */
    uint8_t btn3 = (uint8_t)atomic_get(&inp.hc12.btn3);
    uint8_t btn2 = (uint8_t)atomic_get(&inp.hc12.btn2);
    bool connected = (atomic_get(&inp.hc12.conn) != 0);

    if (connected) {
      float X = (float)atomic_get(&inp.hc12.x10) / 10.0f;
      float Y = (float)atomic_get(&inp.hc12.y10) / 10.0f;

      // THO THUT + LEN XUONG (combo)
      if (btn3 == 12) { // UP + THO (8|4)
        out.can_tx.data[0] = 150;
        out.can_tx.data[1] = 0;
      } else if (btn3 == 3) { // DN + THUT (2|1)
        out.can_tx.data[0] = 50;
        out.can_tx.data[1] = 200;
      } else if (btn3 == 4) { // THO
        out.can_tx.data[0] = 200;
        out.can_tx.data[1] = 100;
      } else if (btn3 == 1) { // THUT
        out.can_tx.data[0] = 0;
        out.can_tx.data[1] = 100;
      } else if (btn3 == 8) { // LEN
        out.can_tx.data[0] = 100;
        out.can_tx.data[1] = 0;
      } else if (btn3 == 2) { // XUONG
        out.can_tx.data[0] = 100;
        out.can_tx.data[1] = 200;
      } else {
        out.can_tx.data[0] = 100;
        out.can_tx.data[1] = 100;
      }

      if (btn3 == 16)
        out.add_rad = 0.0f;
      else if (btn3 == 32)
        out.add_rad = 2.0f * pi4;
      else if (btn3 == 64)
        out.add_rad = pii;
      else if (btn3 == 128)
        out.add_rad = -2.0f * pi4;
      (void)btn2; /* reserved */

      out.rad_dh += (out.add_rad - out.rad_dh) * 0.1f;

      out.goc_quay = inp.imu.goc_tong[0] + (int)out.goc_bu -
                     (int)(out.rad_dh / pii * 180.0f);
      if (out.goc_quay > 20)
        out.goc_quay = 20;
      if (out.goc_quay < -20)
        out.goc_quay = -20;

      float goc_rad =
          ((out.goc_quay + inp.imu.goc_tong[0] + out.goc_bu) * pii) / 180.0f;

      /* Toi uu: chi goi sinf/cosf 1 lan, suy ra 4 banh
       * sin(a+135) =  sin(a)*cos135 + cos(a)*sin135 = (-sinA + cosA)*K
       * cos(a+135) =  cos(a)*cos135 - sin(a)*sin135 = (-cosA - sinA)*K
       * sin(a-135) = (-sinA - cosA)*K, cos(a-135) = (-cosA + sinA)*K
       * sin(a-45)  = ( sinA - cosA)*K, cos(a-45)  = ( cosA + sinA)*K
       * sin(a+45)  = ( sinA + cosA)*K, cos(a+45)  = ( cosA - sinA)*K
       * voi K = 0.7071f (sin45 = cos45)
       */
      float sG = sinf(goc_rad);
      float cG = cosf(goc_rad);
      static const float K = 0.7071f;

      /* Banh 1 (ID3): goc_rad + 135° */
      float s135 = (-sG + cG) * K;
      float c135 = (-cG - sG) * K;
      /* Banh 2 (ID4): goc_rad - 135° */
      float sm135 = (-sG - cG) * K;
      float cm135 = (-cG + sG) * K;
      /* Banh 3 (ID5): goc_rad - 45° */
      float sm45 = (sG - cG) * K;
      float cm45 = (cG + sG) * K;
      /* Banh 4 (ID6): goc_rad + 45° */
      float s45 = (sG + cG) * K;
      float c45 = (cG - sG) * K;

      /* === Van toc hinh thang: tinh target, ramp tung buoc ===
       * Thay vi gan truc tiep data[2-5], ramp tu cur_spd[i] den target
       * moi cycle chi thay doi toi da RAMP_STEP don vi
       * -> tang toc muot (acceleration) va giam toc muot (deceleration)
       */
      float target[4];
      /* Banh data[2]: goc_rad - 45° */
      target[0] = X * sm45 + Y * cm45 + 100.0f + (float)out.goc_quay;
      /* Banh data[3]: goc_rad + 45° */
      target[1] = X * s45 + Y * c45 + 100.0f + (float)out.goc_quay;
      /* Banh data[4]: goc_rad + 135° */
      target[2] = X * s135 + Y * c135 + 100.0f + (float)out.goc_quay;
      /* Banh data[5]: goc_rad - 135° */
      target[3] = X * sm135 + Y * cm135 + 100.0f + (float)out.goc_quay;

      for (int i = 0; i < 4; i++) {
        out.cur_spd[i] = ramp_toward(out.cur_spd[i], target[i], RAMP_STEP);
        out.can_tx.data[2 + i] = f_clamp_u8(out.cur_spd[i]);
      }

      /* --- Code cu (gan truc tiep, khong ramp) ---
      out.can_tx.data[2] =
          f_clamp_u8(X * sm45 + Y * cm45 + 100.0f + (float)out.goc_quay);
      out.can_tx.data[3] =
          f_clamp_u8(X * s45 + Y * c45 + 100.0f + (float)out.goc_quay);
      out.can_tx.data[4] =
          f_clamp_u8(X * s135 + Y * c135 + 100.0f + (float)out.goc_quay);
      out.can_tx.data[5] =
          f_clamp_u8(X * sm135 + Y * cm135 + 100.0f + (float)out.goc_quay);
      */

      out.can_tx.data[6] = out.db;

      // TxData[2]=Chuan_hoa(X*sin(goc_rad-pi4) + Y*cos(goc_rad-pi4)+100 +
      // goc_quay); TxData[3]=Chuan_hoa(X*sin(goc_rad+pi4) +
      // Y*cos(goc_rad+pi4)+100 + goc_quay); TxData[4]=Chuan_hoa(
      // X*sin(goc_rad+pi34) + Y*cos(goc_rad+pi34)+100 + goc_quay);
      // TxData[5]=Chuan_hoa( X*sin(goc_rad-pi34) + Y*cos(goc_rad-pi34)+100 +
      // goc_quay);

    } else {
      /* === MAT KET NOI: dung tat ca motor === */
      /* Reset btn de khong giu trang thai cu */
      atomic_set(&inp.hc12.btn3, 0);
      atomic_set(&inp.hc12.btn2, 0);
      /* Reset goc quay ve 0 de khi ket noi lai khong bi giat */
      out.goc_quay = 0;
      out.add_rad = 0.0f;
      out.rad_dh = 0.0f;

      /* Ramp ve 100 (neutral) khi mat ket noi - van muot */
      for (int i = 0; i < 4; i++) {
        out.cur_spd[i] = ramp_toward(out.cur_spd[i], 100.0f, RAMP_STEP);
        out.can_tx.data[2 + i] = f_clamp_u8(out.cur_spd[i]);
      }
      out.can_tx.data[0] = 100;
      out.can_tx.data[1] = 100;
      /* data[6] = 0 -> Setpoint = (0-7.5)*(data-100) nhung data=100
       * -> Setpoint = 0 bat ke he so nhan. An toan hon de db */
      out.can_tx.data[6] = 0;
      out.can_tx.data[7] = 100;
    }

    /* CAN TX - non-blocking (K_NO_WAIT), khong doi ACK */
    can_send(can1_dev, &out.can_tx, K_NO_WAIT, NULL, NULL);
  }
}

/* ================================================================
 *  watchdog_thread - 5Hz, priority 6
 *  Kiem tra hc12_seq thay doi -> conn/lost
 * ================================================================ */
static void watchdog_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  atomic_val_t prev_seq = atomic_get(&inp.hc12.seq);

  while (1) {
    k_sem_take(&watchdog_sem, K_FOREVER);

    atomic_val_t cur_seq = atomic_get(&inp.hc12.seq);

    if (cur_seq != prev_seq) {
      /* Dang nhan duoc packet */
      atomic_set(&inp.hc12.conn, 1);
    } else {
      /* mat ket noi */
      atomic_set(&inp.hc12.conn, 0);
      atomic_set(&inp.hc12.x10, 0);
      atomic_set(&inp.hc12.y10, 0);
    }
    prev_seq = cur_seq;
  }
}

/* ================================================================
 *  adc_thread - 50Hz, priority 8
 *  Tach khoi ctrl_thread -> khong anh huong control loop
 * ================================================================ */
static struct adc_channel_cfg adc_ch_cfg = {
    .gain = ADC_GAIN_1,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = 0,
    .differential = 0,
};

static int16_t adc_raw_buf;
static struct adc_sequence adc_seq = {
    .channels = BIT(0),
    .buffer = &adc_raw_buf,
    .buffer_size = sizeof(adc_raw_buf),
    .resolution = 12,
};

static void adc_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    k_sem_take(&adc_sem, K_FOREVER);

    MUX_Channel(inp.adc.ch);
    k_usleep(100); /* cho MUX on dinh */

    if (adc_read(adc1_dev, &adc_seq) == 0) {
      inp.adc.val[inp.adc.ch] = (float)adc_raw_buf;
    }
    inp.adc.ch = (inp.adc.ch + 1) & 0x03;
  }
}

/* ================================================================
 *  uart2_send_str - gui chuoi qua UART2 (blocking tung byte)
 * ================================================================ */
static void uart2_send_str(const char *s) {
  while (*s) {
    uart_poll_out(uart2_dev, *s++);
  }
}

/* ================================================================
 *  telem_thread - 10Hz, priority 10
 *  Gui telemetry qua UART2 (PD5/PD6) format CSV:
 *  $X,Y,HEADING,M2,M3,M4,M5,CONN\n
 *  Vi du: $-35.0,20.0,123,145,132,118,140,1\n
 * ================================================================ */
static void telem_thread_fn(void *p1, void *p2, void *p3) {
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  char buf[80];

  while (1) {
    k_sem_take(&telem_sem, K_FOREVER);

    int x10 = (int)atomic_get(&inp.hc12.x10);
    int y10 = (int)atomic_get(&inp.hc12.y10);
    bool conn = (atomic_get(&inp.hc12.conn) != 0);
    int hdg = inp.imu.goc_ht[0]; /* heading 0-359 */

    /* Integer print: nhanh hon snprintf float 10-50x */
    int len = snprintf(buf, sizeof(buf), "$%d.%d,%d.%d,%d,%d,%d,%d,%d,%d\n",
                       x10 / 10, (x10 < 0 ? -x10 : x10) % 10, y10 / 10,
                       (y10 < 0 ? -y10 : y10) % 10, hdg, out.can_tx.data[2],
                       out.can_tx.data[3], out.can_tx.data[4],
                       out.can_tx.data[5], conn ? 1 : 0);

    if (len > 0 && len < (int)sizeof(buf)) {
      uart2_send_str(buf);
    }
  }
}

/* ================================================================
 *  UART3 IRQ callback - HC12
 *  Chi ghi vao atomic -> nhanh nhat co the, khong float, khong lock
 * ================================================================ */
static void uart3_irq_cb(const struct device *dev, void *user_data) {
  static uint8_t rx_buf[5];
  static uint8_t rx_idx = 0;

  if (!uart_irq_update(dev)) {
    return;
  }

  while (uart_irq_rx_ready(dev)) {
    uint8_t byte;
    if (uart_fifo_read(dev, &byte, 1) != 1) {
      break;
    }

    if (byte == 255) {
      /* Sync byte */
      rx_idx = 0;
    } else {
      if (rx_idx < 5) {
        rx_buf[rx_idx++] = byte;
      }
      if (rx_idx >= 5) {
        rx_idx = 0;

        /* Ghi atomic: int*10 tranh float ISR */
        /* Code cu: Y=rx_hc12[0]-100, X=-rx_hc12[1]+100 */
        atomic_set(&inp.hc12.y10, (atomic_val_t)((int)rx_buf[0] - 100) * 10);
        atomic_set(&inp.hc12.x10, (atomic_val_t)(-(int)rx_buf[1] + 100) * 10);
        /* Code cu: kiem tra rx_hc12[2] = buf[2] = button[1] */
        atomic_set(&inp.hc12.btn3, rx_buf[2]);
        atomic_set(&inp.hc12.btn2, rx_buf[3]);
        atomic_inc(&inp.hc12.seq); /* tang sequence -> watchdog phat hien */

        /* LED2 debug: nhay moi packet nhan duoc */
        gpio_pin_toggle_dt(&led2);
      }
    }
  }
}

/* ================================================================
 *  MAIN - khoi tao hardware, start threads, LED blink loop
 * ================================================================ */
int main(void) {
  /* --- GPIO LED --- */
  if (!device_is_ready(led0.port)) {
    while (1) {
    }
  }
  gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

  /* Blink 3 lan = boot OK */
  for (int i = 0; i < 3; i++) {
    gpio_pin_set_dt(&led0, 1);
    k_msleep(100);
    gpio_pin_set_dt(&led0, 0);
    k_msleep(100);
  }

  /* --- GPIO MUX --- */
  for (int i = 0; i < 4; i++) {
    gpio_pin_configure_dt(&mux_s[i], GPIO_OUTPUT_INACTIVE);
  }

  /* --- LED1: HC12 status --- */
  if (device_is_ready(led1.port)) {
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
  }

  /* --- LED2: debug packet HC12 --- */
  if (device_is_ready(led2.port)) {
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    /* Blink LED2 x5 khi boot -> xac nhan LED2 hardware OK */
    for (int i = 0; i < 5; i++) {
      gpio_pin_set_dt(&led2, 1);
      k_msleep(80);
      gpio_pin_set_dt(&led2, 0);
      k_msleep(80);
    }
  }

  /* --- UART3 HC12 --- */
  if (device_is_ready(uart3_dev)) {
    uart_irq_callback_user_data_set(uart3_dev, uart3_irq_cb, NULL);
    uart_irq_rx_enable(uart3_dev);
    gpio_pin_set_dt(&led1, 1); /* LED1 ON = UART3 init OK */
  } else {
    /* LED1 tat = UART3 fail -> kiem tra DTS */
    gpio_pin_set_dt(&led1, 0);
  }

  /* --- CAN1 --- */
  if (device_is_ready(can1_dev)) {
    can_start(can1_dev);
  }

  /* --- I2C1 BNO055 --- */
  if (device_is_ready(bno_i2c.bus)) {
    bno055_assignI2C(&bno_i2c);
    bno055_setup();
    /* Verify BNO055 co that su gan: doc chip ID */
    uint8_t chip_id = 0;
    bno055_readData(BNO055_CHIP_ID, &chip_id, 1);
    if (chip_id == BNO055_ID) {
      bno055_setOperationModeNDOF();
      imu_ok = true;
    }
    /* Neu chip_id != 0xA0 -> khong co BNO055, imu_ok = false, skip Read_IMU */
  }

  /* --- I2C2 PCA9685 --- */
  if (device_is_ready(pca_i2c.bus)) {
    PCA9685_Init_Z(&pca_i2c, PCA9685_I2C_ADDRESS_1);
    PCA9685_SetPWMFreq_Z(&pca_i2c, PCA9685_I2C_ADDRESS_1, 50.0f);
  }

  /* --- ADC1 --- */
  if (device_is_ready(adc1_dev)) {
    adc_channel_setup(adc1_dev, &adc_ch_cfg);
  }

  k_msleep(500); /* cho BNO055 on dinh sau NDOF mode */

  /* --- Start worker threads --- */
  k_thread_create(&ctrl_thread_data, ctrl_stack, CTRL_STACK_SIZE,
                  ctrl_thread_fn, NULL, NULL, NULL, CTRL_THREAD_PRIO, 0,
                  K_NO_WAIT);
  k_thread_name_set(&ctrl_thread_data, "ctrl");

  k_thread_create(&watchdog_thread_data, watchdog_stack, WATCHDOG_STACK_SIZE,
                  watchdog_thread_fn, NULL, NULL, NULL, WATCHDOG_THREAD_PRIO, 0,
                  K_NO_WAIT);
  k_thread_name_set(&watchdog_thread_data, "watchdog");

  k_thread_create(&adc_thread_data, adc_stack, ADC_STACK_SIZE, adc_thread_fn,
                  NULL, NULL, NULL, ADC_THREAD_PRIO, 0, K_NO_WAIT);
  k_thread_name_set(&adc_thread_data, "adc");

  k_thread_create(&telem_thread_data, telem_stack, TELEM_STACK_SIZE,
                  telem_thread_fn, NULL, NULL, NULL, TELEM_THREAD_PRIO, 0,
                  K_NO_WAIT);
  k_thread_name_set(&telem_thread_data, "telem");

  /* --- Start timers (sau khi threads da san sang) --- */
  k_timer_init(&ctrl_timer, ctrl_timer_expiry, NULL);
  k_timer_init(&watchdog_timer, watchdog_timer_expiry, NULL);
  k_timer_init(&adc_timer, adc_timer_expiry, NULL);
  k_timer_init(&telem_timer, telem_timer_expiry, NULL);

  k_timer_start(&ctrl_timer, K_MSEC(10), K_MSEC(10));       /* 100Hz */
  k_timer_start(&watchdog_timer, K_MSEC(200), K_MSEC(200)); /*   5Hz */
  k_timer_start(&adc_timer, K_MSEC(20), K_MSEC(20));        /*  50Hz */
  k_timer_start(&telem_timer, K_MSEC(100), K_MSEC(100));    /*  10Hz */

  /* --- Main loop: LED blink (priority 14, thap nhat) ---
   *   Nhanh 100ms = dang nhan HC12
   *   Cham 500ms  = mat tin hieu
   */
  while (1) {
    gpio_pin_toggle_dt(&led0);
    bool conn = (atomic_get(&inp.hc12.conn) != 0);
    k_msleep(conn ? 100 : 500);
  }

  return 0;
}