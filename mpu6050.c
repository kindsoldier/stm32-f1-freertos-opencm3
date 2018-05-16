
/*
  Original:MPU6050 lib 0x02, copyright (c) Davide Gironi, 2012
  Copyright (c) Oleg Borodin, 2018
  Code updated from http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <mpu6050.h>
#include <i2creg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
    void tim3_setup(void) {
        nvic_enable_irq(NVIC_TIM3_IRQ);

        timer_reset(TIM3);
        timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

        timer_direction_up(TIM3);
        timer_set_prescaler(TIM3, 50);
        timer_disable_preload(TIM3);
        timer_continuous_mode(TIM3);
        timer_set_period(TIM3, 28200);
        timer_set_oc_value(TIM3, TIM_OC1, 1);
        timer_enable_irq(TIM3, TIM_DIER_CC1IE);

        timer_enable_counter(TIM3);
    }

    void tim3_isr(void) {
        if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
            timer_clear_flag(TIM3, TIM_SR_CC1IF);
            mpu_update_quaternion(&mpu);
        }
    }
 */

inline void _delay_ms(uint16_t ms) {
    for (volatile int i = 0; i < ms * 800; i++)
        __asm__("nop");
}

inline void _delay_us(uint16_t ms) {
    for (volatile int i = 0; i < ms * 8; i++)
        __asm__("nop");
}

volatile uint8_t buffer[14];


void mpu_i2c_setup(void) {
    //nvic_enable_irq(NVIC_I2C2_EV_IRQ);
    //nvic_enable_irq(NVIC_I2C2_ER_IRQ);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                            GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                            GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    i2c_peripheral_disable(I2C1);
    i2c_reset(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, I2C_CR2_FREQ_36MHZ);
    i2c_peripheral_enable(I2C1);
}

void mpu_read_reg(mpu_t * mpu, uint8_t reg, uint8_t * data) {
    *data = i2c_read_reg(mpu->i2c, mpu->addr, reg);
}

int16_t mpu_read_seq(mpu_t * mpu, uint8_t reg, uint8_t * data, uint8_t len) {
    return i2c_read_seq(mpu->i2c, mpu->addr, reg, data, len);
}

void mpu_write_reg(mpu_t *mpu, uint8_t reg, uint8_t data) {
    i2c_write_reg(mpu->i2c, mpu->addr, reg, data);
}

void mpu_write_bits(mpu_t *mpu, uint8_t reg, uint8_t base, uint8_t len, uint8_t data) {
    i2c_set_bit_field(mpu->i2c, mpu->addr, reg, base, len, data);
}


void mpu_setup(mpu_t * mpu) {
    mpu_write_bits(mpu, MPU_REG_PWR_MGMT_1, MPU_PWR1_SLEEP_BIT, 1, 0);
    _delay_ms(10);

    mpu_write_bits(mpu, MPU_REG_PWR_MGMT_1,
                        MPU_PWR1_CLKSEL_BASE,
                        MPU_PWR1_CLKSEL_LEN, MPU_PWR1_CLKSEL_PLL_XGYRO);

    mpu_write_bits(mpu, MPU_REG_CONFIG, MPU_CFG_DLPF_CFG_BASE, MPU_CFG_DLPF_CFG_LEN, MPU_DLPF_BW_42);
    mpu_write_reg(mpu, MPU_REG_SMPLRT_DIV, 4);
    mpu_write_bits(mpu, MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_BASE, MPU_GYRO_FS_LEN, MPU_GYRO_FS);
    mpu_write_bits(mpu, MPU_REG_ACCEL_CONFIG, MPU_ACCEL_FS_BASE,  MPU_ACCEL_FS_LEN, MPU_ACCEL_FS);
}

void mpu_get_raw_data(mpu_t * mpu, int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz) {

    volatile uint8_t buffer[14];
    mpu_read_seq(mpu, MPU_REG_ACCEL_XOUT_H, (uint8_t *) buffer, 14);

    *ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    *az = (((int16_t) buffer[4]) << 8) | buffer[5];

    *gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}

void mpu_get_conv_data(mpu_t * mpu, double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds) {

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;

    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    mpu_get_raw_data(mpu, &ax, &ay, &az, &gx, &gy, &gz);

    *axg = ax / MPU_AGAIN;
    *ayg = ay / MPU_AGAIN;
    *azg = az / MPU_AGAIN;

    *gxds = gx / MPU_GGAIN;
    *gyds = gy / MPU_GGAIN;
    *gzds = gz / MPU_GGAIN;
}

/* Update quaternion */
void mpu_update_quaternion(mpu_t * mpu) {

    double axg = 0;
    double ayg = 0;
    double azg = 0;

    double gxrs = 0;
    double gyrs = 0;
    double gzrs = 0;

    mpu_get_conv_data(mpu, &axg, &ayg, &azg, &gxrs, &gyrs, &gzrs);

    gxrs *= 0.01745329;
    gyrs *= 0.01745329;
    gzrs *= 0.01745329;

    mpu_mahony_update(mpu, gxrs, gyrs, gzrs, axg, ayg, azg);
}


inline double inv_sqrt(double x) {
    return 1.0 / sqrt(x);
}

#define MPU_MAHONY_SAMPLE_FREQ  50 /* sample frequency in Hz */

/* IMU algorithm update */
void mpu_mahony_update(mpu_t * qn, double gx, double gy, double gz, double ax, double ay, double az) {

    double twoKp = (2.0 * 0.5);       /* 2 * proportional gain (Kp) */
    double twoKi = (2.0 * 0.0);       /* 2 * integral gain (Ki) */

    double recipNorm;
    double halfVx, halfVy, halfVz;
    double halfEx, halfEy, halfEz;
    double qa, qb, qc;


    /* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
    if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Estimated direction of gravity and vector perpendicular to magnetic flux */
        halfVx = qn->q1 * qn->q3 - qn->q0 * qn->q2;
        halfVy = qn->q0 * qn->q1 + qn->q2 * qn->q3;
        halfVz = qn->q0 * qn->q0 - 0.5L + qn->q3 * qn->q3;

        /* Error is sum of cross product between estimated and measured direction of gravity */
        halfEx = (ay * halfVz - az * halfVy);
        halfEy = (az * halfVx - ax * halfVz);
        halfEz = (ax * halfVy - ay * halfVx);

        /* Compute and apply integral feedback if enabled */
        if (twoKi > 0.0) {
            qn->integralFBx += twoKi * halfEx * (1.0 / MPU_MAHONY_SAMPLE_FREQ); /* Integral error scaled by Ki */
            qn->integralFBy += twoKi * halfEy * (1.0 / MPU_MAHONY_SAMPLE_FREQ);
            qn->integralFBz += twoKi * halfEz * (1.0 / MPU_MAHONY_SAMPLE_FREQ);
            gx += qn->integralFBx;      /* Apply integral feedback */
            gy += qn->integralFBy;
            gz += qn->integralFBz;
        } else {
            qn->integralFBx = 0.0;     /* Prevent integral windup */
            qn->integralFBy = 0.0;
            qn->integralFBz = 0.0;
        }

        /* Apply proportional feedback */
        gx += twoKp * halfEx;
        gy += twoKp * halfEy;
        gz += twoKp * halfEz;
    }

    /* Integrate rate of change of quaternion */
    gx *= (0.5 * (1.0 / MPU_MAHONY_SAMPLE_FREQ));     /* Pre-multiply common factors */
    gy *= (0.5 * (1.0 / MPU_MAHONY_SAMPLE_FREQ));
    gz *= (0.5 * (1.0 / MPU_MAHONY_SAMPLE_FREQ));

    qa = qn->q0;
    qb = qn->q1;
    qc = qn->q2;

    qn->q0 += (-qb * gx - qc * gy - qn->q3 * gz);
    qn->q1 += (qa * gx + qc * gz - qn->q3 * gy);
    qn->q2 += (qa * gy - qb * gz + qn->q3 * gx);
    qn->q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(qn->q0 * qn->q0 + qn->q1 * qn->q1 + qn->q2 * qn->q2 + qn->q3 * qn->q3);
    qn->q0 *= recipNorm;
    qn->q1 *= recipNorm;
    qn->q2 *= recipNorm;
    qn->q3 *= recipNorm;
}


/*
 * Get euler angles aerospace sequence, to obtain sensor attitude:
 * 1. Rotate around sensor Z plane by yaw
 * 2. Rotate around sensor Y plane by pitch
 * 3. Rotate around sensor X plane by roll
 */

void mpu_get_roll_pitch_yaw(mpu_t * qn, double *roll, double *pitch, double *yaw) {
    /* Roll (x-axis rotation) */
    double sinr = 2.0 * (qn->q0 * qn->q1 + qn->q2 * qn->q3);
    double cosr = 1.0 - 2.0 * (qn->q1 * qn->q1 + qn->q2 * qn->q2);
    *roll = atan2(sinr, cosr);

    /* Pitch (Y-axis rotation) */
    double sinp = 2.0 * (qn->q0 * qn->q2 - qn->q3 * qn->q1);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2.0, sinp);
    else
        *pitch = asin(sinp);

    /* Yaw (Z-axis rotation) */
    double siny = 2.0 * (qn->q0 * qn->q3 + qn->q1 * qn->q2);
    double cosy = 1.0 - 2.0 * (qn->q2 * qn->q2 + qn->q3 * qn->q3);
    *yaw = atan2(siny, cosy);
}

/* EOF */
