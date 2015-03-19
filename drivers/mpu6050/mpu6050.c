/*
 * Copyright (C) 2015 airfy GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup drivers_mpu6050
 * @{
 *
 * @file
 * @brief   Device driver implementation for the MPU-6050
 *
 * @author  Ludwig Ortmann <ludwig@airfy.com>
 *
 * @}
 */

#include "mpu6050.h"
#include "mpu6050-regs.h"
#include "periph/i2c.h"
#include "hwtimer.h"

#define ENABLE_DEBUG        (0)
#include "debug.h"

#define REG_RESET           (0x00)
#define MAX_VALUE           (0x7FFF)

/* Default config settings */
static const mpu6050_status_t DEFAULT_STATUS = {
    .accel_pwr = MPU6050_SENSOR_PWR_ON,
    .gyro_pwr = MPU6050_SENSOR_PWR_ON,
    .gyro_fsr = MPU6050_GYRO_FSR_250DPS,
    .accel_fsr = MPU6050_ACCEL_FSR_16G,
    .sample_rate = 0,
};

/**********************************************************************
 * internal API declaration
 **********************************************************************/
static void conf_lpf(mpu6050_t *dev, uint16_t rate);

/**********************************************************************
 * public API implementation
 **********************************************************************/

int mpu6050_init(mpu6050_t *dev, i2c_t i2c, mpu6050_hw_addr_t hw_addr)
{
    char temp;

    dev->i2c_dev = i2c;
    dev->hw_addr = hw_addr;
    dev->conf = DEFAULT_STATUS;

    /* Initialize I2C interface */
    if (i2c_init_master(dev->i2c_dev, I2C_SPEED_FAST)) {
        DEBUG("[Error] I2C device not enabled\n");
        return -1;
    }

    /* Acquire exclusive access */
    i2c_acquire(dev->i2c_dev);

    /* Reset MPU6050 registers and afterwards wake up the chip */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_RESET);
    hwtimer_wait(HWTIMER_TICKS(MPU6050_RESET_SLEEP_US));
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_WAKEUP);

    /* Release the bus, it is acquired again inside each function */
    i2c_release(dev->i2c_dev);

    /* Set default full scale ranges and sample rate */
    mpu6050_set_gyro_fsr(dev, MPU6050_GYRO_FSR_2000DPS);
    mpu6050_set_accel_fsr(dev, MPU6050_ACCEL_FSR_2G);
    mpu6050_set_sample_rate(dev, MPU6050_DEFAULT_SAMPLE_RATE);

    /* Disable interrupt generation */
    i2c_acquire(dev->i2c_dev);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_INT_ENABLE_REG, REG_RESET);

    /* Release the bus, it is acquired again inside each function */
    i2c_release(dev->i2c_dev);
    /* Enable all sensors */
    i2c_acquire(dev->i2c_dev);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_PLL);
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, &temp);
    temp &= ~(MPU6050_PWR_ACCEL | MPU6050_PWR_GYRO);
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, temp);
    i2c_release(dev->i2c_dev);
    hwtimer_wait(HWTIMER_TICKS(MPU6050_PWR_CHANGE_SLEEP_US));

    return 0;
}

int mpu6050_set_accel_power(mpu6050_t *dev, mpu6050_pwr_t pwr_conf)
{
    char pwr_2_setting;

    if (dev->conf.accel_pwr == pwr_conf) {
        return 0;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }

    /* Read current power management 2 configuration */
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, &pwr_2_setting);
    /* Prepare power register settings */
    if (pwr_conf == MPU6050_SENSOR_PWR_ON) {
        pwr_2_setting &= ~(MPU6050_PWR_ACCEL);
    }
    else {
        pwr_2_setting |= MPU6050_PWR_ACCEL;
    }
    /* Enable/disable accelerometer standby in power management 2 register */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    i2c_release(dev->i2c_dev);

    dev->conf.accel_pwr = pwr_conf;
    hwtimer_wait(HWTIMER_TICKS(MPU6050_PWR_CHANGE_SLEEP_US));

    return 0;
}

int mpu6050_set_gyro_power(mpu6050_t *dev, mpu6050_pwr_t pwr_conf)
{
    char pwr_2_setting;

    if (dev->conf.gyro_pwr == pwr_conf) {
        return 0;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }

    /* Read current power management 2 configuration */
    i2c_read_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, &pwr_2_setting);
    /* Prepare power register settings */
    if (pwr_conf == MPU6050_SENSOR_PWR_ON) {
        /* Set clock to pll */
        i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_PLL);
        pwr_2_setting &= ~(MPU6050_PWR_GYRO);
    }
    else {
        /* Configure power management 1 register */
        if (dev->conf.accel_pwr == MPU6050_SENSOR_PWR_OFF) {
            /* All sensors turned off, put the MPU-6050 to sleep */
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU6050_PWR_MGMT_1_REG, BIT_PWR_MGMT1_SLEEP);
        }
        else {
            /* Reset clock to internal oscillator */
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_WAKEUP);
        }
        pwr_2_setting |= MPU6050_PWR_GYRO;
    }
    /* Enable/disable gyroscope standby in power management 2 register */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_PWR_MGMT_2_REG, pwr_2_setting);

    /* Release the bus */
    i2c_release(dev->i2c_dev);

    dev->conf.gyro_pwr = pwr_conf;
    hwtimer_wait(HWTIMER_TICKS(MPU6050_PWR_CHANGE_SLEEP_US));

    return 0;
}

int mpu6050_read_gyro(mpu6050_t *dev, mpu6050_results_t *output)
{
    char data[6];
    int16_t temp;
    float fsr;

    switch (dev->conf.gyro_fsr) {
        case MPU6050_GYRO_FSR_250DPS:
            fsr = 250.0;
            break;
        case MPU6050_GYRO_FSR_500DPS:
            fsr = 500.0;
            break;
        case MPU6050_GYRO_FSR_1000DPS:
            fsr = 1000.0;
            break;
        case MPU6050_GYRO_FSR_2000DPS:
            fsr = 2000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw data */
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU6050_GYRO_START_REG, data, 6);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    /* Normalize data according to configured full scale range */
    temp = (data[0] << 8) | data[1];
    output->x_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[2] << 8) | data[3];
    output->y_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[4] << 8) | data[5];
    output->z_axis = (temp * fsr) / MAX_VALUE;

    return 0;
}

int mpu6050_read_accel(mpu6050_t *dev, mpu6050_results_t *output)
{
    char data[6];
    int16_t temp;
    float fsr;

    switch (dev->conf.accel_fsr) {
        case MPU6050_ACCEL_FSR_2G:
            fsr = 2000.0;
            break;
        case MPU6050_ACCEL_FSR_4G:
            fsr = 4000.0;
            break;
        case MPU6050_ACCEL_FSR_8G:
            fsr = 8000.0;
            break;
        case MPU6050_ACCEL_FSR_16G:
            fsr = 16000.0;
            break;
        default:
            return -2;
    }

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw data */
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU6050_ACCEL_START_REG, data, 6);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    /* Normalize data according to configured full scale range */
    temp = (data[0] << 8) | data[1];
    output->x_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[2] << 8) | data[3];
    output->y_axis = (temp * fsr) / MAX_VALUE;
    temp = (data[4] << 8) | data[5];
    output->z_axis = (temp * fsr) / MAX_VALUE;

    return 0;
}


int mpu6050_read_temperature(mpu6050_t *dev, int32_t *output)
{
    char data[2];
    int16_t temp;

    /* Acquire exclusive access */
    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    /* Read raw temperature value */
    i2c_read_regs(dev->i2c_dev, dev->hw_addr, MPU6050_TEMP_START_REG, data, 2);
    /* Release the bus */
    i2c_release(dev->i2c_dev);

    temp = (data[0] << 8) | data[1];
    *output = ((((int32_t)temp) * 1000) / 340) + (35*1000);

    return 0;
}

int mpu6050_set_gyro_fsr(mpu6050_t *dev, mpu6050_gyro_ranges_t fsr)
{
    if (dev->conf.gyro_fsr == fsr) {
        return 0;
    }

    switch (fsr) {
        case MPU6050_GYRO_FSR_250DPS:
        case MPU6050_GYRO_FSR_500DPS:
        case MPU6050_GYRO_FSR_1000DPS:
        case MPU6050_GYRO_FSR_2000DPS:
            if (i2c_acquire(dev->i2c_dev)) {
                return -1;
            }
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU6050_GYRO_CFG_REG, (char)(fsr << 3));
            i2c_release(dev->i2c_dev);
            dev->conf.gyro_fsr = fsr;
            break;
        default:
            return -2;
    }

    return 0;
}

int mpu6050_set_accel_fsr(mpu6050_t *dev, mpu6050_accel_ranges_t fsr)
{
    if (dev->conf.accel_fsr == fsr) {
        return 0;
    }

    switch (fsr) {
        case MPU6050_ACCEL_FSR_2G:
        case MPU6050_ACCEL_FSR_4G:
        case MPU6050_ACCEL_FSR_8G:
        case MPU6050_ACCEL_FSR_16G:
            if (i2c_acquire(dev->i2c_dev)) {
                return -1;
            }
            i2c_write_reg(dev->i2c_dev, dev->hw_addr,
                    MPU6050_ACCEL_CFG_REG, (char)(fsr << 3));
            i2c_release(dev->i2c_dev);
            dev->conf.accel_fsr = fsr;
            break;
        default:
            return -2;
    }

    return 0;
}

int mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t rate)
{
    uint8_t divider;

    if ((rate < MPU6050_MIN_SAMPLE_RATE) || (rate > MPU6050_MAX_SAMPLE_RATE)) {
        return -2;
    }
    else if (dev->conf.sample_rate == rate) {
        return 0;
    }

    /* Compute divider to achieve desired sample rate and write to rate div register */
    divider = (1000 / rate - 1);

    if (i2c_acquire(dev->i2c_dev)) {
        return -1;
    }
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_RATE_DIV_REG, (char) divider);

    /* Store configured sample rate */
    dev->conf.sample_rate = 1000 / (((uint16_t) divider) + 1);

    /* Always set LPF to a maximum of half the configured sampling rate */
    conf_lpf(dev, (dev->conf.sample_rate >> 1));
    i2c_release(dev->i2c_dev);

    return 0;
}

/**********************************************************************
 * internal API implementation
 **********************************************************************/

/**
 * Configure low pass filter
 * Caution: This internal function does not acquire exclusive access to the I2C bus.
 *          Acquisation and release is supposed to be handled by the calling function.
 */
static void conf_lpf(mpu6050_t *dev, uint16_t half_rate)
{
    mpu6050_lpf_t lpf_setting;

    /* Get target LPF configuration setting */
    if (half_rate >= 188) {
        lpf_setting = MPU6050_FILTER_188HZ;
    }
    else if (half_rate >= 98) {
        lpf_setting = MPU6050_FILTER_98HZ;
    }
    else if (half_rate >= 42) {
        lpf_setting = MPU6050_FILTER_42HZ;
    }
    else if (half_rate >= 20) {
        lpf_setting = MPU6050_FILTER_20HZ;
    }
    else if (half_rate >= 10) {
        lpf_setting = MPU6050_FILTER_10HZ;
    }
    else {
        lpf_setting = MPU6050_FILTER_5HZ;
    }

    /* Write LPF setting to configuration register */
    i2c_write_reg(dev->i2c_dev, dev->hw_addr, MPU6050_LPF_REG, (char)lpf_setting);
}
