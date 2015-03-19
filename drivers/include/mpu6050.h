/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mpu6050 MPU-6050
 * @ingroup     drivers
 * @brief       Device driver for the MPU-6050 6-Axis Motion Sensor
 *
 * ### Addressing:
 * If the device is wired with AD0 low, MPU6050_HW_ADDR_HEX_68 needs to be
 * given, MPU6050_HW_ADDR_HEX_69 if high.
 *
 * @{
 *
 * @file
 * @brief   Device driver interface for the MPU-6050
 *
 * @author  Fabian Nack <nack@inf.fu-berlin.de>
 * @author  Ludwig Ortmann <ludwig@airfy.com>
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "periph/i2c.h"
#include "periph/gpio.h"
#include "msg.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @name Message type definitions
 * @{
 */
#ifndef MPU6050_MSG_START
#define MPU6050_MSG_START 160
#endif

#define MPU6050_MSG_INT             (MPU6050_MSG_START + 0)
/** @} */

/**
 * @name Sample rate macro definitions
 * @{
 */
#define MPU6050_MIN_SAMPLE_RATE     (4)
#define MPU6050_MAX_SAMPLE_RATE     (1000)
#define MPU6050_DEFAULT_SAMPLE_RATE (50)
/** @} */

/**
 * @name Power Management 1 register macros
 * @{
 */
#define MPU6050_PWR_WAKEUP          (0x00)
#define MPU6050_PWR_PLL             (0x01)
#define MPU6050_PWR_RESET           (0x80)
/** @} */

/**
 * @name Power Management 2 register macros
 * @{
 */
#define MPU6050_PWR_GYRO            (0x07)
#define MPU6050_PWR_ACCEL           (0x38)
/** @} */

/**
 * @name Sleep times in microseconds
 * @{
 */
#define MPU6050_BYPASS_SLEEP_US     (3000)
#define MPU6050_PWR_CHANGE_SLEEP_US (50000)
#define MPU6050_RESET_SLEEP_US      (100000)
/** @} */

/**
 * @brief Power enum values
 */
typedef enum {
    MPU6050_SENSOR_PWR_OFF = 0x00,
    MPU6050_SENSOR_PWR_ON = 0x01,
} mpu6050_pwr_t;

/**
 * @brief Possible MPU-6050 hardware addresses (wiring specific)
 */
typedef enum {
    MPU6050_HW_ADDR_HEX_68 = 0x68,
    MPU6050_HW_ADDR_HEX_69 = 0x69,
} mpu6050_hw_addr_t;

/**
 * @brief Possible full scale ranges for the gyroscope
 */
typedef enum {
    MPU6050_GYRO_FSR_250DPS = 0x00,
    MPU6050_GYRO_FSR_500DPS = 0x01,
    MPU6050_GYRO_FSR_1000DPS = 0x02,
    MPU6050_GYRO_FSR_2000DPS = 0x03,
} mpu6050_gyro_ranges_t;

/**
 * @brief Possible full scale ranges for the accelerometer
 */
typedef enum {
    MPU6050_ACCEL_FSR_2G = 0x00,
    MPU6050_ACCEL_FSR_4G = 0x01,
    MPU6050_ACCEL_FSR_8G = 0x02,
    MPU6050_ACCEL_FSR_16G = 0x03,
} mpu6050_accel_ranges_t;

/**
 * @brief Possible low pass filter values
 */
typedef enum {
    MPU6050_FILTER_188HZ    = 0x01,
    MPU6050_FILTER_98HZ     = 0x02,
    MPU6050_FILTER_42HZ     = 0x03,
    MPU6050_FILTER_20HZ     = 0x04,
    MPU6050_FILTER_10HZ     = 0x05,
    MPU6050_FILTER_5HZ      = 0x06,
} mpu6050_lpf_t;

/**
 * @brief Possible low power wakeup values
 */
typedef enum {
    MPU6050_LP_WAKEUP_125mHZ    = 0x00,
    MPU6050_LP_WAKEUP_5HZ       = 0x40,
    MPU6050_LP_WAKEUP_20HZ      = 0x80,
    MPU6050_LP_WAKEUP_40HZ      = 0xC0,
} mpu6050_lp_wakeup_t;

/**
 * @brief MPU-6050 result vector struct
 */
typedef struct {
    int16_t x_axis;             /**< X-Axis measurement result */
    int16_t y_axis;             /**< Y-Axis measurement result */
    int16_t z_axis;             /**< Z-Axis measurement result */
} mpu6050_results_t;

/**
 * @brief Configuration struct for the MPU-6050 sensor
 */
typedef struct {
    mpu6050_pwr_t accel_pwr;            /**< Accel power status (on/off) */
    mpu6050_pwr_t gyro_pwr;             /**< Gyro power status (on/off) */
    mpu6050_gyro_ranges_t gyro_fsr;     /**< Configured gyro full-scale range */
    mpu6050_accel_ranges_t accel_fsr;   /**< Configured accel full-scale range */
    uint16_t sample_rate;               /**< Configured sample rate for accel and gyro */
} mpu6050_status_t;

/**
 * @brief Device descriptor for the MPU-6050 sensor
 */
typedef struct {
    i2c_t i2c_dev;                  /**< I2C device which is used */
    gpio_t gpio_dev;                /**< GPIO device which is used */
    uint8_t hw_addr;                /**< Hardware address of the MPU-6050 */
    kernel_pid_t msg_thread_pid;    /**< thread to msg on interrupt */
    mpu6050_status_t conf;          /**< Device configuration */
} mpu6050_t;

/**
 * @brief Initialize the given MPU6050 device
 *
 * @param[out] dev          Initialized device descriptor of MPU6050 device
 * @param[in]  i2c          I2C bus the sensor is connected to
 * @param[in]  hw_addr      The device's address on the I2C bus
 * @param[in]  gpio         GPIO device the sensors interrupt pin is
 *                          connected to
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu6050_init(mpu6050_t *dev, i2c_t i2c, mpu6050_hw_addr_t hw_addr, gpio_t gpio);

/**
 * @brief   register a thread for notification when motion is detected
 *
 * @note
 * This configures the gpio device for interrupt driven operation
 *
 * @param[in] dev       device descriptor of the motion sensor to register
 *                      for
 *
 * @return              0 on succuess,
 * @return              -1 on internal errors,
 * @return              -2 if another thread is registered already
 */
int mpu6050_register_thread(mpu6050_t *dev);

/**
 * @brief Enable or disable accelerometer power
 *
 * @param[in] dev           Device descriptor of MPU6050 device
 * @param[in] pwr_conf      Target power setting: PWR_ON or PWR_OFF
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu6050_set_accel_power(mpu6050_t *dev, mpu6050_pwr_t pwr_conf);

/**
 * @brief Enable or disable gyroscope power
 *
 * @param[in] dev           Device descriptor of MPU6050 device
 * @param[in] pwr_conf      Target power setting: PWR_ON or PWR_OFF
 *
 * @return                  0 on success
 * @return                  -1 if given I2C is not enabled in board config
 */
int mpu6050_set_gyro_power(mpu6050_t *dev, mpu6050_pwr_t pwr_conf);

/**
 * @brief Read angular speed values from the given MPU6050 device, returned in dps
 *
 * The raw gyroscope data is read from the sensor and normalized with respect to
 * the configured gyroscope full-scale range.
 *
 * @param[in]  dev          Device descriptor of MPU6050 device to read from
 * @param[out] output       Result vector in dps per axis
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if gyro full-scale range is configured wrong
 */
int mpu6050_read_gyro(mpu6050_t *dev, mpu6050_results_t *output);

/**
 * @brief Read acceleration values from the given MPU6050 device, returned in mG
 *
 * The raw acceleration data is read from the sensor and normalized with respect to
 * the configured accelerometer full-scale range.
 *
 * @param[in]  dev          Device descriptor of MPU6050 device to read from
 * @param[out] output       Result vector in mG per axis
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if accel full-scale range is configured wrong
 */
int mpu6050_read_accel(mpu6050_t *dev, mpu6050_results_t *output);

/**
 * @brief Read temperature value from the given MPU6050 device, returned in m°C
 *
 * @note
 * The measured temperature is slightly higher than the real room temperature.
 * Tests showed that the offset varied around 2-3 °C (but no warranties here).
 *
 * @param[in] dev           Device descriptor of MPU6050 device to read from
 * @param[out] output       Temperature in m°C
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 */
int mpu6050_read_temperature(mpu6050_t *dev, int32_t *output);

/**
 * @brief Set the full-scale range for raw gyroscope data
 *
 * @param[in] dev           Device descriptor of MPU6050 device
 * @param[in] fsr           Target full-scale range
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given full-scale target value is not valid
 */
int mpu6050_set_gyro_fsr(mpu6050_t *dev, mpu6050_gyro_ranges_t fsr);

/**
 * @brief Set the full-scale range for raw accelerometer data
 *
 * @param[in] dev           Device descriptor of MPU6050 device
 * @param[in] fsr           Target full-scale range
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given full-scale target value is not valid
 */
int mpu6050_set_accel_fsr(mpu6050_t *dev, mpu6050_accel_ranges_t fsr);

/**
 * @brief Set the rate at which the gyroscope and accelerometer data is sampled
 *
 * Sample rate can be chosen between 4 Hz and 1kHz. The actual set value might
 * slightly differ. If necessary, check the actual set value in the device's
 * config member afterwards.
 *
 * @param[in] dev           Device descriptor of MPU6050 device
 * @param[in] rate          Target sample rate in Hz
 *
 * @return                  0 on success
 * @return                  -1 if device's I2C is not enabled in board config
 * @return                  -2 if given target sample rate is not valid
 */
int mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t rate);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H_ */
/** @} */
