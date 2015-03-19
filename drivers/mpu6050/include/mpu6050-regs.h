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
 * @brief   Register and bit definitions for the MPU-6050
 *
 * @author  Ludwig Ortmann <ludwig@airfy.com>
 */

#ifndef MPU6050_REGS_H_
#define MPU6050_REGS_H_


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @name MPU-6050 register definitions
 * @{
 */
#define MPU6050_YG_OFFS_TC_REG          (0x01)
#define MPU6050_RATE_DIV_REG            (0x19)
#define MPU6050_LPF_REG                 (0x1A)
#define MPU6050_GYRO_CFG_REG            (0x1B)
#define MPU6050_ACCEL_CFG_REG           (0x1C)
#define MPU6050_FIFO_EN_REG             (0x23)
#define MPU6050_I2C_MST_REG             (0x24)
#define MPU6050_SLAVE0_ADDR_REG         (0x25)
#define MPU6050_SLAVE0_REG_REG          (0x26)
#define MPU6050_SLAVE0_CTRL_REG         (0x27)
#define MPU6050_SLAVE1_ADDR_REG         (0x28)
#define MPU6050_SLAVE1_REG_REG          (0x29)
#define MPU6050_SLAVE1_CTRL_REG         (0x2A)
#define MPU6050_SLAVE4_CTRL_REG         (0x34)
#define MPU6050_INT_PIN_CFG_REG         (0x37)
#define MPU6050_INT_ENABLE_REG          (0x38)
#define MPU6050_DMP_INT_STATUS          (0x39)
#define MPU6050_INT_STATUS              (0x3A)
#define MPU6050_ACCEL_START_REG         (0x3B)
#define MPU6050_TEMP_START_REG          (0x41)
#define MPU6050_GYRO_START_REG          (0x43)
#define MPU6050_EXT_SENS_DATA_START_REG (0x49)
#define MPU6050_SLAVE0_DATA_OUT_REG     (0x63)
#define MPU6050_SLAVE1_DATA_OUT_REG     (0x64)
#define MPU6050_SLAVE2_DATA_OUT_REG     (0x65)
#define MPU6050_SLAVE3_DATA_OUT_REG     (0x66)
#define MPU6050_I2C_DELAY_CTRL_REG      (0x67)
#define MPU6050_USER_CTRL_REG           (0x6A)
#define MPU6050_PWR_MGMT_1_REG          (0x6B)
#define MPU6050_PWR_MGMT_2_REG          (0x6C)
#define MPU6050_FIFO_COUNT_START_REG    (0x72)
#define MPU6050_FIFO_RW_REG             (0x74)
#define MPU6050_WHO_AM_I_REG            (0x75)
/** @} */

/**
 * @name MPU6050 bitfield definitions
 * @{
 */
#define BIT_SLV0_DELAY_EN               (0x01)
#define BIT_SLV1_DELAY_EN               (0x02)
#define BIT_I2C_BYPASS_EN               (0x02)
#define BIT_I2C_MST_EN                  (0x20)
#define BIT_PWR_MGMT1_SLEEP             (0x40)
#define BIT_WAIT_FOR_ES                 (0x40)
#define BIT_I2C_MST_VDDIO               (0x80)
#define BIT_SLAVE_RW                    (0x80)
#define BIT_SLAVE_EN                    (0x80)
#define BIT_DMP_EN                      (0x80)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_REGS_H_ */
/** @} */
