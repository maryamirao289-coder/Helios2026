#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"


/* ==================== I2C地址 ==================== */

/* MPU6050 AD0接GND时，7位地址为0x68
 * STM32 HAL使用时左移1位
 */
#define MPU6050_ADDR                 (0x68 << 1)

/* Gyroscope sensitivity at +/-2000 deg/s */
#define MPU6050_GYRO_SENSITIVITY      16.4f

/* Static deadband after calibration, unit: deg/s */
#define MPU6050_GYRO_DEADBAND         0.15f


/* ==================== 寄存器地址 ==================== */

#define MPU6050_REG_SMPLRT_DIV       0x19
#define MPU6050_REG_CONFIG           0x1A
#define MPU6050_REG_GYRO_CONFIG      0x1B
#define MPU6050_REG_ACCEL_CONFIG     0x1C

#define MPU6050_REG_ACCEL_XOUT_H     0x3B

#define MPU6050_REG_PWR_MGMT_1       0x6B
#define MPU6050_REG_WHO_AM_I         0x75


/* ==================== 原始数据结构体 ==================== */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} MPU6050_RawData_t;

/* Gyroscope zero-bias */
typedef struct
{
    float gx;
    float gy;
    float gz;

} MPU6050_GyroBias_t;

typedef struct
{
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float temperature;

} MPU6050_Data_t;
/* ==================== 函数声明 ==================== */

/* 初始化MPU6050 */
HAL_StatusTypeDef MPU6050_Init(void);

/* 读取WHO_AM_I */
HAL_StatusTypeDef MPU6050_ReadID(uint8_t *id);

/* 读取六轴原始数据 */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_RawData_t *data);

void MPU6050_ConvertRaw(
    const MPU6050_RawData_t *raw,
    MPU6050_Data_t *data
);
		
		
HAL_StatusTypeDef MPU6050_CalibrateGyro(
    MPU6050_GyroBias_t *bias,
    uint16_t samples
);

void MPU6050_ApplyGyroBias(
    MPU6050_Data_t *data,
    const MPU6050_GyroBias_t *bias
);
#endif