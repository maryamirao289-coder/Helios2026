#include "mpu6050.h"


/*
 * hi2c1由CubeMX在main.c中定义
 * 这里使用extern引用它
 */
extern I2C_HandleTypeDef hi2c1;


/* =========================================================
 * 内部函数：写一个寄存器
 * ========================================================= */
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );
}


/* =========================================================
 * 内部函数：读一个寄存器
 * ========================================================= */
static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        1,
        100
    );
}


/* =========================================================
 * 读取芯片ID
 * 正常MPU6050应该得到0x68
 * ========================================================= */
HAL_StatusTypeDef MPU6050_ReadID(uint8_t *id)
{
    return MPU6050_ReadReg(
        MPU6050_REG_WHO_AM_I,
        id
    );
}


/* =========================================================
 * MPU6050初始化
 * ========================================================= */
HAL_StatusTypeDef MPU6050_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t id;


    /* 等待MPU6050上电稳定 */
    HAL_Delay(100);


    /* -----------------------------------------------------
     * 1. 检查WHO_AM_I
     * ----------------------------------------------------- */
    status = MPU6050_ReadID(&id);

    if (status != HAL_OK)
    {
        return status;
    }

    if (id != 0x68)
    {
        return HAL_ERROR;
    }


    /* -----------------------------------------------------
     * 2. 复位MPU6050
     *
     * PWR_MGMT_1 bit7 DEVICE_RESET = 1
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_PWR_MGMT_1,
        0x80
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(100);


    /* -----------------------------------------------------
     * 3. 唤醒MPU6050
     *
     * PWR_MGMT_1 = 0x01
     *
     * SLEEP = 0
     * CLKSEL = 001
     * 使用X轴陀螺仪PLL作为时钟源
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_PWR_MGMT_1,
        0x01
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(10);


    /* -----------------------------------------------------
     * 4. 设置DLPF
     *
     * CONFIG = 0x03
     * DLPF_CFG = 3
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_CONFIG,
        0x03
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* -----------------------------------------------------
     * 5. 设置采样分频
     *
     * DLPF开启以后内部采样率为1kHz
     *
     * Sample Rate =
     * 1kHz / (1 + SMPLRT_DIV)
     *
     * SMPLRT_DIV = 0
     * => 1kHz
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_SMPLRT_DIV,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* -----------------------------------------------------
     * 6. 陀螺仪量程
     *
     * GYRO_CONFIG
     *
     * FS_SEL = 3
     * ±2000 deg/s
     *
     * 0b00011000 = 0x18
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_GYRO_CONFIG,
        0x18
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /* -----------------------------------------------------
     * 7. 加速度计量程
     *
     * ACCEL_CONFIG
     *
     * AFS_SEL = 2
     * ±8g
     *
     * 0b00010000 = 0x10
     * ----------------------------------------------------- */
    status = MPU6050_WriteReg(
        MPU6050_REG_ACCEL_CONFIG,
        0x10
    );

    if (status != HAL_OK)
    {
        return status;
    }


    return HAL_OK;
}


/* =========================================================
 * 一次读取14字节
 *
 * 从0x3B开始：
 *
 * AX_H
 * AX_L
 * AY_H
 * AY_L
 * AZ_H
 * AZ_L
 * TEMP_H
 * TEMP_L
 * GX_H
 * GX_L
 * GY_H
 * GY_L
 * GZ_H
 * GZ_L
 * ========================================================= */
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_RawData_t *data)
{
    uint8_t buf[14];

    HAL_StatusTypeDef status;


    if (data == NULL)
    {
        return HAL_ERROR;
    }


    status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        MPU6050_REG_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        14,
        100
    );


    if (status != HAL_OK)
    {
        return status;
    }


    /* 加速度计 */
    data->ax =
        (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);

    data->ay =
        (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);

    data->az =
        (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);


    /* 温度 */
    data->temp =
        (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);


    /* 陀螺仪 */
    data->gx =
        (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);

    data->gy =
        (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);

    data->gz =
        (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);


    return HAL_OK;
}
void MPU6050_ConvertRaw(
    const MPU6050_RawData_t *raw,
    MPU6050_Data_t *data
)
{
    if ((raw == NULL) || (data == NULL))
    {
        return;
    }

    /* Accelerometer
     * Range: +/-8g
     * Sensitivity: 4096 LSB/g
     */
    data->ax = (float)raw->ax / 4096.0f;
    data->ay = (float)raw->ay / 4096.0f;
    data->az = (float)raw->az / 4096.0f;


    /* Gyroscope
     * Range: +/-2000 deg/s
     * Sensitivity: 16.4 LSB/(deg/s)
     */
    data->gx = (float)raw->gx / MPU6050_GYRO_SENSITIVITY;
    data->gy = (float)raw->gy / MPU6050_GYRO_SENSITIVITY;
    data->gz = (float)raw->gz / MPU6050_GYRO_SENSITIVITY;


    /* Temperature */
    data->temperature =
        ((float)raw->temp / 340.0f) + 36.53f;
}
HAL_StatusTypeDef MPU6050_CalibrateGyro(
    MPU6050_GyroBias_t *bias,
    uint16_t samples
)
{
    MPU6050_RawData_t raw;
    HAL_StatusTypeDef status;

    int64_t sum_gx = 0;
    int64_t sum_gy = 0;
    int64_t sum_gz = 0;

    uint16_t i;


    if ((bias == NULL) || (samples == 0))
    {
        return HAL_ERROR;
    }


    /* Clear previous bias */
    bias->gx = 0.0f;
    bias->gy = 0.0f;
    bias->gz = 0.0f;


    /*
     * Wait for the sensor to become stable.
     * Keep MPU6050 completely stationary during calibration.
     */
    HAL_Delay(200);


    for (i = 0; i < samples; i++)
    {
        status = MPU6050_ReadRaw(&raw);

        if (status != HAL_OK)
        {
            return status;
        }


        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;


        HAL_Delay(1);
    }


    /*
     * Calculate average zero-bias.
     * Convert RAW to deg/s.
     */
    bias->gx =
        ((float)sum_gx / (float)samples)
        / MPU6050_GYRO_SENSITIVITY;

    bias->gy =
        ((float)sum_gy / (float)samples)
        / MPU6050_GYRO_SENSITIVITY;

    bias->gz =
        ((float)sum_gz / (float)samples)
        / MPU6050_GYRO_SENSITIVITY;


    return HAL_OK;
}
void MPU6050_ApplyGyroBias(
    MPU6050_Data_t *data,
    const MPU6050_GyroBias_t *bias
)
{
    if ((data == NULL) || (bias == NULL))
    {
        return;
    }


    /* Remove static zero-bias */
    data->gx -= bias->gx;
    data->gy -= bias->gy;
    data->gz -= bias->gz;


    /*
     * Deadband for residual static noise.
     *
     * Very small angular velocity is treated as zero.
     */
    if ((data->gx > -MPU6050_GYRO_DEADBAND) &&
        (data->gx <  MPU6050_GYRO_DEADBAND))
    {
        data->gx = 0.0f;
    }


    if ((data->gy > -MPU6050_GYRO_DEADBAND) &&
        (data->gy <  MPU6050_GYRO_DEADBAND))
    {
        data->gy = 0.0f;
    }


    if ((data->gz > -MPU6050_GYRO_DEADBAND) &&
        (data->gz <  MPU6050_GYRO_DEADBAND))
    {
        data->gz = 0.0f;
    }
}