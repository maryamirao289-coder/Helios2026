#ifndef __MAHONY_H
#define __MAHONY_H


#include "main.h"


/* 角度转换 */
#define MAHONY_DEG_TO_RAD    0.017453292519943295f
#define MAHONY_RAD_TO_DEG    57.29577951308232f


typedef struct
{
    /* Quaternion */
    float q0;
    float q1;
    float q2;
    float q3;


    /* Integral feedback */
    float integral_x;
    float integral_y;
    float integral_z;


    /* Filter gains */
    float kp;
    float ki;


    /* Euler angles, degree */
    float roll;
    float pitch;
    float yaw;

} Mahony_t;


/* Initialize filter */
void Mahony_Init(
    Mahony_t *filter,
    float kp,
    float ki
);


/* Modify gains dynamically */
void Mahony_SetGains(
    Mahony_t *filter,
    float kp,
    float ki
);


/* 6-axis Mahony update */
void Mahony_UpdateIMU(
    Mahony_t *filter,
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt
);


#endif