#include "mahony.h"
#include <math.h>


static void Mahony_UpdateEuler(Mahony_t *filter)
{
    float sin_pitch;


    /*
     * Roll
     */
    filter->roll = atan2f(
        2.0f * (
            filter->q0 * filter->q1 +
            filter->q2 * filter->q3
        ),
        1.0f - 2.0f * (
            filter->q1 * filter->q1 +
            filter->q2 * filter->q2
        )
    );


    /*
     * Pitch
     */
    sin_pitch =
        2.0f * (
            filter->q0 * filter->q2 -
            filter->q3 * filter->q1
        );


    /* Prevent asin numerical overflow */
    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }


    filter->pitch = asinf(sin_pitch);


    /*
     * Yaw
     */
    filter->yaw = atan2f(
        2.0f * (
            filter->q0 * filter->q3 +
            filter->q1 * filter->q2
        ),
        1.0f - 2.0f * (
            filter->q2 * filter->q2 +
            filter->q3 * filter->q3
        )
    );


    /* rad -> degree */
    filter->roll  *= MAHONY_RAD_TO_DEG;
    filter->pitch *= MAHONY_RAD_TO_DEG;
    filter->yaw   *= MAHONY_RAD_TO_DEG;
}


void Mahony_Init(
    Mahony_t *filter,
    float kp,
    float ki
)
{
    if (filter == NULL)
    {
        return;
    }


    /*
     * Initial quaternion:
     * no rotation
     */
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;


    filter->integral_x = 0.0f;
    filter->integral_y = 0.0f;
    filter->integral_z = 0.0f;


    filter->kp = kp;
    filter->ki = ki;


    filter->roll  = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw   = 0.0f;
}


void Mahony_SetGains(
    Mahony_t *filter,
    float kp,
    float ki
)
{
    if (filter == NULL)
    {
        return;
    }


    filter->kp = kp;
    filter->ki = ki;
}


void Mahony_UpdateIMU(
    Mahony_t *filter,
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt
)
{
    float norm;

    float vx;
    float vy;
    float vz;

    float ex;
    float ey;
    float ez;

    float q0;
    float q1;
    float q2;
    float q3;

    float dq0;
    float dq1;
    float dq2;
    float dq3;


    if ((filter == NULL) || (dt <= 0.0f))
    {
        return;
    }


    /*
     * Gyroscope:
     * deg/s -> rad/s
     */
    gx *= MAHONY_DEG_TO_RAD;
    gy *= MAHONY_DEG_TO_RAD;
    gz *= MAHONY_DEG_TO_RAD;


    q0 = filter->q0;
    q1 = filter->q1;
    q2 = filter->q2;
    q3 = filter->q3;


    /*
     * Accelerometer feedback is only used
     * when acceleration vector is valid.
     */
    norm = sqrtf(
        ax * ax +
        ay * ay +
        az * az
    );


    if (norm > 0.000001f)
    {
        /*
         * Normalize accelerometer.
         * Only direction of gravity is needed.
         */
        ax /= norm;
        ay /= norm;
        az /= norm;


        /*
         * Estimated gravity direction
         * calculated from quaternion.
         */
        vx = 2.0f * (
            q1 * q3 -
            q0 * q2
        );

        vy = 2.0f * (
            q0 * q1 +
            q2 * q3
        );

        vz =
            q0 * q0 -
            q1 * q1 -
            q2 * q2 +
            q3 * q3;


        /*
         * Error between measured gravity
         * and estimated gravity.
         *
         * e = measured x estimated
         */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;


        /*
         * Integral feedback.
         *
         * Mainly compensates slowly varying
         * gyro bias.
         */
        if (filter->ki > 0.0f)
        {
            filter->integral_x +=
                filter->ki * ex * dt;

            filter->integral_y +=
                filter->ki * ey * dt;

            filter->integral_z +=
                filter->ki * ez * dt;


            gx += filter->integral_x;
            gy += filter->integral_y;
            gz += filter->integral_z;
        }
        else
        {
            filter->integral_x = 0.0f;
            filter->integral_y = 0.0f;
            filter->integral_z = 0.0f;
        }


        /*
         * Proportional feedback.
         */
        gx += filter->kp * ex;
        gy += filter->kp * ey;
        gz += filter->kp * ez;
    }


    /*
     * Quaternion derivative:
     *
     * q_dot = 0.5 * q (*) omega
     */
    dq0 =
        0.5f * (
            -q1 * gx -
             q2 * gy -
             q3 * gz
        );

    dq1 =
        0.5f * (
             q0 * gx +
             q2 * gz -
             q3 * gy
        );

    dq2 =
        0.5f * (
             q0 * gy -
             q1 * gz +
             q3 * gx
        );

    dq3 =
        0.5f * (
             q0 * gz +
             q1 * gy -
             q2 * gx
        );


    /*
     * Integration
     */
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;


    /*
     * Normalize quaternion
     */
    norm = sqrtf(
        q0 * q0 +
        q1 * q1 +
        q2 * q2 +
        q3 * q3
    );


    if (norm > 0.000001f)
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }


    filter->q0 = q0;
    filter->q1 = q1;
    filter->q2 = q2;
    filter->q3 = q3;


    /*
     * Quaternion -> Euler
     */
    Mahony_UpdateEuler(filter);
}