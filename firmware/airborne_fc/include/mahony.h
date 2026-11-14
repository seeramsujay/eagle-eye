#ifndef MAHONY_H
#define MAHONY_H

#include <math.h>

class MahonyAHRS {
public:
    float Kp;
    float Ki;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float roll_deg, pitch_deg, yaw_deg;

    MahonyAHRS(float kp = 2.0f, float ki = 0.005f) 
        : Kp(kp), Ki(ki), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
          integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f),
          roll_deg(0.0f), pitch_deg(0.0f), yaw_deg(0.0f) {}

    // Fast inverse square root (Quake III algorithm)
    static float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }

    // Update with 6-DOF IMU data (gx, gy, gz in rad/s, ax, ay, az in any consistent unit)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // Compute and apply integral feedback if enabled
            if(Ki > 0.0f) {
                integralFBx += 2.0f * Ki * halfex * dt;
                integralFBy += 2.0f * Ki * halfey * dt;
                integralFBz += 2.0f * Ki * halfez * dt;
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }

            // Apply proportional feedback
            gx += 2.0f * Kp * halfex;
            gy += 2.0f * Kp * halfey;
            gz += 2.0f * Kp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gy - q3 * gz);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        // Compute Euler angles (Roll, Pitch, Yaw in degrees)
        // Roll: phi = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2))
        roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (180.0f / M_PI);

        // Pitch: theta = asin(clamp(2*(q0*q2 - q3*q1), -1, +1))
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f) {
            pitch_deg = copysignf(90.0f, sinp);
        } else {
            pitch_deg = asinf(sinp) * (180.0f / M_PI);
        }

        // Yaw: psi = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2))
        yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / M_PI);
    }
};

#endif // MAHONY_H
