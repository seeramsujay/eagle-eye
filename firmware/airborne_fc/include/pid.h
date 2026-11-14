#ifndef PID_H
#define PID_H

#include <math.h>

class PIDController {
public:
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_output;
    float max_integral;
    float d_filter_alpha;
    float prev_derivative;

    PIDController(float p = 1.0f, float i = 0.0f, float d = 0.0f, float max_out = 500.0f, float max_i = 150.0f, float alpha = 0.7f)
        : kp(p), ki(i), kd(d), integral(0.0f), prev_error(0.0f),
          max_output(max_out), max_integral(max_i), d_filter_alpha(alpha), prev_derivative(0.0f) {}

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
        prev_derivative = 0.0f;
    }

    float update(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measured;

        // Proportional term
        float p_term = kp * error;

        // Integral term with anti-windup clamping
        integral += error * dt;
        if (integral > max_integral) integral = max_integral;
        else if (integral < -max_integral) integral = -max_integral;
        float i_term = ki * integral;

        // Derivative term with low-pass filter
        float raw_derivative = (error - prev_error) / dt;
        float filtered_derivative = d_filter_alpha * prev_derivative + (1.0f - d_filter_alpha) * raw_derivative;
        prev_derivative = filtered_derivative;
        prev_error = error;
        float d_term = kd * filtered_derivative;

        // Total output with saturation limit
        float output = p_term + i_term + d_term;
        if (output > max_output) output = max_output;
        else if (output < -max_output) output = -max_output;

        return output;
    }
};

#endif // PID_H
