#ifndef V0_MICRO_CONFIG_H
#define V0_MICRO_CONFIG_H

// =============================================================================
// Project Eagle-Eye v0 Micro: Twin-Coreless Flying Wing Flight Controller
// =============================================================================

// Motor PWM Pins (Driven via N-MOSFETs SI2302 / AO3400)
#define PIN_MOTOR_LEFT          13
#define PIN_MOTOR_RIGHT         12

// PWM Configuration (20 kHz Ultrasonic Frequency - Silent & Smooth)
#define MOTOR_PWM_FREQ_HZ       20000
#define MOTOR_PWM_RES_BITS      10      // 0 - 1023 duty cycle
#define LEDC_CHAN_LEFT          0
#define LEDC_CHAN_RIGHT         1

// MPU6050 (I2C)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define I2C_FREQ_HZ             400000

// 1S LiPo Battery ADC (Direct or 2:1 Divider)
#define PIN_BATT_ADC            34
#define BATT_DIVIDER            2.0f    // 10k / 10k divider for 1S (3.0V - 4.2V)
#define BATT_CUTOFF_V           3.2f    // Low voltage cutoff

// Aerodynamic Control Constraints
#define MAX_THROTTLE_PCT        100
#define IDLE_THROTTLE_PCT       5
#define MAX_DIFF_THRUST_PCT     45      // Maximum differential split for steering

#endif // V0_MICRO_CONFIG_H
