/**
 * Project Eagle-Eye v0 Micro: Twin-Coreless Flying Wing Flight Controller
 * Controls 2x Brushed Coreless Motors via 20 kHz ultrasonic PWM & differential thrust
 * Features MPU6050 yaw-rate damping, ESP-NOW wireless telemetry, and 1S LiPo protection
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"

// Active Control Setpoints
volatile int8_t cmd_throttle_pct = 0;   // 0 - 100%
volatile int8_t cmd_steer_yaw = 0;      // -100 (Full Left) to +100 (Full Right)
volatile int8_t cmd_steer_roll = 0;     // -100 to +100
volatile uint32_t last_cmd_time_ms = 0;
const uint32_t FAILSAFE_TIMEOUT_MS = 500; // Cut motors after 500ms signal loss

// Gyro Rate State
float gyro_yaw_rate = 0.0f;
float gyro_yaw_offset = 0.0f;
float yaw_damping_gain = 0.15f; // Damps high-speed yaw oscillations

// 1S Battery Monitor
float battery_voltage = 3.8f;

// ESP-NOW Control Packet (Compact 8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t  magic;         // 0xEE
    int8_t   throttle;      // 0 - 100
    int8_t   steering;      // -100 to +100 (Yaw/Roll)
    uint8_t  trim_left;     // Left motor trim offset
    uint8_t  trim_right;    // Right motor trim offset
    uint16_t checksum;
} MicroControlPacket;

void on_esp_now_data_recv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(MicroControlPacket)) {
        MicroControlPacket pkt;
        memcpy(&pkt, incomingData, sizeof(MicroControlPacket));

        if (pkt.magic == 0xEE) {
            cmd_throttle_pct = constrain(pkt.throttle, 0, 100);
            cmd_steer_yaw = constrain(pkt.steering, -100, 100);
            last_cmd_time_ms = millis();
        }
    }
}

// -----------------------------------------------------------------------------
// Coreless Motor PWM Initialization (20 kHz, 10-bit resolution)
// -----------------------------------------------------------------------------
void init_motors() {
    ledcSetup(LEDC_CHAN_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_LEFT, LEDC_CHAN_LEFT);

    ledcSetup(LEDC_CHAN_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcAttachPin(PIN_MOTOR_RIGHT, LEDC_CHAN_RIGHT);

    ledcWrite(LEDC_CHAN_LEFT, 0);
    ledcWrite(LEDC_CHAN_RIGHT, 0);
}

void write_motor_pct(uint8_t channel, float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    uint32_t duty = (uint32_t)((pct / 100.0f) * 1023.0f);
    ledcWrite(channel, duty);
}

// -----------------------------------------------------------------------------
// MPU6050 Gyro Initialization & Fast Read
// -----------------------------------------------------------------------------
void init_gyro() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission(true);

    // Measure initial yaw gyro offset (average 50 samples)
    float sum_gz = 0.0f;
    for (int i = 0; i < 50; i++) {
        Wire.beginTransmission(0x68);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true) == 2) {
            int16_t raw_gz = (Wire.read() << 8) | Wire.read();
            sum_gz += (float)raw_gz / 65.5f; // +/- 500 deg/s
        }
        delay(5);
    }
    gyro_yaw_offset = sum_gz / 50.0f;
}

float read_gyro_yaw_rate() {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    if (Wire.endTransmission(false) != 0) return 0.0f;
    if (Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true) != 2) return 0.0f;
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    return ((float)raw_gz / 65.5f) - gyro_yaw_offset;
}

// -----------------------------------------------------------------------------
// Differential Thrust Flight Mixer
// -----------------------------------------------------------------------------
void compute_differential_thrust(float throttle_pct, float steer_input, float gyro_rate,
                                 float &left_motor_pct, float &right_motor_pct) {
    if (throttle_pct <= 1.0f) {
        left_motor_pct = 0.0f;
        right_motor_pct = 0.0f;
        return;
    }

    // Steering split: positive steer turns right (Left motor speeds up, Right slows down)
    float steer_effective = (steer_input / 100.0f) * MAX_DIFF_THRUST_PCT;

    // Rate damping feedback to suppress high-speed flying wing yaw wag
    float gyro_stabilization = -gyro_rate * yaw_damping_gain;

    float raw_left = throttle_pct + (steer_effective * 0.5f) + gyro_stabilization;
    float raw_right = throttle_pct - (steer_effective * 0.5f) - gyro_stabilization;

    left_motor_pct = constrain(raw_left, 0.0f, 100.0f);
    right_motor_pct = constrain(raw_right, 0.0f, 100.0f);
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Project Eagle-Eye v0 Micro FC Initializing ===");

    init_motors();
    init_gyro();

    // Initialize ESP-NOW fast peer-to-peer radio
    WiFi.mode(WIFI_STA);
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(on_esp_now_data_recv);
        Serial.println("[OK] ESP-NOW Receiver Active.");
    } else {
        Serial.println("[WARN] ESP-NOW failed, using Serial control fallback.");
    }

    Serial.println("[OK] Twin-Coreless Micro FC Ready.");
}

void loop() {
    static uint32_t last_loop_time = millis();
    uint32_t now = millis();

    // 100 Hz Control Loop
    if (now - last_loop_time >= 10) {
        last_loop_time = now;

        // 1. Check Failsafe: Cut motors if no signal received within 500ms
        bool failsafe_active = (now - last_cmd_time_ms) > FAILSAFE_TIMEOUT_MS;
        float base_thr = failsafe_active ? 0.0f : (float)cmd_throttle_pct;
        float steer = failsafe_active ? 0.0f : (float)cmd_steer_yaw;

        // 2. Read Gyro Rate
        gyro_yaw_rate = read_gyro_yaw_rate();

        // 3. Compute Differential Thrust
        float left_motor = 0.0f, right_motor = 0.0f;
        compute_differential_thrust(base_thr, steer, gyro_yaw_rate, left_motor, right_motor);

        // 4. Output Ultrasonic PWM to Motors
        write_motor_pct(LEDC_CHAN_LEFT, left_motor);
        write_motor_pct(LEDC_CHAN_RIGHT, right_motor);

        // 5. Sample 1S Battery Voltage (10 Hz)
        static uint8_t batt_count = 0;
        if (++batt_count >= 10) {
            batt_count = 0;
            int raw = analogRead(PIN_BATT_ADC);
            battery_voltage = ((float)raw / 4095.0f) * 3.3f * BATT_DIVIDER;

            // Cut throttle if cell drops below 3.2V (over-discharge safety)
            if (battery_voltage < BATT_CUTOFF_V && battery_voltage > 1.5f) {
                cmd_throttle_pct = 0;
            }
        }
    }

    // Serial CLI control for bench testing (e.g. "T50 S10" = 50% throttle, 10 steer)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'T' || c == 't') {
            cmd_throttle_pct = Serial.parseInt();
            last_cmd_time_ms = millis();
        } else if (c == 'S' || c == 's') {
            cmd_steer_yaw = Serial.parseInt();
            last_cmd_time_ms = millis();
        }
    }
}
