/**
 * Project Eagle-Eye v0 Micro: 18650 Twin-Coreless Flying Wing with L293D Driver
 * Target: ESP32-WROOM-32 (38-pin DevKit)
 * Actuators: Twin 8520 / 716 Coreless Motors driven via L293D (Channels 1+2 & 3+4 in parallel)
 * PWM: 8 kHz on GPIO 13 (Left) and GPIO 12 (Right)
 * Sensors: GY-521 MPU6050 (I2C @ 400kHz)
 * Telemetry: 16-Byte Packed Binary Protocol matching Eagle-Eye Ground Station
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "../../../common/protocol.h"

// -----------------------------------------------------------------------------
// Pin Configuration & PWM Parameters
// -----------------------------------------------------------------------------
#define PIN_L293D_LEFT_PWM      13  // Inputs 1A & 2A (Channels 1 & 2 in parallel)
#define PIN_L293D_RIGHT_PWM     12  // Inputs 3A & 4A (Channels 3 & 4 in parallel)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define PIN_BATT_ADC            34  // 18650 cell tap (10k/10k divider, 2:1 ratio)

#define L293D_PWM_FREQ_HZ       8000    // 8 kHz (optimal for L293D bipolar Darlingtons)
#define L293D_PWM_RES_BITS      10      // 0 - 1023
#define LEDC_CHAN_LEFT          0
#define LEDC_CHAN_RIGHT         1

#define BATT_CUTOFF_V           3.2f    // 1S 18650 low-voltage cutoff threshold
#define BATT_DIVIDER_RATIO      2.0f

// Flight Commands & Failsafe
volatile FlightMode active_mode = MODE_DISARMED;
volatile float cmd_roll_deg = 0.0f;
volatile float cmd_pitch_deg = 0.0f;
volatile uint8_t cmd_throttle_pct = 0;
volatile uint32_t last_packet_time_ms = 0;
const uint32_t LORA_FAILSAFE_TIMEOUT_MS = 1500;

// Gyro Stabilization
float gyro_yaw_rate = 0.0f;
float gyro_yaw_offset = 0.0f;
const float YAW_DAMPING_GAIN = 0.18f;

// Battery
float battery_voltage = 3.9f;

// -----------------------------------------------------------------------------
// Actuator Initialization & PWM Output
// -----------------------------------------------------------------------------
void init_l293d_motors() {
    ledcSetup(LEDC_CHAN_LEFT, L293D_PWM_FREQ_HZ, L293D_PWM_RES_BITS);
    ledcAttachPin(PIN_L293D_LEFT_PWM, LEDC_CHAN_LEFT);

    ledcSetup(LEDC_CHAN_RIGHT, L293D_PWM_FREQ_HZ, L293D_PWM_RES_BITS);
    ledcAttachPin(PIN_L293D_RIGHT_PWM, LEDC_CHAN_RIGHT);

    ledcWrite(LEDC_CHAN_LEFT, 0);
    ledcWrite(LEDC_CHAN_RIGHT, 0);
}

void write_l293d_throttle(uint8_t channel, float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    uint32_t duty = (uint32_t)((pct / 100.0f) * 1023.0f);
    ledcWrite(channel, duty);
}

// -----------------------------------------------------------------------------
// MPU6050 Rate Gyro
// -----------------------------------------------------------------------------
void init_mpu6050() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Measure initial bias
    float sum_gz = 0.0f;
    for (int i = 0; i < 40; i++) {
        Wire.beginTransmission(0x68);
        Wire.write(0x47);
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true) == 2) {
            int16_t gz = (Wire.read() << 8) | Wire.read();
            sum_gz += (float)gz / 65.5f;
        }
        delay(5);
    }
    gyro_yaw_offset = sum_gz / 40.0f;
}

float read_mpu6050_yaw_rate() {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    if (Wire.endTransmission(false) != 0) return 0.0f;
    if (Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true) != 2) return 0.0f;
    int16_t gz = (Wire.read() << 8) | Wire.read();
    return ((float)gz / 65.5f) - gyro_yaw_offset;
}

// -----------------------------------------------------------------------------
// Differential Thrust Flight Mixer (Coupled Yaw & Roll)
// -----------------------------------------------------------------------------
void compute_differential_mixer(float base_throttle, float roll_cmd, float gyro_rate,
                                float &left_pct, float &right_pct) {
    if (base_throttle <= 1.0f || active_mode == MODE_DISARMED) {
        left_pct = 0.0f;
        right_pct = 0.0f;
        return;
    }

    // Steering split: Roll command from ground station pursuit controller [-35° to +35°]
    // Normalized to differential thrust percentage (max +/- 40% split)
    float steer_split = (roll_cmd / 35.0f) * 40.0f;

    // Rate damping stabilization: damp uncommanded yaw rotation
    float damping = -gyro_rate * YAW_DAMPING_GAIN;

    float raw_left = base_throttle + (steer_split * 0.5f) + damping;
    float raw_right = base_throttle - (steer_split * 0.5f) - damping;

    left_pct = constrain(raw_left, 0.0f, 100.0f);
    right_pct = constrain(raw_right, 0.0f, 100.0f);
}

// -----------------------------------------------------------------------------
// Ground Station Packet Ingestion (Serial / Wireless Bridge)
// -----------------------------------------------------------------------------
void process_ground_station_uplink() {
    if (Serial.available() >= (int)sizeof(UplinkPacket)) {
        UplinkPacket uplink;
        Serial.readBytes((uint8_t *)&uplink, sizeof(UplinkPacket));

        if (validate_uplink_crc(&uplink)) {
            active_mode = (FlightMode)uplink.mode;
            cmd_roll_deg = (float)uplink.roll_cmd_cdeg / 100.0f;
            cmd_pitch_deg = (float)uplink.pitch_cmd_cdeg / 100.0f;
            cmd_throttle_pct = uplink.throttle_pct;
            last_packet_time_ms = millis();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Project Eagle-Eye v0 Micro (18650 + L293D) Initialized ===");

    init_l293d_motors();
    init_mpu6050();
    Serial.println("[OK] L293D Paralleled H-Bridge Drivers Active at 8 kHz.");
}

void loop() {
    static uint32_t last_loop_ms = millis();
    uint32_t now = millis();

    // Ingest Uplink Packets from Laptop Ground Station
    process_ground_station_uplink();

    // 100 Hz Flight Control Loop
    if (now - last_loop_ms >= 10) {
        last_loop_ms = now;

        // Failsafe: if ground station link dropped > 1.5s, cut throttle
        bool is_failsafe = (now - last_packet_time_ms) > LORA_FAILSAFE_TIMEOUT_MS && (last_packet_time_ms > 0);
        float effective_thr = is_failsafe ? 0.0f : (float)cmd_throttle_pct;
        float effective_roll = is_failsafe ? 0.0f : cmd_roll_deg;

        // Read Gyro Rate
        gyro_yaw_rate = read_mpu6050_yaw_rate();

        // Calculate Left & Right Coreless Motor Throttle
        float left_thrust = 0.0f, right_thrust = 0.0f;
        compute_differential_mixer(effective_thr, effective_roll, gyro_yaw_rate, left_thrust, right_thrust);

        // Output PWM to L293D
        write_l293d_throttle(LEDC_CHAN_LEFT, left_thrust);
        write_l293d_throttle(LEDC_CHAN_RIGHT, right_thrust);

        // Sample 18650 Battery Voltage (10 Hz)
        static uint8_t batt_counter = 0;
        if (++batt_counter >= 10) {
            batt_counter = 0;
            int raw_adc = analogRead(PIN_BATT_ADC);
            battery_voltage = ((float)raw_adc / 4095.0f) * 3.3f * BATT_DIVIDER_RATIO;

            // Low-voltage cutoff protection for 18650
            if (battery_voltage < BATT_CUTOFF_V && battery_voltage > 1.5f) {
                cmd_throttle_pct = 0;
            }

            // Downlink Telemetry Packet to Laptop Ground Station (16 Bytes)
            DownlinkPacket downlink;
            memset(&downlink, 0, sizeof(downlink));
            downlink.speed_kmh = (uint8_t)(cmd_throttle_pct * 0.45f); // Estimated airspeed km/h
            downlink.batt_v_x10 = (uint8_t)(battery_voltage * 10.0f);
            downlink.state_flags = (cmd_throttle_pct > 0 ? FLAG_ARMED : 0) | (uint8_t)active_mode;
            downlink.crc16 = calculate_crc16((const uint8_t *)&downlink, sizeof(DownlinkPacket) - 2);

            Serial.write((const uint8_t *)&downlink, sizeof(DownlinkPacket));
        }
    }
}
