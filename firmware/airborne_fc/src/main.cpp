/**
 * Project Eagle-Eye: Autonomous Flying Wing Airborne Flight Controller
 * Target: ESP32-WROOM-32 (38-pin DevKit)
 * Sensors: GY-521 MPU6050 (I2C @ 400kHz), u-blox NEO-6M GPS (UART2 @ 115200 5Hz)
 * Radio: Ai-Thinker Ra-02 SX1278 (VSPI @ 8MHz, 433MHz, SF7, BW 500kHz)
 * Actuators: 50Hz LEDC PWM on GPIO 13 (L Elevon), GPIO 12 (R Elevon), GPIO 27 (ESC)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"
#include "../../common/protocol.h"
#include "mahony.h"
#include "pid.h"
#include "mixer.h"
#include "gps.h"
#include "failsafe.h"

// -------------------------------------------------------------
// Global Subsystems & State
// -------------------------------------------------------------
MahonyAHRS ahrs(2.0f, 0.005f);
PIDController roll_pid(3.5f, 0.05f, 0.45f, 400.0f, 100.0f, 0.7f);
PIDController pitch_pid(3.8f, 0.05f, 0.50f, 400.0f, 100.0f, 0.7f);
ElevonMixer mixer(0, 0, 1.0f, 1.0f, SERVO_MIN_US, SERVO_MAX_US, SERVO_MID_US);
FailsafeManager failsafe(1500);

// Hardware LEDC PWM Channels (ESP32)
#define LEDC_CHAN_LEFT_SERVO  0
#define LEDC_CHAN_RIGHT_SERVO 1
#define LEDC_CHAN_ESC         2
#define LEDC_TIMER_14_BIT     14
#define LEDC_BASE_FREQ        50

// Active Flight Commands
volatile FlightMode current_mode = MODE_DISARMED;
volatile float cmd_roll_deg = 0.0f;
volatile float cmd_pitch_deg = 0.0f;
volatile uint8_t cmd_throttle_pct = 0;

// GPS State
HardwareSerial SerialGPS(2);
double gps_lat = 0.0;
double gps_lon = 0.0;
float gps_alt_m = 0.0f;
float gps_speed_kmh = 0.0f;
float gps_heading_deg = 0.0f;
bool gps_has_fix = false;

// Battery State
float battery_voltage = 11.1f;

// Thread Synchronization Mutex
portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;

// -------------------------------------------------------------
// Helper: Convert Microseconds to LEDC 14-bit Duty Cycle
// 50 Hz -> 20,000 us period -> 14-bit = 16384 ticks -> ~0.8192 ticks/us
// -------------------------------------------------------------
static inline uint32_t us_to_duty(uint32_t us) {
    return (us * 16384UL) / 20000UL;
}

// -------------------------------------------------------------
// MPU6050 Low-level I2C Initialization & Read
// -------------------------------------------------------------
#define MPU_ADDR 0x68
void init_mpu6050() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
    // Wake up MPU6050 (clear sleep bit)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set Gyro full scale range +/- 1000 deg/s (0x10)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x10);
    Wire.endTransmission(true);

    // Set Accel full scale range +/- 4g (0x08)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission(true);

    // Digital Low Pass Filter ~44Hz (DLPF_CFG = 3)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission(true);
}

bool read_mpu6050(float &gx_rad, float &gy_rad, float &gz_rad, float &ax_g, float &ay_g, float &az_g) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;

    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    // Accel scale: +/- 4g -> 8192 LSB/g
    ax_g = (float)raw_ax / 8192.0f;
    ay_g = (float)raw_ay / 8192.0f;
    az_g = (float)raw_az / 8192.0f;

    // Gyro scale: +/- 1000 deg/s -> 32.8 LSB/(deg/s) -> convert to rad/s
    const float deg2rad = 0.0174532925f;
    gx_rad = ((float)raw_gx / 32.8f) * deg2rad;
    gy_rad = ((float)raw_gy / 32.8f) * deg2rad;
    gz_rad = ((float)raw_gz / 32.8f) * deg2rad;

    return true;
}

// -------------------------------------------------------------
// u-blox NEO-6M UBX Configuration Strings
// -------------------------------------------------------------
void configure_ublox_gps() {
    SerialGPS.begin(GPS_BOOT_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX2, PIN_GPS_TX2);
    delay(100);

    // UBX-CFG-PRT: Switch UART to 115200 baud
    const uint8_t ubx_set_baud_115200[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 
        0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
    };
    SerialGPS.write(ubx_set_baud_115200, sizeof(ubx_set_baud_115200));
    SerialGPS.flush();
    delay(100);

    // Switch ESP32 SerialGPS to 115200 baud
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, PIN_GPS_RX2, PIN_GPS_TX2);
    delay(50);

    // UBX-CFG-RATE: Set Navigation Update Rate to 5 Hz (200ms)
    const uint8_t ubx_set_rate_5hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
    };
    SerialGPS.write(ubx_set_rate_5hz, sizeof(ubx_set_rate_5hz));
    SerialGPS.flush();
}

// Simple NMEA Sentence Parser for RMC/GGA
void parse_nmea_sentence(const char *line) {
    if (strstr(line, "$GPRMC") || strstr(line, "$GNRMC")) {
        char status = 'V';
        char lat_dir = 'N', lon_dir = 'E';
        float raw_lat = 0, raw_lon = 0, speed_knots = 0, track_deg = 0;
        
        int parsed = sscanf(line, "%*[^,],%*[^,],%c,%f,%c,%f,%c,%f,%f", 
                            &status, &raw_lat, &lat_dir, &raw_lon, &lon_dir, &speed_knots, &track_deg);
        if (parsed >= 1 && status == 'A') {
            gps_has_fix = true;
            // Convert ddmm.mmmm to decimal degrees
            int lat_deg = (int)(raw_lat / 100);
            float lat_min = raw_lat - (lat_deg * 100);
            gps_lat = (double)lat_deg + (lat_min / 60.0);
            if (lat_dir == 'S') gps_lat = -gps_lat;

            int lon_deg = (int)(raw_lon / 100);
            float lon_min = raw_lon - (lon_deg * 100);
            gps_lon = (double)lon_deg + (lon_min / 60.0);
            if (lon_dir == 'W') gps_lon = -gps_lon;

            gps_speed_kmh = speed_knots * 1.852f;
            gps_heading_deg = track_deg;
        } else {
            gps_has_fix = false;
        }
    }
}

// -------------------------------------------------------------
// SX1278 LoRa Transceiver Setup
// -------------------------------------------------------------
void init_lora() {
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[ERROR] SX1278 LoRa init failed!");
        return;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE_4);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    Serial.println("[OK] SX1278 LoRa 433MHz Ready (SF7, BW500kHz)");
}

// -------------------------------------------------------------
// Core 0 Task: LoRa Telemetry, GPS, ADC & Failsafe (25 Hz)
// -------------------------------------------------------------
void telemetry_and_nav_task(void *pvParameters) {
    char nmea_buf[128];
    uint8_t nmea_idx = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 25 Hz (40 ms)

    while (1) {
        uint32_t now_ms = millis();

        // 1. Process incoming GPS serial stream
        while (SerialGPS.available()) {
            char c = SerialGPS.read();
            if (c == '\n' || c == '\r') {
                if (nmea_idx > 6) {
                    nmea_buf[nmea_idx] = '\0';
                    parse_nmea_sentence(nmea_buf);
                }
                nmea_idx = 0;
            } else if (nmea_idx < sizeof(nmea_buf) - 1) {
                nmea_buf[nmea_idx++] = c;
            }
        }

        // 2. Receive Uplink Commands via LoRa
        int packet_size = LoRa.parsePacket();
        if (packet_size == sizeof(UplinkPacket)) {
            UplinkPacket uplink;
            LoRa.readBytes((uint8_t *)&uplink, sizeof(UplinkPacket));

            if (validate_uplink_crc(&uplink)) {
                portENTER_CRITICAL(&state_spinlock);
                current_mode = (FlightMode)uplink.mode;
                cmd_roll_deg = (float)uplink.roll_cmd_cdeg / 100.0f;
                cmd_pitch_deg = (float)uplink.pitch_cmd_cdeg / 100.0f;
                cmd_throttle_pct = uplink.throttle_pct;

                double car_lat = (double)uplink.car_lat_e7 / 1e7;
                double car_lon = (double)uplink.car_lon_e7 / 1e7;

                failsafe.on_lora_packet_received(now_ms, cmd_roll_deg, cmd_pitch_deg, cmd_throttle_pct, car_lat, car_lon);
                portEXIT_CRITICAL(&state_spinlock);
            }
        }

        // 3. Evaluate 3-Tier Failsafe
        float active_roll = cmd_roll_deg;
        float active_pitch = cmd_pitch_deg;
        uint8_t active_throttle = cmd_throttle_pct;
        FlightMode active_m = current_mode;

        failsafe.evaluate(now_ms, gps_has_fix, gps_lat, gps_lon, gps_alt_m, gps_heading_deg,
                          active_roll, active_pitch, active_throttle, active_m);

        portENTER_CRITICAL(&state_spinlock);
        cmd_roll_deg = active_roll;
        cmd_pitch_deg = active_pitch;
        cmd_throttle_pct = active_throttle;
        current_mode = active_m;
        portEXIT_CRITICAL(&state_spinlock);

        // 4. Sample Battery ADC
        int raw_adc = analogRead(PIN_BATT_ADC);
        float pin_volts = ((float)raw_adc / 4095.0f) * 3.3f;
        battery_voltage = pin_volts * BATT_DIVIDER_RATIO;

        // 5. Send Downlink Telemetry Packet (16 Bytes)
        DownlinkPacket downlink;
        downlink.plane_lat_e7 = (int32_t)(gps_lat * 1e7);
        downlink.plane_lon_e7 = (int32_t)(gps_lon * 1e7);
        downlink.alt_m = (int16_t)gps_alt_m;
        downlink.speed_kmh = (uint8_t)gps_speed_kmh;
        downlink.heading_deg_div2 = (uint8_t)(fmod(gps_heading_deg, 360.0f) / 2.0f);
        downlink.batt_v_x10 = (uint8_t)(battery_voltage * 10.0f);

        uint8_t state_byte = (uint8_t)current_mode;
        if (cmd_throttle_pct > 0) state_byte |= FLAG_ARMED;
        if (gps_has_fix) state_byte |= FLAG_GPS_FIX;
        if (failsafe.current_tier != FAILSAFE_TIER_NONE) state_byte |= FLAG_FAILSAFE_ACTIVE;
        if (battery_voltage < 9.6f) state_byte |= FLAG_LOW_BATT;
        downlink.state_flags = state_byte;

        downlink.crc16 = calculate_crc16((const uint8_t *)&downlink, sizeof(DownlinkPacket) - 2);

        LoRa.beginPacket();
        LoRa.write((const uint8_t *)&downlink, sizeof(DownlinkPacket));
        LoRa.endPacket(true); // Async non-blocking transmit

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// -------------------------------------------------------------
// Arduino Setup & Main Loop (Core 1: 500 Hz Flight Control Loop)
// -------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- Project Eagle-Eye Airborne FC Starting ---");

    // Initialize Actuator PWM pins via LEDC
    ledcSetup(LEDC_CHAN_LEFT_SERVO, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttachPin(PIN_SERVO_LEFT, LEDC_CHAN_LEFT_SERVO);

    ledcSetup(LEDC_CHAN_RIGHT_SERVO, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttachPin(PIN_SERVO_RIGHT, LEDC_CHAN_RIGHT_SERVO);

    ledcSetup(LEDC_CHAN_ESC, LEDC_BASE_FREQ, LEDC_TIMER_14_BIT);
    ledcAttachPin(PIN_ESC_PWM, LEDC_CHAN_ESC);

    // Initial safe outputs: Servos centered (1500us), Motor stopped (1000us)
    ledcWrite(LEDC_CHAN_LEFT_SERVO, us_to_duty(SERVO_MID_US));
    ledcWrite(LEDC_CHAN_RIGHT_SERVO, us_to_duty(SERVO_MID_US));
    ledcWrite(LEDC_CHAN_ESC, us_to_duty(SERVO_MIN_US));

    // Initialize Sensors & Hardware
    init_mpu6050();
    configure_ublox_gps();
    init_lora();

    // Launch Core 0 Telemetry & Navigation Task
    xTaskCreatePinnedToCore(
        telemetry_and_nav_task,
        "TelemetryNav",
        4096,
        NULL,
        1,
        NULL,
        0
    );

    Serial.println("[OK] Flight Controller Initialized. Entering 500 Hz Control Loop.");
}

void loop() {
    static uint32_t last_micros = micros();
    uint32_t current_micros = micros();
    uint32_t dt_micros = current_micros - last_micros;

    // Strict 500 Hz timing: 2000 microseconds per iteration
    if (dt_micros >= 2000) {
        last_micros = current_micros;
        float dt = (float)dt_micros * 1e-6f;

        // 1. Read MPU6050 6-DOF IMU
        float gx, gy, gz, ax, ay, az;
        if (read_mpu6050(gx, gy, gz, ax, ay, az)) {
            // 2. Update Mahony AHRS Attitude Filter
            ahrs.update(gx, gy, gz, ax, ay, az, dt);
        }

        // 3. Fetch thread-safe command targets
        float target_roll, target_pitch;
        uint8_t target_throttle;
        FlightMode mode;

        portENTER_CRITICAL(&state_spinlock);
        target_roll = cmd_roll_deg;
        target_pitch = cmd_pitch_deg;
        target_throttle = cmd_throttle_pct;
        mode = current_mode;
        portEXIT_CRITICAL(&state_spinlock);

        // 4. Compute PID Corrections
        float roll_correction = 0.0f;
        float pitch_correction = 0.0f;
        int16_t left_servo_us = SERVO_MID_US;
        int16_t right_servo_us = SERVO_MID_US;
        int16_t esc_us = SERVO_MIN_US;

        if (mode != MODE_DISARMED) {
            // Cascaded PID stabilization on estimated Euler angles
            roll_correction = roll_pid.update(target_roll, ahrs.roll_deg, dt);
            pitch_correction = pitch_pid.update(target_pitch, ahrs.pitch_deg, dt);

            // Elevon Mixer: Left = Pitch + Roll, Right = Pitch - Roll
            mixer.mix(roll_correction, pitch_correction, left_servo_us, right_servo_us);
            esc_us = ElevonMixer::throttle_to_us(target_throttle);
        } else {
            // Disarmed: reset controllers and center controls
            roll_pid.reset();
            pitch_pid.reset();
            left_servo_us = SERVO_MID_US;
            right_servo_us = SERVO_MID_US;
            esc_us = SERVO_MIN_US;
        }

        // 5. Output to Servos & ESC via LEDC
        ledcWrite(LEDC_CHAN_LEFT_SERVO, us_to_duty(left_servo_us));
        ledcWrite(LEDC_CHAN_RIGHT_SERVO, us_to_duty(right_servo_us));
        ledcWrite(LEDC_CHAN_ESC, us_to_duty(esc_us));
    }
}
