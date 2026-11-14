#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// Project Eagle-Eye Hardware Pin Definitions
// ESP32-WROOM-32 Pinout Specification
// ==========================================

// MPU-6050 (I2C @ 400 kHz)
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22
#define I2C_CLOCK_HZ        400000

// SX1278 Ra-02 LoRa (VSPI @ 8 MHz)
#define PIN_LORA_MOSI       23
#define PIN_LORA_MISO       19
#define PIN_LORA_SCK        18
#define PIN_LORA_CS         5
#define PIN_LORA_RST        14
#define PIN_LORA_DIO0       26
#define LORA_FREQUENCY_HZ   433E6
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH 500E3
#define LORA_CODING_RATE_4  5
#define LORA_SYNC_WORD      0x12
#define LORA_TX_POWER_DBM   17

// u-blox NEO-6M GPS (UART2 @ 115200 baud, 5 Hz UBX)
#define PIN_GPS_RX2         16  // ESP32 RX from GPS TX
#define PIN_GPS_TX2         17  // ESP32 TX to GPS RX
#define GPS_BAUD_RATE       115200
#define GPS_BOOT_BAUD_RATE  9600

// PWM Actuator Outputs (50 Hz LEDC / ESP32 Servo)
#define PIN_SERVO_LEFT      13
#define PIN_SERVO_RIGHT     12
#define PIN_ESC_PWM         27

// Battery Voltage Telemetry ADC (100k / 20k Divider)
#define PIN_BATT_ADC        34
#define BATT_DIVIDER_RATIO  ((100.0f + 20.0f) / 20.0f) // 6.0 multiplier

// Control Surface Constraints
#define SERVO_MIN_US        1000
#define SERVO_MID_US        1500
#define SERVO_MAX_US        2000

#define MAX_ROLL_ANGLE_DEG  35.0f
#define MIN_PITCH_ANGLE_DEG -15.0f
#define MAX_PITCH_ANGLE_DEG 20.0f

#define LOOP_RATE_HZ        500
#define LORA_RATE_HZ        25

#endif // CONFIG_H
