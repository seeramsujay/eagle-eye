# Project Eagle-Eye: Complete Electrical Schematic & Wiring Guide

## 1. ESP32-WROOM-32 Pinout Allocation

```
                              ESP32-WROOM-32
                              +------------+
                        3V3 --|  1      38 |-- GND (Common Ground)
                         EN --|  2      37 |-- GPIO 23 (LoRa MOSI)
       Battery Div (GPIO 34) -|  6      36 |-- GPIO 22 (MPU6050 SCL)
                              |  .      35 |-- GPIO 1 (TX0)
                              |  .      34 |-- GPIO 3 (RX0)
          LoRa DIO0 (GPIO 26) -| 11      33 |-- GPIO 21 (MPU6050 SDA)
          30A ESC PWM (GPIO 27)-| 12      30 |-- GPIO 19 (LoRa MISO)
          LoRa RST (GPIO 14) --| 13      29 |-- GPIO 18 (LoRa SCK)
          Left Elevon (GPIO 13)-| 14      28 |-- GPIO 5  (LoRa CS/NSS)
          Right Elevon (GPIO 12)| 15      27 |-- GPIO 17 (GPS RX2 / TX to GPS)
          GPS TX2 (GPIO 16) ---| 25      26 |-- GPIO 16 (GPS RX2 / RX from GPS)
                        VIN --| 19      20 |-- GND
                              +------------+
```

---

## 2. Pin Interconnection Table

| Subsystem | Peripheral Pin | ESP32 GPIO | Electrical Specs / Notes |
|---|---|---|---|
| **MPU-6050 (GY-521)** | SDA | **GPIO 21** | Fast I2C (400 kHz), internal pullups |
| | SCL | **GPIO 22** | Fast I2C (400 kHz) |
| | VCC | 3.3V Rail | From ESP32 3V3 pin |
| | GND | Common GND | Star ground connection |
| **SX1278 (Ra-02 LoRa)** | MOSI | **GPIO 23** | VSPI Hardware Bus (8 MHz) |
| | MISO | **GPIO 19** | VSPI Hardware Bus |
| | SCK | **GPIO 18** | VSPI Hardware Bus |
| | NSS / CS | **GPIO 5** | Active LOW Chip Select |
| | RST | **GPIO 14** | Hardware Reset |
| | DIO0 | **GPIO 26** | Packet RX/TX interrupt trigger |
| | 3.3V | **Dedicated 3.3V** | Output of AMS1117-3.3V LDO + 100µF Low-ESR cap |
| | GND | Common GND | Direct ground plane |
| **u-blox NEO-6M GPS** | TX | **GPIO 16 (RX2)** | Hardware UART2 @ 115200 baud, 5 Hz UBX |
| | RX | **GPIO 17 (TX2)** | Hardware UART2 (to configure 5Hz / 115200) |
| | VCC | 5V BEC Rail | Fed to NEO-6M 5V pin (onboard 3.3V LDO) |
| | GND | Common GND | |
| **Left Elevon Servo** | Signal (Orange) | **GPIO 13** | 50 Hz PWM (LEDC Channel 0, 1000-2000µs) |
| | VCC (Red) | 5V BEC Rail | From ESC 5V BEC (NOT ESP32 3.3V!) |
| | GND (Brown) | Common GND | |
| **Right Elevon Servo** | Signal (Orange) | **GPIO 12** | 50 Hz PWM (LEDC Channel 1, 1000-2000µs) |
| | VCC (Red) | 5V BEC Rail | From ESC 5V BEC |
| | GND (Brown) | Common GND | |
| **30A Brushless ESC** | Signal (White) | **GPIO 27** | 50 Hz PWM (LEDC Channel 2, 1000-2000µs) |
| | 5V BEC (Red) | 5V Rail | Powers ESP32 VIN, Servos, GPS, AMS1117 |
| | GND (Black) | Common GND | |
| **Battery Telemetry** | Mid-point Tap | **GPIO 34 (ADC1)**| 100kΩ / 20kΩ voltage divider from 3S Li-ion |

---

## 3. Power Architecture & RF Decoupling
1. **Primary Power Source:** 3S1P 18650 Li-ion pack (9.0V cutoff, 11.1V nominal, 12.6V fully charged).
2. **ESC BEC Output (5.0V @ 2A):**
   - Feeds directly to ESP32 `VIN` pin (ESP32 onboard regulator provides 3.3V for ESP32 and MPU6050).
   - Feeds both SG90 servo power leads directly to avoid servo brownouts on the ESP32 logic rail.
   - Feeds the input of an independent `AMS1117-3.3V` low-dropout regulator.
3. **Dedicated LoRa 3.3V Rail:**
   - The SX1278 Ra-02 module draws up to 120 mA peak current during 100 mW (+20 dBm) transmission bursts.
   - Powering Ra-02 directly from the ESP32 3.3V output frequently causes ESP32 brownout resets and ADC noise.
   - The dedicated `AMS1117-3.3V` with a `100µF` electrolytic capacitor placed directly across VCC/GND of the Ra-02 guarantees clean, ripple-free power during RF bursts.
4. **FPV Camera & VTX:**
   - Powered directly from 3S battery (or via LC filter to eliminate motor brushless PWM ripple from analog video).
