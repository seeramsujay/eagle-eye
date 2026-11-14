#ifndef GROUND_BRIDGE_CONFIG_H
#define GROUND_BRIDGE_CONFIG_H

// SX1278 Ra-02 LoRa Pins for Ground Bridge ESP32 DevKit
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

#define SERIAL_BAUD_RATE    921600

#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0xAA

enum FrameType : uint8_t {
    FRAME_TYPE_UPLINK   = 0x01, // Host -> Bridge -> Air (16 bytes)
    FRAME_TYPE_DOWNLINK = 0x02, // Air -> Bridge -> Host (16 bytes)
    FRAME_TYPE_PING     = 0x03, // Host -> Bridge (timestamp probe)
    FRAME_TYPE_PONG     = 0x04  // Bridge -> Host (echoed timestamp + bridge stats)
};

#endif // GROUND_BRIDGE_CONFIG_H
