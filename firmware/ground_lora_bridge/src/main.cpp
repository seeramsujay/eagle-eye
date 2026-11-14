/**
 * Project Eagle-Eye: Ground LoRa USB-Serial Bridge
 * Bridges 433MHz SX1278 Ra-02 RF telemetry with Laptop Python Ground Station
 * Serial Link: 921600 baud deterministic binary packet framing
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "config.h"
#include "../../common/protocol.h"

static uint32_t rx_packet_count = 0;
static uint32_t tx_packet_count = 0;
static uint32_t crc_error_count = 0;

void init_ground_lora() {
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[BRIDGE_ERROR] LoRa initialization failed!");
        return;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE_4);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    LoRa.setTxPower(LORA_TX_POWER_DBM);
}

// Send framed message over USB-Serial to Laptop Ground Station
void send_serial_frame(FrameType type, const uint8_t *payload, uint8_t len) {
    uint8_t header[4];
    header[0] = FRAME_HEADER_1;
    header[1] = FRAME_HEADER_2;
    header[2] = (uint8_t)type;
    header[3] = len;

    uint16_t crc = calculate_crc16(payload, len);

    Serial.write(header, 4);
    Serial.write(payload, len);
    Serial.write((const uint8_t *)&crc, 2);
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(200);
    init_ground_lora();
}

void loop() {
    // ---------------------------------------------------------
    // 1. Receive Downlink telemetry from Airplane over LoRa RF
    // ---------------------------------------------------------
    int packet_size = LoRa.parsePacket();
    if (packet_size == sizeof(DownlinkPacket)) {
        DownlinkPacket downlink;
        LoRa.readBytes((uint8_t *)&downlink, sizeof(DownlinkPacket));

        if (validate_downlink_crc(&downlink)) {
            rx_packet_count++;
            // Forward to Laptop Ground Station over high-speed USB-Serial
            send_serial_frame(FRAME_TYPE_DOWNLINK, (const uint8_t *)&downlink, sizeof(DownlinkPacket));
        } else {
            crc_error_count++;
        }
    }

    // ---------------------------------------------------------
    // 2. Receive Uplink commands from Laptop over USB-Serial
    // ---------------------------------------------------------
    static uint8_t rx_state = 0;
    static uint8_t frame_type = 0;
    static uint8_t frame_len = 0;
    static uint8_t rx_buf[64];
    static uint8_t rx_idx = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (rx_state) {
            case 0: // Expect HEADER_1 (0x55)
                if (b == FRAME_HEADER_1) rx_state = 1;
                break;
            case 1: // Expect HEADER_2 (0xAA)
                if (b == FRAME_HEADER_2) rx_state = 2;
                else rx_state = 0;
                break;
            case 2: // Frame Type
                frame_type = b;
                rx_state = 3;
                break;
            case 3: // Payload Length
                frame_len = b;
                if (frame_len > sizeof(rx_buf) - 2) {
                    rx_state = 0; // Invalid length
                } else {
                    rx_idx = 0;
                    rx_state = 4;
                }
                break;
            case 4: // Payload Data
                rx_buf[rx_idx++] = b;
                if (rx_idx >= frame_len + 2) { // Data + 2-byte Frame CRC
                    uint16_t expected_crc = calculate_crc16(rx_buf, frame_len);
                    uint16_t received_crc = rx_buf[frame_len] | (rx_buf[frame_len + 1] << 8);

                    if (expected_crc == received_crc) {
                        if (frame_type == FRAME_TYPE_UPLINK && frame_len == sizeof(UplinkPacket)) {
                            // Transmit Uplink command over LoRa RF to plane
                            LoRa.beginPacket();
                            LoRa.write(rx_buf, sizeof(UplinkPacket));
                            LoRa.endPacket(true);
                            tx_packet_count++;
                        } else if (frame_type == FRAME_TYPE_PING) {
                            // Reply immediately with PONG and current stats for roundtrip latency measurement
                            uint8_t pong_data[8];
                            memcpy(pong_data, rx_buf, 4); // Echo back 4-byte client timestamp
                            pong_data[4] = (uint8_t)(rx_packet_count & 0xFF);
                            pong_data[5] = (uint8_t)(tx_packet_count & 0xFF);
                            pong_data[6] = (uint8_t)(crc_error_count & 0xFF);
                            pong_data[7] = (uint8_t)LoRa.packetRssi();
                            send_serial_frame(FRAME_TYPE_PONG, pong_data, sizeof(pong_data));
                        }
                    }
                    rx_state = 0;
                }
                break;
            default:
                rx_state = 0;
                break;
        }
    }
}
