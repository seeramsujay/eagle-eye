#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

// Flight Controller Mode Flags
enum FlightMode : uint8_t {
    MODE_DISARMED       = 0x00,
    MODE_MANUAL_FPV     = 0x01,
    MODE_VISION_TRACK   = 0x02,
    MODE_KALMAN_COAST   = 0x03,
    MODE_GPS_PURSUIT    = 0x04,
    MODE_RETURN_TO_CAR  = 0x05,
    MODE_FAILSAFE_BLIND = 0x06
};

// Plane Status Flags (Bitfield)
enum PlaneFlags : uint8_t {
    FLAG_ARMED          = (1 << 0),
    FLAG_GPS_FIX        = (1 << 1),
    FLAG_LORA_CONNECTED = (1 << 2),
    FLAG_FAILSAFE_ACTIVE= (1 << 3),
    FLAG_LOW_BATT       = (1 << 4)
};

// Uplink Packet: Ground Station -> Airborne FC (16 Bytes Total)
typedef struct {
    uint8_t  mode;              // FlightMode enum (1 byte)
    int16_t  roll_cmd_cdeg;     // Desired roll angle: -3500 to +3500 centidegrees (-35.00 to +35.00 deg)
    int16_t  pitch_cmd_cdeg;    // Desired pitch angle: -1500 to +2000 centidegrees (-15.00 to +20.00 deg)
    uint8_t  throttle_pct;      // 0 - 100% throttle
    int32_t  car_lat_e7;        // Car Target Latitude in 1e-7 degrees (WGS84)
    int32_t  car_lon_e7;        // Car Target Longitude in 1e-7 degrees (WGS84)
    uint16_t crc16;             // CRC-16-CCITT across previous 14 bytes
} UplinkPacket;

// Downlink Packet: Airborne FC -> Ground Station (16 Bytes Total)
typedef struct {
    int32_t  plane_lat_e7;      // Plane Latitude in 1e-7 degrees (WGS84)
    int32_t  plane_lon_e7;      // Plane Longitude in 1e-7 degrees (WGS84)
    int16_t  alt_m;             // Baro/GPS Altitude in meters (-500 to 8000m)
    uint8_t  speed_kmh;         // Ground speed in km/h (0 - 255)
    uint8_t  heading_deg_div2;  // Heading 0-358 deg divided by 2 (0 - 179)
    uint8_t  batt_v_x10;        // Battery voltage * 10 (e.g. 111 = 11.1V)
    uint8_t  state_flags;       // PlaneFlags bitfield + active FlightMode
    uint16_t crc16;             // CRC-16-CCITT across previous 14 bytes
} DownlinkPacket;

#pragma pack(pop)

// Standard CRC-16-CCITT (Polynomial 0x1021, Init 0xFFFF)
static inline uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static inline uint8_t validate_uplink_crc(const UplinkPacket *pkt) {
    uint16_t expected = calculate_crc16((const uint8_t *)pkt, sizeof(UplinkPacket) - 2);
    return (pkt->crc16 == expected);
}

static inline uint8_t validate_downlink_crc(const DownlinkPacket *pkt) {
    uint16_t expected = calculate_crc16((const uint8_t *)pkt, sizeof(DownlinkPacket) - 2);
    return (pkt->crc16 == expected);
}

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_H
