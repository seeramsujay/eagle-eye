#include <iostream>
#include <cassert>
#include <cmath>
#include <cstring>
#include "common/protocol.h"
#include "airborne_fc/include/mahony.h"
#include "airborne_fc/include/pid.h"
#include "airborne_fc/include/mixer.h"
#include "airborne_fc/include/gps.h"
#include "airborne_fc/include/failsafe.h"

int main() {
    std::cout << "[TEST] Running Firmware Core Algorithms & Architecture Verification..." << std::endl;

    // 1. Verify Packet Sizes and Memory Alignment
    assert(sizeof(UplinkPacket) == 16);
    assert(sizeof(DownlinkPacket) == 16);
    std::cout << "  [PASS] Packet struct sizes: Uplink=" << sizeof(UplinkPacket) << "B, Downlink=" << sizeof(DownlinkPacket) << "B" << std::endl;

    // 2. Verify CRC16 CCITT
    uint8_t dummy_buf[14] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
    uint16_t crc = calculate_crc16(dummy_buf, 14);
    assert(crc != 0);

    UplinkPacket up;
    std::memset(&up, 0, sizeof(up));
    up.mode = MODE_VISION_TRACK;
    up.roll_cmd_cdeg = 1500;
    up.pitch_cmd_cdeg = 500;
    up.throttle_pct = 60;
    up.car_lat_e7 = 154000000;
    up.car_lon_e7 = 738000000;
    up.crc16 = calculate_crc16((const uint8_t *)&up, sizeof(UplinkPacket) - 2);
    assert(validate_uplink_crc(&up) == 1);
    up.roll_cmd_cdeg = 1501; // Corrupt packet
    assert(validate_uplink_crc(&up) == 0);
    std::cout << "  [PASS] CRC16 calculation and corruption rejection verified." << std::endl;

    // 3. Verify Elevon Mixer
    ElevonMixer mixer;
    int16_t left, right;
    mixer.mix(0.0f, 0.0f, left, right);
    assert(left == 1500 && right == 1500);

    // Roll right: left elevon up/down differential
    mixer.mix(100.0f, 0.0f, left, right);
    assert(left == 1600 && right == 1400);

    // Pitch up: both elevons deflect together
    mixer.mix(0.0f, 100.0f, left, right);
    assert(left == 1600 && right == 1600);

    // Clamping limits
    mixer.mix(800.0f, 800.0f, left, right);
    assert(left <= 2000 && right >= 1000);
    std::cout << "  [PASS] Elevon mixer math and servo microsecond clamping verified." << std::endl;

    // 4. Verify PID Controller
    PIDController pid(2.0f, 0.1f, 0.5f, 400.0f);
    float out1 = pid.update(10.0f, 0.0f, 0.01f);
    assert(out1 > 0.0f);
    std::cout << "  [PASS] PID controller step response verified." << std::endl;

    // 5. Verify GPS Navigation Math (Haversine & Bearing)
    // Goa to Mumbai coordinates (~435 km apart)
    double goa_lat = 15.4909, goa_lon = 73.8278;
    double mum_lat = 18.9220, mum_lon = 72.8347;
    float dist = GPSNavigation::distance_meters(goa_lat, goa_lon, mum_lat, mum_lon);
    assert(dist > 350000.0f && dist < 450000.0f); // ~395-410 km

    float bearing = GPSNavigation::bearing_degrees(goa_lat, goa_lon, mum_lat, mum_lon);
    assert(bearing > 330.0f && bearing < 360.0f); // North-Northwest

    // Heading error wrap-around check (359 deg to 1 deg)
    float herr = GPSNavigation::heading_error_degrees(359.0f, 1.0f);
    assert(std::fabs(herr - 2.0f) < 0.01f);
    float herr2 = GPSNavigation::heading_error_degrees(1.0f, 359.0f);
    assert(std::fabs(herr2 - (-2.0f)) < 0.01f);
    std::cout << "  [PASS] Great-circle Haversine, bearing, and 359->1 degree wrap-around verified." << std::endl;

    // 6. Verify 3-Tier Failsafe State Machine
    FailsafeManager fsm(1500);
    fsm.on_lora_packet_received(1000, 10.0f, 2.0f, 50, 15.49, 73.82);

    float roll_cmd = 10.0f, pitch_cmd = 2.0f;
    uint8_t thr_cmd = 50;
    FlightMode mode = MODE_VISION_TRACK;

    // At t = 2000 ms (1000 ms since last LoRa -> normal operation)
    fsm.evaluate(2000, true, 15.49, 73.82, 50.0f, 0.0f, roll_cmd, pitch_cmd, thr_cmd, mode);
    assert(fsm.current_tier == FAILSAFE_TIER_NONE);

    // At t = 3000 ms (2000 ms since last LoRa -> Tier 2 Return to Car)
    fsm.evaluate(3000, true, 15.50, 73.82, 50.0f, 0.0f, roll_cmd, pitch_cmd, thr_cmd, mode);
    assert(fsm.current_tier == FAILSAFE_TIER_2_RETURN_CAR);
    assert(mode == MODE_RETURN_TO_CAR);

    // If GPS is lost at t = 3000 ms -> Tier 3 Blind Loiter
    fsm.evaluate(3000, false, 0.0, 0.0, 0.0f, 0.0f, roll_cmd, pitch_cmd, thr_cmd, mode);
    assert(fsm.current_tier == FAILSAFE_TIER_3_BLIND_LOITER);
    assert(mode == MODE_FAILSAFE_BLIND);
    assert(std::fabs(roll_cmd - 18.0f) < 0.1f);
    assert(std::fabs(pitch_cmd - 3.0f) < 0.1f);
    assert(thr_cmd == 45);
    std::cout << "  [PASS] 3-Tier Failsafe State Machine verified." << std::endl;

    std::cout << "[SUCCESS] ALL FIRMWARE CORE TESTS PASSED (100%)!" << std::endl;
    return 0;
}
