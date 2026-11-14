#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>
#include "protocol.h"
#include "gps.h"

enum FailsafeTier : uint8_t {
    FAILSAFE_TIER_NONE          = 0,
    FAILSAFE_TIER_1_HOLD        = 1,    // LoRa Drop < 1.5s: Hold last valid attitude/throttle
    FAILSAFE_TIER_2_RETURN_CAR  = 2,    // LoRa Drop > 1.5s (with GPS): Fly to car / +20m boost & 22° bank orbit
    FAILSAFE_TIER_3_BLIND_LOITER= 3     // LoRa Drop > 1.5s (no GPS): 18° bank, +3° nose-up, 45% throttle
};

class FailsafeManager {
public:
    uint32_t last_lora_rx_ms;
    uint32_t timeout_tier1_ms; // 1500 ms
    FailsafeTier current_tier;

    // Last valid commands received via LoRa
    float last_roll_cmd;
    float last_pitch_cmd;
    uint8_t last_throttle_cmd;

    // Home / Car target coordinates
    double target_lat;
    double target_lon;
    bool has_target_coords;
    float baseline_alt_m;
    bool altitude_boost_engaged;

    FailsafeManager(uint32_t timeout_ms = 1500)
        : last_lora_rx_ms(0), timeout_tier1_ms(timeout_ms), current_tier(FAILSAFE_TIER_NONE),
          last_roll_cmd(0.0f), last_pitch_cmd(0.0f), last_throttle_cmd(0),
          target_lat(0.0), target_lon(0.0), has_target_coords(false),
          baseline_alt_m(0.0f), altitude_boost_engaged(false) {}

    void on_lora_packet_received(uint32_t current_ms, float roll_cmd, float pitch_cmd, uint8_t throttle, double car_lat, double car_lon) {
        last_lora_rx_ms = current_ms;
        last_roll_cmd = roll_cmd;
        last_pitch_cmd = pitch_cmd;
        last_throttle_cmd = throttle;

        if (car_lat != 0.0 && car_lon != 0.0) {
            target_lat = car_lat;
            target_lon = car_lon;
            has_target_coords = true;
        }

        current_tier = FAILSAFE_TIER_NONE;
        altitude_boost_engaged = false;
    }

    void evaluate(uint32_t current_ms, bool has_gps_lock, double plane_lat, double plane_lon, float plane_alt, float plane_heading,
                  float &active_roll_cmd, float &active_pitch_cmd, uint8_t &active_throttle_cmd, FlightMode &active_mode) {
        
        uint32_t elapsed_ms = (current_ms >= last_lora_rx_ms) ? (current_ms - last_lora_rx_ms) : 0;

        // Condition 0: Normal LoRa link active (< 1500 ms)
        if (elapsed_ms < timeout_tier1_ms) {
            current_tier = FAILSAFE_TIER_NONE;
            // Commanded values pass through
            return;
        }

        // Condition 1: Short LoRa Drop (< 1.5s is handled by not triggering; if drop is between 1.5s and first detection, tier 1 holds)
        // Here elapsed_ms >= 1500ms:
        if (has_gps_lock && has_target_coords) {
            // Tier 2: Return to Car with +20m terrain clearance & coordinated 22° orbit
            current_tier = FAILSAFE_TIER_2_RETURN_CAR;
            active_mode = MODE_RETURN_TO_CAR;

            if (!altitude_boost_engaged) {
                baseline_alt_m = plane_alt;
                altitude_boost_engaged = true;
            }

            float dist_to_target = GPSNavigation::distance_meters(plane_lat, plane_lon, target_lat, target_lon);
            float target_bearing = GPSNavigation::bearing_degrees(plane_lat, plane_lon, target_lat, target_lon);
            float heading_err = GPSNavigation::heading_error_degrees(plane_heading, target_bearing);

            // Terrain-clearance altitude boost: command +5° nose-up until +20m above baseline
            if (plane_alt < (baseline_alt_m + 20.0f)) {
                active_pitch_cmd = 5.0f; // climb to clear obstacles/coastal ridges
            } else {
                active_pitch_cmd = 2.0f; // cruise level pitch
            }

            if (dist_to_target > 75.0f) {
                // Intercept navigation toward car
                active_roll_cmd = GPSNavigation::coordinated_turn_bank_angle(heading_err, 14.0f, 25.0f);
                active_throttle_cmd = 65; // Cruise throttle (65%)
            } else {
                // Within 75m orbit radius: enter sustained coordinated 22° bank loiter circle
                active_roll_cmd = 22.0f;
                active_pitch_cmd = 3.0f; // slight nose up to maintain altitude in 22° bank
                active_throttle_cmd = 60;
            }
        } else {
            // Tier 3: No GPS lock failsafe - Safe blind loiter circle
            current_tier = FAILSAFE_TIER_3_BLIND_LOITER;
            active_mode = MODE_FAILSAFE_BLIND;

            // Autonomous safe 18° bank, +3° nose-up pitch, 45% throttle blind loiter circle
            active_roll_cmd = 18.0f;
            active_pitch_cmd = 3.0f;
            active_throttle_cmd = 45;
        }
    }
};

#endif // FAILSAFE_H
