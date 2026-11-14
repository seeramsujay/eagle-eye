#ifndef GPS_NAV_H
#define GPS_NAV_H

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class GPSNavigation {
public:
    static constexpr double EARTH_RADIUS_M = 6371000.0;
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    static constexpr float GRAVITY_MPS2 = 9.80665f;

    // Computes Great-Circle distance in meters using Haversine formula
    static float distance_meters(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
        double dlat = (lat2_deg - lat1_deg) * DEG_TO_RAD;
        double dlon = (lon2_deg - lon1_deg) * DEG_TO_RAD;
        double lat1_rad = lat1_deg * DEG_TO_RAD;
        double lat2_rad = lat2_deg * DEG_TO_RAD;

        double a = sin(dlat * 0.5) * sin(dlat * 0.5) +
                   cos(lat1_rad) * cos(lat2_rad) * sin(dlon * 0.5) * sin(dlon * 0.5);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        return (float)(EARTH_RADIUS_M * c);
    }

    // Computes initial Great-Circle bearing in degrees [0, 360) from Point 1 to Point 2
    static float bearing_degrees(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
        double dlon = (lon2_deg - lon1_deg) * DEG_TO_RAD;
        double lat1_rad = lat1_deg * DEG_TO_RAD;
        double lat2_rad = lat2_deg * DEG_TO_RAD;

        double y = sin(dlon) * cos(lat2_rad);
        double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
        double initial_bearing_rad = atan2(y, x);
        double initial_bearing_deg = initial_bearing_rad * RAD_TO_DEG;

        // Normalise to [0, 360)
        double bearing = fmod(initial_bearing_deg + 360.0, 360.0);
        return (float)bearing;
    }

    // Computes heading error in degrees [-180, +180] from current heading to target bearing
    static float heading_error_degrees(float current_heading_deg, float target_bearing_deg) {
        float diff = target_bearing_deg - current_heading_deg;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        return diff;
    }

    // Computes aerodynamic coordinated turn bank angle: tan(phi) = (V * omega) / g
    static float coordinated_turn_bank_angle(float heading_error_deg, float airspeed_mps = 12.0f, float max_bank_deg = 25.0f) {
        // Desired turn rate omega: proportional to heading error (clamp to 15 deg/s)
        float max_turn_rate_deg_s = 15.0f;
        float omega_deg_s = heading_error_deg * 0.4f;
        if (omega_deg_s > max_turn_rate_deg_s) omega_deg_s = max_turn_rate_deg_s;
        if (omega_deg_s < -max_turn_rate_deg_s) omega_deg_s = -max_turn_rate_deg_s;

        float omega_rad_s = omega_deg_s * (float)DEG_TO_RAD;
        float centripetal_accel = airspeed_mps * omega_rad_s;
        float target_bank_rad = atan2f(centripetal_accel, GRAVITY_MPS2);
        float target_bank_deg = target_bank_rad * (float)RAD_TO_DEG;

        if (target_bank_deg > max_bank_deg) target_bank_deg = max_bank_deg;
        if (target_bank_deg < -max_bank_deg) target_bank_deg = -max_bank_deg;
        return target_bank_deg;
    }
};

#endif // GPS_NAV_H
