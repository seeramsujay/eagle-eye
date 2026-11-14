#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include <algorithm>

class ElevonMixer {
public:
    int16_t trim_left_us;
    int16_t trim_right_us;
    float roll_gain;
    float pitch_gain;
    int16_t min_us;
    int16_t max_us;
    int16_t mid_us;

    ElevonMixer(int16_t trim_l = 0, int16_t trim_r = 0, float r_gain = 1.0f, float p_gain = 1.0f,
                int16_t min_val = 1000, int16_t max_val = 2000, int16_t mid_val = 1500)
        : trim_left_us(trim_l), trim_right_us(trim_r), roll_gain(r_gain), pitch_gain(p_gain),
          min_us(min_val), max_us(max_val), mid_us(mid_val) {}

    // Computes left and right servo pulse widths in microseconds
    // roll_corr and pitch_corr are PID corrections (-400 to +400 us)
    void mix(float roll_corr, float pitch_corr, int16_t &left_us, int16_t &right_us) {
        float roll_eff = roll_corr * roll_gain;
        float pitch_eff = pitch_corr * pitch_gain;

        // Elevon mixing rule:
        // Left elevon = Pitch + Roll
        // Right elevon = Pitch - Roll
        float raw_left = (float)mid_us + trim_left_us + (pitch_eff + roll_eff);
        float raw_right = (float)mid_us + trim_right_us + (pitch_eff - roll_eff);

        // Clamping to hardware physical limits
        if (raw_left < min_us) raw_left = min_us;
        if (raw_left > max_us) raw_left = max_us;

        if (raw_right < min_us) raw_right = min_us;
        if (raw_right > max_us) raw_right = max_us;

        left_us = (int16_t)raw_left;
        right_us = (int16_t)raw_right;
    }

    // Convert throttle percentage (0-100%) to ESC pulse width (1000 - 2000 us)
    static int16_t throttle_to_us(uint8_t throttle_pct) {
        if (throttle_pct > 100) throttle_pct = 100;
        return 1000 + ((int32_t)throttle_pct * 1000) / 100;
    }
};

#endif // MIXER_H
