/* 舵机纯逻辑：不做任何 HAL 调用，可在 PC 上用主机测试直接编译运行。 */
#include "servo_math.h"
#include <stddef.h>

int ServoMath_LimitsValid(const ServoLimits *l)
{
    if (!l) return 0;
    if (l->min_us < SERVO_HARD_MIN_US || l->max_us > SERVO_HARD_MAX_US) return 0;
    if (l->min_us > l->max_us) return 0;
    if (l->neutral_us < l->min_us || l->neutral_us > l->max_us) return 0;
    if (l->travel_deg > 360u) return 0;
    return 1;
}

int ServoMath_Resolve(const ServoLimits *l, int32_t want_us, uint16_t *out)
{
    if (!out || !ServoMath_LimitsValid(l)) return 0;
    if (want_us < (int32_t)l->min_us || want_us > (int32_t)l->max_us) return 0;
    *out = (uint16_t)want_us;
    return 1;
}

int32_t ServoMath_Slew(int32_t current_us, int32_t target_us, uint32_t dt_ms,
                       uint16_t slew_us_per_s)
{
    uint32_t fraction = 0u;
    return ServoMath_SlewAccumulated(current_us, target_us, dt_ms,
                                     slew_us_per_s, &fraction);
}

int32_t ServoMath_SlewAccumulated(int32_t current_us, int32_t target_us,
                                 uint32_t dt_ms, uint16_t slew_us_per_s,
                                 uint32_t *fraction)
{
    int32_t step;
    uint32_t budget;
    if (current_us == target_us || !slew_us_per_s) {
        if (fraction) *fraction = 0u;
        return target_us;
    }
    if (!dt_ms) return current_us;          /* 时间没走，绝不推进 */
    /* 保留亚微秒余量；停顿超过1秒时仍只补最多1秒的运动。 */
    budget = (uint32_t)slew_us_per_s * (dt_ms > 1000u ? 1000u : dt_ms);
    if (fraction) budget += *fraction % 1000u;
    step = (int32_t)(budget / 1000u);
    if (fraction) *fraction = budget % 1000u;
    if (target_us > current_us) {
        current_us += step;
        if (current_us > target_us) current_us = target_us;
    } else {
        current_us -= step;
        if (current_us < target_us) current_us = target_us;
    }
    if (current_us == target_us && fraction) *fraction = 0u;
    return current_us;
}

float ServoMath_PulseToAngleDeg(const ServoLimits *l, uint16_t pulse_us)
{
    if (!l || !l->travel_deg || l->max_us <= l->min_us) return -1.0f;
    if (pulse_us < l->min_us || pulse_us > l->max_us) return -1.0f;
    return (float)(pulse_us - l->min_us) * (float)l->travel_deg /
           (float)(l->max_us - l->min_us);
}

int ServoMath_AngleToPulse(const ServoLimits *l, float angle_deg, uint16_t *out)
{
    float span_us;
    int32_t us;
    if (!out || !l || !l->travel_deg || l->max_us <= l->min_us) return 0;
    if (!(angle_deg >= 0.0f)) return 0;                 /* 同时挡住 NaN */
    if (angle_deg > (float)l->travel_deg) return 0;
    span_us = (float)(l->max_us - l->min_us);
    us = (int32_t)((float)l->min_us + angle_deg * span_us / (float)l->travel_deg + 0.5f);
    return ServoMath_Resolve(l, us, out);
}
