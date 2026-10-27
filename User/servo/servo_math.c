/* 舵机纯逻辑：不做任何 HAL 调用，可在 PC 上用主机测试直接编译运行。 */
#include "servo_math.h"
#include <stddef.h>

int ServoMath_LimitsValid(const ServoLimits *l)
{
    if (!l) return 0;
    if (l->min_us < SERVO_HARD_MIN_US || l->max_us > SERVO_HARD_MAX_US) return 0;
    if (l->min_us > l->max_us) return 0;
    if (l->neutral_us < l->min_us || l->neutral_us > l->max_us) return 0;
    return 1;
}

int ServoMath_Resolve(const ServoLimits *l, int32_t want_us, uint16_t *out)
{
    if (!out || !ServoMath_LimitsValid(l)) return 0;
    if (want_us < (int32_t)l->min_us || want_us > (int32_t)l->max_us) return 0;
    *out = (uint16_t)want_us;
    return 1;
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
