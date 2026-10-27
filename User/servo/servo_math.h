#ifndef SERVO_MATH_H
#define SERVO_MATH_H
#include <stdint.h>

/* 构建级上限：双舵机模式2600us、夹爪联调2430us、其他模式2300us。
   每一路仍另有自身范围约束；2600us超出用户提供的舵机手册标称上限。 */
#define SERVO_HARD_MIN_US 700
#if (defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH) || (defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME)
#define SERVO_HARD_MAX_US 2600
#elif defined(SERVO_BENCH_ENABLE) && SERVO_BENCH_ENABLE
#define SERVO_HARD_MAX_US 2430
#else
#define SERVO_HARD_MAX_US 2300
#endif

/* 单个舵机的行程与速率约束。min/max 必须是实测出来的机械端点再留余量，
   不是数据手册的 500/2500。 */
typedef struct {
    uint16_t min_us;
    uint16_t max_us;
    uint16_t neutral_us;      /* 使能后的第一个位置，应处于行程中段 */
    uint16_t slew_us_per_s;   /* 限速；0 表示立即到位（不建议用于带载机构） */
    uint16_t travel_deg;      /* min_us..max_us 对应的机械行程（度）；0 = 不按角度使用 */
} ServoLimits;

/* 目标是否可接受：1 = 通过，*out 写入结果；0 = 拒绝，*out 保持不变。
   故意不做静默钳位——钳位会把"标定写错了"伪装成"动作正常"。 */
int ServoMath_Resolve(const ServoLimits *limits, int32_t want_us, uint16_t *out);

/* 按 slew_us_per_s 逼近目标，返回新脉宽。
   slew=0 表示不限速（直接到目标）；dt_ms=0 表示时间未推进，原值返回。 */
int32_t ServoMath_Slew(int32_t current_us, int32_t target_us, uint32_t dt_ms,
                       uint16_t slew_us_per_s);

/* 连续调度使用此版本：fraction 保存不足 1us 的余量（0..999）。
   首次使能/停用时清零；每路舵机独立保存，避免短周期调用使限速偏快。 */
int32_t ServoMath_SlewAccumulated(int32_t current_us, int32_t target_us,
                                 uint32_t dt_ms, uint16_t slew_us_per_s,
                                 uint32_t *fraction);

/* 配置自检：硬限位内、min<=max、neutral 落在区间内、travel_deg<=360。1 = 合法。 */
int ServoMath_LimitsValid(const ServoLimits *limits);

/* 脉宽 → 角度（0..travel_deg）。travel_deg=0 或参数非法返回 -1.0f。
   角度只有在 min_us/max_us 确实对应机械端点时才有意义。 */
float ServoMath_PulseToAngleDeg(const ServoLimits *limits, uint16_t pulse_us);

/* 角度 → 脉宽（四舍五入）。1 = 通过，0 = 拒绝（越界、NaN 或 travel_deg=0）。 */
int ServoMath_AngleToPulse(const ServoLimits *limits, float angle_deg, uint16_t *out);

#endif
