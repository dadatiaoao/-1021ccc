#ifndef SERVO_MATH_H
#define SERVO_MATH_H
#include <stdint.h>

/* Per-channel limits remain mandatory. Values are pulse commands, not measured positions. */
#define SERVO_HARD_MIN_US 700
#define SERVO_HARD_MAX_US 2600

/* 单个舵机的行程与速率约束。min/max 必须是实测出来的机械端点再留余量，
   不是数据手册的 500/2500。 */
typedef struct {
    uint16_t min_us;
    uint16_t max_us;
    uint16_t neutral_us;      /* 使能后的初始脉宽，不表示实测位置 */
    uint16_t slew_us_per_s;   /* 限速；0 表示立即输出目标脉宽（不建议用于带载机构） */
} ServoLimits;

/* 目标是否可接受：1 = 通过，*out 写入结果；0 = 拒绝，*out 保持不变。
   故意不做静默钳位——钳位会把"标定写错了"伪装成"动作正常"。 */
int ServoMath_Resolve(const ServoLimits *limits, int32_t want_us, uint16_t *out);

/* 连续调度使用此版本：fraction 保存不足 1us 的余量（0..999）。
   首次使能/停用时清零；每路舵机独立保存，避免短周期调用使限速偏快。 */
int32_t ServoMath_SlewAccumulated(int32_t current_us, int32_t target_us,
                                 uint32_t dt_ms, uint16_t slew_us_per_s,
                                 uint32_t *fraction);

/* 配置自检：硬限位内、min<=max、neutral 落在区间内。1 = 合法。 */
int ServoMath_LimitsValid(const ServoLimits *limits);

#endif
