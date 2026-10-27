/* 升降舵机 CH2 —— 独立联调测试（单独编译成 servo_lift_tests.exe）。
 *
 * 升降是转动机构：1200~1800us 对应 0~180 度，1 度约 3.33us。
 * 本文件证明的是逻辑——角度换算、越界拒绝、转动耗时；
 * 舵机是否真的抬到 60 度，只有上机用 SERVO 页看得到。
 *
 * 用法：
 *   servo_lift_tests.exe          跑全部断言
 *   servo_lift_tests.exe table    打印 角度 -> 脉宽 -> 耗时 标定表
 */
#include "servo/servo_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 示例标定值：上车实测后替换。travel_deg=180 表示 1200~1800us 走满 180 度。 */
static const ServoLimits LIFT = { 1200u, 1800u, 1500u, 300u, 180u };

static unsigned checks;

#define CHECK(x) do { ++checks; if (!(x)) { \
    fprintf(stderr, "[升降 CH2] FAIL line %d: %s\n", __LINE__, #x); \
    exit(1); } } while (0)

static int near(float a, float b)
{
    float d = a - b;
    return d > -0.05f && d < 0.05f;
}

/* 模拟主循环：每 dt_ms 调一次 Slew，直到到位，返回耗时(ms)。 */
static uint32_t run_to(int32_t from, int32_t to, uint16_t slew, uint32_t dt_ms,
                       int32_t *end)
{
    int32_t cur = from;
    uint32_t ms = 0;
    unsigned guard = 0;
    while (cur != to && guard++ < 100000u) {
        cur = ServoMath_Slew(cur, to, dt_ms, slew);
        ms += dt_ms;
    }
    if (end) *end = cur;
    return ms;
}

static void print_table(void)
{
    int deg;
    printf("=== 升降舵机 %uus ~ %uus = 0 ~ %u 度，限速 %uus/s ===\n",
           (unsigned)LIFT.min_us, (unsigned)LIFT.max_us,
           (unsigned)LIFT.travel_deg, (unsigned)LIFT.slew_us_per_s);
    printf("角度    脉宽     相对90度    从90度出发耗时\n");
    for (deg = 0; deg <= 180; deg += 15) {
        uint16_t us = 0;
        int32_t end = 0;
        uint32_t ms;
        if (!ServoMath_AngleToPulse(&LIFT, (float)deg, &us)) continue;
        ms = run_to(1500, (int32_t)us, LIFT.slew_us_per_s, 10u, &end);
        printf("%4d    %4uus   %+5dus      %ums\n",
               deg, (unsigned)us, (int)us - 1500, ms);
    }
}

int main(int argc, char **argv)
{
    uint16_t us = 0;
    int32_t end = 0;
    uint32_t ms;

    if (argc > 1 && strcmp(argv[1], "table") == 0) {
        print_table();
        return 0;
    }

    printf("[升降 CH2] %uus ~ %uus = 0 ~ %u 度，限速 %uus/s\n",
           (unsigned)LIFT.min_us, (unsigned)LIFT.max_us,
           (unsigned)LIFT.travel_deg, (unsigned)LIFT.slew_us_per_s);

    /* --- 配置本身 --- */
    CHECK(ServoMath_LimitsValid(&LIFT));

    /* --- 角度 -> 脉宽 --- */
    CHECK(ServoMath_AngleToPulse(&LIFT, 0.0f, &us) && us == 1200u);
    printf("  0 度   -> %uus（最低位）\n", (unsigned)us);

    CHECK(ServoMath_AngleToPulse(&LIFT, 60.0f, &us) && us == 1400u);
    printf("  60 度  -> %uus\n", (unsigned)us);

    CHECK(ServoMath_AngleToPulse(&LIFT, 90.0f, &us) && us == 1500u);
    printf("  90 度  -> %uus（中位，也是上电位置）\n", (unsigned)us);

    CHECK(ServoMath_AngleToPulse(&LIFT, 180.0f, &us) && us == 1800u);
    printf("  180 度 -> %uus（最高位）\n", (unsigned)us);

    /* --- 脉宽 -> 角度（反算） --- */
    CHECK(near(ServoMath_PulseToAngleDeg(&LIFT, 1400u), 60.0f));
    CHECK(near(ServoMath_PulseToAngleDeg(&LIFT, 1500u), 90.0f));
    CHECK(near(ServoMath_PulseToAngleDeg(&LIFT, 1800u), 180.0f));
    printf("  反算 1400us -> %.1f 度，1500us -> %.1f 度\n",
           (double)ServoMath_PulseToAngleDeg(&LIFT, 1400u),
           (double)ServoMath_PulseToAngleDeg(&LIFT, 1500u));

    /* --- 越界角度必须拒绝 --- */
    CHECK(!ServoMath_AngleToPulse(&LIFT, 181.0f, &us));
    CHECK(!ServoMath_AngleToPulse(&LIFT, -1.0f, &us));
    CHECK(!ServoMath_AngleToPulse(&LIFT, NAN, &us));
    printf("  181 度 / -1 度 / NaN -> 拒绝\n");

    /* --- 脉宽越界时反算同样拒绝 --- */
    CHECK(ServoMath_PulseToAngleDeg(&LIFT, 1199u) < 0.0f);
    CHECK(ServoMath_PulseToAngleDeg(&LIFT, 1801u) < 0.0f);
    printf("  1199us / 1801us 反算 -> 拒绝\n");

    /* --- 转动 30 度要多久：90 -> 120 度，走 100us --- */
    ms = run_to(1500, 1600, LIFT.slew_us_per_s, 10u, &end);
    CHECK(end == 1600);
    CHECK(ms == 340u);
    printf("  90 -> 120 度（30 度 = 100us）需要 %ums\n", ms);

    /* --- 整段 0 -> 180 度 --- */
    ms = run_to(1200, 1800, LIFT.slew_us_per_s, 10u, &end);
    CHECK(end == 1800);
    CHECK(ms == 2000u);
    printf("  0 -> 180 度（600us）需要 %ums\n", ms);

    /* --- 不限速：复位路径必须一步到位 --- */
    ms = run_to(1200, 1800, 0u, 10u, &end);
    CHECK(end == 1800);
    CHECK(ms == 10u);
    printf("  slew=0 立即到位：1 步 %ums\n", ms);

    printf("\nPASS: 升降 CH2 共 %u 条断言\n", checks);
    return 0;
}
