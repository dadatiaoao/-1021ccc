/* 夹爪舵机 CH1 —— 独立联调测试（单独编译成 servo_jaw_tests.exe）。
 *
 * 夹爪是线性机构：只关心"张开"和"闭合"两个端点，不按角度使用。
 * 本文件证明的是逻辑——端点拒绝、限速耗时；
 * 舵机是否真的夹到位，只有上机用 SERVO 页看得到。
 *
 * 用法：
 *   servo_jaw_tests.exe          跑全部断言
 *   servo_jaw_tests.exe table    打印夹爪位置表
 */
#include "servo/servo_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 示例标定值：上车实测后替换。travel_deg=0 表示这个舵机不按角度用。 */
static const ServoLimits JAW = { 1050u, 1950u, 1500u, 300u, 0u };

#define JAW_OPEN_US  1950      /* 张开 */
#define JAW_MID_US   1500      /* 中位 */
#define JAW_CLOSE_US 1400      /* 夹持（按物料实测，不要顶到 1050） */

static unsigned checks;

#define CHECK(x) do { ++checks; if (!(x)) { \
    fprintf(stderr, "[夹爪 CH1] FAIL line %d: %s\n", __LINE__, #x); \
    exit(1); } } while (0)

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
    printf("=== 夹爪舵机 %uus ~ %uus（线性机构，无角度） ===\n",
           (unsigned)JAW.min_us, (unsigned)JAW.max_us);
    printf("位置      脉宽     相对中位\n");
    printf("张开      %4uus    %+dus\n", (unsigned)JAW_OPEN_US, JAW_OPEN_US - 1500);
    printf("中位      %4uus      +0us\n", (unsigned)JAW_MID_US);
    printf("夹持      %4uus    %+dus\n", (unsigned)JAW_CLOSE_US, JAW_CLOSE_US - 1500);
    printf("闭合端点  %4uus    %+dus\n", (unsigned)JAW.min_us, (int)JAW.min_us - 1500);
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

    printf("[夹爪 CH1] 行程 %uus ~ %uus，限速 %uus/s\n",
           (unsigned)JAW.min_us, (unsigned)JAW.max_us, (unsigned)JAW.slew_us_per_s);

    /* --- 配置本身 --- */
    CHECK(ServoMath_LimitsValid(&JAW));

    /* --- 两个端点必须允许，端点外必须拒绝 --- */
    CHECK(ServoMath_Resolve(&JAW, JAW_OPEN_US, &us) && us == JAW_OPEN_US);
    printf("  张开 %dus  -> 允许\n", JAW_OPEN_US);

    CHECK(ServoMath_Resolve(&JAW, (int32_t)JAW.min_us, &us) && us == JAW.min_us);
    printf("  闭合端点 %uus  -> 允许\n", (unsigned)JAW.min_us);

    us = 4321u;
    CHECK(!ServoMath_Resolve(&JAW, 1000, &us));
    CHECK(us == 4321u);                 /* 拒绝时不得改写输出 */
    printf("  1000us  -> 拒绝（低于标定端点 50us），输出未被改写\n");

    CHECK(!ServoMath_Resolve(&JAW, 2000, &us));
    printf("  2000us  -> 拒绝（高于标定端点 50us）\n");

    CHECK(!ServoMath_Resolve(&JAW, 699, &us));
    printf("  699us   -> 拒绝（低于硬限位 700us）\n");

    /* --- 线性机构不提供角度语义 --- */
    CHECK(ServoMath_PulseToAngleDeg(&JAW, JAW_MID_US) < 0.0f);
    CHECK(!ServoMath_AngleToPulse(&JAW, 90.0f, &us));
    printf("  角度换算 -> 未启用（travel_deg=0，线性机构没有\"度\"可言）\n");

    /* --- 全行程要多久：900us / 300us 每秒 --- */
    ms = run_to(JAW.min_us, JAW_OPEN_US, JAW.slew_us_per_s, 10u, &end);
    CHECK(end == JAW_OPEN_US);
    CHECK(ms == 3000u);
    printf("  全行程 %u->%d（900us）需要 %ums\n", (unsigned)JAW.min_us, JAW_OPEN_US, ms);

    /* --- 一趟典型夹取：张开 -> 夹持 --- */
    ms = run_to(JAW_OPEN_US, JAW_CLOSE_US, JAW.slew_us_per_s, 10u, &end);
    CHECK(end == JAW_CLOSE_US);
    CHECK(ms == 1840u);
    printf("  夹取 %d->%d（550us）需要 %ums\n", JAW_OPEN_US, JAW_CLOSE_US, ms);

    /* --- 松手回程同样耗时 --- */
    ms = run_to(JAW_CLOSE_US, JAW_OPEN_US, JAW.slew_us_per_s, 10u, &end);
    CHECK(end == JAW_OPEN_US);
    CHECK(ms == 1840u);
    printf("  释放 %d->%d（550us）需要 %ums\n", JAW_CLOSE_US, JAW_OPEN_US, ms);

    /* --- 不限速：急停/复位路径必须一步到位 --- */
    ms = run_to(JAW.min_us, JAW_OPEN_US, 0u, 10u, &end);
    CHECK(end == JAW_OPEN_US);
    CHECK(ms == 10u);
    printf("  slew=0 立即到位：1 步 %ums\n", ms);

    printf("\nPASS: 夹爪 CH1 共 %u 条断言\n", checks);
    return 0;
}
