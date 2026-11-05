/* 两路舵机 PWM 驱动。只占用 TIM3 的 CH1/CH2，不开中断，不阻塞。
 *
 * 时钟：SYSCLK 180MHz → APB1 45MHz → APB1 定时器时钟 90MHz
 *       PSC = 90 - 1  → 计数频率 1MHz
 *       ARR = 20000 - 1 → 20ms 周期 = 50Hz
 *       CCR = 脉宽(us)
 *
 * 为什么不用 K230 的 PWM：K230 跑 YOLO 时 Python 调度会被阻塞，改占空比的
 * 时刻不可控，这正是归档脚本里"unexpected delayed motion"的成因。舵机需要
 * 确定性的时序，归 STM32。
 */
#include "servo_pwm.h"
#include "stm32f4xx_hal.h"

#if (defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH) || (defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME)
/* Dual manual test: PA6/TIM3_CH1 and PC8/TIM3_CH3; PA7 is not driven. */
#define SERVO_GPIO_PORT GPIOA
#define SERVO_GPIO_PINS GPIO_PIN_6
#elif defined(SERVO_TURNTABLE_BENCH) && SERVO_TURNTABLE_BENCH
/* Dedicated test firmware: PC8/TIM3_CH3 only; do not drive PA6 or PA7. */
#define SERVO_GPIO_PORT GPIOC
#define SERVO_GPIO_PINS GPIO_PIN_8
#else
#define SERVO_GPIO_PORT GPIOA
#define SERVO_GPIO_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#endif
#define SERVO_GPIO_AF   GPIO_AF2_TIM3
#define SERVO_TIM       TIM3
#define SERVO_PSC       89u      /* 90MHz / 90 = 1MHz */
#define SERVO_ARR       19999u   /* 1MHz / 20000 = 50Hz */

#if (defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH) || (defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME)
static const uint32_t servo_channel[SERVO_CHANNELS] = { TIM_CHANNEL_1, TIM_CHANNEL_3 };
#elif defined(SERVO_TURNTABLE_BENCH) && SERVO_TURNTABLE_BENCH
static const uint32_t servo_channel[SERVO_CHANNELS] = { TIM_CHANNEL_3, TIM_CHANNEL_2 };
#else
static const uint32_t servo_channel[SERVO_CHANNELS] = { TIM_CHANNEL_1, TIM_CHANNEL_2 };
#endif

static TIM_HandleTypeDef servo_tim;
static ServoLimits servo_limits[SERVO_CHANNELS] = {
    { 1000u, 2000u, 1500u, 300u, 0u },   /* 夹爪：保守窗口；线性机构，不按角度用 */
    { 1000u, 2000u, 1500u, 300u, 0u }    /* 升降：标定后填 travel_deg=180 才能用角度 */
};
static int32_t  servo_current[SERVO_CHANNELS] = { 1500, 1500 };
static int32_t  servo_target[SERVO_CHANNELS]  = { 1500, 1500 };
static uint32_t servo_last_ms[SERVO_CHANNELS];
static uint32_t servo_fraction[SERVO_CHANNELS];
static uint8_t  servo_enabled[SERVO_CHANNELS];
static uint8_t  servo_ready;

static void servo_write(uint8_t ch, int32_t us)
{
    __HAL_TIM_SET_COMPARE(&servo_tim, servo_channel[ch], (uint32_t)(us > 0 ? us : 0));
}

void Servo_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_ClockConfigTypeDef clk = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint8_t i;

#if (defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH) || (defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
#elif defined(SERVO_TURNTABLE_BENCH) && SERVO_TURNTABLE_BENCH
    __HAL_RCC_GPIOC_CLK_ENABLE();
#else
    __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin = SERVO_GPIO_PINS;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = SERVO_GPIO_AF;
    HAL_GPIO_Init(SERVO_GPIO_PORT, &gpio);
#if (defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH) || (defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME)
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &gpio);
#endif

    servo_tim.Instance = SERVO_TIM;
    servo_tim.Init.Prescaler = SERVO_PSC;
    servo_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    servo_tim.Init.Period = SERVO_ARR;
    servo_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&servo_tim) != HAL_OK) return;
    /* This project uses the older HAL without Init.AutoReloadPreload. */
    SET_BIT(servo_tim.Instance->CR1, TIM_CR1_ARPE);

    clk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&servo_tim, &clk) != HAL_OK) return;
    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&servo_tim, &master) != HAL_OK) return;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0u;                 /* 先给 0：不产生脉冲，舵机不出力 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    for (i = 0u; i < SERVO_CHANNELS; ++i) {
        if (HAL_TIM_PWM_ConfigChannel(&servo_tim, &oc, servo_channel[i]) != HAL_OK) return;
        servo_current[i] = servo_limits[i].neutral_us;
        servo_target[i]  = servo_limits[i].neutral_us;
        servo_enabled[i] = 0u;
        servo_last_ms[i] = 0u;
        servo_fraction[i] = 0u;
        if (HAL_TIM_PWM_Start(&servo_tim, servo_channel[i]) != HAL_OK) return;
        servo_write(i, 0);
    }
    servo_ready = 1u;
}

uint8_t Servo_IsReady(void) { return servo_ready; }

ServoStatus Servo_Enable(uint8_t ch)
{
    if (!servo_ready || ch >= SERVO_CHANNELS) return SERVO_REJECT_STATE;
    if (!ServoMath_LimitsValid(&servo_limits[ch])) return SERVO_REJECT_STATE;
    servo_current[ch] = servo_limits[ch].neutral_us;
    servo_target[ch]  = servo_limits[ch].neutral_us;
    servo_last_ms[ch] = HAL_GetTick();
    servo_fraction[ch] = 0u;
    servo_write(ch, servo_current[ch]);
    servo_enabled[ch] = 1u;
    return SERVO_OK;
}

void Servo_Off(uint8_t ch)
{
    if (ch >= SERVO_CHANNELS) return;
    servo_write(ch, 0);
    servo_enabled[ch] = 0u;
    servo_fraction[ch] = 0u;
}

void Servo_OffAll(void)
{
    uint8_t i;
    for (i = 0u; i < SERVO_CHANNELS; ++i) Servo_Off(i);
}

ServoStatus Servo_SetTarget(uint8_t ch, int32_t us)
{
    uint16_t resolved;
    if (!servo_ready || ch >= SERVO_CHANNELS) return SERVO_REJECT_STATE;
    if (!servo_enabled[ch]) return SERVO_REJECT_STATE;
    if (!ServoMath_Resolve(&servo_limits[ch], us, &resolved)) return SERVO_REJECT_RANGE;
    servo_target[ch] = (int32_t)resolved;
    return SERVO_OK;
}

ServoStatus Servo_Jog(uint8_t ch, int32_t delta_us)
{
    if (!servo_ready || ch >= SERVO_CHANNELS) return SERVO_REJECT_STATE;
    return Servo_SetTarget(ch, servo_current[ch] + delta_us);
}

void Servo_Process(uint32_t now_ms)
{
    uint32_t dt;
    uint8_t i;
    if (!servo_ready) return;
    for (i = 0u; i < SERVO_CHANNELS; ++i) {
        if (!servo_enabled[i]) continue;
        if (servo_current[i] == servo_target[i]) {
            servo_last_ms[i] = now_ms;
            servo_fraction[i] = 0u;
            continue;
        }
        dt = (uint32_t)(now_ms - servo_last_ms[i]);
        if (dt < 1u) continue;
        servo_current[i] = ServoMath_SlewAccumulated(servo_current[i], servo_target[i], dt,
                                          servo_limits[i].slew_us_per_s, &servo_fraction[i]);
        servo_last_ms[i] = now_ms;
        servo_write(i, servo_current[i]);
    }
}

ServoStatus Servo_SetLimits(uint8_t ch, const ServoLimits *limits)
{
    if (ch >= SERVO_CHANNELS || !limits) return SERVO_REJECT_STATE;
    if (servo_enabled[ch]) return SERVO_REJECT_STATE;   /* 运行中不改限位 */
    if (!ServoMath_LimitsValid(limits)) return SERVO_REJECT_RANGE;
    servo_limits[ch] = *limits;
    servo_current[ch] = limits->neutral_us;
    servo_target[ch]  = limits->neutral_us;
    return SERVO_OK;
}

const ServoLimits *Servo_GetLimits(uint8_t ch)
{
    return ch < SERVO_CHANNELS ? &servo_limits[ch] : 0;
}

uint16_t Servo_CurrentUs(uint8_t ch)
{
    return ch < SERVO_CHANNELS && servo_current[ch] > 0 ? (uint16_t)servo_current[ch] : 0u;
}

int32_t Servo_TargetUs(uint8_t ch)
{
    return ch < SERVO_CHANNELS ? servo_target[ch] : -1;
}

uint8_t Servo_IsEnabled(uint8_t ch)
{
    return ch < SERVO_CHANNELS ? servo_enabled[ch] : 0u;
}
