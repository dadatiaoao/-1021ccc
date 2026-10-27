#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include <stdint.h>
#include "servo_math.h"

/* 两路 50Hz 舵机：TIM3_CH1 = PA6（夹爪），TIM3_CH3 = PC8（转盘）。
   时基 1MHz、周期 20000 → 1 个计数 = 1us，直接写 CCR 就是脉宽。

   安全约定：
   1) 初始化完成时两路 CCR 均为 0，等于不发脉冲，舵机不出力；
   2) 必须显式 Servo_Enable() 才输出 neutral_us；
   3) 任何超出行程的目标一律拒绝，不做静默钳位；
   4) Servo_Off() 只停脉冲（舵机卸力），夹爪可能失去保持力；不代表机械已停止或到位。 */
#define SERVO_CHANNELS 2u
enum { SERVO_JAW = 0, SERVO_TABLE = 1 };

typedef enum {
    SERVO_OK = 0,
    SERVO_REJECT_RANGE,   /* 超出该通道实测行程或硬限位 */
    SERVO_REJECT_STATE    /* 未初始化 / 通道未使能 / 参数非法 */
} ServoStatus;

void Servo_Init(void);
uint8_t Servo_IsReady(void);

/* 使能：把 CCR 置为 neutral_us 并开始跟踪目标。 */
ServoStatus Servo_Enable(uint8_t ch);
/* 停脉冲：CCR=0，舵机卸力。夹爪可能失去保持力。 */
void Servo_Off(uint8_t ch);
void Servo_OffAll(void);

ServoStatus Servo_SetTarget(uint8_t ch, int32_t us);

/* 每个主循环调用一次：按 slew 逼近目标并写 CCR。 */
void Servo_Process(uint32_t now_ms);

/* 标定结果写回。只在通道未使能时接受，避免运行中改限位。 */
ServoStatus Servo_SetLimits(uint8_t ch, const ServoLimits *limits);
const ServoLimits *Servo_GetLimits(uint8_t ch);

uint16_t Servo_CurrentUs(uint8_t ch);
int32_t Servo_TargetUs(uint8_t ch);
uint8_t Servo_IsEnabled(uint8_t ch);

#endif
