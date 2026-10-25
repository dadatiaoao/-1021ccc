#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H
/*
 * 七轴总线公共接口与状态：应用通过提交/STOP API 表达意图，Process/Receive 推进结果。
 * MotorActionState 是软件事务状态，不能单独代替实物反馈；所有时间使用毫秒。
 */
#include "motor_service.h"

#define MOTOR_COUNT 7u
#define MOTOR_ALL 0x7Fu
#define MOTOR_WHEELS 0x0Fu
#define MOTOR_BUS_GAP_MS 10u
#define MOTOR_BUS_FRESH_MS 1600u
#define MOTOR_BUS_REPLY_MS 60u
#define MOTOR_BUS_QUIET_MS 120u
#define MOTOR_BUS_QUEUE 7u

typedef enum {
    MOTOR_IDLE, MOTOR_QUEUED, MOTOR_SENT, MOTOR_ACCEPTED, MOTOR_EXECUTING,
    MOTOR_COMPLETE, MOTOR_REJECTED, MOTOR_UNKNOWN, MOTOR_CANCELLED,
    MOTOR_STOP_PENDING, MOTOR_STOPPED
} MotorActionState;
typedef enum { MOTOR_SYNC_IDLE, MOTOR_SYNC_PREPARE, MOTOR_SYNC_TRIGGER,
               MOTOR_SYNC_VERIFY, MOTOR_SYNC_DONE, MOTOR_SYNC_LOCKED } MotorSyncState;
/*
 * 动作参数单位：速度由 command 中的 RPM 表达；位置脉冲数需配合 pulses_per_rev
 * 换算到反馈每圈 65536 计数；lease_ms 用于速度租约，timeout_ms 用于位置监视。
 */
typedef struct {
    EmmCommand command;
    uint32_t pulses_per_rev, timeout_ms, lease_ms;
} MotorAction;
/*
 * 单轴状态：data 保存反馈及其独立时间戳/序号；target_* 支持到位判断。
 * stop_unconfirmed 是持续停止意图；stop_sent/stop_attempts 记录提交，
 * stop_speed_seq/stop_status_seq 和 stop_first_ms 约束可用于确认停车的新反馈。
 */
typedef struct {
    MotorSnapshot data;
    MotorActionState action;
    uint32_t request_id, action_ms, deadline_ms, poll_due[3], quiet_until;
    uint32_t stop_speed_seq, stop_status_seq, max_age_ms, max_poll_gap_ms[3];
    uint32_t stop_retry_ms, stop_first_ms;
    uint32_t poll_last_ms[3], poll_samples[3];
    uint32_t rejected_commands;
    int64_t target_counts, previous_target;
    int32_t target_rpm;
    uint8_t target_known, previous_known, motion, failures, stop_sent, requested_enable;
    uint8_t stop_unconfirmed, stop_attempts;
    uint16_t firmware, hardware;
    uint8_t version_valid, last_notice, failure_code, failure_result;
} MotorNode;
typedef struct {
    MotorAction action;
    uint32_t id, expires_ms;
    uint8_t mask, address, code, kind;
} MotorRequest;
/*
 * 总线状态由前台独占更新。queue/wheels/sync 是待发来源，active 是当前事务；
 * pending/sent/tx_wait 区分等待事务、已提交、等待硬件结束。
 * stop_mask 仅表示首次待发位，清除此位不等于停止已确认。
 */
typedef struct {
    MotorTransport transport;
    EmmParser parser;
    MotorNode nodes[MOTOR_COUNT];
    MotorRequest queue[MOTOR_BUS_QUEUE], wheels, active;
    MotorAction sync[MOTOR_COUNT];
    MotorSyncState sync_state;
    uint8_t queue_count, wheels_pending, pending, sent, tx_wait;
    uint8_t stop_mask, poll_cursor, sync_count, sync_index, sync_mask, sync_locked;
    uint8_t group_ack, manual, diagnostic_mask;
    uint8_t stop_cursor, verify_cursor, retry_yield;
    uint32_t now_ms, next_id, next_send_ms, tx_ms, tx_limit_ms, reply_ms;
    uint32_t sync_id, sync_deadline, group_id, next_wheel_ms;
    uint32_t wheel_frames, last_wheel_ms; /* successful transport submissions, not ACKs */
    uint32_t tx_frames, tx_bytes, rx_frames, rx_bytes, ignored_frames, timeout_count, transport_errors;
    uint32_t started_ms, max_step_ms, last_step_ms;
    const char *message;
} MotorBus;

void MotorBus_Init(MotorBus *b, MotorTransport t, uint32_t now);
void MotorBus_Process(MotorBus *b, uint32_t now);
void MotorBus_Receive(MotorBus *b, const uint8_t *bytes, size_t count, uint32_t now);
void MotorBus_TransportError(MotorBus *b, MotorError error, uint32_t now);
/* Motion still requires fresh, healthy, enabled feedback plus a lease/timeout.
   The MANUAL flag is a UI-facing explicit-intent state: when
   MOTOR_DRIVER_REQUIRE_MANUAL is 0 (default) programmatic callers may command
   motion without it, and the caller owns its own explicit-intent/safety policy.
   Leaving MANUAL still requests a full stop. */
void MotorBus_SetManual(MotorBus *b, uint8_t manual, uint32_t now);
int MotorBus_Enable(MotorBus *b, uint8_t address, uint8_t enabled, uint32_t now);
int MotorBus_Submit(MotorBus *b, const MotorAction *action, uint32_t now);
/* Extend only the matching active velocity request; never starts/reverses motion. */
int MotorBus_RenewVelocity(MotorBus *b, uint8_t address, uint32_t request_id,
                          uint32_t lease_ms, uint32_t now);
int MotorBus_Wheels(MotorBus *b, const int32_t rpm[4], uint8_t acceleration,
                    uint32_t lease_ms, uint32_t now);
int MotorBus_Sync(MotorBus *b, const MotorAction *actions, uint8_t count, uint32_t now);
void MotorBus_Stop(MotorBus *b, uint8_t mask, uint32_t now);
void MotorBus_ReadVersions(MotorBus *b);
int MotorBus_Fresh(const MotorNode *n, uint8_t field, uint32_t now);
const char *MotorBus_ActionName(MotorActionState state);
const char *MotorBus_Role(uint8_t address);
#endif
