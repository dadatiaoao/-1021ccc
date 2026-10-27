/*
 * 七电机共享串口调度层：地址 1/2/3/4 对应前左/前右/后左/后右，
 * 5=旋转臂、6=升降、7=前后微调；同一时刻只保留一个可匹配的线上事务。
 * 发送提交、ACK 接受、反馈执行/到位、停车确认分别建模；普通运动不因超时自动重放。
 * STOP 意图持续保留并限频重试，只有首次提交之后的新鲜速度和状态才能确认停车。
 */
#include "motor_bus.h"
#include "motor_test_config.h"
#include <string.h>

enum { QUERY, CONTROL, GROUP, PREPARE, TRIGGER, STOP, DIAGNOSTIC };
static const uint8_t codes[3] = { EMM_SPEED, EMM_POSITION, EMM_STATUS };
static int due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static uint8_t bit(uint8_t a) { return (uint8_t)(1u << (a - 1u)); }
static int address_ok(uint8_t a) { return a >= 1u && a <= MOTOR_COUNT; }
static int mask_ready(const MotorBus *b, uint8_t mask, uint32_t now)
{
    unsigned i;
    for (i = 0u; i < MOTOR_COUNT; ++i)
        if ((mask & (1u << i)) && !due(now, b->nodes[i].quiet_until)) return 0;
    return 1;
}
/*
 * 检查指定单个反馈字段是否有效且未过期；online 或 valid 位本身不等于当前新鲜。
 */
int MotorBus_Fresh(const MotorNode *n, uint8_t field, uint32_t now)
{
    uint32_t at = field == MOTOR_VALID_POSITION ? n->data.position_ms :
                  field == MOTOR_VALID_SPEED ? n->data.speed_ms : n->data.status_ms;
    return (n->data.valid & field) && (uint32_t)(now - at) <= MOTOR_BUS_FRESH_MS;
}
const char *MotorBus_Role(uint8_t a)
{
    static const char *names[7] = {"FL", "FR", "RL", "RR", "ARM", "LIFT", "TRIM"};
    return address_ok(a) ? names[a - 1u] : "?";
}
const char *MotorBus_ActionName(MotorActionState s)
{
    static const char *names[] = {"IDLE", "QUEUED", "SENT", "ACK ONLY", "EXECUTING",
        "COMPLETE", "REJECTED", "UNKNOWN", "CANCELLED", "STOP UNCONF", "STOPPED"};
    return (unsigned)s < sizeof(names) / sizeof(names[0]) ? names[s] : "?";
}
/*
 * 检查掩码内轴的在线/状态新鲜度、故障、使能要求及停止/未知状态，供运动入口复用。
 */
static int healthy(const MotorBus *b, uint8_t mask, uint32_t now, uint8_t enabled)
{
    unsigned i;
    for (i = 0u; i < MOTOR_COUNT; ++i) if (mask & (1u << i)) {
        const MotorNode *n = &b->nodes[i];
        if (!n->data.online || !MotorBus_Fresh(n, MOTOR_VALID_STATUS, now) ||
            (n->data.status_flags & 0x0Cu) || (enabled && !(n->data.status_flags & 1u)) ||
            n->stop_unconfirmed || n->action == MOTOR_STOP_PENDING || n->action == MOTOR_UNKNOWN) return 0;
    }
    return 1;
}
static uint32_t new_id(MotorBus *b) { if (!++b->next_id) ++b->next_id; return b->next_id; }
static void state(MotorBus *b, uint8_t mask, MotorActionState s, uint32_t id, uint32_t now)
{
    unsigned i;
    for (i = 0u; i < MOTOR_COUNT; ++i) if (mask & (1u << i)) {
        b->nodes[i].action = s; b->nodes[i].request_id = id; b->nodes[i].action_ms = now;
    }
}
static void sync_fail(MotorBus *b)
{
    b->sync_locked = 1u; b->sync_state = MOTOR_SYNC_LOCKED;
    b->message = "SYNC CACHE UNKNOWN / LOCKED";
}
/*
 * 清空调度器，绑定传输接口并分散七轴的三类轮询期限；
 * 同步缓存是否可信只由明确的启动配置决定，主控复位不能自动证明驱动缓存干净。
 */
void MotorBus_Init(MotorBus *b, MotorTransport t, uint32_t now)
{
    unsigned i, j;
    memset(b, 0, sizeof(*b)); b->transport = t;
    b->now_ms = b->started_ms = b->last_step_ms = now;
    b->next_send_ms = b->next_wheel_ms = now;
    b->sync_locked = (uint8_t)!MOTOR_SYNC_CLEAN_BOOT_CONFIRMED;
    b->sync_state = b->sync_locked ? MOTOR_SYNC_LOCKED : MOTOR_SYNC_IDLE;
    b->message = "READ ONLY";
    EmmParser_Init(&b->parser, 0u);
    for (i = 0u; i < MOTOR_COUNT; ++i) {
        b->nodes[i].data.address = (uint8_t)(i + 1u); b->nodes[i].quiet_until = now;
        for (j = 0u; j < 3u; ++j) b->nodes[i].poll_due[j] = now + (i * 3u + j) * 24u;
    }
}
/*
 * 验证速度/位置帧、转速、模式及租约/超时范围，阻止无效参数进入调度队列。
 */
static int action_ok(const MotorAction *a)
{
    const EmmCommand *c;
    uint16_t rpm;
    if (!a) return 0;
    c = &a->command;
    if (!address_ok(c->bytes[0]) || c->bytes[1] != c->code || c->bytes[2] > 1u) return 0;
    rpm = (uint16_t)(((uint16_t)c->bytes[3] << 8) | c->bytes[4]);
    if (rpm > 3000u) return 0;
    if (c->code == EMM_VELOCITY) return c->length == 8u && c->bytes[6] <= 1u &&
        c->bytes[7] == 0x6Bu && a->lease_ms >= 100u && a->lease_ms <= 5000u;
    if (c->code == EMM_MOVE) return c->length == 13u && c->bytes[10] <= 2u &&
        c->bytes[11] <= 1u && c->bytes[12] == 0x6Bu && a->pulses_per_rev > 0u &&
        a->pulses_per_rev <= 1000000u && a->timeout_ms >= 100u && a->timeout_ms <= 60000u;
    return 0;
}
/*
 * 把协议中的脉冲目标换算为反馈计数，建立后续到位判断的可观测目标。
 * 相对前目标必须有已知历史，相对当前位置必须有新鲜位置；超出反馈表示范围则拒绝。
 */
static int set_target(MotorNode *n, const MotorAction *a, uint32_t now)
{
    const uint8_t *c = a->command.bytes;
    uint32_t pulses;
    int64_t delta;
    if (a->command.code == EMM_VELOCITY) {
        n->target_rpm = ((int32_t)c[3] << 8) | c[4];
        if (c[2]) n->target_rpm = -n->target_rpm;
        n->target_known = n->previous_known = 0u; return 1;
    }
    pulses = ((uint32_t)c[6] << 24) | ((uint32_t)c[7] << 16) | ((uint32_t)c[8] << 8) | c[9];
    delta = (int64_t)pulses * 65536 / a->pulses_per_rev;
    if (c[2]) delta = -delta;
    if (c[10] == EMM_REL_TARGET && !n->previous_known) return 0;
    if (c[10] == EMM_REL_ACTUAL && !MotorBus_Fresh(n, MOTOR_VALID_POSITION, now)) return 0;
    n->target_counts = delta + (c[10] == EMM_REL_TARGET ? n->previous_target :
        c[10] == EMM_REL_ACTUAL ? n->data.position_counts : 0);
    /* Feedback wire magnitude is 32 bits. Reject targets outside observable range. */
    if (n->target_counts < -INT64_C(4294967295) || n->target_counts > INT64_C(4294967295)) return 0;
    n->target_known = 1u; return 1;
}
/*
 * 检查队列容量、同步占用和轴冲突，分配请求编号及发送前有效期；成功仅代表已排队。
 */
static int enqueue(MotorBus *b, MotorRequest r, uint32_t now)
{
    unsigned i;
#if MOTOR_DRIVER_REQUIRE_MANUAL
    if (!b->manual) { b->message = "MANUAL REQUIRED"; return 0; }
#endif
    if (b->queue_count == MOTOR_BUS_QUEUE) { b->message = "QUEUE FULL"; return 0; }
    if (b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER ||
        b->sync_state == MOTOR_SYNC_VERIFY) { b->message = "SYNC BUSY"; return 0; }
    if (b->stop_mask & r.mask) { b->message = "STOP PENDING"; return 0; }
    for (i = 0u; i < b->queue_count; ++i) if (b->queue[i].mask & r.mask) {
        b->message = "AXIS BUSY"; return 0;
    }
    if ((b->pending && b->active.kind != QUERY && b->active.kind != DIAGNOSTIC &&
        (b->active.mask & r.mask)) || (b->wheels_pending && (r.mask & MOTOR_WHEELS))) {
        b->message = "AXIS BUSY"; return 0;
    }
    r.id = new_id(b); r.expires_ms = now + (r.code == EMM_VELOCITY ? r.action.lease_ms : 1000u);
    b->queue[b->queue_count++] = r;
    state(b, r.mask, MOTOR_QUEUED, r.id, now); b->message = "QUEUED"; return 1;
}
/*
 * 对单轴提交使能/失能请求；先要求健康状态，随后通过新的状态反馈确认结果。
 */
int MotorBus_Enable(MotorBus *b, uint8_t a, uint8_t enabled, uint32_t now)
{
    MotorRequest r;
    if (!address_ok(a) || enabled > 1u || !healthy(b, bit(a), now, 0u)) {
        b->message = "NEED FRESH HEALTHY STATUS"; return 0;
    }
    memset(&r, 0, sizeof(r)); r.action.command = Emm_Enable(a, enabled);
    r.address = a; r.code = EMM_ENABLE; r.mask = bit(a); r.kind = CONTROL;
    return enqueue(b, r, now);
}
/*
 * 提交单轴非同步速度或位置动作；需参数合法、轴空闲且有新鲜健康使能反馈。
 * 返回成功表示入队，执行/完成状态由后续 Process 和反馈更新。
 */
int MotorBus_Submit(MotorBus *b, const MotorAction *a, uint32_t now)
{
    MotorRequest r;
    uint8_t addr;
    if (!action_ok(a)) { b->message = "INVALID MOTION PARAMETERS"; return 0; }
    addr = a->command.bytes[0];
    if (a->command.bytes[a->command.length - 2u] || !healthy(b, bit(addr), now, 1u) ||
        b->nodes[addr - 1u].motion) { b->message = "NEED IDLE ENABLED HEALTHY AXIS"; return 0; }
    memset(&r, 0, sizeof(r)); r.action = *a;
    r.address = addr; r.code = a->command.code; r.mask = bit(addr); r.kind = CONTROL;
    return enqueue(b, r, now);
}
/* Native homing setup/start shares queue ownership, ACK and fault handling. */
int MotorBus_HomeCommand(MotorBus *b, const EmmCommand *c, uint32_t timeout_ms, uint32_t now)
{
    MotorRequest r; uint8_t a;
    if (!c || !address_ok(c->bytes[0]) || c->bytes[1]!=c->code) return 0;
    a=c->bytes[0];
    if (c->code==EMM_HOME_CONFIG) {
        uint32_t tm=((uint32_t)c->bytes[8]<<24)|((uint32_t)c->bytes[9]<<16)|
            ((uint32_t)c->bytes[10]<<8)|c->bytes[11];
        unsigned rpm=((unsigned)c->bytes[6]<<8)|c->bytes[7];
        if (c->length!=20u || c->bytes[2]!=0xAEu || c->bytes[3] || c->bytes[4]!=2u ||
            c->bytes[5]>1u || !rpm || rpm>3000u || tm<100u || tm>60000u ||
            c->bytes[18] || c->bytes[19]!=0x6Bu) return 0;
    } else if (c->code==EMM_HOME_BACKOFF) {
        if (c->length!=7u || c->bytes[2]!=0xACu || c->bytes[3] ||
            !(c->bytes[4]|c->bytes[5]) || c->bytes[6]!=0x6Bu) return 0;
    } else if (c->code==EMM_HOME) {
        if (c->length!=5u || c->bytes[2]!=2u || c->bytes[3] || c->bytes[4]!=0x6Bu ||
            timeout_ms<100u || timeout_ms>60000u) return 0;
    } else return 0;
    if (!healthy(b,bit(a),now,1u) || b->nodes[a-1u].motion || b->nodes[a-1u].homing) return 0;
    memset(&r,0,sizeof(r)); r.action.command=*c; r.action.timeout_ms=timeout_ms;
    r.address=a; r.code=c->code; r.mask=bit(a); r.kind=CONTROL;
    return enqueue(b,r,now);
}
int MotorBus_Wheels(MotorBus *b, const int32_t rpm[4], uint8_t acc, uint32_t lease, uint32_t now)
{
    MotorRequest r;
    unsigned i;
    memset(&r, 0, sizeof(r));
#if MOTOR_DRIVER_REQUIRE_MANUAL
    if (!b->manual) { b->message = "WHEELS NOT READY"; return 0; }
#endif
    if (lease < 100u || lease > 5000u ||
        !Emm_Wheels(&r.action.command, rpm, acc) ||
        !healthy(b, MOTOR_WHEELS, now, 1u) || (b->stop_mask & MOTOR_WHEELS) ||
        b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER ||
        b->sync_state == MOTOR_SYNC_VERIFY) { b->message = "WHEELS NOT READY"; return 0; }
    for (i = 0u; i < b->queue_count; ++i) if (b->queue[i].mask & MOTOR_WHEELS) return 0;
    if (b->pending && b->active.kind == CONTROL && (b->active.mask & MOTOR_WHEELS)) return 0;
    if (b->wheels_pending) state(b, MOTOR_WHEELS, MOTOR_CANCELLED, b->wheels.id, now);
    r.mask = MOTOR_WHEELS; r.address = 1u; r.code = EMM_VELOCITY; r.kind = GROUP;
    r.id = new_id(b); r.action.lease_ms = lease; r.expires_ms = now + lease;
    b->wheels = r; b->wheels_pending = 1u; b->group_id = r.id; b->group_ack = 0u;
    /* A new pending vector does not extend the already running vector's lease. */
    b->message = "WHEELS QUEUED"; return 1;
}
/*
 * 开启“逐轴写同步缓存→统一触发→反馈验证”的事务。
 * 要求缓存可信且总线无冲突；准备或触发出现不确定性后锁定同步流程，避免误用残留缓存。
 */
int MotorBus_Sync(MotorBus *b, const MotorAction *a, uint8_t count, uint32_t now)
{
    unsigned i;
    uint8_t mask = 0u;
#if MOTOR_DRIVER_REQUIRE_MANUAL
    if (!b->manual) { b->message = "SYNC LOCKED / BUSY"; return 0; }
#endif
    if (b->sync_locked || !a || !count ||
        count > MOTOR_COUNT || b->queue_count ||
        b->wheels_pending || b->stop_mask || b->sync_state == MOTOR_SYNC_PREPARE ||
        b->sync_state == MOTOR_SYNC_TRIGGER || b->sync_state == MOTOR_SYNC_VERIFY) {
        b->message = "SYNC LOCKED / BUSY"; return 0;
    }
    for (i = 0u; i < count; ++i) {
        uint8_t addr = a[i].command.bytes[0];
        if (!action_ok(&a[i]) || (mask & bit(addr)) || !healthy(b, bit(addr), now, 1u) ||
            b->nodes[addr - 1u].motion || a[i].command.bytes[a[i].command.length - 2u] != 1u) return 0;
        mask |= bit(addr);
    }
    /* Do not prepare while any unrelated action is moving, even outside the mask. */
    for (i = 0u; i < MOTOR_COUNT; ++i) if (b->nodes[i].motion) return 0;
    memcpy(b->sync, a, count * sizeof(*a)); b->sync_count = count; b->sync_index = 0u;
    b->sync_mask = mask; b->sync_id = new_id(b); b->sync_deadline = now + 3000u;
    b->sync_state = MOTOR_SYNC_PREPARE;
    state(b, mask, MOTOR_QUEUED, b->sync_id, now); b->message = "SYNC PREPARE"; return 1;
}
/*
 * 登记掩码内轴的持久停止意图，取消相关待发动作，并优先安排非同步单播 STOP。
 * 重复请求合并，不重置首次期限/反馈基线/限频；motion 清零表示软件取消，
 * 实物停车仍待 stop_unconfirmed 清除。正在发送的 DMA 帧保持完整，升降轴不以失能代替 STOP。
 */
void MotorBus_Stop(MotorBus *b, uint8_t mask, uint32_t now)
{
    unsigned i, write = 0u;
    uint8_t interrupted_sync = 0u;
    mask &= MOTOR_ALL;
    if (!mask) return;
    if (b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER ||
        b->sync_state == MOTOR_SYNC_VERIFY) {
        mask |= b->sync_mask; sync_fail(b); interrupted_sync = 1u;
    }
    /* Merge repeated UI/internal requests without resetting evidence, deadline
       or the rate limiter, and without aborting a verification query. */
    for (i = 0u; i < MOTOR_COUNT; ++i)
        if (b->nodes[i].stop_unconfirmed) mask &= (uint8_t)~(1u << i);
    if (!mask) return;
    for (i = 0u; i < b->queue_count; ++i) {
        if (b->queue[i].mask & mask) state(b, b->queue[i].mask, MOTOR_CANCELLED, b->queue[i].id, now);
        else b->queue[write++] = b->queue[i];
    }
    b->queue_count = (uint8_t)write;
    if (b->wheels_pending && (mask & MOTOR_WHEELS)) { b->wheels_pending = 0u; mask |= MOTOR_WHEELS; }
    /* Never cut a DMA frame: tx_wait remains set until hardware reports completion. */
    if (b->pending && (b->active.kind == QUERY || b->active.kind == DIAGNOSTIC ||
        (b->active.mask & mask))) {
        if (b->sent) b->nodes[b->active.address - 1u].quiet_until = now + MOTOR_BUS_QUIET_MS;
        b->pending = 0u;
    }
    b->stop_mask |= mask;
    for (i = 0u; i < MOTOR_COUNT; ++i) if (mask & (1u << i)) {
        MotorNode *n = &b->nodes[i];
        n->motion = 0u; n->previous_known = 0u; n->stop_sent = 0u;
        n->stop_unconfirmed = 1u; n->stop_attempts = 0u; n->stop_retry_ms = now;
        n->action = MOTOR_STOP_PENDING; n->action_ms = now;
        n->deadline_ms = now + MOTOR_STOP_TIMEOUT_MS;
        n->stop_speed_seq = n->data.speed_seq; n->stop_status_seq = n->data.status_seq;
    }
    if (!interrupted_sync) b->message = "STOP REQUESTED / VERIFY";
}
/*
 * 切换界面显式操作模式；退出手动请求全轴 STOP。
 * 程序调用是否也必须手动模式由 MOTOR_DRIVER_REQUIRE_MANUAL 决定。
 */
void MotorBus_SetManual(MotorBus *b, uint8_t manual, uint32_t now)
{
    if (b->manual && !manual) MotorBus_Stop(b, MOTOR_ALL, now);
    b->manual = (uint8_t)!!manual;
    b->message = b->manual ? "MANUAL / SELECT ACTION" : b->stop_mask ? "READ ONLY / STOP VERIFY" : "READ ONLY";
}
void MotorBus_ReadVersions(MotorBus *b) { b->diagnostic_mask = MOTOR_ALL; }

/*
 * 按事务种类收敛超时/拒绝：查询退避并更新在线状态，STOP 保留停止意图，
 * 普通控制失败取消运动并请求停止；同步失败额外锁定缓存，不重放原运动。
 */
static void failed(MotorBus *b, uint8_t result)
{
    MotorRequest r = b->active;
    unsigned i;
    b->pending = 0u;
    if (r.kind == QUERY || r.kind == DIAGNOSTIC) {
        MotorNode *n = &b->nodes[r.address - 1u];
        if (n->failures < 3u) ++n->failures;
        n->data.last_error = MOTOR_ERROR_TIMEOUT;
        n->failure_code = r.code; n->failure_result = result;
        if (n->failures >= 2u) { n->data.online = 0u; n->data.valid = 0u; }
        n->quiet_until = b->now_ms + MOTOR_BUS_QUIET_MS;
        for (i = 0u; i < 3u; ++i) n->poll_due[i] = b->now_ms + (n->failures >= 2u ? 2000u : 500u);
        if (n->motion) MotorBus_Stop(b, (uint8_t)(r.mask & MOTOR_WHEELS ? MOTOR_WHEELS : r.mask), b->now_ms);
        return;
    }
    if (r.kind == STOP) {
        MotorNode *n = &b->nodes[r.address - 1u];
        n->data.command_code = r.code; n->data.command_result = result; ++n->data.command_seq;
        n->data.last_error = result == EMM_ACK_TIMEOUT ? MOTOR_ERROR_TIMEOUT : MOTOR_ERROR_REJECTED;
        if (n->stop_unconfirmed && !due(b->now_ms, n->deadline_ms)) n->action = MOTOR_STOP_PENDING;
        return;
    }
    for (i = 0u; i < MOTOR_COUNT; ++i) if (r.mask & (1u << i)) {
        MotorNode *n = &b->nodes[i];
        n->action = result == EMM_ACK_TIMEOUT || result == EMM_ACK_TRANSPORT ? MOTOR_UNKNOWN : MOTOR_REJECTED;
        n->data.command_code = r.code; n->data.command_result = result; ++n->data.command_seq;
        n->data.last_error = result == 0xEEu ? MOTOR_ERROR_FORMAT :
            result == EMM_ACK_TIMEOUT ? MOTOR_ERROR_TIMEOUT : MOTOR_ERROR_REJECTED;
        if (result != EMM_ACK_TIMEOUT && result != EMM_ACK_TRANSPORT) ++n->rejected_commands;
        n->previous_known = 0u;
        n->failure_code = r.code; n->failure_result = result;
    }
    if (r.kind == PREPARE || r.kind == TRIGGER) sync_fail(b);
    MotorBus_Stop(b, (uint8_t)((r.mask & MOTOR_WHEELS) ? r.mask | MOTOR_WHEELS : r.mask), b->now_ms);
    b->message = b->sync_locked && (r.kind == PREPARE || r.kind == TRIGGER) ?
        "SYNC CACHE UNKNOWN / LOCKED" : result == 0x12u || result == 0x22u ? "LIMIT / NO MOTION (12/22)" :
        result == 0xE2u ? "E2 CONDITION/PARAM REJECTED" : result == 0xEEu ? "EE FORMAT REJECTED" :
        result == EMM_ACK_TIMEOUT ? "ACK TIMEOUT / STOP VERIFY" : "TRANSPORT ERROR / STOP VERIFY";
}
/*
 * 命令提交到传输层后建立运动监视期限与目标检查状态；这一步不证明电机已经转动。
 */
static void begin_motion(MotorBus *b, MotorNode *n, const MotorAction *a)
{
    uint8_t was_moving = n->motion;
    n->motion = 1u; n->action = MOTOR_SENT; n->action_ms = b->now_ms;
    n->deadline_ms = b->now_ms + (a->command.code == EMM_MOVE || a->command.code == EMM_HOME ? a->timeout_ms : a->lease_ms);
    n->data.command_code = a->command.code;
    if (!was_moving) n->poll_due[0] = n->poll_due[1] = n->poll_due[2] = b->now_ms;
}
/*
 * 解析器的前台回调：只接纳与当前已发事务地址/功能码一致的应答。
 * 查询更新字段时间戳与序号；ACK 只推进命令接受状态，停车/到位依赖独立反馈验证。
 */
static void received(void *context, const EmmFrame *f)
{
    MotorBus *b = (MotorBus *)context;
    MotorNode *n = &b->nodes[f->address - 1u];
    MotorSnapshot *d = &n->data;
    MotorRequest r;
    uint8_t index = 0u;
    ++b->rx_frames;
    if (f->result == 0x9Fu) { n->last_notice = f->result; ++b->ignored_frames; return; }
    if (!b->pending || !b->sent || f->address != b->active.address || f->code != b->active.code) {
        ++b->ignored_frames; return;
    }
    r = b->active;
    ++d->rx_count; d->last_rx_ms = b->now_ms; d->online = 1u; n->failures = 0u;
    b->next_send_ms = b->now_ms + MOTOR_BUS_GAP_MS;
    if (r.kind == QUERY) {
        if (f->code == EMM_SPEED) {
            d->speed_rpm = f->speed; d->speed_ms = b->now_ms; ++d->speed_seq; d->valid |= MOTOR_VALID_SPEED;
        } else if (f->code == EMM_POSITION) {
            index = 1u; d->position_counts = f->position; d->position_ms = b->now_ms;
            ++d->position_seq; d->valid |= MOTOR_VALID_POSITION;
        } else {
            index = 2u; if (f->code==EMM_HOME_STATUS) n->home_flags=f->home_flags;
            d->status_flags = f->flags; d->status_ms = b->now_ms;
            ++d->status_seq; d->valid |= MOTOR_VALID_STATUS;
        }
        if (n->poll_samples[index] && b->now_ms - n->poll_last_ms[index] > n->max_poll_gap_ms[index])
            n->max_poll_gap_ms[index] = b->now_ms - n->poll_last_ms[index];
        n->poll_last_ms[index] = b->now_ms; ++n->poll_samples[index];
        /* Position feedback also drives wheel odometry; refresh moving wheels
           at the same 200ms target as speed, through the existing poll queue. */
        n->poll_due[index] = b->now_ms + (n->motion && index != 2u ? 200u : 500u);
        d->last_error = MOTOR_ERROR_NONE; b->pending = 0u; return;
    }
    if (r.kind == DIAGNOSTIC) {
        n->firmware = f->firmware; n->hardware = f->hardware; n->version_valid = 1u;
        b->pending = 0u; return;
    }
    if (f->result != EMM_ACK_OK) { failed(b, f->result); return; }
    d->command_code = r.code; d->command_result = f->result; ++d->command_seq;
    b->pending = 0u;
    if (r.kind == STOP) return; /* Only fresh speed+status may confirm a stop. */
    if (r.kind == GROUP) {
        b->group_ack = f->result; b->message = "GROUP ACK1 / VERIFY EACH";
        /* Do not mark nodes 2..4 accepted based on address 1. */
        if (n->request_id == r.id) n->action = MOTOR_ACCEPTED;
    } else if (r.kind == PREPARE) {
        n->action = MOTOR_ACCEPTED; ++b->sync_index;
        if (b->sync_index == b->sync_count) b->sync_state = MOTOR_SYNC_TRIGGER;
    } else if (r.kind == TRIGGER) {
        b->sync_state = MOTOR_SYNC_VERIFY; b->message = "SYNC TRIGGER ACK / VERIFY";
    } else {
        n->action = (r.code==EMM_HOME_CONFIG || r.code==EMM_HOME_BACKOFF) ? MOTOR_COMPLETE : MOTOR_ACCEPTED;
        b->message = "ACK ONLY / VERIFY FEEDBACK";
        if (r.code == EMM_MOVE) { n->previous_target = n->target_counts; n->previous_known = n->target_known; }
    }
}
/*
 * 由串口 Pump 在前台喂入字节流；协议解析及业务状态更新不在串口中断中执行。
 */
void MotorBus_Receive(MotorBus *b, const uint8_t *bytes, size_t count, uint32_t now)
{ b->now_ms = now; b->rx_bytes += (uint32_t)count; EmmParser_Feed(&b->parser, bytes, count, received, b); }
/*
 * 处理传输故障边界：丢弃协议半帧、待发普通动作和旧反馈有效性，
 * 对已运动轴保留/请求停止，同步缓存转锁定；链路恢复后不会自动重放位置或速度动作。
 */
void MotorBus_TransportError(MotorBus *b, MotorError error, uint32_t now)
{
    unsigned i;
    uint8_t moving = 0u;
    b->now_ms = now; ++b->transport_errors;
    for (i = 0u; i < MOTOR_COUNT; ++i) if (b->nodes[i].motion) moving |= (uint8_t)(1u << i);
    if (b->pending && b->active.kind != QUERY && b->active.kind != DIAGNOSTIC) failed(b, EMM_ACK_TRANSPORT);
    b->pending = 0u;
    for (i = 0u; i < b->queue_count; ++i) state(b, b->queue[i].mask, MOTOR_CANCELLED, b->queue[i].id, now);
    b->queue_count = b->wheels_pending = 0u;
    if (b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER ||
        b->sync_state == MOTOR_SYNC_VERIFY) { moving |= b->sync_mask; sync_fail(b); }
    if (moving) MotorBus_Stop(b, moving, now);
    for (i = 0u; i < MOTOR_COUNT; ++i) {
        MotorNode *n = &b->nodes[i]; n->data.online = n->data.valid = 0u;
        n->data.last_error = error; n->previous_known = 0u; n->quiet_until = now + MOTOR_BUS_QUIET_MS;
    }
    b->parser.used = 0u; b->next_send_ms = now + MOTOR_BUS_QUIET_MS;
    b->message = "BUS ERROR / NO REPLAY";
}
/*
 * 用新鲜反馈监视七轴：确认停止、使能、位置到位与速度执行，并处理租约/超时和故障。
 * 未确认 STOP 超时只转 UNKNOWN，停止意图和重试继续保留；
 * 任一轮故障扩大为四轮停止，普通 UNKNOWN 恢复也不能清掉未确认 STOP。
 */
static void monitor(MotorBus *b)
{
    unsigned i;
    uint8_t fault_mask = 0u, all_done = 1u;
    for (i = 0u; i < MOTOR_COUNT; ++i) {
        MotorNode *n = &b->nodes[i];
        uint32_t age = b->now_ms - n->data.last_rx_ms;
        if (n->data.rx_count && age > n->max_age_ms) n->max_age_ms = age;
        if (age > MOTOR_BUS_FRESH_MS) n->data.online = 0u;
        if (n->stop_unconfirmed && n->stop_sent &&
            n->data.speed_seq != n->stop_speed_seq && n->data.status_seq != n->stop_status_seq &&
            due(n->data.speed_ms, n->stop_first_ms + 1u) && due(n->data.status_ms, n->stop_first_ms + 1u) &&
            MotorBus_Fresh(n, MOTOR_VALID_SPEED, b->now_ms) &&
            MotorBus_Fresh(n, MOTOR_VALID_STATUS, b->now_ms) && n->data.speed_rpm == 0 &&
            !(n->data.status_flags & 0x0Cu) && (!n->homing || !(n->home_flags & 0x34u))) {
            n->homing = 0u; n->stop_unconfirmed = 0u; b->stop_mask &= (uint8_t)~(1u << i);
            n->action = n->action == MOTOR_UNKNOWN && !(n->data.status_flags & 1u) ? MOTOR_IDLE : MOTOR_STOPPED;
        } else if (n->stop_unconfirmed && n->action != MOTOR_UNKNOWN && due(b->now_ms, n->deadline_ms)) {
            n->action = MOTOR_UNKNOWN; n->action_ms = b->now_ms;
        }
        if (n->stop_unconfirmed) continue; /* UNKNOWN recovery cannot erase STOP intent. */
        if (n->data.command_code == EMM_ENABLE && n->action == MOTOR_ACCEPTED) {
            if (MotorBus_Fresh(n, MOTOR_VALID_STATUS, b->now_ms) &&
                due(n->data.status_ms, n->action_ms + 1u) &&
                (n->data.status_flags & 1u) == n->requested_enable) n->action = MOTOR_COMPLETE;
            else if (due(b->now_ms, n->deadline_ms)) {
                n->action = MOTOR_UNKNOWN; n->action_ms = b->now_ms;
            }
        }
        if (n->action == MOTOR_UNKNOWN && !n->motion) {
#if MOTOR_UNKNOWN_AUTO_RECOVER_MS
            /* Leave UNKNOWN only after fresh, post-UNKNOWN evidence proves a safe
               stopped state: online, enabled-state known, zero speed, no stall or
               protection flag and the isolation window has passed. */
            if (n->data.online && MotorBus_Fresh(n, MOTOR_VALID_STATUS, b->now_ms) &&
                MotorBus_Fresh(n, MOTOR_VALID_SPEED, b->now_ms) &&
                due(n->data.status_ms, n->action_ms) && due(n->data.speed_ms, n->action_ms) &&
                due(b->now_ms, n->action_ms + MOTOR_UNKNOWN_AUTO_RECOVER_MS) &&
                due(b->now_ms, n->quiet_until) && !(n->data.status_flags & 0x0Cu) &&
                n->data.speed_rpm == 0) {
                if (n->data.status_flags & 1u) {
                    n->action = MOTOR_STOPPED; b->message = "UNKNOWN RECOVERED / STOPPED";
                } else {
                    n->action = MOTOR_IDLE; b->message = "UNKNOWN RECOVERED / IDLE";
                }
            }
#endif
            continue;
        }
        if (!n->motion) continue;
        if (!n->data.online || !MotorBus_Fresh(n, MOTOR_VALID_STATUS, b->now_ms) ||
            (n->data.status_flags & 0x0Cu) || !(n->data.status_flags & 1u)) {
            fault_mask |= (uint8_t)(1u << i); continue;
        }
        if (n->homing && (n->home_flags & 0x38u)) {
            n->data.last_error=MOTOR_ERROR_STALL; fault_mask|=(uint8_t)(1u<<i); continue;
        }
        if (due(b->now_ms, n->deadline_ms)) {
            n->data.last_error = MOTOR_ERROR_MOTION_TIMEOUT;
            fault_mask |= (uint8_t)(1u << i); continue;
        }
        if ((b->pending && b->active.id == n->request_id) ||
            b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER) continue;
        if (n->homing && n->action==MOTOR_ACCEPTED &&
            MotorBus_Fresh(n,MOTOR_VALID_STATUS,b->now_ms) &&
            MotorBus_Fresh(n,MOTOR_VALID_SPEED,b->now_ms) &&
            due(n->data.status_ms,n->action_ms+1u) && due(n->data.speed_ms,n->action_ms+1u) &&
            (n->home_flags & 0x3Fu)==3u && !n->data.speed_rpm) {
            n->homing=n->motion=0u; n->action=MOTOR_COMPLETE;
        }
        if (n->data.command_code == EMM_MOVE && n->target_known &&
            MotorBus_Fresh(n, MOTOR_VALID_POSITION, b->now_ms) &&
            MotorBus_Fresh(n, MOTOR_VALID_SPEED, b->now_ms) &&
            due(n->data.position_ms, n->action_ms + 1u) && due(n->data.speed_ms, n->action_ms + 1u) &&
            due(n->data.status_ms, n->action_ms + 1u)) {
            int64_t delta = n->data.position_counts - n->target_counts;
            if (delta >= -(int64_t)MOTOR_POSITION_TOLERANCE_COUNTS && delta <= MOTOR_POSITION_TOLERANCE_COUNTS &&
                n->data.speed_rpm == 0 && (n->data.status_flags & 2u)) {
                n->motion = 0u; n->action = MOTOR_COMPLETE;
                n->previous_target = n->target_counts; n->previous_known = 1u;
            } else n->action = MOTOR_EXECUTING;
        } else if (n->data.command_code == EMM_VELOCITY &&
            MotorBus_Fresh(n, MOTOR_VALID_SPEED, b->now_ms) && due(n->data.speed_ms, n->action_ms + 1u) &&
            n->data.speed_rpm == n->target_rpm) n->action = MOTOR_EXECUTING;
    }
    if (fault_mask) {
        if (fault_mask & MOTOR_WHEELS) fault_mask |= MOTOR_WHEELS;
        MotorBus_Stop(b, fault_mask, b->now_ms);
    }
    if (b->sync_state == MOTOR_SYNC_VERIFY) {
        for (i = 0u; i < MOTOR_COUNT; ++i) if ((b->sync_mask & (1u << i)) && b->nodes[i].motion) all_done = 0u;
        if (all_done) b->sync_state = MOTOR_SYNC_DONE;
    }
}
static void activate(MotorBus *b, MotorRequest r)
{ b->active = r; b->pending = 1u; b->sent = 0u; }
/*
 * 选择下一事务：首次 STOP 优先，再轮转限频补发，并为查询/其它事务让出槽位；
 * 随后处理同步、普通队列、四轮向量、停车验证、背景轮询及版本诊断。
 * STOP 补发不重置首次反馈基线，避免持续重试令停车永远无法确认。
 */
static int choose(MotorBus *b)
{
    MotorRequest r;
    unsigned i, scan;
    memset(&r, 0, sizeof(r));
    if (b->stop_mask) {
        for (scan = 0u; scan < MOTOR_COUNT; ++scan) {
            i = b->stop_cursor; b->stop_cursor = (uint8_t)((i + 1u) % MOTOR_COUNT);
            if (b->stop_mask & (1u << i)) break;
        }
        r.address = (uint8_t)(i + 1u); r.mask = bit(r.address); r.code = EMM_STOP; r.kind = STOP;
        r.id = new_id(b); r.action.command = b->nodes[r.address-1u].homing ? Emm_HomeAbort(r.address) : Emm_Stop(r.address);
        r.code=r.action.command.code; activate(b, r); return 1;
    }
    /* One retry at most before yielding a transaction slot to feedback/other
       polls. Rotate axes; never restart the stop's evidence baseline. */
    if (!b->retry_yield) for (scan = 0u; scan < MOTOR_COUNT; ++scan) {
        MotorNode *n;
        i = b->stop_cursor; b->stop_cursor = (uint8_t)((i + 1u) % MOTOR_COUNT); n = &b->nodes[i];
        if (!n->stop_unconfirmed || !n->stop_sent || !due(b->now_ms, n->stop_retry_ms) ||
            !due(b->now_ms, n->quiet_until)) continue;
        r.address = n->data.address; r.mask = bit(r.address); r.code = EMM_STOP; r.kind = STOP;
        r.id = new_id(b); r.action.command = b->nodes[r.address-1u].homing ? Emm_HomeAbort(r.address) : Emm_Stop(r.address);
        r.code=r.action.command.code; b->retry_yield = 1u;
        activate(b, r); return 1;
    }
    b->retry_yield = 0u;
    if (b->sync_state == MOTOR_SYNC_PREPARE || b->sync_state == MOTOR_SYNC_TRIGGER) {
        if (due(b->now_ms, b->sync_deadline) || !healthy(b, b->sync_mask, b->now_ms, 1u)) {
            MotorBus_Stop(b, b->sync_mask, b->now_ms); sync_fail(b); return 0;
        }
        if (!mask_ready(b, b->sync_mask, b->now_ms)) return 0;
        r.id = b->sync_id;
        if (b->sync_state == MOTOR_SYNC_PREPARE) {
            r.action = b->sync[b->sync_index]; r.address = r.action.command.bytes[0];
            r.mask = bit(r.address); r.code = r.action.command.code; r.kind = PREPARE;
        } else {
            r.action.command = Emm_SyncTrigger(); r.address = 1u; r.mask = b->sync_mask;
            r.code = EMM_SYNC; r.kind = TRIGGER;
        }
        activate(b, r); return 1;
    }
    if (b->queue_count) {
        r = b->queue[0];
        if (!due(b->now_ms, b->nodes[r.address - 1u].quiet_until)) return 0;
        --b->queue_count; memmove(b->queue, b->queue + 1, b->queue_count * sizeof(b->queue[0]));
        if (due(b->now_ms, r.expires_ms) || !healthy(b, r.mask, b->now_ms, (uint8_t)(r.code != EMM_ENABLE))) {
            state(b, r.mask, MOTOR_CANCELLED, r.id, b->now_ms); return 0;
        }
        activate(b, r); return 1;
    }
    if (b->wheels_pending && due(b->now_ms, b->next_wheel_ms)) {
        if (!mask_ready(b, MOTOR_WHEELS, b->now_ms)) return 0;
        r = b->wheels; b->wheels_pending = 0u;
        if (due(b->now_ms, r.expires_ms) || !healthy(b, MOTOR_WHEELS, b->now_ms, 1u)) {
            b->message = "EXPIRED WHEEL VECTOR"; return 0;
        }
        activate(b, r); return 1;
    }
    /* Stop verification runs before background polling, even for a full stop group. */
    for (scan = 0u; scan < MOTOR_COUNT * 2u; ++scan) {
        uint8_t k = b->verify_cursor;
        MotorNode *n = &b->nodes[k / 2u];
        unsigned j = (k % 2u) * 2u;
        b->verify_cursor = (uint8_t)((k + 1u) % (MOTOR_COUNT * 2u));
        if (!n->stop_unconfirmed || !n->stop_sent || !due(b->now_ms, n->quiet_until)) continue;
        if (due(b->now_ms, n->poll_due[j])) {
            r.address = n->data.address; r.mask = bit(r.address); r.code = codes[j]; r.kind = QUERY;
            if (r.code==EMM_STATUS && b->nodes[r.address-1u].homing) r.code=EMM_HOME_STATUS;
            r.action.command = Emm_Query(r.address, r.code); activate(b, r); return 1;
        }
    }
    for (scan = 0u; scan < 21u; ++scan) {
        uint8_t k = b->poll_cursor;
        MotorNode *n = &b->nodes[k / 3u];
        b->poll_cursor = (uint8_t)((k + 1u) % 21u);
        if (!due(b->now_ms, n->poll_due[k % 3u]) || !due(b->now_ms, n->quiet_until)) continue;
        r.address = n->data.address; r.mask = bit(r.address); r.code = codes[k % 3u]; r.kind = QUERY;
        if (r.code==EMM_STATUS && n->homing) r.code=EMM_HOME_STATUS;
        r.action.command = Emm_Query(r.address, r.code); activate(b, r); return 1;
    }
    if (b->diagnostic_mask) for (i = 0u; i < MOTOR_COUNT; ++i) if (b->diagnostic_mask & (1u << i)) {
        b->diagnostic_mask &= (uint8_t)~(1u << i);
        r.address = (uint8_t)(i + 1u); r.mask = bit(r.address); r.code = EMM_VERSION; r.kind = DIAGNOSTIC;
        r.action.command = Emm_Query(r.address, r.code); activate(b, r); return 1;
    }
    return 0;
}
/*
 * 前台单步推进：检查 DMA 完成/应答超时，监视动作，选择待发事务，再检查期限和健康状态。
 * 发送成功后才记录提交时刻、停止重试计划或运动租约；
 * 传输 busy 时立即返回，DMA 缓冲的终止/恢复由 BSP 管理，不在此层强行复用。
 */
void MotorBus_Process(MotorBus *b, uint32_t now)
{
    MotorRequest *r = &b->active;
    unsigned i;
    b->now_ms = now;
    if (now - b->last_step_ms > b->max_step_ms) b->max_step_ms = now - b->last_step_ms;
    b->last_step_ms = now;
    if (b->tx_wait) {
        if (!b->transport.busy(b->transport.context)) {
            b->tx_wait = 0u; b->reply_ms = now; b->next_send_ms = now + MOTOR_BUS_GAP_MS;
        } else if (now - b->tx_ms >= b->tx_limit_ms) {
            /* BSP owns abort/recovery; never reuse a still-busy DMA buffer here. */
            MotorBus_TransportError(b, MOTOR_ERROR_UART, now); b->tx_ms = now;
        }
    }
    if (b->pending && b->sent && !b->tx_wait && now - b->reply_ms >= MOTOR_BUS_REPLY_MS) {
        ++b->timeout_count; ++b->nodes[r->address - 1u].data.timeouts;
        b->nodes[r->address - 1u].quiet_until = now + MOTOR_BUS_QUIET_MS;
        b->parser.used = 0u; failed(b, EMM_ACK_TIMEOUT);
    }
    monitor(b);
    if (b->tx_wait || b->transport.busy(b->transport.context) || !due(now, b->next_send_ms)) return;
    if (!b->pending && !choose(b)) return;
    if (b->sent) return;
    r = &b->active;
    if (r->kind == STOP && !b->nodes[r->address - 1u].stop_unconfirmed) { b->pending = 0u; return; }
    if ((r->kind == CONTROL || r->kind == GROUP) && due(now, r->expires_ms)) {
        state(b, r->mask, MOTOR_CANCELLED, r->id, now); b->pending = 0u;
        b->message = "REQUEST EXPIRED / NOT SENT"; return;
    }
    if ((r->kind == CONTROL || r->kind == GROUP || r->kind == PREPARE || r->kind == TRIGGER) &&
        !healthy(b, r->mask, now, (uint8_t)(r->code != EMM_ENABLE))) { failed(b, 0xE2u); return; }
    if ((r->kind == CONTROL && (r->code == EMM_MOVE || r->code == EMM_VELOCITY)) || r->kind == PREPARE) {
        if (!set_target(&b->nodes[r->address - 1u], &r->action, now)) { failed(b, 0xE2u); return; }
    }
    if (b->transport.send(b->transport.context, r->action.command.bytes, r->action.command.length)) {
        ++b->tx_frames; b->tx_bytes += r->action.command.length;
        b->sent = b->tx_wait = 1u; b->tx_ms = now;
        b->tx_limit_ms = 20u + ((uint32_t)r->action.command.length * 10000u + 115199u) / 115200u;
        ++b->nodes[r->address - 1u].data.tx_count;
        if (r->kind == STOP) {
            MotorNode *n = &b->nodes[r->address - 1u];
            b->stop_mask &= (uint8_t)~r->mask;
            if (!n->stop_sent) {
                n->stop_first_ms = now;
                n->stop_speed_seq = n->data.speed_seq; n->stop_status_seq = n->data.status_seq;
                n->poll_due[0] = n->poll_due[2] = now;
            }
            n->stop_sent = 1u;
            if (n->stop_attempts < 255u) ++n->stop_attempts;
            n->stop_retry_ms = now + (n->stop_attempts < MOTOR_STOP_FAST_ATTEMPTS ?
                MOTOR_STOP_RETRY_MS : MOTOR_STOP_BACKOFF_MS);
        } else if (r->kind == GROUP) {
            ++b->wheel_frames; b->last_wheel_ms = now;
            b->next_wheel_ms = now + 100u;
            for (i = 0u; i < 4u; ++i) {
                MotorAction a = r->action;
                a.command = Emm_VelocityEx((uint8_t)(i + 1u), r->action.command.bytes[6u + i * 8u],
                    (uint16_t)(((uint16_t)r->action.command.bytes[7u + i * 8u] << 8) |
                    r->action.command.bytes[8u + i * 8u]), r->action.command.bytes[9u + i * 8u], 0u);
                (void)set_target(&b->nodes[i], &a, now); begin_motion(b, &b->nodes[i], &a);
                b->nodes[i].request_id = r->id; b->nodes[i].deadline_ms = r->expires_ms;
            }
        } else if (r->kind == TRIGGER) {
            for (i = 0u; i < b->sync_count; ++i) {
                MotorNode *n = &b->nodes[b->sync[i].command.bytes[0] - 1u];
                begin_motion(b, n, &b->sync[i]);
                if (n->target_known) { n->previous_target = n->target_counts; n->previous_known = 1u; }
            }
        } else if (r->kind == CONTROL && (r->code == EMM_MOVE || r->code == EMM_VELOCITY || r->code == EMM_HOME)) {
            if (r->code==EMM_HOME) {
                MotorNode *n=&b->nodes[r->address-1u];
                n->homing=1u; n->home_flags=4u; n->target_known=n->previous_known=0u;
            }
            begin_motion(b, &b->nodes[r->address - 1u], &r->action);
            if (r->code == EMM_VELOCITY) b->nodes[r->address - 1u].deadline_ms = r->expires_ms;
        } else if (r->kind != QUERY && r->kind != DIAGNOSTIC) {
            b->nodes[r->address - 1u].action = MOTOR_SENT;
            if (r->code == EMM_ENABLE) {
                MotorNode *n = &b->nodes[r->address - 1u];
                n->data.command_code = r->code; n->requested_enable = r->action.command.bytes[3];
                n->action_ms = now; n->deadline_ms = now + 3000u; n->poll_due[2] = now;
            }
        }
    } else if (now - b->next_send_ms >= 100u) MotorBus_TransportError(b, MOTOR_ERROR_UART, now);
}
