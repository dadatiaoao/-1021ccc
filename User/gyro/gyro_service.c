/*
 * 单旋转轴陀螺服务：分步启动 ICM-42688，轮询 DRDY，静止校准零偏，
 * 发布选定轴角速度（deg/s）及连续有效样本的相对转角（deg）。
 * 无 FIFO 时缺口不可补算；积分是相对量，会漂移，不是绝对航向或完整姿态融合。
 */
#include "gyro_service.h"
#include "gyro_config.h"
#include <string.h>
#include <limits.h>
static float absf(float x) { return x<0.0f ? -x : x; }
static GyroCalibration motor_rejection(uint8_t condition)
{
    return condition == GYRO_MOTOR_STOP ? GYRO_CAL_STOP :
        condition == GYRO_MOTOR_UNKNOWN ? GYRO_CAL_UNKNOWN : GYRO_CAL_MOTOR;
}
/*
 * 锁存角度连续性中断，清积分历史和 angle_valid，保留旧角数值作诊断；
 * 后续新速率不会自动把旧轨迹恢复为有效，需 ZERO 或成功 CAL 建立新起点。
 */
static void gap(GyroService *s,GyroError e)
{
    if (!s->data.angle_gap) ++s->data.gaps;
    s->data.angle_gap=1u; s->data.angle_valid=0u; s->have_previous=0u;
    s->data.error=s->data.last_fault=e;
}
/*
 * 终止本次校准并中断角度轨迹，保留原偏置；失败样本不能覆盖已知偏置。
 */
static void cal_fail(GyroService *s,GyroCalibration c)
{
    s->data.calibration=c;
    gap(s,GYRO_GAP); /* Keep the old bias, never resume its old trajectory. */
}
static GyroError chip_error(int r)
{
    if (r==ICM_ID_ERROR) return GYRO_ID;
    if (r==ICM_RESET_ERROR || r==ICM_RESET_DETECTED) return GYRO_RESET;
    if (r==ICM_CONFIG_ERROR) return GYRO_CONFIG;
    return GYRO_BUS;
}
/*
 * 总线/器件异常使样本和积分无效，若在校准则失败；按退避期限安排重新初始化。
 */
static void fault(GyroService *s,int r,uint32_t now)
{
    if (r==ICM_BUS_ERROR) {
        GyroBusFailure *f=&s->data.last_bus_failure;
        f->valid=1u; f->stage=s->chip.trace_stage;
        f->phase=s->chip.phase;
        f->index=f->stage==GYRO_TRACE_CHECK ? s->chip.check_index : s->chip.index;
        f->op=s->chip.trace_op; f->reg=s->chip.trace_reg; f->bus=s->chip.bus_error;
        /* Retained across retries, successful samples, ZERO and CAL.
           Only a full Gyro_Init starts a new diagnostic session. */
        if (!s->data.first_bus_failure.valid) s->data.first_bus_failure=*f;
    }
    if (s->data.calibration==GYRO_CAL_RUNNING) cal_fail(s,GYRO_CAL_ERROR);
    gap(s,chip_error(r)); s->data.state=GYRO_ERROR;
    s->data.raw_valid=s->data.rate_valid=0u;
    s->data.bus_error=s->chip.bus_error; ++s->data.errors;
    if (s->chip.bus_error!=GYRO_BUS_OK) s->data.last_bus_error=s->chip.bus_error;
    s->due=now+s->retry_ms;
    if (s->retry_ms<GYRO_RETRY_MAX_MS) s->retry_ms*=2u;
}
/*
 * 启动一次器件重建：清校准有效性和样本连续性，保留失败原因供诊断，
 * 器件复位后必须重新校准，不能沿用旧轨迹。
 */
static void start(GyroService *s,uint32_t now)
{
    GyroBusError e;
    s->data.state=GYRO_INIT; s->data.error=GYRO_OK;
    s->data.calibrated=s->data.have_sample=s->data.raw_valid=0u;
    s->data.rate_valid=s->data.angle_valid=s->have_previous=0u;
    /* Preserve a failed calibration result for diagnosis until next CAL. */
    if (s->data.calibration==GYRO_CAL_DONE) s->data.calibration=GYRO_CAL_NONE;
#if GYRO_TEMP_FIXED_BIAS_ENABLED
    /*
     * TEMPORARY bring-up workaround, see gyro_config.h: substitute the measured
     * stationary offset for a real calibration so RATE/REL stay usable while the
     * out-of-spec module is pending replacement. It does not claim a CAL result
     * (the CAL row stays CAL REQUIRED). Real CAL still checks raw samples and
     * ends in CAL:MOVING for this module; after such a failure use ZERO to
     * start a new angle segment.
     */
    s->data.bias_dps=GYRO_TEMP_FIXED_BIAS_DPS;
    s->data.calibrated=1u;
    s->data.calibration=GYRO_CAL_NONE;
    s->data.angle_deg=0.0;
    s->data.angle_valid=1u;
    s->data.angle_gap=0u;
#endif
    Icm_Begin(&s->chip,now);
    e=s->io.init(s->io.ctx);
    if (e!=GYRO_BUS_OK) { s->chip.bus_error=e; fault(s,ICM_BUS_ERROR,now); }
}
/*
 * 绑定 I2C/时钟接口并应用安装轴与符号配置，启动分阶段初始化。
 */
void Gyro_Init(GyroService *s,GyroIo io,uint32_t now)
{
    memset(s,0,sizeof(*s)); s->io=io;
    s->data.axis=GYRO_AXIS; s->data.sign=GYRO_SIGN;
    if (GYRO_COMM_TEST) {
        s->data.probe.mode=GYRO_COMM_TEST;
        s->data.probe.init_bus=s->io.init(s->io.ctx);
        s->data.bus_error=s->data.probe.init_bus;
        s->data.state=GYRO_INIT; s->due=now+3u;
        return;
    }
    s->retry_ms=GYRO_RETRY_MIN_MS; start(s,now);
}
/* A controlled experiment, deliberately separate from fault()/start():
 * no recovery, backoff, bank writes, calibration or normal initialization.
 * Each call performs at most ONE bounded bus transaction. */
static void probe_process(GyroService *s,uint32_t now)
{
    GyroProbe *p=&s->data.probe; GyroProbeCounts *c;
    GyroBusError e; uint8_t v=0u;
    if (p->init_bus!=GYRO_BUS_OK || (int32_t)(now-s->due)<0) return;
    if (p->phase==GYRO_PROBE_UNSUPPORTED) return;
    if (p->phase==GYRO_PROBE_RELEASE_WAIT || p->phase==GYRO_PROBE_RELEASED) {
        p->lines_now=s->io.lines(s->io.ctx);
        if (!p->after_valid) { p->lines_after=p->lines_now; p->after_valid=1u; }
        p->phase=GYRO_PROBE_RELEASED;
        s->due=now+100u;
        return;
    }
    if (p->phase==GYRO_PROBE_BANK) {
        e=s->io.read(s->io.ctx,0x76u,&v,1u);
        p->bank_bus=e;
        s->data.bus_error=e; p->bank_seen=(uint8_t)(e==GYRO_BUS_OK); p->bank=v;
        p->phase=(e==GYRO_BUS_OK && v==0u) ? GYRO_PROBE_RESET : GYRO_PROBE_HOLD;
    } else if (p->phase==GYRO_PROBE_RESET) {
        p->reset_sent=1u;
        p->reset_bus=s->io.write(s->io.ctx,0x11u,1u);
        s->data.bus_error=p->reset_bus;
        /* Even NACK/TIMEOUT can leave the write outcome uncertain: NEVER retry.
         * POST means after the attempt, not proof that a reset completed. */
        p->phase=GYRO_PROBE_POST;
        s->due=s->io.millis(s->io.ctx)+2u;
        return;
    } else {
        c=p->phase==GYRO_PROBE_POST ? &p->after : &p->before;
        if (c->total==UINT32_MAX) return; /* Preserve consistent counters. */
        e=s->io.read(s->io.ctx,0x75u,&v,1u); ++c->total;
        s->data.bus_error=e; s->data.who=e==GYRO_BUS_OK ? v : 0u;
        if (e==GYRO_BUS_OK) { if (v==0x47u) ++c->ok; else ++c->bad_id; }
        else if (e==GYRO_BUS_NACK) ++c->nack;
        else if (e==GYRO_BUS_TIMEOUT) ++c->timeout;
        else if (e==GYRO_BUS_BUSY || (e>=GYRO_BUS_SCL_LOW && e<=GYRO_BUS_HW_BUSY)) ++c->busy;
        else ++c->other;
        if (p->mode==3u && e!=GYRO_BUS_OK) {
            p->release_cause=e;
            if (!s->io.lines || !s->io.release_lines) { p->phase=GYRO_PROBE_UNSUPPORTED; return; }
            p->lines_before=s->io.lines(s->io.ctx);
            s->io.release_lines(s->io.ctx);
            p->phase=GYRO_PROBE_RELEASE_WAIT;
            s->due=s->io.millis(s->io.ctx)+10u;
            return;
        }
        if (p->mode==2u && p->phase==GYRO_PROBE_PRE && c->total==GYRO_COMM_TEST_READS)
            p->phase=c->ok==c->total ? GYRO_PROBE_BANK : GYRO_PROBE_HOLD;
    }
    s->due=s->io.millis(s->io.ctx)+GYRO_COMM_TEST_PERIOD_MS;
}
static int fresh(const GyroService *s,uint32_t now)
{
    return s->data.state==GYRO_RUNNING && s->data.have_sample && s->data.raw_valid &&
        (uint32_t)(now-s->data.sample_ms)<=GYRO_STALE_MS;
}
/*
 * 仅在样本新鲜且无电机活动/未知/未确认停止原因时启动静止校准。
 * 清本次统计和积分历史，偏置要等完整连续窗口通过质量检查后才更新；电机门控仅为辅助。
 */
int Gyro_RequestCalibration(GyroService *s,uint32_t now,uint8_t motor_active)
{
    if (s->data.probe.mode) return 0;
    if (s->data.calibration==GYRO_CAL_RUNNING) return 0;
    ++s->data.cal_requests;
    if (motor_active) { s->data.calibration=motor_rejection(motor_active); return 0; }
    if (!fresh(s,now)) { s->data.calibration=GYRO_CAL_NOT_READY; return 0; }
    s->data.angle_valid=0u; s->have_previous=0u;
    s->data.calibration=GYRO_CAL_RUNNING; s->data.rate_valid=0u;
    s->data.cal_count=s->data.cal_elapsed_ms=0u; s->cal_start=now;
    memset(s->mean,0,sizeof(s->mean)); memset(s->m2,0,sizeof(s->m2));
    return 1;
}
/*
 * 在已有校准且样本新鲜时，把最近已获取样本作为相对角零点；
 * 不改偏置，也不把触摸或绘图时刻当采样边界。
 */
int Gyro_ZeroAngle(GyroService *s,uint32_t now)
{
    if (s->data.probe.mode) return 0;
    if (!fresh(s,now) || !s->data.calibrated || s->data.calibration==GYRO_CAL_RUNNING) return 0;
    /* Boundary is latest acquired sample, not touch/UI refresh time. */
    s->data.angle_deg=0.0; s->data.angle_valid=1u; s->data.angle_gap=0u;
    s->previous_rate=s->data.rate_dps; s->have_previous=1u; ++s->data.zeros;
    return 1;
}
/*
 * 用 Welford 在线算法累计三轴均值/方差，并检查幅值和极差。
 * 满足样本数、持续时间及静止阈值后，只发布安装轴零偏并把相对角置零；运动则立即失败。
 */
static int calibrate(GyroService *s,uint32_t now)
{
    unsigned i; float x,delta; GyroSnapshot *d=&s->data;
    if (!d->cal_count) s->cal_first=now;
    ++d->cal_count; d->cal_elapsed_ms=now-s->cal_first;
    for (i=0u;i<3u;++i) {
        x=(float)d->raw[i]/GYRO_LSB_PER_DPS;
        if (d->cal_count==1u) s->minimum[i]=s->maximum[i]=x;
        if (x<s->minimum[i]) s->minimum[i]=x;
        if (x>s->maximum[i]) s->maximum[i]=x;
        delta=x-s->mean[i]; s->mean[i]+=delta/(float)d->cal_count;
        s->m2[i]+=delta*(x-s->mean[i]);
        if (absf(x)>GYRO_CAL_MAX_ABS_DPS || s->maximum[i]-s->minimum[i]>GYRO_CAL_MAX_RANGE_DPS) {
            cal_fail(s,GYRO_CAL_MOVING); return 0;
        }
    }
    if (d->cal_count<GYRO_CAL_SAMPLES || d->cal_elapsed_ms<GYRO_CAL_MS) return 0;
    for (i=0u;i<3u;++i) if (absf(s->mean[i])>GYRO_CAL_MAX_MEAN_DPS ||
        s->m2[i]/(float)(d->cal_count-1u)>GYRO_CAL_MAX_VARIANCE) {
        cal_fail(s,GYRO_CAL_MOVING); return 0;
    }
    d->bias_dps=(float)GYRO_SIGN*s->mean[GYRO_AXIS]; d->calibrated=1u;
    d->calibration=GYRO_CAL_DONE; d->angle_deg=0.0;
    d->angle_gap=0u; d->angle_valid=1u; s->have_previous=0u;
    return 1;
}
/*
 * 单步推进错误退避、器件初始化、校准期限、轮询/配置复查及 DRDY 采样。
 * 无 FIFO 时轮询空档或样本间隔超限会断开积分；只对有效连续样本做梯形积分，
 * 用实际样本 dt（毫秒）换算秒，不以调用次数或假定周期累加角度。
 */
static void process(GyroService *s,uint32_t now,uint8_t motor_active)
{
    GyroSnapshot *d=&s->data; uint32_t at,dt,poll_dt; int r; int16_t raw[3];
    if (d->state==GYRO_ERROR) {
        if ((int32_t)(now-s->due)>=0) { ++d->restarts; start(s,now); }
        return;
    }
    if (d->state==GYRO_INIT) {
        r=Icm_Step(&s->chip,&s->io,now); d->who=s->chip.who;
        if (s->chip.config4_read) {
            d->config4_seen=1u; d->config4_before=s->chip.config4_before;
        }
        if (r==ICM_READY) {
            d->state=GYRO_RUNNING; d->bus_error=GYRO_BUS_OK;
            s->last_poll=s->last_check=s->ready_ms=s->io.millis(s->io.ctx);
            s->retry_ms=GYRO_RETRY_MIN_MS;
        } else if (r<0) fault(s,r,s->io.millis(s->io.ctx));
        return;
    }
    if (d->calibration==GYRO_CAL_RUNNING) {
        if (motor_active) cal_fail(s,motor_rejection(motor_active));
        else if ((uint32_t)(now-s->cal_start)>=GYRO_CAL_TIMEOUT_MS) cal_fail(s,GYRO_CAL_TIMEOUT);
    }
    if ((uint32_t)(now-(d->have_sample ? d->sample_ms : s->ready_ms))>GYRO_STALE_MS) {
        if (d->calibration==GYRO_CAL_RUNNING) cal_fail(s,GYRO_CAL_GAP);
        gap(s,GYRO_STALE); d->raw_valid=d->rate_valid=0u;
        if ((uint32_t)(now-(d->have_sample ? d->sample_ms : s->ready_ms))>=GYRO_NO_DATA_RESTART_MS) {
            fault(s,ICM_RESET_ERROR,now); return;
        }
    }
    poll_dt=now-s->last_poll;
    if (poll_dt<GYRO_POLL_MS) return;
    if (poll_dt>d->max_poll_ms) d->max_poll_ms=poll_dt;
    /* With no FIFO a polling blackout of one ODR can hide a lost frame.
       Conservatively break continuity even if the next value looks plausible. */
    if (d->have_sample && poll_dt>=GYRO_PERIOD_MS) {
        gap(s,GYRO_GAP);
        if (d->calibration==GYRO_CAL_RUNNING) cal_fail(s,GYRO_CAL_GAP);
    }
    if ((uint32_t)(now-s->last_check)>=GYRO_CHECK_MS) {
        s->last_check=now; r=Icm_Check(&s->chip,&s->io); d->who=s->chip.who;
        if (r<0) fault(s,r,s->io.millis(s->io.ctx));
        return; /* No DRDY poll occurred; do not postpone the next fetch. */
    }
    s->last_poll=now;
    r=Icm_Fetch(&s->chip,&s->io,raw,&at); d->bus_error=s->chip.bus_error;
    if (r==ICM_NO_DATA) return;
    if (r==ICM_INVALID || r==ICM_SATURATED) {
        if (r==ICM_INVALID) ++d->invalid; else ++d->saturations;
        if (d->calibration==GYRO_CAL_RUNNING) cal_fail(s,GYRO_CAL_ERROR);
        gap(s,r==ICM_INVALID ? GYRO_BAD_SAMPLE : GYRO_SATURATED);
        d->raw_valid=d->rate_valid=0u; return;
    }
    if (r<0) { fault(s,r,s->io.millis(s->io.ctx)); return; }
    dt=d->have_sample ? at-d->sample_ms : 0u;
    d->interval_ms=dt;
    if (dt>d->max_interval_ms) d->max_interval_ms=dt;
    if (d->have_sample && (dt==0u || dt>GYRO_MAX_SAMPLE_MS)) {
        if (dt>=2u*GYRO_PERIOD_MS) d->missed+=dt/GYRO_PERIOD_MS-1u;
        gap(s,GYRO_GAP);
        if (d->calibration==GYRO_CAL_RUNNING) cal_fail(s,GYRO_CAL_GAP);
    }
    memcpy(d->raw,raw,sizeof(raw)); d->raw_dps=(float)(GYRO_SIGN*(int32_t)raw[GYRO_AXIS])/GYRO_LSB_PER_DPS;
    d->sample_ms=at; d->have_sample=d->raw_valid=1u; ++d->sequence;
    d->error=GYRO_OK;
    if (d->calibration==GYRO_CAL_RUNNING) (void)calibrate(s,at);
    d->rate_dps=d->raw_dps-d->bias_dps;
    d->rate_valid=(uint8_t)(d->calibrated && d->calibration!=GYRO_CAL_RUNNING);
    if (d->rate_valid && d->angle_valid) {
        if (s->have_previous) d->angle_deg+=(double)(s->previous_rate+d->rate_dps)*0.0005*(double)dt;
        s->previous_rate=d->rate_dps; s->have_previous=1u;
    }
}
/*
 * 统一主循环的陀螺服务入口，内部限频处理；额外记录单步最大微秒耗时供调度验收。
 */
void Gyro_Process(GyroService *s,uint32_t now,uint8_t motor_active)
{
    uint32_t began=s->io.cycles(s->io.ctx),cost;
    if (s->data.probe.mode) probe_process(s,now);
    else process(s,now,motor_active);
    cost=(uint32_t)(s->io.cycles(s->io.ctx)-began)/s->io.cycles_per_us;
    if (cost>s->data.max_process_us) s->data.max_process_us=cost;
}
/*
 * 只复制数据并按当前时刻检查新鲜度；过期时清输出有效位，旧角数值仍可用于诊断。
 * 使用方必须检查 raw_valid/rate_valid/angle_valid，不能仅因数值存在就认为可用于控制。
 */
void Gyro_GetSnapshot(const GyroService *s,uint32_t now,GyroSnapshot *out)
{
    *out=s->data;
    if (s->io.wire_trace) out->wire_trace=*s->io.wire_trace; out->age_ms=out->have_sample ? now-out->sample_ms : UINT32_MAX;
    if (!fresh(s,now)) {
        out->raw_valid=out->rate_valid=out->angle_valid=0u;
        if (out->state==GYRO_RUNNING) out->error=GYRO_STALE;
    }
}
const char *Gyro_StateName(const GyroSnapshot *d)
{
    if (d->probe.mode) return "COMM TEST";
    if (d->state==GYRO_INIT) return "INIT";
    if (d->state==GYRO_ERROR) return d->error==GYRO_ID ? "NOT FOUND" : "ERROR";
    if (d->calibration==GYRO_CAL_RUNNING) return "CALIBRATING";
    if (!d->raw_valid) return d->error==GYRO_STALE ? "STALE" : "INVALID";
    if (!d->calibrated) return "UNCAL / RAW";
    if (!d->angle_valid) return "ANGLE GAP";
    return "READY";
}
const char *Gyro_ErrorName(GyroError e)
{
    static const char *names[]={"NONE","I2C","BAD ID","RESET/NO DRDY","CONFIG",
        "INVALID DATA","SATURATED","SAMPLE GAP","STALE"};
    return (unsigned)e<sizeof(names)/sizeof(names[0]) ? names[e] : "UNKNOWN";
}
const char *Gyro_CalName(GyroCalibration c)
{
    static const char *names[]={"CAL REQUIRED","KEEP STILL","OK","MOVING","TIMEOUT",
        "ERROR","MOTOR ACTIVE","SAMPLE GAP","NOT READY","MOTOR UNKNOWN","STOP UNCONF"};
    return (unsigned)c<sizeof(names)/sizeof(names[0]) ? names[c] : "UNKNOWN";
}
