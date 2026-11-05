/*
 * 陀螺页面文本格式化：根据快照有效位显示角速度、相对角和校准/GAP 原因。
 * 只生成字符串，不访问传感器或更新积分；避免浮点 printf 带入较大的运行库和栈成本。
 */
#include "gyro_ui.h"
#include "gyro_config.h"
#include <stdio.h>
#include <string.h>
/* Avoid pulling floating-point printf and its stack cost into AC5. */
static void decimal(char t[20],double value)
{
    long v; unsigned long magnitude;
    if (!(value>=-20000000.0 && value<=20000000.0)) { strcpy(t,"RANGE"); return; }
    v=(long)(value*100.0+(value<0.0 ? -0.5 : 0.5));
    magnitude=(unsigned long)(v<0 ? -v : v);
    snprintf(t,20,"%s%lu.%02lu",v<0 ? "-" : "",magnitude/100u,magnitude%100u);
}
static const char *bus_name(GyroBusError e)
{
    static const char * const names[]={"OK","NACK","TIMEOUT","BUSY","IO",
        "SCL_LOW","SDA_LOW","BOTH_LOW","HW_BUSY"};
    return (unsigned)e<sizeof(names)/sizeof(names[0]) ? names[e] : "UNKNOWN";
}
static void probe_counts(char t[43],const char *label,const GyroProbeCounts *c)
{
    unsigned rate=c->total ? (unsigned)(((uint64_t)c->ok*10000u)/c->total) : 0u;
    if (!c->total) { snprintf(t,43,"%s:0/0 --",label); return; }
    snprintf(t,43,"%s:%lu/%lu %u.%02u%%",label,(unsigned long)c->ok,
        (unsigned long)c->total,rate/100u,rate%100u);
}
static void probe_line(char t[43],unsigned row,const GyroSnapshot *d)
{
    const GyroProbe *p=&d->probe;
    const GyroProbeCounts *c;
    static const char * const phases[]={"PRE","BANK","RESET","POST","HOLD"};
    char bank[12];
    if (!row) { strcpy(t,"GYRO communication test"); return; }
    if (p->mode==3u) {
        switch (row) {
        case 1u: snprintf(t,43,"TEST3 %s",p->phase==GYRO_PROBE_RELEASE_WAIT ? "RELEASE WAIT" :
            p->phase==GYRO_PROBE_RELEASED ? "PINS RELEASED" :
            p->phase==GYRO_PROBE_UNSUPPORTED ? "UNSUPPORTED" : "READ ONLY"); break;
        case 2u: probe_counts(t,"READ",&p->before); break;
        case 3u: snprintf(t,43,"FAULT:%s INIT:%s",bus_name(p->release_cause),bus_name(p->init_bus)); break;
        case 4u:
            if (p->phase==GYRO_PROBE_RELEASE_WAIT || p->phase==GYRO_PROBE_RELEASED)
                snprintf(t,43,"BEFORE SCL:%u SDA:%u BUSY:%u",p->lines_before&1u,
                    (p->lines_before>>1)&1u,(p->lines_before>>2)&1u);
            else strcpy(t,"BEFORE:--"); break;
        case 5u:
            if (p->after_valid) snprintf(t,43,"AFTER10 SCL:%u SDA:%u",p->lines_after&1u,(p->lines_after>>1)&1u);
            else strcpy(t,"AFTER10:--"); break;
        case 6u:
            if (p->after_valid) snprintf(t,43,"NOW SCL:%u SDA:%u",p->lines_now&1u,(p->lines_now>>1)&1u);
            else strcpy(t,"NOW:--"); break;
        case 7u:
            if (d->wire_trace.valid) snprintf(t,43,"FIRST S:%u R:%02X C1:%04X",d->wire_trace.stage,d->wire_trace.reg,d->wire_trace.cr1);
            else strcpy(t,"FIRST:-- (NO HAL CAPTURE)"); break;
        case 8u:
            if (d->wire_trace.valid) snprintf(t,43,"SR1:%04X SR2:%04X E:%u",d->wire_trace.sr1,d->wire_trace.sr2,d->wire_trace.reason);
            else strcpy(t,"WAIT FIRST HAL ERROR"); break;
        default: break;
        }
        return;
    }
    switch (row) {
    case 1u: snprintf(t,43,"TEST%u %s ID:%02X %s",(unsigned)p->mode,
        p->phase<=GYRO_PROBE_HOLD ? phases[p->phase] : "?",(unsigned)d->who,
        bus_name(d->bus_error)); break;
    case 2u: probe_counts(t,"PRE",&p->before); break;
    case 3u: probe_counts(t,"POST",&p->after); break;
    case 4u:
        if (p->bank_seen) snprintf(bank,sizeof(bank),"%02X",(unsigned)p->bank);
        else snprintf(bank,sizeof(bank),"%s",p->bank_bus ? bus_name(p->bank_bus) : "--");
        snprintf(t,43,"RST:%s INIT:%s BANK:%s",p->reset_sent ? bus_name(p->reset_bus) : "--",
            bus_name(p->init_bus),bank); break;
    case 5u: case 6u:
        c=row==5u ? &p->before : &p->after;
        snprintf(t,43,"%s N:%lu T:%lu B:%lu",row==5u ? "PRE" : "POST",
            (unsigned long)c->nack,(unsigned long)c->timeout,(unsigned long)c->busy); break;
    case 7u: case 8u:
        c=row==7u ? &p->before : &p->after;
        snprintf(t,43,"%s BADID:%lu OTHER:%lu",row==7u ? "PRE" : "POST",
            (unsigned long)c->bad_id,(unsigned long)c->other); break;
    default: break;
    }
}
/* Compact records fit the existing eight-row MORE page. No new touch target. */
static void failure_line(char t[43],const char *label,const GyroBusFailure *f)
{
    char stage[16];
    if (!f->valid) { snprintf(t,43,"%s:NONE",label); return; }
    if (f->stage==GYRO_TRACE_BUS_INIT) {
        snprintf(t,43,"%s:BUS_INIT -- %s",label,bus_name(f->bus)); return;
    }
    if (f->stage==GYRO_TRACE_INIT)
        snprintf(stage,sizeof(stage),"INIT%u.%u",(unsigned)f->phase,(unsigned)f->index);
    else if (f->stage==GYRO_TRACE_CHECK)
        snprintf(stage,sizeof(stage),"CHECK%u",(unsigned)f->index);
    else strcpy(stage,"SAMPLE");
    snprintf(t,43,"%s:%s %c:%02X %s",label,stage,(char)f->op,
             (unsigned)f->reg,bus_name(f->bus));
}
/*
 * 输出概览或诊断行；角度无效时显示 --/GAP，不能把保留的旧角度显示成当前可用结果。
 */
void GyroUi_Line(char t[43],unsigned row,uint8_t detail,const GyroSnapshot *d)
{
    char a[20],b[20]; t[0]='\0';
    if (!d) { if (!row) strcpy(t,"GYRO unavailable"); return; }
    if (d->probe.mode) { probe_line(t,row,d); return; }
    if (!row) { strcpy(t,detail ? "GYRO diagnostics" : "GYRO relative rotation"); return; }
    if (detail) {
        switch (row) {
        case 1u: snprintf(t,43,"%s ID:%02X A:%02X",Gyro_StateName(d),d->who,GYRO_ADDRESS_7BIT); break;
        case 2u: snprintf(t,43,"SEQ:%lu DT:%lums",(unsigned long)d->sequence,(unsigned long)d->interval_ms); break;
        case 3u: snprintf(t,43,"MAXdt:%lu POLL:%lums",(unsigned long)d->max_interval_ms,(unsigned long)d->max_poll_ms); break;
        case 4u: failure_line(t,"FIRST",&d->first_bus_failure); break;
        case 5u:
            if (d->config4_seen)
                snprintf(t,43,"LAST7A:%02X %s GAP:%lu",(unsigned)d->config4_before,
                         d->config4_before==0x83u ? "SKIP" : "WRITE",(unsigned long)d->gaps);
            else snprintf(t,43,"LAST7A:-- GAP:%lu",(unsigned long)d->gaps);
            break;
        case 6u: failure_line(t,"RECENT",&d->last_bus_failure); break;
        case 7u: snprintf(t,43,"LAST:%s LBUS:%s",Gyro_ErrorName(d->last_fault),bus_name(d->last_bus_error)); break;
        case 8u: snprintf(t,43,"REINIT:%lu E:%lu",(unsigned long)d->restarts,(unsigned long)d->errors); break;
        default: break;
        }
        return;
    }
    switch (row) {
    case 1u: snprintf(t,43,"%s ID:%02X AXIS:%c%c",Gyro_StateName(d),d->who,"XYZ"[d->axis],d->sign<0 ? '-' : '+'); break;
    case 2u: if (d->rate_valid) decimal(a,d->rate_dps); else strcpy(a,"--"); snprintf(t,43,"RATE:%s deg/s",a); break;
    case 3u: if (d->angle_valid) decimal(a,d->angle_deg); else strcpy(a,"--"); snprintf(t,43,"REL:%s deg%s",a,d->angle_gap ? " GAP" : ""); break;
    case 4u:
        if (d->raw_valid) decimal(a,d->raw_dps); else strcpy(a,"--");
        if (d->calibrated) decimal(b,d->bias_dps); else strcpy(b,"--");
        snprintf(t,43,"RAW:%.12s BIAS:%.12s",a,b); break;
    case 5u:
#if GYRO_TEMP_FIXED_BIAS_ENABLED
        if (d->calibrated && d->calibration==GYRO_CAL_NONE) {
            decimal(a,d->bias_dps);
            snprintf(t,43,"BIAS:FIXED %.12s / NO REAL CAL",a); break;
        }
#endif
        snprintf(t,43,"CAL:%s %lu/%u",Gyro_CalName(d->calibration),(unsigned long)d->cal_count,GYRO_CAL_SAMPLES); break;
    case 6u:
        /* Six overview rows fit above the buttons. Put failures in the visible
           timing row rather than an unreachable seventh row. */
        if (d->state==GYRO_INIT) strcpy(t,"INIT: WAIT SENSOR");
        else if (d->error!=GYRO_OK || d->angle_gap)
            snprintf(t,43,"ERR:%s BUS:%s",Gyro_ErrorName(d->error!=GYRO_OK ? d->error : d->last_fault),bus_name(d->bus_error));
        else if (!d->have_sample) strcpy(t,"WAIT FIRST SAMPLE");
        else if (d->calibration==GYRO_CAL_RUNNING) strcpy(t,"CAL IN PROGRESS");
        else if (!d->calibrated) strcpy(t,"CAL REQUIRED");
        else snprintf(t,43,"DT:%lu AGE:%lu ms",(unsigned long)d->interval_ms,(unsigned long)d->age_ms);
        break;
    case 7u: snprintf(t,43,"ERR:%s BUS:%s",Gyro_ErrorName(d->error),bus_name(d->bus_error)); break;
    case 8u: strcpy(t,"Relative integral / drifts with time"); break;
    default: break;
    }
}
