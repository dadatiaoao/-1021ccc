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
/*
 * 输出概览或诊断行；角度无效时显示 --/GAP，不能把保留的旧角度显示成当前可用结果。
 */
void GyroUi_Line(char t[43],unsigned row,uint8_t detail,const GyroSnapshot *d)
{
    char a[20],b[20]; t[0]='\0';
    if (!d) { if (!row) strcpy(t,"GYRO unavailable"); return; }
    if (!row) { strcpy(t,detail ? "GYRO diagnostics" : "GYRO relative rotation"); return; }
    if (detail) {
        switch (row) {
        case 1u: snprintf(t,43,"%s ID:%02X A:%02X",Gyro_StateName(d),d->who,GYRO_ADDRESS_7BIT); break;
        case 2u: snprintf(t,43,"SEQ:%lu DT:%lums",(unsigned long)d->sequence,(unsigned long)d->interval_ms); break;
        case 3u: snprintf(t,43,"MAXdt:%lu POLL:%lums",(unsigned long)d->max_interval_ms,(unsigned long)d->max_poll_ms); break;
        case 4u: snprintf(t,43,"PROC max:%luus",(unsigned long)d->max_process_us); break;
        case 5u: snprintf(t,43,"MISS:%lu GAP:%lu",(unsigned long)d->missed,(unsigned long)d->gaps); break;
        case 6u: snprintf(t,43,"SAT:%lu BAD:%lu",(unsigned long)d->saturations,(unsigned long)d->invalid); break;
        case 7u: snprintf(t,43,"LAST:%s BUS:%u",Gyro_ErrorName(d->last_fault),(unsigned)d->bus_error); break;
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
    case 6u: if (!d->have_sample) strcpy(t,"DT:-- AGE:-- ms"); else snprintf(t,43,"DT:%lu AGE:%lu ms",(unsigned long)d->interval_ms,(unsigned long)d->age_ms); break;
    case 7u: snprintf(t,43,"ERR:%s BUS:%u",Gyro_ErrorName(d->error),(unsigned)d->bus_error); break;
    case 8u: strcpy(t,"Relative integral / drifts with time"); break;
    default: break;
    }
}
