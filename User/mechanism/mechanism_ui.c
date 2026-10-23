#include "mechanism_ui.h"
#include <stdio.h>
#include <string.h>
static int milli(double v)
{
    if (!(v==v)) return 0;
    if (v>99999.0) return 99999000;
    if (v< -99999.0) return -99999000;
    return (int)(v*1000.0);
}
static void units(char out[20], double v)
{
    int value=milli(v), magnitude=value<0 ? -value : value;
    snprintf(out,20,"%s%d.%03d",value<0 ? "-" : "",magnitude/1000,magnitude%1000);
}
void MechanismUi_Line(char out[43], unsigned row, uint8_t detail, const Mechanism *m, uint32_t now)
{
    const MechanismSnapshot *d=&m->data;
    unsigned a=detail ? (detail-1u)%3u : d->axis;
    const MotorNode *n=&m->bus->nodes[a+4u];
    char value[20],target[20];
    static const char * const refs[]={"NO ZERO","HOMED","SESSION"};
    static const char * const results[]={"NONE","RUNNING","SUCCESS","FAILED","CANCELLED"};
    if (row==0u) snprintf(out,43,"MECH %s [tap]",detail ? MotorBus_Role((uint8_t)(a+5u)) : "ALL");
    else if (row==1u && d->homing && d->result==MECH_RUNNING) snprintf(out,43,"%s HOMING",MotorBus_Role((uint8_t)(d->axis+5u)));
    else if (row==1u && d->homing && d->result==MECH_SUCCESS) snprintf(out,43,"HOME READY TRIM:%s",d->trim_ready ? "SESSION ZERO" : "NOT READY");
    else if (row==1u) snprintf(out,43,"%s %s %u/8",MotorBus_Role((uint8_t)(d->axis+5u)),d->done ? "DONE" : results[d->result],d->stage);
    else if (row==2u) snprintf(out,43,"%s",Mechanism_StateName(d->state));
    else if (!detail && row>=3u && row<=5u) {
        a=row-3u; n=&m->bus->nodes[a+4u];
        if (d->reference[a]) units(value,d->position[a]); else strcpy(value,"--");
        snprintf(out,43,"%u %s P:%s%s %s E:%u",a+5u,refs[d->reference[a]],value,a==0u ? "deg" : "mm",
            n->data.online && MotorBus_Fresh(n,MOTOR_VALID_POSITION,now) ? "ON" : "--",n->data.status_flags & 1u);
    } else if (detail && row==3u) snprintf(out,43,"COUNTS:%lld REF:%s",(long long)n->data.position_counts,refs[d->reference[a]]);
    else if (detail && row==4u) {
        if (d->reference[a]) units(value,d->position[a]); else strcpy(value,"--");
        if (!d->homing && d->request_id && a==d->axis) units(target,d->target); else strcpy(target,"--");
        snprintf(out,43,"P:%s T:%s %s",value,target,a==0u ? "deg" : "mm");
    }
    else if (detail && row==5u) snprintf(out,43,"AGE p/s/t:%lu/%lu/%lu",(unsigned long)(now-n->data.position_ms),
        (unsigned long)(now-n->data.speed_ms),(unsigned long)(now-n->data.status_ms));
    else if (row==6u) snprintf(out,43,"PG2:%u/%s PG3:%u/%s GAP:%lu",d->limit[0].raw,
        d->limit[0].valid ? (d->limit[0].stable ? "HIGH" : "LOW") : "?",d->limit[1].raw,
        d->limit[1].valid ? (d->limit[1].stable ? "HIGH" : "LOW") : "?",(unsigned long)d->max_sample_gap_ms);
    else if (row==7u) snprintf(out,43,"STOP:%s CFG 5/6/7:%lu/%lu/%lu",d->stop_pending ? "UNCONF" : "OK",
        (unsigned long)d->config_issues[0],(unsigned long)d->config_issues[1],(unsigned long)d->config_issues[2]);
    else if (d->reason==MECH_OK && d->rejection==MECH_OK && d->reference[0]==MECH_HOMED &&
        d->reference[1]==MECH_HOMED && !d->trim_ready) snprintf(out,43,"TRIM NOT READY");
    else snprintf(out,43,"%s",Mechanism_ReasonName(d->reason!=MECH_OK ? d->reason :
        d->rejection!=MECH_OK ? d->rejection : (d->config_issues[0] || d->config_issues[1]) ? MECH_CONFIG :
        !d->trim_ready ? MECH_REFERENCE : MECH_OK));
}
