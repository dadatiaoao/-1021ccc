#include "chassis_odometry.h"
#include "chassis_kinematics.h"
#include <math.h>
#define RAD 0.01745329251994329577
int Chassis_GyroValid(const GyroSnapshot *g, uint32_t now)
{
    return g && g->state==GYRO_RUNNING && g->calibrated && g->have_sample &&
        g->angle_valid && g->rate_valid && !g->angle_gap && g->calibration!=GYRO_CAL_RUNNING &&
        now-g->sample_ms<=30u && Chassis_Finite(g->angle_deg) && Chassis_Finite(g->rate_dps);
}
int Chassis_GyroContinuous(const GyroSnapshot *a, const GyroSnapshot *b)
{
    return a->gaps==b->gaps && a->restarts==b->restarts && a->zeros==b->zeros &&
        a->cal_requests==b->cal_requests && a->invalid==b->invalid &&
        a->errors==b->errors && a->saturations==b->saturations &&
        (int32_t)(b->sequence-a->sequence)>=0;
}
static int flow_valid(const FlowSnapshot *f, uint32_t now)
{
    return f && f->state==FLOW_READY && f->have_sample && f->valid && f->tracking &&
        f->comm_ok && !f->overflow && now-f->sample_ms<=100u;
}
static ChassisOdometryIssue flow_change(const FlowSnapshot *a, const FlowSnapshot *b)
{
    if (a->clears!=b->clears) return CHASSIS_ODOM_FLOW_CLEAR;
    if (a->segments!=b->segments) return CHASSIS_ODOM_FLOW_RESTART;
    if (a->gaps!=b->gaps) return CHASSIS_ODOM_FLOW_GAP;
    if (a->overflow!=b->overflow) return CHASSIS_ODOM_FLOW_OVERFLOW;
    if (a->rejected!=b->rejected) return CHASSIS_ODOM_FLOW_REJECTED;
    if (a->errors!=b->errors) return CHASSIS_ODOM_FLOW_INVALID;
    if ((int32_t)(b->sequence-a->sequence)<0) return CHASSIS_ODOM_FLOW_RESET;
    return CHASSIS_ODOM_OK;
}
static void invalidate(ChassisOdometry *o, ChassisOdometryIssue issue,
                       const FlowSnapshot *f, uint32_t now)
{
    if (o->baseline || o->valid || !o->generation) {
        if (o->baseline || o->valid) ++o->generation;
        o->last_issue=issue; o->break_ms=now;
        o->break_flow_age_ms=f->have_sample ? now-f->sample_ms : UINT32_MAX;
        o->break_paired_age_ms=o->baseline ? now-o->flow_guard.sample_ms : UINT32_MAX;
        o->break_flow_error=f->error; o->break_quality=f->raw.squal;
        if (f->rejected!=o->flow_guard.rejected && f->last_rejection!=FLOW_OK) {
            o->break_flow_error=f->last_rejection; o->break_quality=f->last_rejected_quality;
        }
        o->break_rejected=f->rejected; o->break_gaps=f->gaps;
        o->break_segments=f->segments; o->break_clears=f->clears;
    }
    o->baseline=o->valid=o->pending=0u;
}
static int aligned(const ChassisOdometry *o, uint32_t time, double *angle)
{
    const ChassisGyroPoint *a=&o->gyro[0], *b=&o->gyro[1];
    uint32_t dt;
    if (!o->gyro_count) return 0;
    if (b->sample_ms==time) { *angle=b->angle; return 1; }
    if (o->gyro_count<2u || (int32_t)(time-a->sample_ms)<0 ||
        (int32_t)(b->sample_ms-time)<0) return 0;
    dt=b->sample_ms-a->sample_ms;
    if (!dt || dt>14u) return 0;
    *angle=a->angle+(b->angle-a->angle)*(time-a->sample_ms)/dt;
    return 1;
}
static void consume(ChassisOdometry *o, const ChassisConfig *c,
                    const FlowSnapshot *f, double angle, uint32_t now)
{
    double a, b, m, mount, dx, dy, bx, by;
    if (!o->baseline) {
        o->x=o->y=0.0; o->baseline=o->valid=1u;
    } else {
        dx=((double)f->total_x-(double)o->flow_guard.total_x)*c->flow_x_mm_count;
        dy=((double)f->total_y-(double)o->flow_guard.total_y)*c->flow_y_mm_count;
        mount=(double)c->flow_mount_deg*RAD; bx=cos(mount)*dx-sin(mount)*dy;
        by=sin(mount)*dx+cos(mount)*dy;
        a=o->flow_angle*RAD; b=angle*RAD; m=(a+b)*0.5;
        o->x+=cos(m)*bx-sin(m)*by-
            ((cos(b)-cos(a))*c->flow_offset_x_mm-(sin(b)-sin(a))*c->flow_offset_y_mm);
        o->y+=sin(m)*bx+cos(m)*by-
            ((sin(b)-sin(a))*c->flow_offset_x_mm+(cos(b)-cos(a))*c->flow_offset_y_mm);
        if (!Chassis_Finite(o->x) || !Chassis_Finite(o->y)) {
            invalidate(o,CHASSIS_ODOM_NUMERIC,f,now); return;
        }
    }
    o->flow_angle=angle; o->flow_guard=*f;
}
void Chassis_OdometryProcess(ChassisOdometry *o, const ChassisConfig *c,
    const FlowSnapshot *f, const GyroSnapshot *g, uint32_t now)
{
    double angle; ChassisOdometryIssue issue;
    if (!Chassis_GyroValid(g,now)) {
        invalidate(o,CHASSIS_ODOM_GYRO_INVALID,f,now); o->gyro_count=0u; return;
    }
    if (o->gyro_count && !Chassis_GyroContinuous(&o->gyro_guard,g)) {
        invalidate(o,CHASSIS_ODOM_GYRO_CHANGED,f,now); o->gyro_count=0u;
    }
    if (!o->gyro_count || g->sequence!=o->gyro[1].sequence) {
        o->gyro[0]=o->gyro[1]; o->gyro[1].angle=g->angle_deg;
        o->gyro[1].sequence=g->sequence; o->gyro[1].sample_ms=g->sample_ms;
        if (o->gyro_count<2u) ++o->gyro_count;
    }
    o->gyro_guard=*g;
    if (Chassis_ConfigIssues(c,1u)) { invalidate(o,CHASSIS_ODOM_CONFIG,f,now); return; }
    if (!flow_valid(f,now)) {
        issue=f->overflow ? CHASSIS_ODOM_FLOW_OVERFLOW :
            f->have_sample && now-f->sample_ms>100u ? CHASSIS_ODOM_FLOW_STALE : CHASSIS_ODOM_FLOW_INVALID;
        invalidate(o,issue,f,now); return;
    }
    issue=o->baseline ? flow_change(&o->flow_guard,f) :
        o->pending ? flow_change(&o->pending_flow,f) : CHASSIS_ODOM_OK;
    if (issue!=CHASSIS_ODOM_OK) invalidate(o,issue,f,now);
    if ((o->baseline && now-o->flow_guard.sample_ms>100u) ||
        (o->pending && now-o->pending_flow.sample_ms>100u))
        invalidate(o,CHASSIS_ODOM_SYNC_TIMEOUT,f,now);
    /* Consume the OLDER pending sample before considering the newer snapshot.
       Otherwise a fixed service phase can replace every sample before its
       future gyro bracket arrives, despite both sensors remaining healthy. */
    if (o->pending) {
        if (aligned(o,o->pending_flow.sample_ms,&angle)) {
            consume(o,c,&o->pending_flow,angle,now); o->pending=0u;
        } else if (o->gyro_count==2u && (int32_t)(o->gyro[0].sample_ms-o->pending_flow.sample_ms)>0) {
            /* This timestamp has left the available gyro bracket. Retain its
               counts via the cumulative total; do not extrapolate its angle. */
            o->pending=0u;
        }
    }
    if (o->pending || (o->baseline && f->sequence==o->flow_guard.sequence)) return;
    if (aligned(o,f->sample_ms,&angle)) consume(o,c,f,angle,now);
    else { o->pending_flow=*f; o->pending=1u; }
}
const char *Chassis_OdometryIssueName(ChassisOdometryIssue issue)
{
    static const char * const names[]={"OK","CONFIG","GYRO INVALID","GYRO CHANGED","FLOW DATA",
        "STALE","REJECTED SAMPLE","SAMPLE GAP","RESTART","CLEARED","OVERFLOW","SEQ RESET","TIME ALIGN","NUMERIC"};
    return (unsigned)issue<14u ? names[issue] : "UNKNOWN";
}
const char *Chassis_FlowFaultName(ChassisOdometryIssue issue, FlowError error)
{
    if (issue==CHASSIS_ODOM_FLOW_INVALID || issue==CHASSIS_ODOM_FLOW_REJECTED) {
        switch (error) {
        case FLOW_SPI: return "SPI/TIMEBASE";
        case FLOW_ID: return "BAD ID";
        case FLOW_BUS_DATA: return "BUS 00/FF";
        case FLOW_LOW_QUALITY: return "LOW QUALITY";
        case FLOW_STATUS: return "BAD STATUS";
        case FLOW_GAP: return "SAMPLE GAP";
        case FLOW_NO_VALID: return "NO VALID DATA";
        case FLOW_OVERFLOW: return "OVERFLOW";
        default: break;
        }
    }
    return Chassis_OdometryIssueName(issue);
}
