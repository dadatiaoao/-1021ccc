#include "chassis_odometry.h"
#include "chassis_kinematics.h"
#include <math.h>
#include <string.h>
#define RAD 0.01745329251994329577
#define PI 3.14159265358979323846
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
static void invalidate(ChassisOdometry *o, ChassisOdometryIssue issue,
                       uint8_t wheel, uint32_t now)
{
    if (o->baseline || o->valid || !o->last_issue) {
        if (o->baseline || o->valid) ++o->generation;
        o->last_issue=issue; o->break_ms=now; o->break_wheel=wheel;
        o->break_age_ms=o->baseline ? now-o->sample_ms : UINT32_MAX;
    }
    o->baseline=o->valid=0u; o->gyro_head=o->gyro_count=0u;
    memset(o->wheel_count,0,sizeof(o->wheel_count));
}
static const ChassisGyroPoint *gyro_point(const ChassisOdometry *o, unsigned index)
{ return &o->gyro[(o->gyro_head+index)%CHASSIS_GYRO_HISTORY]; }
static void gyro_add(ChassisOdometry *o, const GyroSnapshot *g)
{
    unsigned index;
    if (o->gyro_count && g->sample_ms-gyro_point(o,o->gyro_count-1u)->sample_ms<10u) return;
    if (o->gyro_count==CHASSIS_GYRO_HISTORY) {
        o->gyro_head=(uint16_t)((o->gyro_head+1u)%CHASSIS_GYRO_HISTORY); --o->gyro_count;
    }
    index=(o->gyro_head+o->gyro_count++)%CHASSIS_GYRO_HISTORY;
    o->gyro[index].angle=g->angle_deg; o->gyro[index].sample_ms=g->sample_ms;
}
static int gyro_at(const ChassisOdometry *o, const GyroSnapshot *g, uint32_t time, double *angle)
{
    unsigned i;
    for (i=0u;i<o->gyro_count;++i) {
        const ChassisGyroPoint *a=gyro_point(o,i);
        uint32_t end=i+1u<o->gyro_count ? gyro_point(o,i+1u)->sample_ms : g->sample_ms;
        double next=i+1u<o->gyro_count ? gyro_point(o,i+1u)->angle : g->angle_deg;
        if (time==a->sample_ms) { *angle=a->angle; return 1; }
        if ((int32_t)(time-a->sample_ms)>0 && (int32_t)(end-time)>=0 && end!=a->sample_ms) {
            *angle=a->angle+(next-a->angle)*(time-a->sample_ms)/(end-a->sample_ms); return 1;
        }
    }
    return 0;
}
static int wheel_at(const ChassisOdometry *o, unsigned wheel, uint32_t time, double *counts)
{
    unsigned j;
    for (j=0u;j<o->wheel_count[wheel];++j) {
        const ChassisWheelPoint *a=&o->wheel[wheel][j];
        if (time==a->sample_ms) { *counts=a->counts; return 1; }
        if (j+1u<o->wheel_count[wheel]) {
            const ChassisWheelPoint *b=&o->wheel[wheel][j+1u];
            if ((int32_t)(time-a->sample_ms)>0 && (int32_t)(b->sample_ms-time)>=0) {
                *counts=a->counts+(b->counts-a->counts)*(time-a->sample_ms)/(b->sample_ms-a->sample_ms);
                return 1;
            }
        }
    }
    return 0;
}
void Chassis_OdometryProcess(ChassisOdometry *o, const ChassisConfig *c,
    const MotorBus *bus, const GyroSnapshot *g, uint32_t now)
{
    double counts[4], distance[4], angle, dx, dy, middle, factor;
    uint32_t age=0u, time; unsigned i;
    if (!Chassis_GyroValid(g,now)) { invalidate(o,CHASSIS_ODOM_GYRO_INVALID,0u,now); return; }
    if (o->gyro_count && !Chassis_GyroContinuous(&o->gyro_guard,g))
        invalidate(o,CHASSIS_ODOM_GYRO_CHANGED,0u,now);
    o->gyro_guard=*g; gyro_add(o,g);
    if (Chassis_ConfigIssues(c,1u)) { invalidate(o,CHASSIS_ODOM_CONFIG,0u,now); return; }
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&bus->nodes[i];
        unsigned size=o->wheel_count[i];
        if (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_POSITION,now)) {
            invalidate(o,CHASSIS_ODOM_WHEEL_STALE,(uint8_t)(i+1u),now); return;
        }
        if (now-n->data.position_ms>age) age=now-n->data.position_ms;
        if (size) {
            const ChassisWheelPoint *p=&o->wheel[i][size-1u];
            uint32_t dt=n->data.position_ms-p->sample_ms;
            if (n->data.position_seq==p->sequence) continue;
            if ((int32_t)(n->data.position_seq-p->sequence)<0 || (int32_t)dt<=0) {
                invalidate(o,CHASSIS_ODOM_WHEEL_RESET,(uint8_t)(i+1u),now); return;
            }
            /* Position is signed motor revolutions * 65536. Reject resets or
               jumps larger than travel possible at the configured RPM cap. */
            if (fabs((double)n->data.position_counts-p->counts)>
                ((double)c->max_rpm+1.0)*65536.0*dt/60000.0+4.0) {
                invalidate(o,CHASSIS_ODOM_WHEEL_JUMP,(uint8_t)(i+1u),now); return;
            }
        }
        if (size==CHASSIS_WHEEL_HISTORY) {
            memmove(&o->wheel[i][0],&o->wheel[i][1],(size-1u)*sizeof(o->wheel[i][0])); --size;
        }
        o->wheel[i][size].counts=(double)n->data.position_counts;
        o->wheel[i][size].sample_ms=n->data.position_ms;
        o->wheel[i][size].sequence=n->data.position_seq;
        o->wheel_count[i]=(uint8_t)(size+1u);
    }
    time=now-age;
    if (o->baseline && time==o->sample_ms) return;
    if (o->baseline && (int32_t)(time-o->sample_ms)<0) {
        invalidate(o,CHASSIS_ODOM_WHEEL_RESET,0u,now); return;
    }
    for (i=0u;i<4u;++i) if (!wheel_at(o,i,time,&counts[i])) {
        if (o->baseline) invalidate(o,CHASSIS_ODOM_SYNC_TIMEOUT,(uint8_t)(i+1u),now);
        return;
    }
    if (!gyro_at(o,g,time,&angle)) {
        if (o->baseline) invalidate(o,CHASSIS_ODOM_SYNC_TIMEOUT,0u,now);
        return;
    }
    if (!o->baseline) { o->x=o->y=0.0; o->baseline=1u; }
    else {
        factor=PI*(double)c->wheel_diameter_mm/(65536.0*(double)c->gear_ratio);
        for (i=0u;i<4u;++i) distance[i]=(counts[i]-o->previous_counts[i])*factor*c->motor_sign[i];
        dx=(distance[0]+distance[1]+distance[2]+distance[3])*0.25;
        dy=(-distance[0]+distance[1]+distance[2]-distance[3])*0.25;
        middle=(o->angle+angle)*0.5*RAD;
        o->x+=cos(middle)*dx-sin(middle)*dy;
        o->y+=sin(middle)*dx+cos(middle)*dy;
        if (!Chassis_Finite(o->x) || !Chassis_Finite(o->y)) {
            invalidate(o,CHASSIS_ODOM_NUMERIC,0u,now); return;
        }
    }
    memcpy(o->previous_counts,counts,sizeof(counts));
    o->angle=angle; o->sample_ms=time; ++o->sequence; o->valid=1u;
}
const char *Chassis_OdometryIssueName(ChassisOdometryIssue issue)
{
    static const char * const names[]={"OK","CONFIG","GYRO INVALID","GYRO CHANGED",
        "POSITION STALE","POSITION RESET","POSITION JUMP","TIME ALIGN","NUMERIC"};
    return (unsigned)issue<9u ? names[issue] : "UNKNOWN";
}
