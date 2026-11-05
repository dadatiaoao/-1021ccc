#include "wheel_only_trial.h"
#include <math.h>
#include <string.h>
#define WT_PI 3.14159265358979323846
#define WT_TARGET_MM 300.0
#define WT_TARGET_DEG 45.0
#define WT_TIMEOUT_MS 22000u
#define WT_STALL_MS 4000u
static const int8_t patterns[6][4]={{1,1,1,1},{-1,-1,-1,-1},{-1,1,1,-1},{1,-1,-1,1},{-1,1,-1,1},{1,-1,1,-1}};
double WheelTrial_Estimate(const ChassisConfig *c,WheelTrialMove move,const int64_t origin[4],const int64_t current[4])
{
    double sum=0.0, mm; unsigned i;
    if (!c || (unsigned)move>=6u || c->gear_ratio<=0.0f || c->wheel_diameter_mm<=0.0f) return -1.0;
    for(i=0;i<4u;++i) sum+=((double)current[i]-(double)origin[i])*c->motor_sign[i]*patterns[move][i];
    mm=sum*0.25*WT_PI*c->wheel_diameter_mm/(65536.0*c->gear_ratio);
    if (move>=WT_CCW) {
        double k=(c->length_mm+c->width_mm)*0.5;
        if(k<=0.0) return -1.0;
        return mm/k*180.0/WT_PI;
    }
    return mm;
}
static int position_ready(const WheelOnlyTrial *t,uint32_t now)
{
    unsigned i;
    for(i=0;i<4u;++i) if(!t->bus->nodes[i].data.online ||
        !MotorBus_Fresh(&t->bus->nodes[i],MOTOR_VALID_POSITION,now)) return 0;
    return 1;
}
void WheelTrial_Init(WheelOnlyTrial *t,Chassis *c,MotorBus *b)
{ memset(t,0,sizeof(*t)); t->chassis=c; t->bus=b; t->note="WAIT / PREPARE"; }
void WheelTrial_Stop(WheelOnlyTrial *t,uint32_t now)
{
    if(t->state==WT_STOPPING) return;
    Chassis_Stop(t->chassis,now); t->state=WT_STOPPING;
    if(!t->fault) t->note="STOPPING";
}
static void fail(WheelOnlyTrial *t,const char *why,uint32_t now)
{ t->fault=1u; t->note=why; WheelTrial_Stop(t,now); }
int WheelTrial_Start(WheelOnlyTrial *t,WheelTrialMove move,uint32_t now)
{
    unsigned i;
    if((unsigned)move>=6u || t->state==WT_RUNNING || t->state==WT_STOPPING ||
       t->chassis->data.state!=CHASSIS_READY || !position_ready(t,now)) {
        t->note="NEED READY / WHEEL DATA"; return 0;
    }
    for(i=0;i<4u;++i) {
        const MotorSnapshot *d=&t->bus->nodes[i].data;
        t->origin[i]=t->last_count[i]=d->position_counts; t->seq[i]=d->position_seq; t->sample_ms[i]=d->position_ms;
    }
    t->move=move; t->progress=t->max_progress=0; t->fault=0u;
    t->start_ms=t->progress_ms=now; t->publish_ms=now-100u; t->state=WT_RUNNING; t->note="RUNNING";
    return 1;
}
void WheelTrial_Process(WheelOnlyTrial *t,uint32_t now)
{
    unsigned i; int64_t counts[4]; float vx=0,vy=0,omega=0; double target;
    if(t->state==WT_STOPPING) {
        if(t->chassis->data.state==CHASSIS_READY || t->chassis->data.state==CHASSIS_DISARMED) {
            t->state=t->fault ? WT_FAULT : WT_DONE; if(!t->fault) t->note="STOPPED";
        } else if(t->chassis->data.state==CHASSIS_FAULT) { t->state=WT_FAULT; t->note="STOP UNCONFIRMED"; }
        return;
    }
    if(t->state!=WT_RUNNING) return;
    if(t->chassis->data.state!=CHASSIS_READY && t->chassis->data.state!=CHASSIS_VELOCITY) {
        fail(t,"CHASSIS FAULT / STOP",now); return;
    }
    if(!position_ready(t,now)) { fail(t,"POSITION STALE / STOP",now); return; }
    for(i=0;i<4u;++i) {
        const MotorSnapshot *d=&t->bus->nodes[i].data;
        counts[i]=d->position_counts;
        if(d->position_seq!=t->seq[i]) {
            uint32_t dt=d->position_ms-t->sample_ms[i];
            double change=fabs((double)counts[i]-(double)t->last_count[i]);
            if((int32_t)(d->position_seq-t->seq[i])<=0 || (int32_t)dt<=0 ||
               change>((double)t->chassis->config.max_rpm+1.0)*65536.0*dt/60000.0+4.0) {
                fail(t,"ENCODER JUMP / STOP",now); return;
            }
            t->seq[i]=d->position_seq; t->sample_ms[i]=d->position_ms; t->last_count[i]=counts[i];
        }
    }
    t->progress=WheelTrial_Estimate(&t->chassis->config,t->move,t->origin,counts);
    if(!Chassis_Finite(t->progress) || t->progress < -20.0) { fail(t,"DIRECTION ERROR / STOP",now); return; }
    if(t->progress>t->max_progress+2.0) { t->max_progress=t->progress; t->progress_ms=now; }
    target=t->move>=WT_CCW ? WT_TARGET_DEG : WT_TARGET_MM;
    if(t->progress>=target) { t->note="TARGET ESTIMATE / STOP"; WheelTrial_Stop(t,now); return; }
    if(now-t->start_ms>WT_TIMEOUT_MS || now-t->progress_ms>WT_STALL_MS) { fail(t,"TIMEOUT / STOP",now); return; }
    if(now-t->publish_ms<100u) return;
    if(t->move==WT_FORWARD) vx=30; else if(t->move==WT_BACK) vx=-30;
    else if(t->move==WT_LEFT) vy=30; else if(t->move==WT_RIGHT) vy=-30;
    else omega=t->move==WT_CCW ? 10.0f : -10.0f;
    if(Chassis_SetBodyVelocity(t->chassis,vx,vy,omega,now)!=CHASSIS_ACCEPTED) { fail(t,"COMMAND REJECT / STOP",now); return; }
    t->publish_ms=now;
}
