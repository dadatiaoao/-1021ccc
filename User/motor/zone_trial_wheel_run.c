#include "wheel_run.h"
#include "motor_test_config.h"
#include <string.h>
static int healthy(const WheelRun *r,uint32_t now,int rest)
{
    unsigned i;
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&r->bus->nodes[i];
        if (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
            !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || n->stop_unconfirmed ||
            n->action==MOTOR_UNKNOWN || (n->data.status_flags & 12u) ||
            (rest && (n->motion || n->data.speed_rpm))) return 0;
    }
    return !(r->bus->stop_mask & MOTOR_WHEELS);
}
void WheelRun_Init(WheelRun *r,MotorBus *bus)
{ memset(r,0,sizeof(*r)); r->bus=bus; r->note="WAIT 1-4 ONLINE / TAP FORWARD"; }
void WheelRun_Stop(WheelRun *r,uint32_t now)
{
    if (r->state==WHEEL_RUN_STOPPING) return;
    MotorBus_Stop(r->bus,MOTOR_WHEELS,now);
    r->state=WHEEL_RUN_STOPPING; r->phase_ms=now;
    if (!r->fault) r->note=r->task_valid ? "QR FOUND / STOPPING" : "STOPPING / VERIFY ZERO RPM";
}
static void fail(WheelRun *r,const char *note,uint32_t now)
{ r->fault=1u; r->note=note; WheelRun_Stop(r,now); }
int WheelRun_Start(WheelRun *r,uint8_t reverse,uint32_t now)
{
    if (r->state!=WHEEL_RUN_READY && r->state!=WHEEL_RUN_DONE) return 0;
    if (reverse>1u || !healthy(r,now,1) || r->bus->queue_count || r->bus->wheels_pending ||
        r->bus->sync_state==MOTOR_SYNC_PREPARE || r->bus->sync_state==MOTOR_SYNC_TRIGGER ||
        r->bus->sync_state==MOTOR_SYNC_VERIFY) {
        r->note="NEED 1-4 ONLINE / HEALTHY / STOPPED"; return 0;
    }
    r->reverse=reverse; r->index=r->waiting=r->fault=r->until_qr=r->task_valid=0u;
    r->phase_ms=now; r->state=WHEEL_RUN_ENABLE; r->note="ENABLING WHEELS 1-4";
    return 1;
}
void WheelRun_BindQr(WheelRun *r,QrService *qr,void (*new_scan)(QrService *,uint32_t))
{ r->qr=qr; r->new_scan=new_scan; }
int WheelRun_StartUntilQr(WheelRun *r,uint32_t now)
{
    uint32_t old_session;
    if (r->state!=WHEEL_RUN_READY && r->state!=WHEEL_RUN_DONE) return 0;
    if (!r->qr || !r->new_scan || !r->qr->data.uart_ready) {
        r->note="XM1605 UART NOT READY"; return 0;
    }
    old_session=r->qr->data.session;
    if (!WheelRun_Start(r,0u,now)) return 0;
    r->new_scan(r->qr,now); /* BSP clears DMA/SW backlog before a new session. */
    if (r->qr->data.session==old_session || r->qr->data.valid || !r->qr->data.uart_ready) {
        fail(r,"SCAN SESSION FAILED / STOP",now); return 0;
    }
    r->until_qr=1u; r->scan_ms=now; r->scan_session=r->qr->data.session;
    r->scan_seq=r->qr->data.accepted_seq; r->scan_errors=r->qr->data.uart_errors;
    r->scan_overflows=r->qr->data.overflows; return 1;
}
void WheelRun_Process(WheelRun *r,uint32_t now)
{
    static const int8_t sign[4]={1,-1,1,-1};
    unsigned i; int32_t rpm[4];
    if (r->state==WHEEL_RUN_STOPPING) {
        if (healthy(r,now,1)) {
            r->state=r->fault ? WHEEL_RUN_FAULT : WHEEL_RUN_DONE;
            if (!r->fault) r->note=r->task_valid ? "QR OK / STOPPED" : "STOPPED / TAP TO START";
        } else if ((uint32_t)(now-r->phase_ms)>=MOTOR_STOP_TIMEOUT_MS) {
            r->fault=1u; r->note="STOP NOT CONFIRMED / CHECK POWER";
        }
        return;
    }
    if (r->state!=WHEEL_RUN_ENABLE && r->state!=WHEEL_RUN_MOVING) return;
    if (r->until_qr) {
        const QrSnapshot *q=&r->qr->data;
        if (!q->uart_ready || q->session!=r->scan_session || q->uart_errors!=r->scan_errors ||
            q->overflows!=r->scan_overflows) { fail(r,"SCANNER ERROR / STOP",now); return; }
        if (q->valid && q->accepted_seq!=r->scan_seq && (int32_t)(q->accepted_ms-r->scan_ms)>=0) {
            if (QrTaskCode_Parse((const uint8_t *)q->current.text,QR_TASK_LENGTH,&r->task)!=QR_CODE_OK) {
                fail(r,"INVALID SCAN RESULT / STOP",now); return;
            }
            r->task_valid=1u; WheelRun_Stop(r,now); return;
        }
    }
    if (!healthy(r,now,r->state==WHEEL_RUN_ENABLE)) { fail(r,"WHEEL FEEDBACK FAULT / STOP",now); return; }
    if (r->state==WHEEL_RUN_ENABLE) {
        if ((uint32_t)(now-r->phase_ms)>=5000u) { fail(r,"ENABLE TIMEOUT / STOP",now); return; }
        if (r->waiting) {
            const MotorNode *n=&r->bus->nodes[r->index];
            if (n->action==MOTOR_REJECTED || n->action==MOTOR_CANCELLED) { fail(r,"ENABLE REJECTED / STOP",now); return; }
            if (n->action!=MOTOR_COMPLETE) return;
            r->waiting=0u; ++r->index;
        }
        while (r->index<4u && (r->bus->nodes[r->index].data.status_flags & 1u)) ++r->index;
        if (r->index<4u) {
            if (MotorBus_Enable(r->bus,(uint8_t)(r->index+1u),1u,now)) r->waiting=1u;
            return;
        }
        r->state=WHEEL_RUN_MOVING; r->phase_ms=now; r->publish_ms=now-100u;
        r->note=r->until_qr ? "FORWARD / SCANNING XM1605" :
            r->reverse ? "REVERSE / 20RPM / 2s" : "FORWARD / 20RPM / 2s";
    }
    if (!r->until_qr && (uint32_t)(now-r->phase_ms)>=8000u) { WheelRun_Stop(r,now); return; }
    for (i=0u;i<4u;++i) {
        if (!(r->bus->nodes[i].data.status_flags & 1u)) { fail(r,"WHEEL DISABLED / STOP",now); return; }
        rpm[i]=(r->reverse ? -1 : 1)*sign[i]*WHEEL_RUN_RPM;
    }
    if ((uint32_t)(now-r->publish_ms)>=100u) {
        if (!MotorBus_Wheels(r->bus,rpm,100u,WHEEL_RUN_LEASE_MS,now)) { fail(r,"COMMAND REJECTED / STOP",now); return; }
        r->publish_ms=now;
    }
}
