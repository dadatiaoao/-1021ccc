/* Isolated first-leg trial: no delays, no raw UART writes, no cargo actions.
 * Chassis supplies acceleration limiting, heading hold, feedback supervision,
 * and verified stopping. Only XM1605's NEW session can complete this trial. */
#include "departure.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

int Departure_IsRunning(const Departure *d)
{ return d && d->state>=DEPARTURE_PREPARE && d->state<=DEPARTURE_SCAN; }
static int motors_healthy(const Departure *d, uint32_t now, int rest)
{
    unsigned i;
    if (d->motor->stop_mask) return 0;
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&d->motor->nodes[i];
        if (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
            !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || n->stop_unconfirmed ||
            (n->data.status_flags & 12u) || n->action==MOTOR_UNKNOWN ||
            n->action==MOTOR_STOP_PENDING ||
            (rest && (n->motion || n->data.speed_rpm))) return 0;
    }
    return 1;
}
static int sensors_healthy(const Departure *d, uint32_t now)
{
    const Chassis *c=d->chassis;
    return Chassis_GyroValid(&c->gyro,now) && c->odometry.valid &&
        (uint32_t)(now-c->odometry.sample_ms)<=MOTOR_BUS_FRESH_MS;
}
static void fail(Departure *d, DepartureError error, uint32_t now)
{
    d->state=DEPARTURE_FAULT; d->error=error; d->task_valid=0u;
    Chassis_Stop(d->chassis,now); MotorBus_Stop(d->motor,MOTOR_WHEELS,now);
}
void Departure_Init(Departure *d, Chassis *c, MotorBus *m, QrService *q, DeparturePort port)
{
    memset(d,0,sizeof(*d)); d->chassis=c; d->motor=m; d->qr=q; d->port=port;
}
int Departure_Start(Departure *d, uint32_t now)
{
    Chassis *c=d->chassis;
    if (d->started || d->state!=DEPARTURE_IDLE) return 0;
    d->error=DEPARTURE_OK;
    if (!Chassis_Finite(DEPARTURE_FORWARD_MM) || DEPARTURE_FORWARD_MM<=0.0f ||
        DEPARTURE_FORWARD_MM>4000.0f || !Chassis_Finite(DEPARTURE_SPEED_MM_S) ||
        DEPARTURE_SPEED_MM_S<=0.0f || DEPARTURE_SPEED_MM_S>c->config.max_speed_mm_s ||
        Chassis_ConfigIssues(&c->config,1u)) d->error=DEPARTURE_CONFIG;
    else if (Chassis_OwnsWheels(c) || c->data.state==CHASSIS_FAULT ||
        d->motor->manual || d->motor->queue_count || d->motor->wheels_pending ||
        (c->external_busy && *c->external_busy)) d->error=DEPARTURE_BUSY;
    else if (!d->port.arm_ready || !d->port.arm_ready(d->port.context,now)) d->error=DEPARTURE_ARM;
    else if (!sensors_healthy(d,now)) d->error=DEPARTURE_SENSOR;
    else if (!motors_healthy(d,now,1)) d->error=DEPARTURE_MOTOR;
    else if (!d->qr->data.uart_ready || !d->port.new_scan) d->error=DEPARTURE_SCANNER;
    if (d->error!=DEPARTURE_OK) return 0;
    if (Chassis_Prepare(c,now)!=CHASSIS_ACCEPTED) { d->error=DEPARTURE_CHASSIS; return 0; }
    d->started=1u; d->state=DEPARTURE_PREPARE; d->started_ms=d->phase_ms=now;
    d->generation=c->odometry.generation; d->start_gyro=c->gyro;
    d->request_id=c->data.request_id; return 1;
}
void Departure_Stop(Departure *d, uint32_t now)
{
    if (!Departure_IsRunning(d)) return;
    d->state=DEPARTURE_ABORTED; d->error=DEPARTURE_STOPPED; d->task_valid=0u;
    Chassis_Stop(d->chassis,now); MotorBus_Stop(d->motor,MOTOR_WHEELS,now);
}
void Departure_Process(Departure *d, uint32_t now)
{
    Chassis *c=d->chassis;
    QrTaskCode checked;
    if (!Departure_IsRunning(d)) return;
    if (!sensors_healthy(d,now) || c->odometry.generation!=d->generation ||
        !Chassis_GyroContinuous(&d->start_gyro,&c->gyro)) { fail(d,DEPARTURE_SENSOR,now); return; }
    if (!d->port.arm_ready || !d->port.arm_ready(d->port.context,now)) { fail(d,DEPARTURE_ARM,now); return; }
    if (c->data.request_id!=d->request_id || c->data.state==CHASSIS_FAULT ||
        c->data.result==CHASSIS_FAILED || c->data.result==CHASSIS_CANCELLED) {
        fail(d,DEPARTURE_CHASSIS,now); return;
    }
    /* STOP verification inside an active move owns its transient stop_mask. */
    if (!(d->state==DEPARTURE_MOVE && c->data.state==CHASSIS_STOPPING) &&
        !motors_healthy(d,now,d->state>=DEPARTURE_SETTLE)) { fail(d,DEPARTURE_MOTOR,now); return; }
    if (d->state<DEPARTURE_SCAN && (uint32_t)(now-d->started_ms)>=DEPARTURE_NAV_TIMEOUT_MS) {
        fail(d,DEPARTURE_TIMEOUT,now); return;
    }
    if (d->state==DEPARTURE_PREPARE) {
        if (c->data.state!=CHASSIS_READY || c->data.result!=CHASSIS_NONE) return;
        if (Chassis_MoveRelative(c,DEPARTURE_FORWARD_MM,0.0f,DEPARTURE_SPEED_MM_S,now)!=CHASSIS_ACCEPTED) {
            fail(d,DEPARTURE_CHASSIS,now); return;
        }
        d->request_id=c->data.request_id; d->state=DEPARTURE_MOVE; d->phase_ms=now;
    } else if (d->state==DEPARTURE_MOVE) {
        if (c->data.state!=CHASSIS_READY || c->data.result!=CHASSIS_SUCCESS || !motors_healthy(d,now,1)) return;
        d->state=DEPARTURE_SETTLE; d->phase_ms=now;
    } else {
        if (c->data.state!=CHASSIS_READY || c->data.result!=CHASSIS_SUCCESS ||
            c->data.error_mm>c->config.position_window_mm ||
            fabs(c->data.error_deg)>c->config.angle_window_deg) { fail(d,DEPARTURE_CHASSIS,now); return; }
        if (!d->qr->data.uart_ready) { fail(d,DEPARTURE_SCANNER,now); return; }
        if (d->state==DEPARTURE_SETTLE) {
            if ((uint32_t)(now-d->phase_ms)<DEPARTURE_SETTLE_MS) return;
            d->scan_session=d->qr->data.session;
            d->port.new_scan(d->port.context,now);
            if (d->qr->data.session==d->scan_session || d->qr->data.valid) { fail(d,DEPARTURE_SCANNER,now); return; }
            d->scan_session=d->qr->data.session; d->scan_seq=d->qr->data.accepted_seq;
            d->scan_uart_errors=d->qr->data.uart_errors; d->scan_overflows=d->qr->data.overflows;
            d->phase_ms=now; d->state=DEPARTURE_SCAN;
        } else {
            if (d->qr->data.session!=d->scan_session ||
                d->qr->data.uart_errors!=d->scan_uart_errors || d->qr->data.overflows!=d->scan_overflows) {
                fail(d,DEPARTURE_SCANNER,now); return;
            }
            if ((uint32_t)(now-d->phase_ms)>=DEPARTURE_SCAN_TIMEOUT_MS) { fail(d,DEPARTURE_QR_TIMEOUT,now); return; }
            if (!d->qr->data.valid || d->qr->data.accepted_seq==d->scan_seq ||
                (int32_t)(d->qr->data.accepted_ms-d->phase_ms)<0) return;
            if (QrTaskCode_Parse((const uint8_t *)d->qr->data.current.text,QR_TASK_LENGTH,&checked)!=QR_CODE_OK) {
                fail(d,DEPARTURE_SCANNER,now); return;
            }
            d->task=checked; d->task_valid=1u; d->state=DEPARTURE_DONE;
        }
    }
}
const char *Departure_StateName(DepartureState state)
{
    static const char *names[]={"WAIT START","PREPARE WHEELS","FORWARD 1000mm",
        "PARK / SETTLE","READ XM1605","QR TEST DONE","ABORTED","FAULT"};
    return (unsigned)state<8u ? names[state] : "UNKNOWN";
}
const char *Departure_ErrorName(DepartureError error)
{
    static const char *names[]={"NONE","BUSY","ARM NOT STOWED","GYRO / ODOMETRY",
        "WHEEL FEEDBACK","SCANNER / SESSION","CHASSIS","MOVE TIMEOUT","QR TIMEOUT","USER STOP","CONFIG"};
    return (unsigned)error<11u ? names[error] : "UNKNOWN";
}
void Departure_Line(char out[43], unsigned row, const Departure *d)
{
    switch (row) {
    case 0u: snprintf(out,43,"ZONE 2 -> QR / FIRST LEG ONLY"); break;
    case 1u: snprintf(out,43,"%s",Departure_StateName(d->state)); break;
    case 2u: snprintf(out,43,"FWD:1000mm SPEED:60mm/s HOLD HEADING"); break;
    case 3u: snprintf(out,43,"QR:%s",d->task_valid ? d->task.text : "--"); break;
    case 4u: snprintf(out,43,"XM1605:9600 SESSION:%lu",(unsigned long)d->qr->data.session); break;
    case 5u: snprintf(out,43,"ERR:%s",Departure_ErrorName(d->error)); break;
    default: snprintf(out,43,"SCAN AT REST / NO AUTO PICK"); break;
    }
}
