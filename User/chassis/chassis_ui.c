#include "chassis_ui.h"
#include <stdio.h>
#include <string.h>
static int number(double value)
{
    if (!Chassis_Finite(value)) return 0;
    if (value>999999.0) return 999999;
    if (value< -999999.0) return -999999;
    return (int)value;
}
void ChassisUi_Line(char out[43], unsigned row, uint8_t detail, const Chassis *c, uint32_t now)
{
    const ChassisSnapshot *d=&c->data;
    uint32_t issues=Chassis_ConfigIssues(&c->config,1u);
    static const char * const results[]={"NONE","RUNNING","SUCCESS","CANCELLED","FAILED"};
    if (row==0u) snprintf(out,43,"CHASSIS %s [tap]",detail ? "WHEELS" : "BODY");
    else if (row==1u) snprintf(out,43,"%s %s",Chassis_StateName(d->state),d->done ? "DONE" : results[d->result]);
    else if (row==2u) snprintf(out,43,"%u/8 %s",(unsigned)d->stage,Chassis_StageName(d->stage));
    else if (detail && row>=3u && row<=6u) {
        unsigned i=row-3u;
        const MotorNode *n=&c->bus->nodes[i];
        if (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now))
            snprintf(out,43,"%s T:%ld RPM:%ld STOP:%s",MotorBus_Role((uint8_t)(i+1u)),
                (long)d->wheels.quantized[i],(long)n->data.speed_rpm,n->stop_unconfirmed ? "?" : "OK");
        else snprintf(out,43,"%s T:%ld RPM:-- STOP:%s",MotorBus_Role((uint8_t)(i+1u)),
                (long)d->wheels.quantized[i],n->stop_unconfirmed ? "?" : "OK");
    } else if (issues && row==3u) snprintf(out,43,"CONFIRM MECH:%u FEEDBACK:%u",
        c->config.mechanical_confirmed,c->config.feedback_confirmed);
    else if (issues && row==4u) snprintf(out,43,"D:%s L:%s W:%s GEAR:%s",
        c->config.wheel_diameter_mm>0 ? "SET" : "--",c->config.length_mm>0 ? "SET" : "--",
        c->config.width_mm>0 ? "SET" : "--",c->config.gear_ratio>0 ? "SET" : "--");
    else if (issues && row==5u) snprintf(out,43,"SIGNS FL/FR/RL/RR:%d/%d/%d/%d",
        c->config.motor_sign[0],c->config.motor_sign[1],c->config.motor_sign[2],c->config.motor_sign[3]);
    else if (issues && row==6u) snprintf(out,43,"FLOW X:%s Y:%s OFFSET:VERIFY",
        c->config.flow_x_mm_count>0 ? "SET" : "--",c->config.flow_y_mm_count>0 ? "SET" : "--");
    else if (row==3u) snprintf(out,43,"TARGET X:%d Y:%d A:%d",number(d->target_x_mm),number(d->target_y_mm),number(d->target_angle_deg));
    else if (row==4u) {
        if (d->pose_valid) snprintf(out,43,"NOW X:%d Y:%d A:%d",number(d->x_mm),number(d->y_mm),number(d->angle_deg));
        else if (c->have_origin && d->gyro_valid && Chassis_GyroContinuous(&c->action_gyro,&c->gyro))
            snprintf(out,43,"NOW X:-- Y:-- A:%d",number(d->angle_deg));
        else strcpy(out,"NOW X:-- Y:-- A:--");
    } else if (row==5u) snprintf(out,43,"V mm/s:%d,%d W d/s:%d",(int)d->output.vx,(int)d->output.vy,(int)d->output.omega);
    else if (row==6u) snprintf(out,43,"FLOW:%s GYRO:%s STOP:%s",d->flow_valid ? "OK" : "--",
        d->gyro_valid ? "OK" : "--",d->stop_pending ? "VERIFY" : "OK");
    else if (row==7u && d->reason==CHASSIS_FLOW_INVALID && d->fault.valid)
        snprintf(out,43,"AT Q:%u BAD:%lu G:%lu PAIR:%lu",d->fault.quality,
            (unsigned long)d->fault.rejected,(unsigned long)d->fault.gaps,(unsigned long)d->fault.paired_age_ms);
    else if (row==7u && issues) snprintf(out,43,"CFG:%s%s%s",issues & 3u ? "MECH " : "",
        issues & 4u ? "SIGN " : "",issues & 56u ? "FLOW/CTRL" : "");
    else if (row==7u) snprintf(out,43,"TX:%lu REJ:%lu AGE:%lu",(unsigned long)d->submitted_frames,
        (unsigned long)d->publish_rejected,(unsigned long)d->feedback_age_ms);
    else if (row==8u && d->reason==CHASSIS_FLOW_INVALID && d->fault.valid)
        snprintf(out,43,"FLOW INVALID: %s",Chassis_FlowFaultName(d->fault.issue,d->fault.flow_error));
    else snprintf(out,43,"%s",Chassis_ReasonName(d->reason!=CHASSIS_OK ? d->reason :
        d->rejection!=CHASSIS_OK ? d->rejection : issues ? CHASSIS_CONFIG : CHASSIS_OK));
}
