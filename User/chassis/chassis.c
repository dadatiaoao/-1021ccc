#include "chassis.h"
#include "./motor/motor_test_config.h"
#include <math.h>
#include <string.h>
#define RAD 0.01745329251994329577
static int due(uint32_t now, uint32_t at) { return (int32_t)(now-at)>=0; }
static float minimum(float a, float b) { return a<b ? a : b; }
static ChassisRequestResult reject(Chassis *c, ChassisRequestResult r, ChassisReason why)
{ c->data.rejection=why; return r; }
int Chassis_OwnsWheels(const Chassis *c)
{
    return c->data.sequence_active || (c->data.state!=CHASSIS_DISARMED &&
        c->data.state!=CHASSIS_READY && c->data.state!=CHASSIS_FAULT);
}
static int stopped(const Chassis *c)
{
    unsigned i;
    for (i=0u;i<4u;++i) if (c->bus->nodes[i].stop_unconfirmed ||
        (c->bus->stop_mask & (1u<<i))) return 0;
    return 1;
}
static ChassisReason health(const Chassis *c, uint32_t now, uint8_t enabled, uint8_t rest)
{
    unsigned i;
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&c->bus->nodes[i];
        if (n->stop_unconfirmed || n->action==MOTOR_STOP_PENDING) return CHASSIS_STOP_UNCONFIRMED;
        if (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
            !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now)) return CHASSIS_FEEDBACK;
        if ((n->data.status_flags & 0x0Cu) || n->action==MOTOR_UNKNOWN) return CHASSIS_MOTOR_FAULT;
        if (enabled && !(n->data.status_flags & 1u)) return CHASSIS_ENABLE_FAILED;
        if (rest && (n->motion || n->data.speed_rpm!=0)) return CHASSIS_AXIS_BUSY;
    }
    return CHASSIS_OK;
}
static int bus_occupied(const Chassis *c)
{
    unsigned i;
    if (c->bus->wheels_pending || c->bus->sync_state==MOTOR_SYNC_PREPARE ||
        c->bus->sync_state==MOTOR_SYNC_TRIGGER || c->bus->sync_state==MOTOR_SYNC_VERIFY) return 1;
    for (i=0u;i<c->bus->queue_count;++i) if (c->bus->queue[i].mask & MOTOR_WHEELS) return 1;
    for (i=0u;i<4u;++i) if (c->bus->nodes[i].action==MOTOR_QUEUED ||
        c->bus->nodes[i].action==MOTOR_SENT || c->bus->nodes[i].action==MOTOR_ACCEPTED) return 1;
    return 0;
}
void Chassis_Init(Chassis *c, MotorBus *bus, const ChassisConfig *config, uint32_t now)
{
    memset(c,0,sizeof(*c)); c->bus=bus;
    if (config) c->config=*config; else Chassis_DefaultConfig(&c->config);
    c->control_ms=c->publish_ms=now;
    c->data.config_issues=Chassis_ConfigIssues(&c->config,1u);
}
static void begin_stop(Chassis *c, ChassisResult result, ChassisReason why, uint32_t now)
{
    /* STOP cannot erase a latched failure or silently re-arm a faulted instance. */
    if (c->data.state==CHASSIS_FAULT ||
        (c->data.state==CHASSIS_STOPPING && c->stop_result==CHASSIS_FAILED)) {
        result=CHASSIS_FAILED; why=c->data.reason;
    }
    if (result==CHASSIS_FAILED && !c->data.fault.valid) {
        ChassisFault *fault=&c->data.fault;
        fault->valid=1u; fault->at_ms=now;
        if (why==CHASSIS_ODOMETRY_INVALID) {
            const ChassisOdometry *o=&c->odometry;
            fault->issue=o->last_issue; fault->sample_age_ms=o->break_age_ms;
            fault->wheel=o->break_wheel;
        }
    }
    c->data.sequence_active=(uint8_t)(result==CHASSIS_SUCCESS && c->data.sequence_active);
    c->between=0u; c->data.done=0u;
    if (c->data.state==CHASSIS_STOPPING) {
        if (result!=CHASSIS_SUCCESS) { c->stop_result=result; c->data.result=result; c->data.reason=why; }
        return;
    }
    c->stop_result=result; c->data.reason=why; c->stop_ms=now; c->verifying=0u;
    c->data.result=result==CHASSIS_SUCCESS ? CHASSIS_RUNNING : result;
    c->data.state=CHASSIS_STOPPING; c->data.stop_pending=1u; c->stable=0u;
    memset(&c->data.output,0,sizeof(c->data.output));
    MotorBus_Stop(c->bus,MOTOR_WHEELS,now);
}
void Chassis_Stop(Chassis *c, uint32_t now)
{ begin_stop(c,CHASSIS_CANCELLED,CHASSIS_OK,now); }
static void request_id(Chassis *c)
{
    if (!++c->data.request_id) ++c->data.request_id;
    memset(&c->data.fault,0,sizeof(c->data.fault));
    c->data.rejection=CHASSIS_OK; c->data.reason=CHASSIS_OK; c->data.done=0u;
}
ChassisRequestResult Chassis_Prepare(Chassis *c, uint32_t now)
{
    if (Chassis_OwnsWheels(c)) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    if (!stopped(c)) return reject(c,CHASSIS_NOT_READY,CHASSIS_STOP_UNCONFIRMED);
    c->data.config_issues=Chassis_ConfigIssues(&c->config,0u);
    if (c->data.config_issues) return reject(c,CHASSIS_INVALID,CHASSIS_CONFIG);
    if (bus_occupied(c)) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    request_id(c); c->data.state=CHASSIS_PREPARING; c->data.result=CHASSIS_RUNNING;
    c->deadline=now+10000u; c->enable_index=c->enable_wait=0u; c->finite=0u;
    return CHASSIS_ACCEPTED;
}
static ChassisRequestResult motion_ready(Chassis *c, uint32_t now, uint8_t closed)
{
    ChassisReason why;
    if (c->data.sequence_active || (c->data.state!=CHASSIS_READY &&
        c->data.state!=CHASSIS_VELOCITY)) return reject(c,
            Chassis_OwnsWheels(c) ? CHASSIS_BUSY : CHASSIS_NOT_READY,CHASSIS_AXIS_BUSY);
    c->data.config_issues=Chassis_ConfigIssues(&c->config,closed);
    if (c->data.config_issues) return reject(c,CHASSIS_INVALID,CHASSIS_CONFIG);
    why=health(c,now,1u,(uint8_t)(c->data.state==CHASSIS_READY));
    if (why!=CHASSIS_OK) return reject(c,CHASSIS_NOT_READY,why);
    if (c->data.state==CHASSIS_READY && bus_occupied(c)) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    return CHASSIS_ACCEPTED;
}
ChassisRequestResult Chassis_SetBodyVelocity(Chassis *c, float vx, float vy, float omega, uint32_t now)
{
    ChassisRequestResult r;
    if (!Chassis_Finite(vx) || !Chassis_Finite(vy) || !Chassis_Finite(omega))
        return reject(c,CHASSIS_INVALID,CHASSIS_BAD_REQUEST);
    if (c->data.state==CHASSIS_VELOCITY && due(now,c->input_deadline)) {
        begin_stop(c,CHASSIS_FAILED,CHASSIS_INPUT_EXPIRED,now);
        return reject(c,CHASSIS_NOT_READY,CHASSIS_INPUT_EXPIRED);
    }
    r=motion_ready(c,now,0u); if (r!=CHASSIS_ACCEPTED) return r;
    request_id(c); c->finite=0u;
    if (vx==0.0f && vy==0.0f && omega==0.0f) {
        begin_stop(c,CHASSIS_NONE,CHASSIS_OK,now); return CHASSIS_ACCEPTED;
    }
    if (c->data.state!=CHASSIS_VELOCITY) {
        memset(&c->data.output,0,sizeof(c->data.output));
        c->control_ms=now; c->publish_ms=now-c->config.publish_ms;
    }
    c->data.state=CHASSIS_VELOCITY; c->data.result=CHASSIS_RUNNING;
    c->data.requested.vx=vx; c->data.requested.vy=vy; c->data.requested.omega=omega;
    c->input_deadline=now+c->config.input_lease_ms; return CHASSIS_ACCEPTED;
}
static void pose(Chassis *c)
{
    double a=c->origin_angle*RAD;
    double dx=c->odometry.x-c->origin_x, dy=c->odometry.y-c->origin_y;
    c->data.x_mm=cos(a)*dx+sin(a)*dy; c->data.y_mm= -sin(a)*dx+cos(a)*dy;
    c->data.angle_deg=c->gyro.angle_deg-c->origin_angle;
    dx=c->data.target_x_mm-c->data.x_mm; dy=c->data.target_y_mm-c->data.y_mm;
    c->data.error_mm=(float)sqrt(dx*dx+dy*dy);
    c->data.error_deg=(float)(c->data.target_angle_deg-c->data.angle_deg);
}
static ChassisRequestResult begin_action(Chassis *c, float x, float y, float angle,
    float limit, uint8_t kind, uint32_t now)
{
    double seconds, distance=sqrt((double)x*x+(double)y*y);
    if (!Chassis_GyroValid(&c->gyro,now)) return reject(c,CHASSIS_NOT_READY,CHASSIS_GYRO_INVALID);
    if (kind==1u && (!c->odometry.valid || now-c->odometry.sample_ms>MOTOR_BUS_FRESH_MS))
        return reject(c,CHASSIS_NOT_READY,CHASSIS_ODOMETRY_INVALID);
    request_id(c); c->finite=kind; c->have_origin=1u; c->action_limit=limit; c->action_gyro=c->gyro;
    c->origin_angle=c->gyro.angle_deg; c->origin_x=c->odometry.x; c->origin_y=c->odometry.y;
    c->odometry_generation=c->odometry.generation;
    c->data.target_x_mm=x; c->data.target_y_mm=y; c->data.target_angle_deg=angle;
    c->data.state=kind==1u ? CHASSIS_MOVING : CHASSIS_ROTATING; c->data.result=CHASSIS_RUNNING;
    seconds=5.0+3.0*(kind==1u ? distance : fabs(angle))/limit;
    if (seconds>60.0) seconds=60.0;
    c->deadline=now+(uint32_t)(seconds*1000.0);
    c->control_ms=now; c->publish_ms=now-c->config.publish_ms; c->stable=0u;
    c->stable_odometry_seq=c->odometry.sequence; c->stable_gyro_seq=c->gyro.sequence;
    memset(&c->data.output,0,sizeof(c->data.output)); pose(c);
    return CHASSIS_ACCEPTED;
}
ChassisRequestResult Chassis_MoveRelative(Chassis *c, float dx, float dy, float limit, uint32_t now)
{
    ChassisRequestResult r;
    if (!Chassis_Finite(dx) || !Chassis_Finite(dy) || fabs(dx)>10000.0 || fabs(dy)>10000.0 ||
        !Chassis_Finite(limit) || limit<=0.0f || limit>c->config.max_speed_mm_s)
        return reject(c,CHASSIS_INVALID,CHASSIS_BAD_REQUEST);
    if (c->data.state==CHASSIS_VELOCITY) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    r=motion_ready(c,now,1u); if (r!=CHASSIS_ACCEPTED) return r;
    return begin_action(c,dx,dy,0.0f,limit,1u,now);
}
ChassisRequestResult Chassis_RotateRelative(Chassis *c, float angle, float limit, uint32_t now)
{
    ChassisRequestResult r;
    if (!Chassis_Finite(angle) || fabs(angle)>360.0 || !Chassis_Finite(limit) ||
        limit<=0.0f || limit>c->config.max_rate_deg_s) return reject(c,CHASSIS_INVALID,CHASSIS_BAD_REQUEST);
    if (c->data.state==CHASSIS_VELOCITY) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    r=motion_ready(c,now,1u); if (r!=CHASSIS_ACCEPTED) return r;
    return begin_action(c,0.0f,0.0f,angle,limit,2u,now);
}
ChassisRequestResult Chassis_StartAcceptance(Chassis *c, uint32_t now)
{
    ChassisRequestResult r;
    if (Chassis_OwnsWheels(c)) return reject(c,CHASSIS_BUSY,CHASSIS_AXIS_BUSY);
    c->data.config_issues=Chassis_ConfigIssues(&c->config,1u);
    if (c->data.config_issues) return reject(c,CHASSIS_INVALID,CHASSIS_CONFIG);
    if (!Chassis_GyroValid(&c->gyro,now)) return reject(c,CHASSIS_NOT_READY,CHASSIS_GYRO_INVALID);
    if (!c->odometry.valid || now-c->odometry.sample_ms>MOTOR_BUS_FRESH_MS)
        return reject(c,CHASSIS_NOT_READY,CHASSIS_ODOMETRY_INVALID);
    r=Chassis_Prepare(c,now);
    if (r==CHASSIS_ACCEPTED) {
        c->data.sequence_active=1u; c->data.stage=0u;
        c->action_gyro=c->gyro; c->odometry_generation=c->odometry.generation;
    }
    return r;
}
static void prepare_process(Chassis *c, uint32_t now)
{
    ChassisReason why=health(c,now,0u,1u);
    if (due(now,c->deadline)) { begin_stop(c,CHASSIS_FAILED,CHASSIS_PREPARE_TIMEOUT,now); return; }
    if (why==CHASSIS_FEEDBACK) return;
    if (why!=CHASSIS_OK) { begin_stop(c,CHASSIS_FAILED,why,now); return; }
#if MOTOR_DRIVER_REQUIRE_MANUAL
    MotorBus_SetManual(c->bus,1u,now);
#endif
    if (c->enable_wait) {
        MotorNode *n=&c->bus->nodes[c->enable_index];
        if (n->action==MOTOR_REJECTED || n->action==MOTOR_UNKNOWN || n->action==MOTOR_CANCELLED) {
            begin_stop(c,CHASSIS_FAILED,CHASSIS_ENABLE_FAILED,now); return;
        }
        if (n->action!=MOTOR_COMPLETE) return;
        c->enable_wait=0u; ++c->enable_index;
    }
    while (c->enable_index<4u && (c->bus->nodes[c->enable_index].data.status_flags & 1u)) ++c->enable_index;
    if (c->enable_index<4u) {
        if (MotorBus_Enable(c->bus,(uint8_t)(c->enable_index+1u),1u,now)) c->enable_wait=1u;
        return;
    }
    c->data.state=CHASSIS_READY; c->data.result=CHASSIS_NONE;
}
static int in_window(const Chassis *c)
{
    return (c->finite!=1u || c->data.error_mm<=c->config.position_window_mm) &&
        fabs(c->data.error_deg)<=c->config.angle_window_deg &&
        fabs(c->gyro.rate_dps)<=c->config.rest_rate_deg_s;
}
static void stop_process(Chassis *c, uint32_t now)
{
    if (!stopped(c)) {
        if (now-c->stop_ms>=MOTOR_STOP_TIMEOUT_MS) {
            c->stop_result=CHASSIS_FAILED;
            c->data.result=CHASSIS_FAILED;
            if (c->data.reason==CHASSIS_OK) c->data.reason=CHASSIS_STOP_TIMEOUT;
            c->data.sequence_active=0u;
        }
        return;
    }
    if (c->stop_result==CHASSIS_SUCCESS) {
        if (!c->verifying) { c->verify_ms=now; c->verifying=1u; return; }
        if (!due(c->gyro.sample_ms,c->verify_ms+1u) ||
            (c->finite==1u && !due(c->odometry.sample_ms,c->verify_ms+1u))) return;
        if (!in_window(c)) { c->stop_result=CHASSIS_FAILED; c->data.reason=CHASSIS_FINAL_ERROR; }
    }
    c->data.stop_pending=0u; c->data.result=c->stop_result;
    c->data.state=c->stop_result==CHASSIS_FAILED ? CHASSIS_FAULT : CHASSIS_READY;
    c->finite=0u;
    if (c->data.result==CHASSIS_SUCCESS && c->data.sequence_active) {
        if (c->data.stage==8u) { c->data.done=1u; c->data.sequence_active=0u; }
        else { c->between=1u; c->between_due=now+c->config.between_ms; }
    } else c->data.sequence_active=0u;
}
static ChassisVelocity closed_velocity(const Chassis *c)
{
    ChassisVelocity v={0.0f,0.0f,0.0f};
    float rate_limit=c->config.max_rate_deg_s;
    if (c->finite==1u) {
        double ex=c->data.target_x_mm-c->data.x_mm, ey=c->data.target_y_mm-c->data.y_mm;
        double length=c->data.error_mm, angle=c->data.angle_deg*RAD;
        double speed=minimum(c->action_limit,(float)sqrt(2.0*(double)c->config.acceleration_mm_s2*length));
        if (length>0.0) {
            double gain=minimum(c->config.position_kp,(float)(speed/length));
            v.vx=(float)(gain*(cos(angle)*ex+sin(angle)*ey));
            v.vy=(float)(gain*(-sin(angle)*ex+cos(angle)*ey));
        }
    } else rate_limit=minimum(c->action_limit,(float)sqrt(2.0*(double)c->config.acceleration_deg_s2*fabs(c->data.error_deg)));
    v.omega=c->config.heading_kp*c->data.error_deg-c->config.rate_damping*c->gyro.rate_dps;
    if (v.omega>rate_limit) v.omega=rate_limit;
    if (v.omega< -rate_limit) v.omega= -rate_limit;
    return v;
}
static void slew(Chassis *c, ChassisVelocity target, float dt)
{
    double dx=target.vx-c->data.output.vx, dy=target.vy-c->data.output.vy;
    double length=sqrt(dx*dx+dy*dy), step=c->config.acceleration_mm_s2*dt;
    float da=target.omega-c->data.output.omega, angular=c->config.acceleration_deg_s2*dt;
    if (length>step) { dx*=step/length; dy*=step/length; }
    c->data.output.vx+=(float)dx; c->data.output.vy+=(float)dy;
    if (da>angular) da=angular; if (da< -angular) da= -angular;
    c->data.output.omega+=da;
}
static void sequence_process(Chassis *c, uint32_t now)
{
    static const float x[6]={300.0f,-300.0f,0.0f,0.0f,200.0f,-200.0f};
    static const float y[6]={0.0f,0.0f,300.0f,-300.0f,200.0f,-200.0f};
    ChassisRequestResult r;
    uint8_t stage;
    if (!c->data.sequence_active || c->data.state!=CHASSIS_READY ||
        (c->between && !due(now,c->between_due))) return;
    c->between=0u; stage=++c->data.stage;
    r=stage<=6u ? begin_action(c,x[stage-1u],y[stage-1u],0.0f,c->config.max_speed_mm_s,1u,now) :
        begin_action(c,0.0f,0.0f,stage==7u ? 90.0f : -90.0f,c->config.max_rate_deg_s,2u,now);
    if (r!=CHASSIS_ACCEPTED) begin_stop(c,CHASSIS_FAILED,c->data.rejection,now);
}
void Chassis_Process(Chassis *c, const GyroSnapshot *g, uint32_t now)
{
    unsigned i; ChassisReason why; uint32_t lease;
    c->gyro=*g;
    Chassis_OdometryProcess(&c->odometry,&c->config,c->bus,g,now);
    c->data.gyro_valid=(uint8_t)Chassis_GyroValid(g,now);
    c->data.odometry_valid=c->odometry.valid;
    if (c->have_origin) pose(c);
    c->data.pose_valid=(uint8_t)(c->have_origin && c->data.gyro_valid && c->odometry.valid &&
        c->odometry.generation==c->odometry_generation && Chassis_GyroContinuous(&c->action_gyro,g));
    c->data.feedback_age_ms=0u;
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&c->bus->nodes[i];
        uint32_t age=now-n->data.speed_ms, status_age=now-n->data.status_ms;
        c->data.measured_rpm[i]=n->data.speed_rpm;
        if (!(n->data.valid & MOTOR_VALID_SPEED) || !(n->data.valid & MOTOR_VALID_STATUS)) age=UINT32_MAX;
        if (status_age>age) age=status_age;
        if (age>c->data.feedback_age_ms) c->data.feedback_age_ms=age;
    }
    c->data.submitted_frames=c->bus->wheel_frames; c->data.last_submit_ms=c->bus->last_wheel_ms;
    if (c->observed_frames!=c->bus->wheel_frames) {
        if (c->last_submitted && c->bus->last_wheel_ms-c->last_submitted>c->data.max_submit_gap_ms)
            c->data.max_submit_gap_ms=c->bus->last_wheel_ms-c->last_submitted;
        c->last_submitted=c->bus->last_wheel_ms;
        c->observed_frames=c->bus->wheel_frames;
    }
    if (c->data.state==CHASSIS_PREPARING) prepare_process(c,now);
    if (c->data.sequence_active && !c->finite) {
        if (!c->data.gyro_valid) begin_stop(c,CHASSIS_FAILED,CHASSIS_GYRO_INVALID,now);
        else if (!Chassis_GyroContinuous(&c->action_gyro,g)) begin_stop(c,CHASSIS_FAILED,CHASSIS_SENSOR_CHANGED,now);
        else if (c->data.stage<6u && (!c->odometry.valid || c->odometry.generation!=c->odometry_generation))
            begin_stop(c,CHASSIS_FAILED,CHASSIS_ODOMETRY_INVALID,now);
    }
    if (c->finite) {
        if (!c->data.gyro_valid) begin_stop(c,CHASSIS_FAILED,CHASSIS_GYRO_INVALID,now);
        else if (!Chassis_GyroContinuous(&c->action_gyro,g)) begin_stop(c,CHASSIS_FAILED,CHASSIS_SENSOR_CHANGED,now);
        else if (c->finite==1u && (!c->odometry.valid || c->odometry.generation!=c->odometry_generation))
            begin_stop(c,CHASSIS_FAILED,CHASSIS_ODOMETRY_INVALID,now);
    }
    if (c->data.state==CHASSIS_STOPPING) { stop_process(c,now); return; }
    if (c->data.state==CHASSIS_VELOCITY && due(now,c->input_deadline)) {
        begin_stop(c,CHASSIS_FAILED,CHASSIS_INPUT_EXPIRED,now); return;
    }
    if (c->data.sequence_active || c->data.state==CHASSIS_VELOCITY || c->finite) {
        why=health(c,now,1u,(uint8_t)(c->data.state==CHASSIS_READY));
        /* Enable confirmation is asynchronous while preparing. */
        if (c->data.state!=CHASSIS_PREPARING && why!=CHASSIS_OK) {
            begin_stop(c,CHASSIS_FAILED,why,now); return;
        }
    }
    sequence_process(c,now);
    if (c->finite) {
        if (due(now,c->deadline)) { begin_stop(c,CHASSIS_FAILED,CHASSIS_ACTION_TIMEOUT,now); return; }
        if (!in_window(c)) c->stable=0u;
        else if (g->sequence!=c->stable_gyro_seq &&
            (c->finite!=1u || c->odometry.sequence!=c->stable_odometry_seq)) {
            c->stable_gyro_seq=g->sequence; c->stable_odometry_seq=c->odometry.sequence;
            if (!c->stable) { c->stable=1u; c->stable_since=now; }
            else if (now-c->stable_since>=c->config.stable_ms) {
                begin_stop(c,CHASSIS_SUCCESS,CHASSIS_OK,now); return;
            }
        }
    }
    if (c->data.state!=CHASSIS_VELOCITY && !c->finite) return;
    if (now-c->control_ms>=c->config.control_ms) {
        ChassisVelocity v=c->finite ? closed_velocity(c) : c->data.requested;
        ChassisWheels limited;
        float dt=(now-c->control_ms)*0.001f; c->control_ms=now;
        if (!Chassis_Solve(&c->config,v,&limited)) { begin_stop(c,CHASSIS_FAILED,CHASSIS_CONFIG,now); return; }
        if (c->finite) c->data.requested=v;
        slew(c,limited.limited,dt);
        if (!Chassis_Solve(&c->config,c->data.output,&c->data.wheels)) {
            begin_stop(c,CHASSIS_FAILED,CHASSIS_CONFIG,now); return;
        }
        c->data.output=c->data.wheels.limited;
    }
    if (now-c->publish_ms<c->config.publish_ms) return;
    lease=c->data.state==CHASSIS_VELOCITY ? c->input_deadline-now : 300u;
    if (lease<100u) return; /* Never extend the original caller deadline. */
    c->publish_ms=now;
    if (MotorBus_Wheels(c->bus,c->data.wheels.quantized,c->config.driver_acceleration,lease,now))
        ++c->data.publish_accepted;
    else { ++c->data.publish_rejected; c->data.rejection=CHASSIS_BUS_REJECTED; }
}
void Chassis_GetSnapshot(const Chassis *c, ChassisSnapshot *out) { *out=c->data; }
const char *Chassis_StateName(ChassisState s)
{
    static const char * const names[]={"DISARMED","PREPARING","READY","VELOCITY","MOVING","ROTATING","STOPPING","FAULT"};
    return (unsigned)s<8u ? names[s] : "?";
}
const char *Chassis_ReasonName(ChassisReason r)
{
    static const char * const names[]={"OK","CONFIG REQUIRED","WHEEL FEEDBACK","WHEEL FAULT","STOP UNCONF",
        "CHASSIS BUSY","ENABLE FAILED","PREPARE TIMEOUT","INPUT EXPIRED","GYRO INVALID","ODOM INVALID",
        "SENSOR CHANGED","ACTION TIMEOUT","STOP TIMEOUT","FINAL ERROR","INVALID REQUEST","BUS REJECTED"};
    return (unsigned)r<17u ? names[r] : "?";
}
const char *Chassis_StageName(uint8_t s)
{
    static const char * const names[]={"PREPARE","FORWARD","BACKWARD","LEFT","RIGHT","DIAG FL","DIAG RR","CCW","CW"};
    return s<=8u ? names[s] : "?";
}
