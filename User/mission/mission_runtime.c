#include "mission_runtime.h"
#include <math.h>
#include <string.h>
#define RAD 0.01745329251994329577
static double wrap(double a)
{ a=fmod(a,360.0); if (a>180.0) a-=360.0; if (a< -180.0) a+=360.0; return a; }
static int wheels_rest(const MissionRuntime *r, uint32_t now)
{
    unsigned i;
    if (r->motor->queue_count || r->motor->wheels_pending || r->motor->stop_mask ||
        r->motor->sync_state==MOTOR_SYNC_PREPARE || r->motor->sync_state==MOTOR_SYNC_TRIGGER ||
        r->motor->sync_state==MOTOR_SYNC_VERIFY) return 0;
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&r->motor->nodes[i];
        if (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
            !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || n->stop_unconfirmed ||
            n->motion || n->data.speed_rpm || (n->data.status_flags & 0x0Cu) ||
            n->action==MOTOR_UNKNOWN || n->action==MOTOR_STOP_PENDING) return 0;
    }
    return 1;
}
static int vision_ready(const MissionRuntime *r, uint32_t now)
{
    const VisionService *v=r->vision;
    return v->transport.send && v->online && v->heartbeat_seen &&
        (uint32_t)(now-v->last_heartbeat_ms)<VISION_LINK_MS && !v->waiting &&
        (v->state==VISION_IDLE || v->state==VISION_STOPPED);
}
static uint32_t blocked(void *context, uint32_t now)
{
    MissionRuntime *r=(MissionRuntime *)context;
    uint32_t mask=0u;
    if (Chassis_ConfigIssues(&r->chassis->config,1u)) mask|=MISSION_BLOCK_CHASSIS;
    if (!Mission_ConfigValid(&r->config) || r->config.speed_mm_s>r->chassis->config.max_speed_mm_s ||
        r->config.turn_deg_s>r->chassis->config.max_rate_deg_s) mask|=MISSION_BLOCK_ROUTES;
    if (!r->hardware.ready || !r->hardware.begin || !r->hardware.poll || !r->hardware.stop ||
        !r->hardware.ready(r->hardware.context,now)) mask|=MISSION_BLOCK_MECHANISM;
    if (!r->hardware.path_clear) mask|=MISSION_BLOCK_ROUTES;
    if (!vision_ready(r,now)) mask|=MISSION_BLOCK_VISION;
    if (!wheels_rest(r,now) || !Chassis_GyroValid(&r->chassis->gyro,now) ||
        !r->chassis->odometry.valid || (uint32_t)(now-r->chassis->odometry.flow_guard.sample_ms)>100u)
        mask|=MISSION_BLOCK_FEEDBACK;
    if (r->motor->manual || Chassis_OwnsWheels(r->chassis) || r->chassis->data.state==CHASSIS_FAULT)
        mask|=MISSION_BLOCK_BUSY;
    return mask;
}
static int route_for(const MissionAction *a, MissionRouteId *route)
{
    switch (a->step) {
    case MISSION_NAV_QR: *route=MISSION_ROUTE_HOME_QR; return 1;
    case MISSION_NAV_SOURCE: *route=a->batch ? MISSION_ROUTE_STORAGE_SOURCE : MISSION_ROUTE_QR_SOURCE; return 1;
    case MISSION_NAV_ROUGH: *route=MISSION_ROUTE_SOURCE_ROUGH; return 1;
    case MISSION_NAV_STORAGE: *route=MISSION_ROUTE_ROUGH_STORAGE; return 1;
    case MISSION_NAV_HOME: *route=MISSION_ROUTE_STORAGE_HOME; return 1;
    default: return 0;
    }
}
static int begin(void *context, const MissionAction *a, uint32_t now)
{
    MissionRuntime *r=(MissionRuntime *)context;
    r->action=*a;
    if (route_for(a,&r->route)) {
        if (!Mission_ConfigValid(&r->config)) return 0;
        if (a->step==MISSION_NAV_QR) {
            if (blocked(r,now)) return 0;
            r->origin_x=r->chassis->odometry.x; r->origin_y=r->chassis->odometry.y;
            r->origin_angle=r->chassis->gyro.angle_deg;
            r->origin_generation=r->chassis->odometry.generation; r->have_origin=1u;
        }
        if (!r->have_origin || !wheels_rest(r,now)) return 0;
        r->waypoint=r->nav_phase=0u; r->chassis_id=0u;
        if (r->chassis->data.state!=CHASSIS_READY) {
            if (Chassis_Prepare(r->chassis,now)!=CHASSIS_ACCEPTED) return 0;
            r->chassis_id=r->chassis->data.request_id; r->nav_phase=1u;
        }
        return 1;
    }
    if (!wheels_rest(r,now) || Chassis_OwnsWheels(r->chassis)) return 0;
    if (a->step==MISSION_READ_QR) {
        r->nav_phase=0u;
        if (!VisionService_Start(r->vision,0u,1u,1u,1u,(uint16_t)(r->vision->pose_id+1u),now)) return 0;
        r->vision_id=r->vision->request_id; return 1;
    }
    return r->hardware.begin && r->hardware.begin(r->hardware.context,a,now);
}
static MissionActionResult navigation(MissionRuntime *r, uint32_t now)
{
    Chassis *c=r->chassis;
    const MissionRoute *route=&r->config.routes[r->route];
    const MissionWaypoint *p;
    double angle, target_x,target_y,dx,dy,body_x,body_y,turn;
    if (!r->have_origin || !c->odometry.valid || c->odometry.generation!=r->origin_generation ||
        !Chassis_GyroValid(&c->gyro,now) || (uint32_t)(now-c->odometry.flow_guard.sample_ms)>100u ||
        c->data.state==CHASSIS_FAULT) return MISSION_ACTION_FAILED;
    /* A clear observation must be maintained while executing a segment. */
    if (r->waypoint<route->count && (!r->hardware.path_clear ||
        !r->hardware.path_clear(r->hardware.context,r->route,r->waypoint,now))) return MISSION_ACTION_FAILED;
    if (r->nav_phase==1u || r->nav_phase==2u || r->nav_phase==4u) {
        if (c->data.request_id!=r->chassis_id || c->data.result==CHASSIS_FAILED ||
            c->data.result==CHASSIS_CANCELLED) return MISSION_ACTION_FAILED;
        if (c->data.state!=CHASSIS_READY ||
            (r->nav_phase==1u ? c->data.result!=CHASSIS_NONE : c->data.result!=CHASSIS_SUCCESS) ||
            !wheels_rest(r,now)) return MISSION_WAIT;
        if (r->nav_phase==4u) { ++r->waypoint; r->nav_phase=0u; }
        else r->nav_phase=r->nav_phase==2u ? 3u : 0u;
        return MISSION_WAIT;
    }
    if (r->waypoint>=route->count) return wheels_rest(r,now) ? MISSION_SUCCEEDED : MISSION_WAIT;
    p=&route->points[r->waypoint]; angle=r->origin_angle*RAD;
    target_x=r->origin_x+cos(angle)*p->x_mm-sin(angle)*p->y_mm;
    target_y=r->origin_y+sin(angle)*p->x_mm+cos(angle)*p->y_mm;
    dx=target_x-c->odometry.x; dy=target_y-c->odometry.y;
    angle=c->gyro.angle_deg*RAD;
    body_x=cos(angle)*dx+sin(angle)*dy; body_y= -sin(angle)*dx+cos(angle)*dy;
    if (r->nav_phase==0u) {
        if (sqrt(dx*dx+dy*dy)<=c->config.position_window_mm) { r->nav_phase=3u; return MISSION_WAIT; }
        if (Chassis_MoveRelative(c,(float)body_x,(float)body_y,r->config.speed_mm_s,now)!=CHASSIS_ACCEPTED)
            return MISSION_ACTION_FAILED;
        r->chassis_id=c->data.request_id; r->nav_phase=2u; return MISSION_WAIT;
    }
    /* A translation is verified at rest by Chassis before any turn is issued. */
    turn=wrap(r->origin_angle+p->heading_deg-c->gyro.angle_deg);
    if (fabs(turn)<=c->config.angle_window_deg) { ++r->waypoint; r->nav_phase=0u; return MISSION_WAIT; }
    if (Chassis_RotateRelative(c,(float)turn,r->config.turn_deg_s,now)!=CHASSIS_ACCEPTED)
        return MISSION_ACTION_FAILED;
    r->chassis_id=c->data.request_id; r->nav_phase=4u; return MISSION_WAIT;
}
static MissionActionResult poll(void *context, uint32_t id, QrTaskCode *task, uint32_t now)
{
    MissionRuntime *r=(MissionRuntime *)context;
    MissionRouteId route;
    MissionActionResult result;
    if (id!=r->action.id) return MISSION_ACTION_FAILED;
    if (route_for(&r->action,&route)) return navigation(r,now);
    if (r->action.step==MISSION_READ_QR) {
        VisionService *v=r->vision;
        if (v->request_id!=r->vision_id || v->state==VISION_FAULT) return MISSION_ACTION_FAILED;
        if (r->nav_phase) {
            if (v->state!=VISION_STOPPED) return MISSION_WAIT;
            *task=r->qr_result; return MISSION_SUCCEEDED;
        }
        if (!v->task_valid || v->state!=VISION_ACTIVE) return MISSION_WAIT;
        r->qr_result=v->task; r->nav_phase=1u; VisionService_Stop(v,now); return MISSION_WAIT;
    }
    if (r->chassis->data.state==CHASSIS_FAULT || !r->hardware.poll) return MISSION_ACTION_FAILED;
    result=r->hardware.poll(r->hardware.context,id,now);
    if (result==MISSION_SUCCEEDED && (Chassis_OwnsWheels(r->chassis) || !wheels_rest(r,now)))
        return MISSION_ACTION_FAILED;
    return result;
}
static void stop(void *context, uint32_t now)
{
    MissionRuntime *r=(MissionRuntime *)context;
    if (r->hardware.stop) r->hardware.stop(r->hardware.context,now);
    Chassis_Stop(r->chassis,now); MotorBus_Stop(r->motor,MOTOR_ALL,now);
    VisionService_Stop(r->vision,now);
}
void MissionRuntime_Init(MissionRuntime *r, Chassis *c, MotorBus *m,
    VisionService *v, const MissionConfig *config)
{
    memset(r,0,sizeof(*r)); r->chassis=c; r->motor=m; r->vision=v;
    if (config) r->config=*config; else Mission_DefaultConfig(&r->config);
}
MissionPort MissionRuntime_Port(MissionRuntime *r)
{ MissionPort p={r,blocked,begin,poll,stop}; return p; }
void MissionRuntime_SetHardware(MissionRuntime *r, MissionHardware hardware)
{ r->hardware=hardware; }
