/* Runtime uses the real vision parser/service; chassis calls are recorded fakes.
   Real chassis dynamics/STOP verification remain in test_chassis.c. */
#include "mission/mission_runtime.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,moves,turns,stops;
static float last_x,last_y,last_turn;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"RUNTIME FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
#define NEAR(a,b) CHECK(fabs((double)(a)-(double)(b))<0.001)
#define RAD 0.01745329251994329577
int MotorBus_Fresh(const MotorNode *n,uint8_t field,uint32_t now)
{ (void)now; return (n->data.valid & field)!=0u; }
void MotorBus_Stop(MotorBus *b,uint8_t mask,uint32_t now)
{ (void)b; (void)now; CHECK(mask==MOTOR_ALL); ++stops; }
int Chassis_GyroValid(const GyroSnapshot *g,uint32_t now)
{ return g && g->angle_valid && (uint32_t)(now-g->sample_ms)<=100u; }
int Chassis_OwnsWheels(const Chassis *c)
{ return c->data.state!=CHASSIS_DISARMED && c->data.state!=CHASSIS_READY && c->data.state!=CHASSIS_FAULT; }
void Chassis_Stop(Chassis *c,uint32_t now)
{ (void)now; c->data.state=CHASSIS_STOPPING; }
ChassisRequestResult Chassis_Prepare(Chassis *c,uint32_t now)
{ (void)now; ++c->data.request_id; c->data.state=CHASSIS_PREPARING; c->data.result=CHASSIS_RUNNING; return CHASSIS_ACCEPTED; }
ChassisRequestResult Chassis_MoveRelative(Chassis *c,float x,float y,float speed,uint32_t now)
{
    (void)now; CHECK(speed>0.0f && c->data.state==CHASSIS_READY);
    last_x=x; last_y=y; ++moves; ++c->data.request_id;
    c->data.state=CHASSIS_MOVING; c->data.result=CHASSIS_RUNNING; return CHASSIS_ACCEPTED;
}
ChassisRequestResult Chassis_RotateRelative(Chassis *c,float angle,float rate,uint32_t now)
{
    (void)now; CHECK(rate>0.0f && c->data.state==CHASSIS_READY);
    last_turn=angle; ++turns; ++c->data.request_id;
    c->data.state=CHASSIS_ROTATING; c->data.result=CHASSIS_RUNNING; return CHASSIS_ACCEPTED;
}
typedef struct {
    MissionRuntime runtime; Chassis chassis; MotorBus motor; VisionService vision;
    uint8_t arm_ready,path;
    unsigned arm_stops;
    MissionActionResult arm_result;
} Fixture;
static int ready(void *ctx,uint32_t now) { (void)now; return ((Fixture *)ctx)->arm_ready; }
static int path_clear(void *ctx,MissionRouteId route,uint8_t point,uint32_t now)
{ (void)route; (void)point; (void)now; return ((Fixture *)ctx)->path; }
static int arm_begin(void *ctx,const MissionAction *a,uint32_t now)
{ (void)ctx; (void)a; (void)now; return 1; }
static MissionActionResult arm_poll(void *ctx,uint32_t id,uint32_t now)
{ (void)id; (void)now; return ((Fixture *)ctx)->arm_result; }
static void arm_stop(void *ctx,uint32_t now) { (void)now; ++((Fixture *)ctx)->arm_stops; }
static int send_wire(void *ctx,const uint8_t *bytes,uint8_t n)
{ (void)ctx; CHECK(n>=4u && bytes[0]==0xAAu); return 1; }
static void fixture(Fixture *f)
{
    MissionConfig cfg; MissionHardware hw;
    VisionTransport t={0,send_wire,0}; unsigned i;
    memset(f,0,sizeof(*f)); moves=turns=stops=0u;
    Mission_DefaultConfig(&cfg); CHECK(!Mission_ConfigValid(&cfg));
    cfg.initial_pose_confirmed=1u;
    for (i=0u;i<MISSION_ROUTE_COUNT;++i) cfg.routes[i].confirmed=cfg.routes[i].count=1u;
    cfg.routes[0].count=2u;
    cfg.routes[0].points[0].x_mm=100.0f; cfg.routes[0].points[0].y_mm=50.0f;
    cfg.routes[0].points[0].heading_deg=90.0f;
    cfg.routes[0].points[1].x_mm=200.0f; cfg.routes[0].points[1].y_mm=50.0f;
    CHECK(Mission_ConfigValid(&cfg));
    Chassis_DefaultConfig(&f->chassis.config);
    f->chassis.config.mechanical_confirmed=f->chassis.config.feedback_confirmed=1u;
    f->chassis.config.wheel_diameter_mm=100.0f;
    f->chassis.config.length_mm=f->chassis.config.width_mm=200.0f;
    f->chassis.config.gear_ratio=1.0f;
    f->chassis.config.flow_x_mm_count=f->chassis.config.flow_y_mm_count=1.0f;
    for (i=0u;i<4u;++i) {
        f->chassis.config.motor_sign[i]=1;
        f->motor.nodes[i].data.valid=MOTOR_VALID_SPEED|MOTOR_VALID_STATUS;
        f->motor.nodes[i].data.online=1u; f->motor.nodes[i].data.status_flags=1u;
    }
    f->chassis.odometry.valid=1u; f->chassis.odometry.x=100.0; f->chassis.odometry.y= -200.0;
    f->chassis.gyro.angle_valid=1u; f->chassis.gyro.angle_deg=90.0;
    VisionService_Init(&f->vision,t,0u);
    f->vision.online=f->vision.heartbeat_seen=1u;
    MissionRuntime_Init(&f->runtime,&f->chassis,&f->motor,&f->vision,&cfg);
    f->arm_ready=f->path=1u;
    f->arm_result=MISSION_SUCCEEDED;
    memset(&hw,0,sizeof(hw)); hw.context=f; hw.ready=ready; hw.path_clear=path_clear;
    hw.begin=arm_begin; hw.poll=arm_poll; hw.stop=arm_stop;
    MissionRuntime_SetHardware(&f->runtime,hw);
}
static MissionAction nav_action(void)
{ MissionAction a={0}; a.id=1u; a.step=MISSION_NAV_QR; return a; }
static void complete(Fixture *f)
{
    Chassis *c=&f->chassis;
    if (c->data.state==CHASSIS_MOVING) {
        double a=c->gyro.angle_deg*RAD;
        c->odometry.x+=cos(a)*last_x-sin(a)*last_y;
        c->odometry.y+=sin(a)*last_x+cos(a)*last_y;
    } else if (c->data.state==CHASSIS_ROTATING) c->gyro.angle_deg+=last_turn;
    c->data.result=c->data.state==CHASSIS_PREPARING ? CHASSIS_NONE : CHASSIS_SUCCESS;
    c->data.state=CHASSIS_READY;
}
static void navigation(void)
{
    Fixture f; MissionPort p; MissionAction a=nav_action(); QrTaskCode task;
    unsigned i;
    fixture(&f); p=MissionRuntime_Port(&f.runtime); CHECK(!p.blocked(p.context,0u));
    CHECK(p.begin(p.context,&a,0u));
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && !moves);
    complete(&f);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT); /* prepared, result NONE */
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && moves==1u);
    NEAR(last_x,100.0); NEAR(last_y,50.0);
    for (i=0u;i<3u;++i) CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && !turns);
    complete(&f); NEAR(f.chassis.odometry.x,50.0); NEAR(f.chassis.odometry.y,-100.0);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && turns==1u); NEAR(last_turn,90.0);
    complete(&f); CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && moves==2u);
    NEAR(last_x,0.0); NEAR(last_y,-100.0); /* absolute waypoint, rotated body */
    complete(&f); CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT && turns==2u); NEAR(last_turn,-90.0);
    complete(&f); CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT);
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_SUCCEEDED);
    NEAR(f.chassis.odometry.x,50.0); NEAR(f.chassis.odometry.y,0.0);
    CHECK(p.poll(p.context,999u,&task,0u)==MISSION_ACTION_FAILED);
    puts("PASS: absolute zone-2 waypoints, heading transforms, prepare NONE, wait for verified rest, request IDs");
}
static void faults(void)
{
    Fixture f; MissionPort p; MissionAction a=nav_action(); QrTaskCode task;
    MissionHardware empty={0}; MissionConfig cfg;
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    MissionRuntime_SetHardware(&f.runtime,empty);
    CHECK(p.blocked(p.context,0u)&MISSION_BLOCK_MECHANISM);
    CHECK(p.blocked(p.context,0u)&MISSION_BLOCK_ROUTES);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    f.runtime.config.initial_pose_confirmed=0u;
    CHECK(!p.begin(p.context,&a,0u) && !moves);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    f.chassis.config.mechanical_confirmed=0u; CHECK(p.blocked(p.context,0u)&MISSION_BLOCK_CHASSIS);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    f.motor.manual=1u; CHECK(p.blocked(p.context,0u)&MISSION_BLOCK_BUSY);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    f.vision.online=0u; CHECK(p.blocked(p.context,0u)&MISSION_BLOCK_VISION);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    CHECK(p.begin(p.context,&a,0u)); complete(&f);
    f.path=0u; CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_ACTION_FAILED && !moves);
    p.stop(p.context,0u); CHECK(f.arm_stops==1u && stops==1u && f.vision.state==VISION_STOPPING);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    CHECK(p.begin(p.context,&a,0u)); ++f.chassis.odometry.generation;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_ACTION_FAILED);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    CHECK(p.begin(p.context,&a,0u)); ++f.chassis.data.request_id;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_ACTION_FAILED);
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    CHECK(p.begin(p.context,&a,0u));
    CHECK(p.poll(p.context,a.id,&task,101u)==MISSION_ACTION_FAILED);
    fixture(&f); cfg=f.runtime.config;
    cfg.routes[MISSION_ROUTE_STORAGE_HOME].points[0].x_mm=1.0f; CHECK(!Mission_ConfigValid(&cfg));
    cfg=f.runtime.config; cfg.routes[0].count=9u; CHECK(!Mission_ConfigValid(&cfg));
    puts("PASS: uncalibrated/missing hardware, busy/link guards, blocked paths, changed odometry/request, stale sensors");
}
static void receive(VisionService *v,uint8_t cmd,const uint8_t *data,uint8_t count,uint32_t now)
{
    uint8_t wire[VISION_FRAME_MAX],n=VisionProtocol_Build(cmd,data,count,wire);
    CHECK(n!=0u); VisionService_Receive(v,wire,n,now);
}
static void qr(void)
{
    Fixture f; MissionPort p; MissionAction a={0}; QrTaskCode task;
    const uint8_t code[12]={1,3,6,2,3,1,6,1,3,1,2,3};
    uint8_t ack[2]; uint32_t now=0u;
    fixture(&f); f.chassis.data.state=CHASSIS_READY;
    p=MissionRuntime_Port(&f.runtime); a.id=2u; a.step=MISSION_READ_QR;
    CHECK(p.begin(p.context,&a,now)); CHECK(!f.vision.task_valid);
    while (f.vision.state==VISION_CONFIGURING && now<20u) {
        VisionService_Process(&f.vision,now);
        CHECK(f.vision.waiting); ack[0]=f.vision.pending_cmd; ack[1]=0u;
        receive(&f.vision,VISION_ACK,ack,2u,now++);
    }
    CHECK(f.vision.state==VISION_ACTIVE && now==5u);
    receive(&f.vision,VISION_TASKCODE,code,12u,now);
    CHECK(f.vision.task_valid);
    CHECK(p.poll(p.context,a.id,&task,now)==MISSION_WAIT && f.vision.state==VISION_STOPPING);
    CHECK(p.poll(p.context,a.id,&task,now)==MISSION_WAIT);
    VisionService_Process(&f.vision,now); ack[0]=VISION_STOP; ack[1]=0u;
    receive(&f.vision,VISION_ACK,ack,2u,now);
    CHECK(p.poll(p.context,a.id,&task,now)==MISSION_SUCCEEDED);
    CHECK(!strcmp(task.text,"136+231+613+123") && !moves && !turns);
    CHECK(task.positions[1][0]==1u && task.colors[1][0]==6u);
    puts("PASS: real K230 QR command/ACK sequence, fresh task, STOP ACK before advancing");
}
static void mechanism(void)
{
    Fixture f; MissionPort p; MissionAction a={0}; QrTaskCode task;
    fixture(&f); p=MissionRuntime_Port(&f.runtime);
    a.id=3u; a.step=MISSION_SOURCE_TO_SLOT;
    CHECK(p.begin(p.context,&a,0u));
    f.arm_result=MISSION_WAIT; f.chassis.data.state=CHASSIS_MOVING;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_WAIT); /* alignment owns body */
    f.arm_result=MISSION_SUCCEEDED;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_ACTION_FAILED); /* cannot finish moving */
    f.chassis.data.state=CHASSIS_READY; f.motor.nodes[0].data.speed_rpm=1;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_ACTION_FAILED);
    f.motor.nodes[0].data.speed_rpm=0;
    CHECK(p.poll(p.context,a.id,&task,0u)==MISSION_SUCCEEDED);
    puts("PASS: mechanism may align body while waiting; completion requires verified rest");
}
int main(void)
{ navigation(); faults(); qr(); mechanism(); printf("PASS: %u runtime assertions (recorded chassis calls)\n",checks); return 0; }
