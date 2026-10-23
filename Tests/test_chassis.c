/* Real MotorBus + incremental wire replies + independent wheel/body plant.
   ALL values here are synthetic; none are vehicle calibration measurements. */
#include "chassis/chassis.h"
#include "chassis/chassis_ui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"CHASSIS FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
#define NEAR(a,b,e) CHECK(fabs((double)(a)-(double)(b))<=(e))
#define RAD 0.01745329251994329577
static ChassisConfig config(void)
{
    ChassisConfig c; Chassis_DefaultConfig(&c);
    c.mechanical_confirmed=c.feedback_confirmed=1u;
    c.wheel_diameter_mm=100.0f; c.length_mm=200.0f; c.width_mm=180.0f; c.gear_ratio=1.0f;
    c.motor_sign[0]=c.motor_sign[2]=1; c.motor_sign[1]=c.motor_sign[3]=-1;
    c.flow_x_mm_count=0.2f; c.flow_y_mm_count=0.25f; c.flow_mount_deg=15.0f;
    c.flow_offset_x_mm=45.0f; c.flow_offset_y_mm=-30.0f;
    return c;
}
typedef struct {
    MotorBus bus; Chassis chassis; ChassisConfig cfg;
    FlowSnapshot flow; GyroSnapshot gyro;
    uint32_t now, tx_done;
    uint8_t bytes[128], length, busy, response, online, flags[7];
    uint8_t drop_ack, drop_stop, hold_tx, freeze_flow, freeze_gyro, stuck, reject_enable;
    int32_t command[7]; double wheel[4];
    double x,y,angle,rate, previous_x,previous_y,previous_angle, count_x,count_y;
    double history[64][4];
    unsigned sends[8][256], groups, stop_applied, bad_axis;
} Sim;
static int send_wire(void *ctx, const uint8_t *b, uint8_t length)
{
    Sim *s=ctx;
    CHECK(!s->busy && length<=128u);
    memcpy(s->bytes,b,length); s->length=length; s->busy=1u;
    s->tx_done=s->now+1u+length/11u; ++s->sends[b[0]][b[1]];
    if ((b[1]==EMM_ENABLE || b[1]==EMM_VELOCITY || b[1]==EMM_STOP) && b[0]>4u) ++s->bad_axis;
    return 1;
}
static int wire_busy(void *ctx) { return ((Sim *)ctx)->busy; }
static int32_t rounded(double x) { return (int32_t)(x+(x<0.0 ? -0.5 : 0.5)); }
static int32_t velocity(const uint8_t *c)
{ int32_t r=((int32_t)c[3]<<8)|c[4]; return c[2] ? -r : r; }
static void reply(Sim *s)
{
    uint8_t *b=s->bytes, r[8]={0};
    unsigned a=b[0], i=a-1u, n=4u; uint32_t v; int32_t rpm;
    r[0]=(uint8_t)a; r[1]=b[1]; r[2]=EMM_ACK_OK;
    if (b[1]==EMM_MULTI) {
        ++s->groups;
        for (i=0u;i<4u;++i) if (s->online & (1u<<i)) s->command[i]=velocity(b+4u+i*8u);
        if (s->drop_ack) return;
        a=r[0]=1u; r[1]=EMM_VELOCITY;
    } else if (!(s->online & (1u<<i))) return;
    else if (b[1]==EMM_ENABLE) {
        if (a==s->reject_enable) r[2]=0xE2u;
        else s->flags[i]=(uint8_t)(2u|b[3]);
    } else if (b[1]==EMM_STOP) {
        if (s->drop_stop) { --s->drop_stop; return; }
        s->command[i]=0; ++s->stop_applied;
    } else if (b[1]==EMM_SPEED) {
        rpm=i<4u ? rounded(s->wheel[i]) : 0;
        n=6u; r[2]=(uint8_t)(rpm<0); v=(uint32_t)(rpm<0 ? -rpm : rpm);
        r[3]=(uint8_t)(v>>8); r[4]=(uint8_t)v;
    } else if (b[1]==EMM_STATUS) r[2]=s->flags[i];
    else if (b[1]==EMM_POSITION) { n=8u; r[2]=0u; }
    r[n-1u]=0x6Bu;
    MotorBus_Receive(&s->bus,r,2u,s->now);
    MotorBus_Receive(&s->bus,r+2u,n-2u,s->now);
}
static void plant(Sim *s)
{
    double w[4], vx,vy,omega,a=s->angle*RAD; unsigned i;
    /* Independent forward model: 100mm wheels, half-sum 190mm; 80ms inertia,
       integer wire RPM and a 5ms sensor delay. No target-pose shortcut. */
    for (i=0u;i<4u;++i) {
        s->wheel[i]+=(s->command[i]-s->wheel[i])/80.0;
        w[i]=s->wheel[i]*(i & 1u ? -1.0 : 1.0)*5.235987755982989;
    }
    vx=(w[0]+w[1]+w[2]+w[3])*0.25;
    vy=(-w[0]+w[1]+w[2]-w[3])*0.25;
    omega=(-w[0]+w[1]-w[2]+w[3])/760.0/RAD;
    if (s->stuck) vx=vy=omega=0.0;
    s->x+=(cos(a)*vx-sin(a)*vy)*0.001;
    s->y+=(sin(a)*vx+cos(a)*vy)*0.001; s->angle+=omega*0.001; s->rate=omega;
    s->history[s->now%64u][0]=s->x; s->history[s->now%64u][1]=s->y;
    s->history[s->now%64u][2]=s->angle; s->history[s->now%64u][3]=omega;
}
static void sensors(Sim *s)
{
    const double *h=s->history[(s->now-5u)%64u];
    if (s->now%5u==0u && !s->freeze_gyro) {
        ++s->gyro.sequence; s->gyro.sample_ms=s->now-5u;
        s->gyro.angle_deg=h[2]; s->gyro.rate_dps=(float)h[3];
    }
    if (s->now%10u==0u && !s->freeze_flow) {
        double a=h[2]*RAD, m=(h[2]+s->previous_angle)*0.5*RAD;
        double sx=h[0]+cos(a)*45.0+sin(a)*30.0, sy=h[1]+sin(a)*45.0-cos(a)*30.0;
        double dx=sx-s->previous_x, dy=sy-s->previous_y;
        double bx=cos(m)*dx+sin(m)*dy, by=-sin(m)*dx+cos(m)*dy;
        double mount=15.0*RAD;
        s->count_x+=(cos(mount)*bx+sin(mount)*by)/0.2;
        s->count_y+=(-sin(mount)*bx+cos(mount)*by)/0.25;
        s->flow.total_x=(int64_t)(s->count_x+(s->count_x<0 ? -0.5 : 0.5));
        s->flow.total_y=(int64_t)(s->count_y+(s->count_y<0 ? -0.5 : 0.5));
        ++s->flow.sequence; s->flow.sample_ms=s->now-5u;
        s->previous_x=sx; s->previous_y=sy; s->previous_angle=h[2];
    }
}
static void run(Sim *s, uint32_t duration)
{
    uint32_t t;
    for (t=0u;t<duration;++t,++s->now) {
        plant(s); sensors(s);
        if (s->busy && !s->hold_tx && (int32_t)(s->now-s->tx_done)>=0) {
            s->busy=0u; s->response=1u;
        }
        MotorBus_Process(&s->bus,s->now);
        if (s->response) { s->response=0u; reply(s); }
        Chassis_Process(&s->chassis,&s->flow,&s->gyro,s->now);
    }
}
static void init_at(Sim *s, uint32_t time)
{
    MotorTransport t; unsigned i;
    memset(s,0,sizeof(*s)); s->now=time; s->online=MOTOR_WHEELS; s->cfg=config();
    memset(s->flags,2,sizeof(s->flags));
    t.context=s; t.send=send_wire; t.busy=wire_busy;
    MotorBus_Init(&s->bus,t,time); Chassis_Init(&s->chassis,&s->bus,&s->cfg,time);
    s->gyro.state=GYRO_RUNNING; s->gyro.calibrated=s->gyro.rate_valid=s->gyro.angle_valid=s->gyro.have_sample=1u;
    s->gyro.calibration=GYRO_CAL_DONE;
    s->flow.state=FLOW_READY; s->flow.have_sample=s->flow.comm_ok=s->flow.tracking=s->flow.valid=1u;
    s->flow.segments=1u; s->previous_x=45.0; s->previous_y=-30.0;
    for (i=0u;i<64u;++i) s->history[i][0]=s->history[i][1]=s->history[i][2]=s->history[i][3]=0.0;
    run(s,2500u);
    CHECK(!s->groups && !s->bad_axis && s->chassis.odometry.valid);
}
static void init(Sim *s) { init_at(s,0u); }
static void ready(Sim *s)
{
    uint32_t t;
    CHECK(Chassis_Prepare(&s->chassis,s->now)==CHASSIS_ACCEPTED);
    for (t=0u;t<10000u && s->chassis.data.state==CHASSIS_PREPARING;++t) run(s,1u);
    CHECK(s->chassis.data.state==CHASSIS_READY);
    CHECK(!s->bad_axis);
}
static void finish(Sim *s, uint32_t max)
{
    uint32_t t;
    for (t=0u;t<max && Chassis_OwnsWheels(&s->chassis);++t) run(s,1u);
    if (Chassis_OwnsWheels(&s->chassis)) fprintf(stderr,"state=%s reason=%s stage=%u err=%f/%f\n",
        Chassis_StateName(s->chassis.data.state),Chassis_ReasonName(s->chassis.data.reason),
        s->chassis.data.stage,s->chassis.data.error_mm,s->chassis.data.error_deg);
    CHECK(!Chassis_OwnsWheels(&s->chassis));
}
static void kinematics(void)
{
    ChassisConfig c=config(); ChassisWheels w;
    ChassisVelocity v={50.0f,0.0f,0.0f}; unsigned i;
    const float vectors[10][3]={{50,0,0},{-50,0,0},{0,50,0},{0,-50,0},
        {30,30,0},{30,-30,0},{-30,30,0},{-30,-30,0},{0,0,20},{0,0,-20}};
    const int signs[10][4]={{1,-1,1,-1},{-1,1,-1,1},{-1,-1,1,1},{1,1,-1,-1},
        {0,-1,1,0},{1,0,0,-1},{-1,0,0,1},{0,1,-1,0},{-1,-1,-1,-1},{1,1,1,1}};
    CHECK(!Chassis_ConfigIssues(&c,1u));
    for (i=0u;i<10u;++i) {
        unsigned j; v.vx=vectors[i][0]; v.vy=vectors[i][1]; v.omega=vectors[i][2];
        CHECK(Chassis_Solve(&c,v,&w));
        for (j=0u;j<4u;++j) CHECK((w.quantized[j]>0)-(w.quantized[j]<0)==signs[i][j]);
    }
    v.vx=50; v.vy=0; v.omega=0; CHECK(Chassis_Solve(&c,v,&w)); NEAR(w.rpm[0],9.5493,0.001);
    c.gear_ratio=2.0f; CHECK(Chassis_Solve(&c,v,&w)); NEAR(w.rpm[0],19.0986,0.001);
    c=config(); c.max_rpm=10; v.vx=100; v.vy=60; v.omega=30;
    CHECK(Chassis_Solve(&c,v,&w)); CHECK(w.scale<1.0f);
    for (i=0u;i<4u;++i) CHECK(fabs(w.rpm[i])<=10.00001);
    v.vx=FLT_MAX; v.vy= -FLT_MAX; v.omega=30; CHECK(Chassis_Solve(&c,v,&w));
    v.vx=0.1f; v.vy=0; v.omega=0; CHECK(Chassis_Solve(&c,v,&w)); CHECK(!w.quantized[0]);
    v.vx=-5; CHECK(Chassis_Solve(&c,v,&w)); CHECK(w.quantized[0]==-1 && w.quantized[1]==1);
    v.vx=(float)NAN; CHECK(!Chassis_Solve(&c,v,&w)); v.vx=(float)INFINITY; CHECK(!Chassis_Solve(&c,v,&w));
    c.wheel_diameter_mm=0; v.vx=1; CHECK(!Chassis_Solve(&c,v,&w));
    /* Invalid-config fixture is independent of the user's measured build config. */
    Chassis_DefaultConfig(&c); c.mechanical_confirmed=c.feedback_confirmed=0u;
    c.wheel_diameter_mm=c.length_mm=c.width_mm=c.gear_ratio=0.0f;
    memset(c.motor_sign,0,sizeof(c.motor_sign)); c.flow_x_mm_count=c.flow_y_mm_count=0.0f;
    CHECK(Chassis_ConfigIssues(&c,1u)==55u);
    puts("PASS: X kinematics, units/gearing/signs, proportional limits, quantization, invalid config/NaN/Inf");
}
static void velocity_tests(void)
{
    Sim s; unsigned i, groups; uint32_t original;
    init(&s); ready(&s);
    CHECK(Chassis_SetBodyVelocity(&s.chassis,80,0,0,s.now)==CHASSIS_ACCEPTED);
    original=s.chassis.input_deadline;
    CHECK(Chassis_SetBodyVelocity(&s.chassis,(float)NAN,0,0,s.now)==CHASSIS_INVALID);
    CHECK(s.chassis.input_deadline==original);
    for (i=0u;i<40u;++i) {
        CHECK(Chassis_SetBodyVelocity(&s.chassis,i<20u ? 80.0f : -80.0f,0,0,s.now)==CHASSIS_ACCEPTED);
        run(&s,50u);
    }
    CHECK(s.groups>=15u && s.bus.wheel_frames==s.groups && s.bus.queue_count<=1u);
    CHECK(s.bus.nodes[3].data.speed_seq>3u && s.chassis.data.publish_accepted>=15u);
    CHECK(s.chassis.data.max_submit_gap_ms<300u);
    groups=s.groups; finish(&s,6000u);
    CHECK(s.chassis.data.result==CHASSIS_FAILED && s.chassis.data.reason==CHASSIS_INPUT_EXPIRED);
    CHECK(!s.bus.wheels_pending); run(&s,2000u); CHECK(s.groups<=groups+2u);
    CHECK(Chassis_SetBodyVelocity(&s.chassis,1,0,0,s.now)==CHASSIS_NOT_READY);
    ready(&s); CHECK(Chassis_SetBodyVelocity(&s.chassis,10,0,0,s.now)==CHASSIS_ACCEPTED);
    run(&s,100u); CHECK(Chassis_SetBodyVelocity(&s.chassis,0,0,0,s.now)==CHASSIS_ACCEPTED);
    CHECK(s.chassis.data.state==CHASSIS_STOPPING); finish(&s,5000u);
    CHECK(s.chassis.data.state==CHASSIS_READY);
    CHECK(!s.bad_axis);
    init_at(&s,UINT32_MAX-4000u); ready(&s);
    CHECK(Chassis_SetBodyVelocity(&s.chassis,10,0,0,s.now)==CHASSIS_ACCEPTED);
    for (i=0u;i<100u;++i) { CHECK(Chassis_SetBodyVelocity(&s.chassis,10,0,0,s.now)==CHASSIS_ACCEPTED); run(&s,50u); }
    Chassis_Stop(&s.chassis,s.now); finish(&s,5000u); CHECK(s.chassis.data.result==CHASSIS_CANCELLED);
    init(&s); ready(&s);
    CHECK(Chassis_SetBodyVelocity(&s.chassis,50,0,0,s.now)==CHASSIS_ACCEPTED);
    run(&s,200u); s.hold_tx=1u; groups=s.groups;
    /* New caller commands cannot keep an unsent/failed bus vector alive. */
    for (i=0u;i<12u && s.chassis.data.state==CHASSIS_VELOCITY;++i) {
        CHECK(Chassis_SetBodyVelocity(&s.chassis,50,0,0,s.now)==CHASSIS_ACCEPTED);
        run(&s,50u);
    }
    CHECK(s.chassis.data.state==CHASSIS_STOPPING && s.chassis.data.reason!=CHASSIS_INPUT_EXPIRED);
    CHECK(!s.bus.wheels_pending && s.groups==groups);
    s.hold_tx=0u; finish(&s,6000u); CHECK(s.chassis.data.state==CHASSIS_FAULT);
    puts("PASS: async enable, 4 present/5-7 absent, 10Hz speed/poll coexistence, leases, reversal, zero, wrap");
}
static void odometry_tests(void)
{
    ChassisConfig c=config(); ChassisOdometry o;
    GyroSnapshot g; FlowSnapshot f; uint32_t generation;
    memset(&o,0,sizeof(o)); memset(&g,0,sizeof(g)); memset(&f,0,sizeof(f));
    c.flow_mount_deg=c.flow_offset_x_mm=c.flow_offset_y_mm=0.0f;
    c.flow_x_mm_count=2.0f; c.flow_y_mm_count=3.0f;
    g.state=GYRO_RUNNING; g.calibrated=g.have_sample=g.angle_valid=g.rate_valid=1u;
    g.sequence=1u; g.sample_ms=100u;
    f.state=FLOW_READY; f.have_sample=f.valid=f.tracking=f.comm_ok=1u; f.sequence=1u; f.sample_ms=100u;
    Chassis_OdometryProcess(&o,&c,&f,&g,100u); CHECK(o.valid); NEAR(o.x,0,0.001);
    g.sequence=2u; g.sample_ms=110u; g.angle_deg=90;
    f.sequence=2u; f.sample_ms=105u; f.total_x=10;
    Chassis_OdometryProcess(&o,&c,&f,&g,110u);
    NEAR(o.x,20*cos(22.5*RAD),0.001); NEAR(o.y,20*sin(22.5*RAD),0.001);
    Chassis_OdometryProcess(&o,&c,&f,&g,111u); NEAR(o.x,20*cos(22.5*RAD),0.001);
    f.sequence=3u; f.sample_ms=110u; f.total_x=20;
    Chassis_OdometryProcess(&o,&c,&f,&g,111u);
    NEAR(o.x,20*(cos(22.5*RAD)+cos(67.5*RAD)),0.001);
    g.sequence=3u; g.sample_ms=120u;
    f.sequence=5u; f.sample_ms=125u; f.total_x=40;
    Chassis_OdometryProcess(&o,&c,&f,&g,125u); CHECK(o.flow_guard.sequence==3u);
    g.sequence=4u; g.sample_ms=130u;
    Chassis_OdometryProcess(&o,&c,&f,&g,130u);
    NEAR(o.y,20*(sin(22.5*RAD)+sin(67.5*RAD))+40,0.001);
    generation=o.generation; ++f.clears;
    Chassis_OdometryProcess(&o,&c,&f,&g,130u); CHECK(o.generation!=generation);
    generation=o.generation; ++g.zeros;
    Chassis_OdometryProcess(&o,&c,&f,&g,130u); CHECK(o.generation!=generation);
    /* Known pure rotation at an offset sensor, including crossing 360 degrees. */
    memset(&o,0,sizeof(o)); c.flow_x_mm_count=c.flow_y_mm_count=0.001f;
    c.flow_offset_x_mm=40; c.flow_offset_y_mm=-20;
    g.sequence=1u; g.sample_ms=200u; g.angle_deg=359; f.sequence=1u; f.sample_ms=200u; f.total_x=f.total_y=0;
    Chassis_OdometryProcess(&o,&c,&f,&g,200u);
    {
        double a=359*RAD,b=361*RAD,m=360*RAD;
        double dx=(cos(b)-cos(a))*40+(sin(b)-sin(a))*20;
        double dy=(sin(b)-sin(a))*40-(cos(b)-cos(a))*20;
        f.total_x=rounded((cos(m)*dx+sin(m)*dy)/0.001);
        f.total_y=rounded((-sin(m)*dx+cos(m)*dy)/0.001);
    }
    g.sequence=2u; g.sample_ms=210u; g.angle_deg=361; f.sequence=2u; f.sample_ms=210u;
    Chassis_OdometryProcess(&o,&c,&f,&g,210u); NEAR(o.x,0,0.002); NEAR(o.y,0,0.002);
    generation=o.generation; f.sample_ms=211u; ++f.sequence;
    Chassis_OdometryProcess(&o,&c,&f,&g,211u); CHECK(o.flow_guard.sample_ms==210u);
    g.sample_ms=320u; ++g.sequence;
    Chassis_OdometryProcess(&o,&c,&f,&g,320u); CHECK(!o.valid && o.generation!=generation);
    puts("PASS: odometry scale, bracket interpolation/wait, dedup/cumulative counts, resets, offset rotation/360 crossing");
}
static void alignment_phase_test(void)
{
    ChassisConfig c=config(); ChassisOdometry o; GyroSnapshot g; FlowSnapshot f;
    unsigned i,phase,wrap;
    c.flow_mount_deg=c.flow_offset_x_mm=c.flow_offset_y_mm=0.0f;
    c.flow_x_mm_count=c.flow_y_mm_count=1.0f;
    for (wrap=0u;wrap<2u;++wrap) for (phase=1u;phase<10u;++phase) {
        uint32_t base=wrap ? UINT32_MAX-160u : 0u;
        memset(&o,0,sizeof(o)); memset(&g,0,sizeof(g)); memset(&f,0,sizeof(f));
        g.state=GYRO_RUNNING; g.calibrated=g.have_sample=g.angle_valid=g.rate_valid=1u;
        f.state=FLOW_READY; f.have_sample=f.valid=f.tracking=f.comm_ok=1u; f.raw.squal=150u;
        g.sequence=f.sequence=1u; g.sample_ms=f.sample_ms=base;
        Chassis_OdometryProcess(&o,&c,&f,&g,base); CHECK(o.valid);
        /* Both healthy 100Hz services. The older pending flow has a new gyro
           bracket, even when every newest flow timestamp is still ahead. */
        for (i=1u;i<=30u;++i) {
            g.sequence=i+1u; g.sample_ms=base+i*10u;
            f.sequence=i+1u; f.sample_ms=base+i*10u+phase; f.total_x=i;
            Chassis_OdometryProcess(&o,&c,&f,&g,f.sample_ms);
            CHECK(o.valid && !o.generation);
        }
        NEAR(o.x,29.0,0.001);
    }
    /* A genuine lack of usable gyro brackets must still time out at 100ms.
       Snapshot consumption below misses every other gyro sample, making the
       observed 20ms spans inadmissible under the unchanged 14ms limit. */
    memset(&o,0,sizeof(o)); g.sequence=f.sequence=1u; g.sample_ms=f.sample_ms=0u;
    f.total_x=0; Chassis_OdometryProcess(&o,&c,&f,&g,0u); CHECK(o.valid);
    for (i=1u;i<=6u;++i) {
        g.sequence=i*2u+1u; g.sample_ms=i*20u;
        f.sequence=i*2u+1u; f.sample_ms=i*20u+5u; f.total_x=i*2u;
        Chassis_OdometryProcess(&o,&c,&f,&g,f.sample_ms);
    }
    CHECK(!o.valid && o.last_issue==CHASSIS_ODOM_SYNC_TIMEOUT);
    CHECK(o.break_quality==150u && !o.break_rejected && !o.break_gaps && o.break_paired_age_ms==105u);
    puts("PASS: flow/gyro phase 1..9ms, clock wrap, retained pending sample, real alignment timeout");
}
static void fault_evidence_tests(void)
{
    Sim s; ChassisFault first; char text[43];
    init(&s); ready(&s); s.flow.raw.squal=150u;
    CHECK(Chassis_MoveRelative(&s.chassis,300,0,100,s.now)==CHASSIS_ACCEPTED);
    run(&s,200u); s.freeze_flow=1u; run(&s,150u);
    CHECK(s.chassis.data.reason==CHASSIS_FLOW_INVALID && s.chassis.data.fault.valid);
    CHECK(s.chassis.data.fault.issue==CHASSIS_ODOM_FLOW_STALE);
    CHECK(s.chassis.data.fault.quality==150u && s.chassis.data.fault.rejected==0u);
    first=s.chassis.data.fault; s.freeze_flow=0u; run(&s,2500u);
    CHECK(s.chassis.data.flow_valid && s.chassis.data.state==CHASSIS_FAULT);
    CHECK(!memcmp(&first,&s.chassis.data.fault,sizeof(first)));
    ChassisUi_Line(text,8u,0u,&s.chassis,s.now); CHECK(!strcmp(text,"FLOW INVALID: STALE"));
    ready(&s); CHECK(!s.chassis.data.fault.valid);
    CHECK(Chassis_MoveRelative(&s.chassis,300,0,100,s.now)==CHASSIS_ACCEPTED);
    run(&s,200u);
    /* Flow service recovered before the chassis next observed it. Retain the
       historical rejection reason and quality, not the current healthy Q. */
    ++s.flow.rejected; s.flow.last_rejection=FLOW_LOW_QUALITY;
    s.flow.last_rejected_quality=12u; s.flow.last_rejected_ms=s.now-1u;
    run(&s,1u);
    CHECK(s.chassis.data.reason==CHASSIS_FLOW_INVALID && s.chassis.data.fault.quality==12u);
    CHECK(s.chassis.data.fault.issue==CHASSIS_ODOM_FLOW_REJECTED);
    ChassisUi_Line(text,8u,1u,&s.chassis,s.now); CHECK(!strcmp(text,"FLOW INVALID: LOW QUALITY"));
    first=s.chassis.data.fault; run(&s,2500u);
    CHECK(!memcmp(&first,&s.chassis.data.fault,sizeof(first)));
    CHECK(!s.bus.wheels_pending && !s.chassis.data.sequence_active);
    puts("PASS: first fault evidence survives live recovery; real rejection still stops the action");
}
static void closed_tests(void)
{
    Sim s; uint32_t t; unsigned stage=0u, count=0u; double sx=0,sy=0,sa=0;
    init(&s);
    CHECK(Chassis_StartAcceptance(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    CHECK(Chassis_StartAcceptance(&s.chassis,s.now)==CHASSIS_BUSY);
    for (t=0u;t<120000u && Chassis_OwnsWheels(&s.chassis);++t) {
        if (s.chassis.data.stage!=stage) { stage=s.chassis.data.stage; sx=s.x; sy=s.y; sa=s.angle; }
        if (s.chassis.data.result==CHASSIS_SUCCESS && s.chassis.data.state==CHASSIS_READY && stage>count) {
            double ex=s.chassis.data.target_x_mm, ey=s.chassis.data.target_y_mm;
            double dx=cos(sa*RAD)*(s.x-sx)+sin(sa*RAD)*(s.y-sy);
            double dy=-sin(sa*RAD)*(s.x-sx)+cos(sa*RAD)*(s.y-sy);
            if (stage<=6u) { CHECK(hypot(dx-ex,dy-ey)<20.0); CHECK(fabs(s.angle-sa)<5.0); }
            else CHECK(fabs((s.angle-sa)-s.chassis.data.target_angle_deg)<5.0);
            ++count;
        }
        CHECK(Chassis_SetBodyVelocity(&s.chassis,10,0,0,s.now)==CHASSIS_BUSY);
        run(&s,1u);
    }
    if (!s.chassis.data.done) fprintf(stderr,"sequence state %s reason %s stage %u errors %f %f\n",
        Chassis_StateName(s.chassis.data.state),Chassis_ReasonName(s.chassis.data.reason),
        s.chassis.data.stage,s.chassis.data.error_mm,s.chassis.data.error_deg);
    CHECK(s.chassis.data.done && s.chassis.data.result==CHASSIS_SUCCESS && stage==8u && count==7u);
    CHECK(!s.bad_axis && s.stop_applied>=32u); NEAR(s.angle-sa,-90,5.0);
    { unsigned n=s.groups; run(&s,3000u); CHECK(s.groups==n && s.chassis.data.done); }
    printf("SIMULATION ONLY: eight segments complete in %lu ms, return %.1f/%.1f mm %.1f deg\n",
        (unsigned long)t,s.x,s.y,s.angle);
    init(&s); ready(&s); s.freeze_flow=1u;
    CHECK(Chassis_RotateRelative(&s.chassis,270,30,s.now)==CHASSIS_ACCEPTED);
    finish(&s,40000u); CHECK(s.chassis.data.result==CHASSIS_SUCCESS); NEAR(s.angle,270,5.0);
    init(&s); ready(&s); s.stuck=1u;
    CHECK(Chassis_MoveRelative(&s.chassis,100,0,100,s.now)==CHASSIS_ACCEPTED);
    finish(&s,15000u); CHECK(s.chassis.data.result==CHASSIS_FAILED && s.chassis.data.reason==CHASSIS_ACTION_TIMEOUT);
    puts("PASS: independent delayed/inertial plant, eight segments, unwrapped +270 rotation, stuck timeout");
}
static void faults(void)
{
    Sim s; unsigned i,groups;
    for (i=0u;i<9u;++i) {
        init(&s); ready(&s);
        CHECK(Chassis_MoveRelative(&s.chassis,300,0,100,s.now)==CHASSIS_ACCEPTED);
        run(&s,500u);
        if (i==0u) s.freeze_flow=1u;
        if (i==1u) s.freeze_gyro=1u;
        if (i==2u) ++s.flow.clears;
        if (i==3u) ++s.flow.rejected;
        if (i==4u) ++s.gyro.zeros;
        if (i==5u) ++s.gyro.restarts;
        if (i==6u) s.flags[2]|=4u;
        if (i==7u) s.online=7u;
        if (i==8u) s.drop_ack=1u;
        run(&s,3500u);
        CHECK(s.chassis.data.state==CHASSIS_STOPPING || s.chassis.data.state==CHASSIS_FAULT);
        CHECK(!s.chassis.data.sequence_active && !s.bus.wheels_pending);
        groups=s.groups; run(&s,2000u); CHECK(s.groups==groups);
    }
    init(&s); ready(&s); CHECK(Chassis_SetBodyVelocity(&s.chassis,50,0,0,s.now)==CHASSIS_ACCEPTED);
    run(&s,250u); s.drop_stop=1u; Chassis_Stop(&s.chassis,s.now); finish(&s,6000u);
    CHECK(s.chassis.data.result==CHASSIS_CANCELLED && s.stop_applied>=4u);
    CHECK(s.sends[1][EMM_STOP]+s.sends[2][EMM_STOP]+s.sends[3][EMM_STOP]+s.sends[4][EMM_STOP]>=5u);
    init(&s); ready(&s); CHECK(Chassis_SetBodyVelocity(&s.chassis,50,0,0,s.now)==CHASSIS_ACCEPTED);
    run(&s,100u); s.hold_tx=1u; run(&s,7000u);
    CHECK(s.chassis.data.result==CHASSIS_FAILED || s.chassis.data.state==CHASSIS_STOPPING);
    puts("PASS: stale/rebased/rejected sensors, wheel fault/loss, missing group ACK, lost STOP retry, busy bus");
}
static void result_tests(void)
{
    Sim s; uint32_t t; unsigned groups;
    init(&s); ready(&s);
    CHECK(Chassis_MoveRelative(&s.chassis,100,0,100,s.now)==CHASSIS_ACCEPTED);
    for (t=0u;t<15000u && s.chassis.data.state!=CHASSIS_STOPPING;++t) run(&s,1u);
    CHECK(s.chassis.data.state==CHASSIS_STOPPING && s.chassis.data.result!=CHASSIS_SUCCESS);
    s.x+=50.0; finish(&s,6000u);
    CHECK(s.chassis.data.result==CHASSIS_FAILED && s.chassis.data.reason==CHASSIS_FINAL_ERROR);
    Chassis_Stop(&s.chassis,s.now); finish(&s,5000u); CHECK(s.chassis.data.state==CHASSIS_FAULT);
    CHECK(Chassis_SetBodyVelocity(&s.chassis,10,0,0,s.now)==CHASSIS_NOT_READY); ready(&s);
    init(&s); ready(&s); CHECK(Chassis_SetBodyVelocity(&s.chassis,50,0,0,s.now)==CHASSIS_ACCEPTED);
    run(&s,250u); s.drop_stop=255u; Chassis_Stop(&s.chassis,s.now); run(&s,5000u);
    CHECK(s.chassis.data.state==CHASSIS_STOPPING && s.chassis.data.reason==CHASSIS_STOP_TIMEOUT);
    CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_BUSY);
    groups=s.groups; s.drop_stop=0u; finish(&s,5000u);
    CHECK(s.chassis.data.state==CHASSIS_FAULT && s.groups==groups);
    init(&s); CHECK(Chassis_StartAcceptance(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    for (t=0u;t<20000u && !s.chassis.between;++t) run(&s,1u);
    CHECK(s.chassis.between && s.chassis.data.stage==1u);
    ++s.flow.clears; run(&s,1u); CHECK(s.chassis.data.state==CHASSIS_STOPPING);
    CHECK(!s.chassis.data.sequence_active); finish(&s,5000u); CHECK(s.chassis.data.result==CHASSIS_FAILED);
    init(&s); s.reject_enable=2u; CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    finish(&s,6000u); CHECK(s.chassis.data.result==CHASSIS_FAILED && !s.groups);
    init(&s); s.online=7u; CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    run(&s,12000u); CHECK(!s.groups && s.chassis.data.state==CHASSIS_STOPPING);
    puts("PASS: post-STOP error recheck, failure latch, unconfirmed STOP/retry, between-stage rebase, enable/missing-wheel rejection");
}
int main(void)
{
    kinematics(); odometry_tests(); alignment_phase_test(); velocity_tests(); closed_tests(); faults(); result_tests(); fault_evidence_tests();
    printf("PASS: %u chassis checks (software and simulation; no physical acceptance)\n",checks); return 0;
}
