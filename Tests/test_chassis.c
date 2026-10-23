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

    return c;
}
typedef struct {
    MotorBus bus; Chassis chassis; ChassisConfig cfg;
    GyroSnapshot gyro;
    uint32_t now, tx_done;
    uint8_t bytes[128], length, busy, response, online, flags[7];
    uint8_t drop_ack, drop_stop, hold_tx, freeze_position, freeze_gyro, stuck, reject_enable, slip;
    int32_t command[7]; double wheel[4], position[4];
    double x,y,angle,rate;
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
    else if (b[1]==EMM_POSITION) {
        int32_t counts=i<4u ? rounded(s->position[i]) : 0;
        if (a==s->freeze_position) return;
        n=8u; r[2]=(uint8_t)(counts<0); v=(uint32_t)(counts<0 ? -counts : counts);
        r[3]=(uint8_t)(v>>24); r[4]=(uint8_t)(v>>16); r[5]=(uint8_t)(v>>8); r[6]=(uint8_t)v;
    }
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
        if (s->stuck) s->wheel[i]=0.0;
        s->position[i]+=s->wheel[i]*65536.0/60000.0;
        w[i]=s->wheel[i]*(i & 1u ? -1.0 : 1.0)*5.235987755982989;
    }
    vx=(w[0]+w[1]+w[2]+w[3])*0.25;
    vy=(-w[0]+w[1]+w[2]-w[3])*0.25;
    omega=(-w[0]+w[1]-w[2]+w[3])/760.0/RAD;
    if (s->slip) vx=vy=omega=0.0;
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
        Chassis_Process(&s->chassis,&s->gyro,s->now);
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
    memset(c.motor_sign,0,sizeof(c.motor_sign));
    CHECK(Chassis_ConfigIssues(&c,1u)==23u);
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
    ChassisConfig c=config(); ChassisOdometry o; MotorBus b; GyroSnapshot g;
    unsigned wrap,kind,i,t; uint32_t generation;
    const double linear[4][4]={{100,100,100,100},{-100,100,100,-100},
        {0,100,100,0},{-60,60,-60,60}};
    for(wrap=0u;wrap<2u;++wrap) for(kind=0u;kind<4u;++kind) {
        uint32_t base=wrap ? UINT32_MAX-1500u : 0u;
        memset(&o,0,sizeof(o)); memset(&b,0,sizeof(b)); memset(&g,0,sizeof(g));
        c.gear_ratio=2.0f;
        g.state=GYRO_RUNNING; g.calibrated=g.have_sample=g.angle_valid=g.rate_valid=1u;
        g.sequence=1u; g.sample_ms=base;
        for(i=0u;i<4u;++i) {
            b.nodes[i].data.online=1u; b.nodes[i].data.valid=MOTOR_VALID_POSITION;
            b.nodes[i].data.position_seq=1u; b.nodes[i].data.position_ms=base;
        }
        Chassis_OdometryProcess(&o,&c,&b,&g,base); CHECK(o.valid);
        for(t=1u;t<=3000u;++t) {
            if(t%10u==0u) { ++g.sequence; g.sample_ms=base+t; }
            for(i=0u;i<4u;++i) if(t%500u==i*70u) {
                MotorSnapshot *n=&b.nodes[i].data;
                ++n->position_seq; n->position_ms=base+t;
                n->position_counts=rounded(linear[kind][i]*c.motor_sign[i]*t*0.417215134018818);
            }
            Chassis_OdometryProcess(&o,&c,&b,&g,base+t); CHECK(o.valid && !o.generation);
        }
        {
            double seconds=(uint32_t)(o.sample_ms-base)*0.001;
            NEAR(o.x,(kind==0u ? 100.0 : kind==2u ? 50.0 : 0.0)*seconds,0.01);
            NEAR(o.y,(kind==1u ? 100.0 : kind==2u ? 50.0 : 0.0)*seconds,0.01);
        }
        generation=o.generation;
        ++b.nodes[2].data.position_seq; b.nodes[2].data.position_ms=base+3001u;
        b.nodes[2].data.position_counts+=1000000;
        Chassis_OdometryProcess(&o,&c,&b,&g,base+3001u);
        CHECK(!o.valid && o.generation!=generation && o.last_issue==CHASSIS_ODOM_WHEEL_JUMP);
        CHECK(o.break_wheel==3u);
    }
    /* Clock/sequence reset and missing positions cannot masquerade as zero travel. */
    memset(&o,0,sizeof(o)); memset(&b,0,sizeof(b)); g.sample_ms=5000u; ++g.sequence;
    for(i=0u;i<4u;++i) {
        b.nodes[i].data.online=1u; b.nodes[i].data.valid=MOTOR_VALID_POSITION;
        b.nodes[i].data.position_seq=3u; b.nodes[i].data.position_ms=5000u;
    }
    Chassis_OdometryProcess(&o,&c,&b,&g,5000u); CHECK(o.valid);
    b.nodes[0].data.position_seq=1u;
    Chassis_OdometryProcess(&o,&c,&b,&g,5001u);
    CHECK(!o.valid && o.last_issue==CHASSIS_ODOM_WHEEL_RESET);
    memset(&o,0,sizeof(o)); b.nodes[0].data.position_seq=3u;
    Chassis_OdometryProcess(&o,&c,&b,&g,5000u); CHECK(o.valid);
    g.sample_ms=6601u; ++g.sequence;
    Chassis_OdometryProcess(&o,&c,&b,&g,6601u);
    CHECK(!o.valid && o.last_issue==CHASSIS_ODOM_WHEEL_STALE);
    puts("PASS: motor 65536 counts/rev, gearing/signs, staggered replies/common-time interpolation, rotation cancellation, wrap/reset/jump/stale");
}
static void fault_evidence_tests(void)
{
    Sim s; ChassisFault first; char text[43]; MotorSnapshot *n;
    init(&s); ready(&s);
    CHECK(Chassis_MoveRelative(&s.chassis,300,0,100,s.now)==CHASSIS_ACCEPTED);
    run(&s,200u); n=&s.bus.nodes[2].data;
    ++n->position_seq; n->position_ms=s.now; n->position_counts+=1000000;
    run(&s,1u);
    CHECK(s.chassis.data.reason==CHASSIS_ODOMETRY_INVALID && s.chassis.data.fault.valid);
    CHECK(s.chassis.data.fault.issue==CHASSIS_ODOM_WHEEL_JUMP && s.chassis.data.fault.wheel==3u);
    first=s.chassis.data.fault; run(&s,5000u);
    CHECK(s.chassis.data.odometry_valid && s.chassis.data.state==CHASSIS_FAULT);
    CHECK(!memcmp(&first,&s.chassis.data.fault,sizeof(first)));
    ChassisUi_Line(text,8u,0u,&s.chassis,s.now); CHECK(!strcmp(text,"ODOM INVALID: POSITION JUMP"));
    ready(&s); CHECK(!s.chassis.data.fault.valid);
    puts("PASS: first motor-position fault survives recovery; explicit prepare clears evidence");
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
    init(&s); ready(&s);
    CHECK(Chassis_RotateRelative(&s.chassis,270,30,s.now)==CHASSIS_ACCEPTED);
    finish(&s,40000u); CHECK(s.chassis.data.result==CHASSIS_SUCCESS); NEAR(s.angle,270,5.0);
    {
        double x=s.x,y=s.y;
        CHECK(Chassis_MoveRelative(&s.chassis,100,0,80,s.now)==CHASSIS_ACCEPTED);
        finish(&s,15000u); CHECK(s.chassis.data.result==CHASSIS_SUCCESS);
        NEAR(s.x-x,0,20.0); NEAR(s.y-y,-100,20.0);
    }
    init(&s); ready(&s); s.stuck=1u;
    CHECK(Chassis_MoveRelative(&s.chassis,100,0,100,s.now)==CHASSIS_ACCEPTED);
    finish(&s,15000u); CHECK(s.chassis.data.result==CHASSIS_FAILED && s.chassis.data.reason==CHASSIS_ACTION_TIMEOUT);
    init(&s); ready(&s); s.slip=1u;
    CHECK(Chassis_MoveRelative(&s.chassis,100,0,100,s.now)==CHASSIS_ACCEPTED);
    finish(&s,15000u); CHECK(s.chassis.data.result==CHASSIS_SUCCESS);
    NEAR(s.x,0.0,0.001); CHECK(s.chassis.data.x_mm>90.0);
    puts("PASS: independent plant/eight segments/+270 then local move/stuck timeout; wheel slip is explicitly unobservable");
}
static void faults(void)
{
    Sim s; unsigned i,groups;
    for (i=0u;i<9u;++i) {
        init(&s); ready(&s);
        CHECK(Chassis_MoveRelative(&s.chassis,300,0,100,s.now)==CHASSIS_ACCEPTED);
        run(&s,500u);
        if (i==0u) s.freeze_position=1u;
        if (i==1u) s.freeze_gyro=1u;
        if (i==2u) { ++s.bus.nodes[0].data.position_seq; s.bus.nodes[0].data.position_ms=s.now; s.bus.nodes[0].data.position_counts+=1000000; }
        if (i==3u) s.bus.nodes[1].data.position_seq=0u;
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
    { unsigned j; for(j=0u;j<4u;++j) s.position[j]+=(j & 1u ? -1.0 : 1.0)*10430.37835047; } finish(&s,6000u);
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
    ++s.gyro.zeros; run(&s,1u); CHECK(s.chassis.data.state==CHASSIS_STOPPING);
    CHECK(!s.chassis.data.sequence_active); finish(&s,5000u); CHECK(s.chassis.data.result==CHASSIS_FAILED);
    init(&s); s.reject_enable=2u; CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    finish(&s,6000u); CHECK(s.chassis.data.result==CHASSIS_FAILED && !s.groups);
    init(&s); s.online=7u; CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_ACCEPTED);
    run(&s,12000u); CHECK(!s.groups && s.chassis.data.state==CHASSIS_STOPPING);
    puts("PASS: post-STOP error recheck, failure latch, unconfirmed STOP/retry, between-stage rebase, enable/missing-wheel rejection");
}
int main(void)
{
    kinematics(); odometry_tests(); velocity_tests(); closed_tests(); faults(); result_tests(); fault_evidence_tests();
    printf("PASS: %u chassis checks (software and simulation; no physical acceptance)\n",checks); return 0;
}
