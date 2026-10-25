/* Synthetic mechanics + real MotorBus/Emm framing and incremental replies.
   None of these distances, signs or scales are vehicle calibration data. */
#include "mechanism/mechanism.h"
#include "mechanism/bsp_limit_gpio.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"MECH FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
#define NEAR(a,b,e) CHECK(fabs((double)(a)-(double)(b))<=(e))
GPIO_TypeDef fake_gpiog;
static uint8_t gpio_levels[2];
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *g)
{ CHECK(port==GPIOG && g->Pin==(GPIO_PIN_2|GPIO_PIN_3) && g->Mode==GPIO_MODE_INPUT && g->Pull==GPIO_PULLUP); }
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ CHECK(port==GPIOG && (pin==GPIO_PIN_2 || pin==GPIO_PIN_3)); return gpio_levels[pin==GPIO_PIN_2 ? 0u : 1u]; }
static MechanismConfig config(void)
{
    unsigned a; MechanismConfig c; Mechanism_DefaultConfig(&c);
    for (a=0u;a<3u;++a) {
        c.axis[a].units_per_rev=a==0u ? 100.0 : 10.0;
        c.axis[a].pulses_per_rev=3200u; c.axis[a].positive_sign=a==1u ? 1 : -1;
        c.axis[a].calibrated=1u; c.axis[a].max_rpm=200.0f;
        c.axis[a].move_speed=a==0u ? 20.0f : 5.0f; c.axis[a].tolerance=a==0u ? 0.1f : a==1u ? 0.02f : 0.005f;
    }
    for (a=0u;a<2u;++a) {
        c.home[a].wiring_confirmed=c.home[a].home_confirmed=1u;
        c.home[a].max_position=a==0u ? 100.0f : 30.0f;
        c.home[a].initial_release=a==0u ? 5.0f : 1.0f;
        c.home[a].fast_seek=a==0u ? 30.0f : 3.0f;
        c.home[a].backoff=c.home[a].clearance=a==0u ? 2.0f : 0.3f;
        c.home[a].slow_seek=a==0u ? 5.0f : 1.0f;
    }
    c.outside_angle=20.0f; c.top_window=0.1f; c.work_window=0.5f;
    return c;
}
typedef struct {
    MotorBus bus; Mechanism mech; Chassis chassis; MechanismConfig cfg;
    uint32_t now,tx_done; uint8_t busy,bytes[128],length,online;
    uint8_t flags[7],freeze_position,drop_stop,reject_enable,hold_stop,stuck;
    uint8_t raw_mode[2],switch_high[2],drop_move_ack;
    uint8_t velocity[7];
    uint8_t delayed[8],delayed_length,delay_ack;
    uint32_t reply_due; int32_t bias_counts[7];
    double counts[7],target[7]; int32_t rpm[7];
    unsigned moves[7],stops[7],enables[7],order[128],order_rpm[128],order_count;
    uint32_t last_trigger,first_stop;
} Sim;
static int send_wire(void *ctx, const uint8_t *b, uint8_t length)
{
    Sim *s=ctx; unsigned a=b[0]-1u;
    CHECK(!s->busy && length<=128u && a<7u);
    s->busy=1u; memcpy(s->bytes,b,length); s->length=length; s->tx_done=s->now+2u;
    if (b[1]==EMM_MOVE) {
        ++s->moves[a];
        if (s->order_count<128u) {
            s->order_rpm[s->order_count]=((unsigned)b[3]<<8)|b[4];
            s->order[s->order_count++]=a+1u;
        }
    }
    if (b[1]==EMM_STOP) { ++s->stops[a]; if (!s->first_stop) s->first_stop=s->now; }
    if (b[1]==EMM_ENABLE) ++s->enables[a];
    return 1;
}
static int wire_busy(void *ctx) { return ((Sim *)ctx)->busy; }
static int64_t rounded(double v) { return (int64_t)(v+(v<0.0 ? -0.5 : 0.5)); }
static void reply(Sim *s)
{
    uint8_t *b=s->bytes,r[8]={0}; unsigned a=b[0]-1u,n=4u; uint32_t v; int64_t counts;
    if (!(s->online & (1u<<a))) return;
    r[0]=b[0]; r[1]=b[1]; r[2]=EMM_ACK_OK;
    if (b[1]==EMM_ENABLE) {
        if (s->reject_enable==b[0]) r[2]=0xE2u;
        else s->flags[a]=(uint8_t)(2u|b[3]);
    } else if (b[1]==EMM_MOVE) {
        v=((uint32_t)b[6]<<24)|((uint32_t)b[7]<<16)|((uint32_t)b[8]<<8)|b[9];
        counts=(int64_t)v*65536/s->cfg.axis[a-4u].pulses_per_rev;
        if (b[2]) counts= -counts;
        s->target[a]=s->counts[a]+(double)counts;
        s->rpm[a]=((int32_t)b[3]<<8)|b[4]; if (b[2]) s->rpm[a]= -s->rpm[a];
        s->flags[a]&=0xFDu;
        if (s->drop_move_ack) return; /* physical command accepted, reply lost */
    } else if (b[1]==EMM_VELOCITY) {
        s->velocity[a]=1u; ++s->moves[a];
        s->rpm[a]=((int32_t)b[3]<<8)|b[4]; if (b[2]) s->rpm[a]= -s->rpm[a];
        s->flags[a]&=0xFDu;
        if (s->drop_move_ack) return;
    } else if (b[1]==EMM_STOP) {
        if (s->hold_stop || s->drop_stop) { if (s->drop_stop) --s->drop_stop; return; }
        s->rpm[a]=0; s->flags[a]|=2u;
    } else if (b[1]==EMM_SPEED) {
        n=6u; r[2]=(uint8_t)(s->rpm[a]<0); v=(uint32_t)abs(s->rpm[a]); r[3]=(uint8_t)(v>>8); r[4]=(uint8_t)v;
    } else if (b[1]==EMM_STATUS) r[2]=s->flags[a];
    else if (b[1]==EMM_POSITION) {
        if (s->freeze_position==b[0]) return;
        counts=rounded(s->counts[a])+s->bias_counts[a]; n=8u; r[2]=(uint8_t)(counts<0); v=(uint32_t)(counts<0 ? -counts : counts);
        r[3]=(uint8_t)(v>>24); r[4]=(uint8_t)(v>>16); r[5]=(uint8_t)(v>>8); r[6]=(uint8_t)v;
    }
    r[n-1u]=0x6Bu;
    if (s->delay_ack && b[1]==EMM_MOVE) {
        memcpy(s->delayed,r,n); s->delayed_length=(uint8_t)n; s->reply_due=s->now+50u; return;
    }
    MotorBus_Receive(&s->bus,r,2u,s->now); MotorBus_Receive(&s->bus,r+2u,n-2u,s->now);
}
static uint8_t limit(Sim *s, unsigned a)
{
    double physical=s->counts[a+4u]*s->cfg.axis[a].positive_sign*s->cfg.axis[a].units_per_rev/65536.0;
    if (s->raw_mode[a]==1u) return 1u; /* pressed or open circuit */
    if (s->raw_mode[a]==2u) return 0u; /* short to GND */
    if (s->raw_mode[a]==3u) return (uint8_t)(s->now%8u<4u); /* never stable */
    if (physical<=0.0) s->switch_high[a]=1u;
    if (physical>0.05) s->switch_high[a]=0u;
    return s->switch_high[a];
}
static void run(Sim *s, uint32_t duration)
{
    uint32_t t; unsigned a;
    for (t=0u;t<duration;++t,++s->now) {
        for (a=0u;a<7u;++a) if (s->rpm[a] && !s->stuck) {
            double step=s->rpm[a]*65536.0/60000.0;
            if (!s->velocity[a] && !s->hold_stop && fabs(s->target[a]-s->counts[a])<=fabs(step)) {
                s->counts[a]=s->target[a]; s->rpm[a]=0; s->flags[a]|=2u;
            } else s->counts[a]+=step;
        }
        Mechanism_SampleLimits(&s->mech,limit(s,0u),limit(s,1u),s->now);
        Mechanism_Process(&s->mech,s->now);
        if (s->mech.triggered && !s->last_trigger) s->last_trigger=s->now;
        if (s->busy && (int32_t)(s->now-s->tx_done)>=0) { s->busy=0u; reply(s); }
        if (s->delayed_length && (int32_t)(s->now-s->reply_due)>=0) {
            MotorBus_Receive(&s->bus,s->delayed,s->delayed_length,s->now); s->delayed_length=0u;
        }
        MotorBus_Process(&s->bus,s->now);
    }
}
static void init_at(Sim *s, uint32_t now)
{
    MotorTransport t; unsigned a;
    memset(s,0,sizeof(*s)); s->now=now; s->online=0x70u; s->cfg=config();
    for (a=0u;a<7u;++a) s->flags[a]=2u;
    s->counts[4]= -2.0*65536.0/100.0; s->counts[5]=0.3*65536.0/10.0;
    s->counts[6]= -8.0*65536.0/10.0;
    t.context=s; t.send=send_wire; t.busy=wire_busy;
    MotorBus_Init(&s->bus,t,now); Chassis_Init(&s->chassis,&s->bus,0,now);
    Mechanism_Init(&s->mech,&s->bus,&s->cfg,now);
    s->mech.auto_pending=1u; /* Legacy HOME scenarios explicitly request auto HOME. */
    s->mech.chassis=&s->chassis; s->chassis.external_busy=&s->mech.owned;
}
static void init(Sim *s) { init_at(s,0u); }
static void finish(Sim *s, uint32_t max)
{
    uint32_t t;
    for (t=0u;t<max && (s->mech.auto_pending || s->mech.owned);++t) run(s,1u);
}
static void homed(Sim *s)
{
    finish(s,40000u);
    if (s->mech.data.result!=MECH_SUCCESS) fprintf(stderr,"home: %s %s axis %u time %u seg %u\n",
        Mechanism_StateName(s->mech.data.state),Mechanism_ReasonName(s->mech.data.reason),s->mech.data.axis,s->now,s->mech.segment);
    CHECK(s->mech.data.result==MECH_SUCCESS && s->mech.data.reference[0]==MECH_HOMED && s->mech.data.reference[1]==MECH_HOMED);
}
static void wait_step(Sim *s, MechanismState state, unsigned axis)
{
    uint32_t t;
    for (t=0u;t<30000u && !(s->mech.data.state==state && s->mech.data.axis==axis && s->mech.segment==1u);++t) run(s,1u);
    CHECK(t<30000u);
}
static void default_and_gpio(void)
{
    Sim s; MechanismConfig c; unsigned a;
    BspLimit_Init(); gpio_levels[0]=0u; gpio_levels[1]=1u;
    CHECK(!BspLimit_Read(0u) && BspLimit_Read(1u)); gpio_levels[0]=1u; gpio_levels[1]=0u;
    CHECK(BspLimit_Read(0u) && !BspLimit_Read(1u));
    Mechanism_DefaultConfig(&c); for (a=0u;a<3u;++a) CHECK(Mechanism_ConfigIssues(&c,a));
    init(&s); Mechanism_Init(&s.mech,&s.bus,0,0u); run(&s,12000u);
    CHECK(!s.order_count && !s.enables[4] && !s.enables[5] && s.mech.data.rejection==MECH_CONFIG);
    CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_INVALID);
    c=config(); c.axis[0].positive_sign=0; CHECK(Mechanism_ConfigIssues(&c,0u)&MECH_CFG_DIRECTION);
    c=config(); c.axis[1].units_per_rev=NAN; CHECK(Mechanism_ConfigIssues(&c,1u)&MECH_CFG_SCALE);
    init(&s); s.mech.auto_pending=0u;
    for (a=0u;a<100u;++a) Mechanism_SampleLimits(&s.mech,0u,0u,0u);
    CHECK(!s.mech.data.limit[0].valid);
    Mechanism_SampleLimits(&s.mech,0u,0u,10u); CHECK(!s.mech.data.limit[0].valid);
    Mechanism_SampleLimits(&s.mech,0u,0u,20u); CHECK(s.mech.data.limit[0].valid);
    Mechanism_SampleLimits(&s.mech,0u,0u,31u); CHECK(!s.mech.data.limit[0].valid);
    puts("PASS: PG2/PG3 NC GPIO; uncalibrated firmware never enables/moves");
}
static void home_and_acceptance(void)
{
    Sim s; unsigned a,old; uint32_t id; double start;
    init(&s); homed(&s);
    CHECK(s.order[0]==6u && s.order[4]==5u && s.order_count==8u);
    CHECK(s.mech.data.reference[2]==MECH_SESSION_REFERENCE && !s.moves[6]);
    CHECK(s.mech.data.stop_submit_ms>=s.mech.data.stop_request_ms);
    CHECK(s.mech.data.stop_confirm_ms>s.mech.data.stop_submit_ms);
    for (a=0u;a<4u;++a) CHECK(!s.moves[a] && !s.enables[a] && !s.stops[a]);
    CHECK(s.flags[5]&1u); /* lift remains enabled */
    NEAR(s.mech.data.position[0],0.0,0.001); NEAR(s.mech.data.position[1],0.0,0.001);
    CHECK(Mechanism_StartAcceptance(&s.mech,s.now)==MECH_ACCEPTED); id=s.mech.data.request_id;
    CHECK(Mechanism_StartAcceptance(&s.mech,s.now)==MECH_BUSY && s.mech.data.request_id==id);
    CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_BUSY);
    CHECK(Chassis_StartAcceptance(&s.chassis,s.now)==CHASSIS_BUSY);
    CHECK(Chassis_Prepare(&s.chassis,s.now)==CHASSIS_BUSY);
    finish(&s,45000u);
    if (s.mech.data.reason) fprintf(stderr,"test: %s stage %u\n",Mechanism_ReasonName(s.mech.data.reason),s.mech.data.stage);
    CHECK(s.mech.data.done && s.mech.data.result==MECH_SUCCESS && s.mech.data.stage==8u);
    CHECK(s.moves[6]==4u && s.moves[4]==6u && s.moves[5]==6u);
    start=(double)s.mech.data.origin_counts[2]; NEAR(s.counts[6],start,2.0);
    old=s.order_count; run(&s,3000u); CHECK(s.order_count==old); /* one round */
    init_at(&s,UINT32_MAX-4000u); homed(&s); CHECK(s.mech.data.reference[2]==MECH_SESSION_REFERENCE);
    init(&s); s.online=0x30u; homed(&s);
    CHECK(!s.mech.data.trim_ready && !s.stops[6] && Mechanism_StartAcceptance(&s.mech,s.now)==MECH_NOT_READY);
    init(&s); s.online=0x71u; s.rpm[0]=10; s.target[0]=100000000.0;
    s.mech.auto_pending=0u; run(&s,2500u);
    CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_ACCEPTED); homed(&s);
    CHECK(s.stops[0] && !s.stops[1] && !s.stops[2] && !s.stops[3]);
    puts("PASS: lift->arm double approach, park origins, optional TRIM, wraparound, 8-step one-shot acceptance");
}
static void conversion_and_gates(void)
{
    Sim s; unsigned before; uint32_t id; double origin;
    init(&s); homed(&s);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_LIFT,1.0f,2.0f,s.now)==MECH_NOT_READY && s.mech.data.rejection==MECH_INTERLOCK);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,-1.0f,2.0f,s.now)==MECH_INVALID);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,101.0f,2.0f,s.now)==MECH_INVALID);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,NAN,2.0f,s.now)==MECH_INVALID);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,1.0f,INFINITY,s.now)==MECH_INVALID);
    CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,FLT_MAX,2.0f,s.now)==MECH_INVALID && !s.mech.owned);
    before=s.moves[6];
    CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,0.00001f,2.0f,s.now)==MECH_ACCEPTED && !s.mech.owned);
    CHECK(s.moves[6]==before);
    origin=(double)s.mech.data.origin_counts[2];
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,-20.0f,5.0f,s.now)==MECH_ACCEPTED); id=s.mech.data.request_id;
    CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,1.0f,2.0f,s.now)==MECH_BUSY && s.mech.data.request_id==id);
    finish(&s,20000u); CHECK(s.mech.data.result==MECH_SUCCESS); NEAR(s.mech.data.position[2],-20.0,0.02);
    NEAR(s.counts[6]-origin,2.0*65536.0,2.0); /* negative mm / negative protocol sign */
    CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,25.0f,5.0f,s.now)==MECH_ACCEPTED);
    finish(&s,25000u); CHECK(s.mech.data.result==MECH_SUCCESS); NEAR(s.mech.data.position[2],5.0,0.02);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,20.0f,20.0f,s.now)==MECH_ACCEPTED); finish(&s,12000u);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_LIFT,1.0f,5.0f,s.now)==MECH_ACCEPTED); finish(&s,12000u);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,0.0f,20.0f,s.now)==MECH_NOT_READY && s.mech.data.rejection==MECH_INTERLOCK);
    s.chassis.data.state=CHASSIS_MOVING;
    CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_BUSY && Mechanism_StartAcceptance(&s.mech,s.now)==MECH_BUSY);
    puts("PASS: degree/mm conversion, signs, relative and absolute requests, NaN/Inf/range, no TRIM boundaries, pose/peer gates");
}
static void fault_tests(void)
{
    Sim s; unsigned n; uint32_t t;
    init(&s); s.counts[4]=s.counts[5]=0.0; homed(&s); CHECK(s.order_count==10u && s.order[0]==6u);
    init(&s); s.raw_mode[1]=1u; finish(&s,30000u);
    CHECK(s.mech.data.result==MECH_FAILED && s.mech.data.reason==MECH_NOT_RELEASED && s.moves[5]==1u);
    n=s.order_count; run(&s,5000u); CHECK(s.order_count==n && !s.mech.auto_pending);
    init(&s); s.raw_mode[1]=2u; finish(&s,30000u);
    CHECK(s.mech.data.result==MECH_FAILED && s.mech.data.reason==MECH_NOT_FOUND && s.moves[5]==1u);
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.raw_mode[1]=1u;
    run(&s,1u); CHECK(s.bus.nodes[5].stop_unconfirmed); /* raw edge before 20ms debounce */
    s.raw_mode[1]=3u;
    finish(&s,12000u); CHECK(s.mech.data.reason==MECH_LIMIT_UNSTABLE);
    init(&s); s.drop_stop=1u; homed(&s); CHECK(s.stops[5]>=3u);
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.hold_stop=1u; s.raw_mode[1]=1u;
    run(&s,5000u); CHECK(s.mech.owned && s.mech.data.stop_pending && s.bus.nodes[5].stop_unconfirmed);
    CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_BUSY);
    s.hold_stop=0u; finish(&s,15000u); CHECK(s.mech.data.result==MECH_FAILED && !s.mech.owned);
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.now+=11u; run(&s,1u);
    CHECK(s.mech.data.reason==MECH_SAMPLE_GAP && s.bus.nodes[5].stop_unconfirmed);
    init(&s); s.reject_enable=6u; finish(&s,20000u); CHECK(s.mech.data.result==MECH_FAILED && !s.moves[5]);
    init(&s); s.online=0x50u; finish(&s,20000u); CHECK(s.mech.data.result==MECH_FAILED && !s.order_count);
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.drop_move_ack=1u;
    /* Losing the next movement ACK cannot be treated as completion. */
    finish(&s,40000u); CHECK(s.mech.data.result==MECH_FAILED);
    init(&s); homed(&s); CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,5.0f,5.0f,s.now)==MECH_ACCEPTED);
    run(&s,100u); Mechanism_Stop(&s.mech,s.now); finish(&s,8000u);
    CHECK(s.mech.data.result==MECH_CANCELLED && !s.mech.auto_pending && s.mech.data.reference[2]==MECH_SESSION_REFERENCE);
    init(&s); homed(&s); s.raw_mode[0]=1u; run(&s,1u);
    CHECK(s.mech.data.reason==MECH_LIMIT && s.bus.nodes[4].stop_unconfirmed && !s.mech.data.reference[0]);
    finish(&s,5000u); Mechanism_Stop(&s.mech,s.now);
    CHECK(s.mech.data.result==MECH_FAILED && s.mech.data.state==MECH_FAULT && s.mech.data.reason==MECH_LIMIT);
    init(&s); homed(&s); s.freeze_position=7u; run(&s,2200u); CHECK(!s.mech.data.reference[2]);
    init(&s); homed(&s); CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,5.0f,5.0f,s.now)==MECH_ACCEPTED);
    s.flags[6]|=4u; finish(&s,10000u); CHECK(s.mech.data.result==MECH_FAILED && !s.mech.data.reference[2]);
    init(&s); homed(&s); s.delay_ack=1u;
    CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,5.0f,5.0f,s.now)==MECH_ACCEPTED);
    run(&s,40u); CHECK(s.mech.data.result==MECH_RUNNING); finish(&s,10000u); CHECK(s.mech.data.result==MECH_SUCCESS);
    init(&s); homed(&s); CHECK(Mechanism_MoveRelative(&s.mech,MECH_TRIM,5.0f,5.0f,s.now)==MECH_ACCEPTED);
    run(&s,100u); s.bias_counts[6]=64; finish(&s,10000u);
    if (s.mech.data.reason!=MECH_FINAL_ERROR) fprintf(stderr,"final test result %u reason %s pos %.5f target %.5f bus %s\n",s.mech.data.result,
        Mechanism_ReasonName(s.mech.data.reason),s.mech.data.position[2],s.mech.data.target,MotorBus_ActionName(s.bus.nodes[6].action));
    CHECK(s.mech.data.result==MECH_FAILED && s.mech.data.reason==MECH_FINAL_ERROR);
    init(&s); wait_step(&s,MECH_SLOW_SEEK,1u);
    s.freeze_position=6u; s.raw_mode[1]=1u; run(&s,1200u);
    CHECK(!s.mech.data.reference[1] && s.mech.data.state!=MECH_PARK_RELEASE); /* STOP ACK does not supply an origin */
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.cfg.axis[1].positive_sign= -1;
    /* Plant switch mapping reversed: search cannot auto-invert or extend itself. */
    finish(&s,30000u); CHECK(s.mech.data.result==MECH_FAILED);
    init(&s); wait_step(&s,MECH_FAST_SEEK,1u); s.stuck=1u;
    for (t=0u;t<20000u && s.mech.data.result==MECH_RUNNING;++t) run(&s,1u);
    CHECK(s.mech.data.result==MECH_FAILED);
    puts("PASS: pressed boot, open/short/bounce, raw STOP, lost STOP retry, no stop bypass, gaps, enable/offline/ACK/stale/protection/stall failures");
}
static void trial_tests(void)
{
    /* LIFT seek/backoff/seek/park; ARM initially pressed, then the same four.
       Check transmitted RPM so conversion/caps cannot silently defeat 2x home. */
    static const unsigned home_rpm[]={10u,4u,4u,4u,2u,4u,2u,2u,2u};
    Sim s; MechanismConfig c; unsigned a;
    Mechanism_DefaultConfig(&c);
    for (a=0u;a<3u;++a) CHECK(!Mechanism_ConfigIssues(&c,a));
    CHECK(c.axis[0].units_per_rev==120.0 && c.axis[1].units_per_rev==2.0);
    init(&s); s.cfg=c;
    s.counts[4]= -2.0*65536.0/c.axis[0].units_per_rev;
    s.counts[5]=0.3*65536.0/c.axis[1].units_per_rev;
    s.counts[6]=8.0*65536.0/c.axis[2].units_per_rev;
    Mechanism_Init(&s.mech,&s.bus,&c,s.now); s.mech.chassis=&s.chassis;
    s.mech.auto_pending=1u;
    homed(&s);
    CHECK(s.order[0]==6u && s.order[4]==5u && !s.moves[6]);
    CHECK(s.order_count==sizeof(home_rpm)/sizeof(home_rpm[0]));
    for (a=0u;a<s.order_count;++a) CHECK(s.order_rpm[a]==home_rpm[a]);
    CHECK(Mechanism_StartAcceptance(&s.mech,s.now)==MECH_ACCEPTED);
    finish(&s,150000u);
    if (s.mech.data.reason) fprintf(stderr,"TRIAL: %s stage %u\n",Mechanism_ReasonName(s.mech.data.reason),s.mech.data.stage);
    CHECK(s.mech.data.result==MECH_SUCCESS && s.mech.data.done && s.mech.data.stage==8u);
    CHECK(s.moves[6]==4u && s.flags[5]&1u);
    puts("TRIAL CONFIG PASS: 2x homing RPM on wire, 120deg/2mm/3200 profile, power-on 6->5 home, session zero, all eight stages (synthetic mechanics only)");
}
static void measurement_fixture(Sim *s)
{
    init(s); s->mech.auto_pending=0u; s->online=0x20u;
    s->counts[5]=3.0*65536.0; /* 30 synthetic mm, beyond the old short seek. */
    /* Measurement must work without an established length or pulse conversion. */
    s->mech.config.axis[1].units_per_rev=0.0; s->mech.config.axis[1].pulses_per_rev=0u;
    run(s,3000u);
    CHECK(!s->moves[5]);
    CHECK(Mechanism_MeasureLift(&s->mech,s->now)==MECH_ACCEPTED);
}
static void measurement_tests(void)
{
    Sim s; uint32_t id; int64_t first;
    measurement_fixture(&s); id=s.mech.data.request_id;
    CHECK(Mechanism_MeasureLift(&s.mech,s.now)==MECH_BUSY && s.mech.data.request_id==id);
    finish(&s,30000u);
    CHECK(s.mech.data.lift_measure_result==MECH_SUCCESS && !s.mech.owned);
    first=s.mech.data.lift_measure_counts;
    NEAR(first,3.0*65536.0,500.0);
    CHECK(!s.moves[4] && !s.moves[6] && s.moves[5]==1u && !s.rpm[5]);
    CHECK(s.mech.data.reference[1]==MECH_UNREFERENCED);
    CHECK(!MotorBus_RenewVelocity(&s.bus,6u,s.mech.segment_request,3000u,s.now));
    CHECK(Mechanism_MeasureLift(&s.mech,s.now)==MECH_NOT_READY); /* still at TOP */
    CHECK(s.mech.data.lift_measure_result==MECH_SUCCESS && s.mech.data.lift_measure_counts==first);
    s.counts[5]=3.0*65536.0; run(&s,2000u);
    CHECK(Mechanism_MeasureLift(&s.mech,s.now)==MECH_ACCEPTED);
    finish(&s,30000u); CHECK(s.mech.data.lift_measure_result==MECH_SUCCESS);
    NEAR(s.mech.data.lift_measure_counts,first,500.0);

    measurement_fixture(&s); run(&s,3000u);
    CHECK(!MotorBus_RenewVelocity(&s.bus,6u,s.mech.segment_request+1u,3000u,s.now));
    CHECK(!MotorBus_RenewVelocity(&s.bus,6u,s.mech.segment_request,3000u,s.bus.nodes[5].deadline_ms));
    Mechanism_Stop(&s.mech,s.now); finish(&s,5000u);
    CHECK(s.mech.data.lift_measure_result==MECH_CANCELLED && !s.mech.owned && !s.rpm[5]);
    run(&s,3000u); CHECK(s.moves[5]==1u);

    measurement_fixture(&s); s.drop_stop=1u; finish(&s,30000u);
    CHECK(s.mech.data.lift_measure_result==MECH_SUCCESS && s.stops[5]>=2u);
    measurement_fixture(&s); run(&s,3000u); s.raw_mode[1]=3u; finish(&s,5000u);
    CHECK(s.mech.data.lift_measure_result==MECH_FAILED && s.mech.data.reason==MECH_LIMIT_UNSTABLE);
    measurement_fixture(&s); run(&s,3000u); s.freeze_position=6u; finish(&s,5000u);
    CHECK(s.mech.data.lift_measure_result==MECH_FAILED && !s.rpm[5]);
    measurement_fixture(&s); s.raw_mode[1]=2u; finish(&s,130000u);
    CHECK(s.mech.data.lift_measure_result==MECH_FAILED && !s.rpm[5]);
    CHECK(s.mech.data.reason==MECH_TIMEOUT || s.mech.data.reason==MECH_NOT_FOUND);
    measurement_fixture(&s); s.drop_move_ack=1u; finish(&s,6000u);
    CHECK(s.mech.data.lift_measure_result==MECH_FAILED && !s.rpm[5]);
    measurement_fixture(&s); run(&s,3000u); s.hold_stop=1u; s.raw_mode[1]=1u; run(&s,5000u);
    CHECK(s.mech.data.lift_measure_result==MECH_FAILED && s.mech.owned);
    s.hold_stop=0u; finish(&s,5000u); CHECK(!s.mech.owned && !s.rpm[5]);
    puts("PASS: lift measurement raw counts, lease renewal, only axis 6, repeat, TOP gate, STOP, lost STOP/ACK, bounce, stale position, timeout and unconfirmed stop");
}
int main(int argc, char **argv)
{
    if (argc>1 && !strcmp(argv[1],"measure")) { measurement_tests(); return 0; }
    if (argc>1 && !strcmp(argv[1],"trial")) { trial_tests(); return 0; }
    default_and_gpio(); home_and_acceptance(); conversion_and_gates(); fault_tests();
    printf("MECHANISM PASS (%u checks), sizeof(Mechanism)=%zu\n",checks,sizeof(Mechanism)); return 0;
}
