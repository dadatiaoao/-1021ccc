/* Synthetic mechanics + real MotorBus/Emm framing and incremental replies.
   config() uses synthetic fixture values; trial_tests() reads the vehicle
   profile from mechanism_config.h. Neither test establishes physical calibration. */
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
    MechanismConfig c; unsigned a; Mechanism_DefaultConfig(&c);
    for (a=0u;a<3u;++a) {
        c.axis[a].calibrated=1u; c.axis[a].pulses_per_rev=3200u;
        c.axis[a].positive_sign=1;
        c.axis[a].max_rpm=a==0u ? 20.1f : a==1u ? 60.1f : 200.1f;
        c.axis[a].move_speed=a==0u ? 40.0f : a==1u ? 40.0f : 3.333333f;
    }
    return c;
}
typedef struct {
    MotorBus bus; Mechanism mech; Chassis chassis; MechanismConfig cfg;
    uint32_t now,tx_done; uint8_t busy,bytes[128],length,online;
    uint8_t flags[7],freeze_position,drop_stop,reject_enable,hold_stop;
    uint8_t raw_mode[2],drop_move_ack,stuck_axis;
    uint8_t delayed[8],delayed_length,delay_ack;
    uint32_t reply_due; int32_t bias_counts[7];
    double counts[7],target[7]; int32_t rpm[7];
    uint8_t home_flags[7],home_phase[7],home_dir[7],native[3];
    uint8_t reject_home,drop_home_ack,drop_home_status;
    uint16_t home_rpm[7],home_backoff[7];
    unsigned homes[7],aborts[7],moves[7],stops[7],enables[7];
    unsigned order[128],order_rpm[128],order_count;
    uint32_t home_at[7],park_at[7];
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
    if (b[1]==EMM_STOP) ++s->stops[a];
    if (b[1]==EMM_ENABLE) ++s->enables[a];
    if (b[1]==EMM_HOME) { ++s->homes[a]; s->home_at[a]=s->now; }
    if (b[1]==EMM_HOME_ABORT) ++s->aborts[a];
    if (b[1]==EMM_MOVE && a>=4u && !s->park_at[a]) s->park_at[a]=s->now;
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
    } else if (b[1]==EMM_HOME_CONFIG) {
        CHECK(a>=4u && b[2]==0xAEu && !b[3] && b[4]==2u && !b[18]);
        if (s->reject_home) r[2]=0xE2u;
        s->home_dir[a]=b[5]; s->home_rpm[a]=(uint16_t)(((unsigned)b[6]<<8)|b[7]);
    } else if (b[1]==EMM_HOME_BACKOFF) {
        CHECK(a>=4u && b[2]==0xACu && !b[3]);
        s->home_backoff[a]=(uint16_t)(((unsigned)b[4]<<8)|b[5]);
    } else if (b[1]==EMM_HOME) {
        CHECK(a>=4u && b[2]==2u && !b[3]);
        s->home_phase[a]=1u; s->home_flags[a]=7u;
        s->target[a]=a==6u ? 50.0*65536.0 : 0.0;
        if (a==6u) s->target[a]/=s->cfg.axis[2].units_per_rev;
        s->rpm[a]=s->home_dir[a] ? -(int32_t)s->home_rpm[a] : s->home_rpm[a];
        if (s->drop_home_ack) return;
    } else if (b[1]==EMM_HOME_ABORT) {
        if (s->hold_stop || s->drop_stop) { if (s->drop_stop) --s->drop_stop; return; }
        s->home_phase[a]=0u; s->home_flags[a]=11u; s->rpm[a]=0; s->flags[a]|=2u;
    } else if (b[1]==EMM_HOME_STATUS) {
        if (s->drop_home_status) return;
        n=5u; r[2]=s->home_flags[a]; r[3]=s->flags[a];
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
    if (s->raw_mode[a]==1u) return 1u;
    if (s->raw_mode[a]==2u) return 0u;
    if (s->raw_mode[a]==3u) return (uint8_t)(s->now%8u<4u);
    return 0u;
}
static void run(Sim *s, uint32_t duration)
{
    uint32_t t; unsigned a;
    for (t=0u;t<duration;++t,++s->now) {
        for (a=0u;a<7u;++a) if (s->rpm[a] && s->stuck_axis!=a+1u) {
            double step=s->rpm[a]*65536.0/60000.0;
            if (a>=4u && s->home_phase[a]==1u &&
                ((s->rpm[a]<0 && s->counts[a]+step<=s->target[a]) ||
                 (s->rpm[a]>0 && s->counts[a]+step>=s->target[a]))) {
                s->counts[a]=s->target[a];
                if (s->native[a-4u]) {
                    s->home_phase[a]=2u; s->rpm[a]= -s->rpm[a];
                    s->target[a]=s->counts[a]+s->home_backoff[a]/3600.0*65536.0*
                        (s->home_dir[a] ? 1.0 : -1.0);
                } else s->rpm[a]=0;
            } else if (s->home_phase[a]==1u && !s->native[a-4u] && !s->rpm[a]) {
                /* Static contact waits for the controller's 1500 ms detector. */
            } else if (s->home_phase[a]==1u) s->counts[a]+=step;
            else if (fabs(s->target[a]-s->counts[a])<=fabs(step)) {
                s->counts[a]=s->target[a]; s->rpm[a]=0; s->flags[a]|=2u;
                if (s->home_phase[a]==2u) { s->home_phase[a]=0u; s->home_flags[a]=3u; }
            } else s->counts[a]+=step;
        }
        Mechanism_SampleLimits(&s->mech,limit(s,0u),limit(s,1u),s->now);
        Mechanism_Process(&s->mech,s->now);
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
    memset(s,0,sizeof(*s)); s->now=now; s->online=0x7Fu; s->cfg=config();
    for (a=0u;a<7u;++a) s->flags[a]=2u;
    for (a=4u;a<7u;++a) s->home_flags[a]=3u;
    for (a=0u;a<3u;++a) s->native[a]=1u;
    s->counts[4]=20.0*65536.0/s->cfg.axis[0].units_per_rev;
    s->counts[5]=20.0*65536.0/s->cfg.axis[1].units_per_rev;
    s->counts[6]=45.0*65536.0/s->cfg.axis[2].units_per_rev;
    t.context=s; t.send=send_wire; t.busy=wire_busy;
    MotorBus_Init(&s->bus,t,now); Chassis_Init(&s->chassis,&s->bus,0,now);
    Mechanism_Init(&s->mech,&s->bus,&s->cfg,now);
    s->mech.chassis=&s->chassis; s->chassis.external_busy=&s->mech.owned;
}
static void init(Sim *s) { init_at(s,0u); }
static void finish(Sim *s, uint32_t max)
{
    uint32_t t;
    for (t=0u;t<max && (s->mech.auto_pending || s->mech.owned);++t) run(s,1u);
}
static void until_home(Sim *s, uint32_t max)
{
    uint32_t t;
    for (t=0u;t<max && (s->mech.auto_pending || s->mech.owned);++t) run(s,1u);
    if (s->mech.data.result!=MECH_SUCCESS)
        fprintf(stderr,"HOME %s %s axis %u t %u segment %u counts %.1f target %.1f rpm %d action %u\n",
            Mechanism_StateName(s->mech.data.state),
            Mechanism_ReasonName(s->mech.data.reason),
            s->mech.data.axis,s->now,s->mech.segment,s->counts[6],s->target[6],s->rpm[6],s->bus.nodes[6].action);
    CHECK(s->mech.data.result==MECH_SUCCESS);
    CHECK(s->mech.data.reference[0]==MECH_HOMED &&
        s->mech.data.reference[1]==MECH_HOMED &&
        s->mech.data.reference[2]==MECH_HOMED);
    NEAR(s->mech.data.position[0],s->cfg.outside_angle,s->cfg.axis[0].tolerance);
    NEAR(s->mech.data.position[1],s->cfg.observation_mm,s->cfg.axis[1].tolerance);
    NEAR(s->mech.data.position[2],0.0,s->cfg.axis[2].tolerance);
}
static void until_state(Sim *s, MechanismState state, unsigned axis)
{
    uint32_t t;
    for (t=0u;t<50000u && !(s->mech.data.state==state &&
        s->mech.data.axis==axis);++t) run(s,1u);
    CHECK(t<50000u);
}
static void config_and_startup(void)
{
    Sim s; MechanismConfig c; unsigned i;
    BspLimit_Init(); gpio_levels[0]=0u; gpio_levels[1]=1u;
    CHECK(!BspLimit_Read(0u) && BspLimit_Read(1u));
    Mechanism_DefaultConfig(&c);
#if MECHANISM_TRIAL_RUN
    for (i=0u;i<3u;++i) CHECK(!Mechanism_ConfigIssues(&c,i));
    CHECK(c.collision[0].rpm==15u && c.collision[1].rpm==40u &&
        c.collision[2].rpm==150u && MECHANISM_NO_PROGRESS_MS==1500u);
    NEAR(c.axis[0].move_speed/c.axis[0].units_per_rev*60.0,20.0,0.01);
    NEAR(c.axis[1].move_speed/c.axis[1].units_per_rev*60.0,60.0,0.01);
    NEAR(c.axis[2].move_speed/c.axis[2].units_per_rev*60.0,200.0,0.01);
    CHECK(c.home[0].max_position==180.0f && c.observation_mm==10.0f &&
        c.grab_mm==140.0f && c.home[1].max_position>=c.grab_mm);
#else
    for (i=0u;i<3u;++i) CHECK(Mechanism_ConfigIssues(&c,i)&MECH_CFG_CONFIRM);
#endif
    init(&s); s.online=0x3Fu; run(&s,8000u);
    CHECK(s.mech.auto_pending && !s.homes[4] && !s.homes[5] && !s.homes[6]);
    s.online=0x7Fu; until_home(&s,60000u);
    CHECK(s.homes[5]==1u && s.homes[4]==1u && s.homes[6]==1u);
    CHECK(s.home_at[5]<s.home_at[4] && s.park_at[4]<s.home_at[6]);
    CHECK(s.home_rpm[5]==40u && s.home_rpm[4]==15u && s.home_rpm[6]==150u);
    CHECK(s.home_backoff[5]==900u && s.home_backoff[6]==3600u);
    CHECK(s.bus.nodes[5].max_poll_gap_ms[1]<=MECHANISM_MAX_PROGRESS_SAMPLE_GAP_MS);
    CHECK(s.bus.nodes[4].max_poll_gap_ms[1]<=MECHANISM_MAX_PROGRESS_SAMPLE_GAP_MS);
    CHECK(s.bus.nodes[6].max_poll_gap_ms[1]<=MECHANISM_MAX_PROGRESS_SAMPLE_GAP_MS);
    CHECK(s.mech.data.result==MECH_SUCCESS && !s.mech.auto_pending);
    run(&s,3000u); CHECK(s.homes[5]==1u);
    init(&s); s.online=0x3Fu; run(&s,4000u); Mechanism_Stop(&s.mech,s.now);
    run(&s,4000u); s.online=0x7Fu; run(&s,5000u);
    CHECK(!s.mech.auto_pending && !s.homes[5] && s.mech.data.result==MECH_CANCELLED);
}
static void home_paths(void)
{
    Sim s; unsigned a,mode,i; uint8_t saw_arm,saw_trim,saw_lift;
    for (a=0u;a<3u;++a) for (mode=0u;mode<2u;++mode) {
        init(&s); if (mode) s.native[a]=0u;
        if (mode && a==MECH_ARM) s.counts[4]=0.0;
        if (mode && a==MECH_LIFT) s.counts[5]=0.0;
        if (mode && a==MECH_TRIM) s.counts[6]=50.0*65536.0;
        until_home(&s,65000u);
        CHECK(s.homes[4+a]==1u);
        CHECK((s.aborts[4+a]>0u)==(mode!=0u));
        saw_arm=saw_trim=saw_lift=0u;
        for (i=0u;i<s.order_count;++i) {
            if (s.order[i]==5u && s.order_rpm[i]==20u) saw_arm=1u;
            if (s.order[i]==6u && s.order_rpm[i]==60u) saw_lift=1u;
            if (s.order[i]==7u && s.order_rpm[i]==200u) saw_trim=1u;
        }
        CHECK(saw_arm && saw_trim);
        if (mode && a==MECH_LIFT) CHECK(saw_lift);
        if (mode) {
            CHECK(s.mech.data.stop_confirm_ms>s.mech.data.stop_submit_ms);
            CHECK(s.park_at[4+a]>s.mech.data.stop_confirm_ms || a!=MECH_TRIM);
        }
        NEAR(s.counts[4]*s.cfg.axis[0].units_per_rev/65536.0,90.0,1.0);
        NEAR(s.counts[5]*s.cfg.axis[1].units_per_rev/65536.0,10.0,0.06);
        NEAR(s.counts[6]*s.cfg.axis[2].units_per_rev/65536.0,25.0,0.06);
    }
    init_at(&s,UINT32_MAX-30000u); s.native[2]=0u;
    until_home(&s,65000u); CHECK(s.aborts[6]>0u);
    init(&s); until_state(&s,MECH_CENTERING,MECH_TRIM);
    CHECK(s.mech.data.result==MECH_RUNNING && s.mech.data.reference[2]!=MECH_HOMED);
    until_home(&s,35000u);
}
static void progress_observation(void)
{
    Sim s; uint32_t start,t;
    init(&s); s.native[1]=0u; s.counts[5]=0.0;
    while (!s.mech.progress_ms && s.now<10000u) run(&s,1u);
    CHECK(s.mech.progress_ms && s.mech.data.state==MECH_COLLISION_HOME);
    start=s.mech.progress_ms;
    for (t=0u;t<800u;++t) {
        s.bias_counts[5]=(t&1u) ? 10 : -10;
        run(&s,1u);
    }
    CHECK(s.mech.data.state==MECH_COLLISION_HOME && !s.aborts[5]);
    while (s.mech.data.state!=MECH_CONTACT_STOP && s.now-start<2200u) run(&s,1u);
    CHECK(s.mech.data.state==MECH_CONTACT_STOP && s.now-start>=1500u);
    s.bias_counts[5]=0; until_home(&s,60000u);
    init(&s); s.native[1]=0u; s.counts[5]=0.0;
    while (!s.mech.progress_ms && s.now<10000u) run(&s,1u);
    CHECK(s.mech.progress_ms);
    s.bus.nodes[5].poll_due[1]=s.now+800u;
    run(&s,750u);
    CHECK(s.mech.data.state==MECH_COLLISION_HOME && !s.aborts[5]);
    run(&s,160u);
    CHECK(s.mech.data.state==MECH_COLLISION_HOME && !s.aborts[5]);
    until_home(&s,60000u);
}
static void limits_and_moves(void)
{
    Sim s; unsigned i; uint32_t id;
    for (i=0u;i<4u;++i) {
        init(&s); s.raw_mode[0]=(uint8_t)i; s.raw_mode[1]=(uint8_t)(3u-i);
        until_home(&s,60000u);
        CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,180.0f,18.0f,s.now)==MECH_ACCEPTED);
        finish(&s,15000u); CHECK(s.mech.data.result==MECH_SUCCESS);
        CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,180.1f,18.0f,s.now)==MECH_INVALID);
        CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,-0.1f,18.0f,s.now)==MECH_INVALID);
    }
    init(&s); until_home(&s,60000u);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_LIFT,140.0f,20.0f,s.now)==MECH_ACCEPTED);
    finish(&s,15000u); CHECK(s.mech.data.result==MECH_SUCCESS);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_ARM,80.0f,18.0f,s.now)==MECH_NOT_READY);
    CHECK(s.mech.data.rejection==MECH_INTERLOCK);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,1.0f,2.0f,s.now)==MECH_NOT_READY);
    CHECK(s.mech.data.rejection==MECH_INTERLOCK);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_LIFT,10.0f,20.0f,s.now)==MECH_ACCEPTED);
    finish(&s,15000u); CHECK(s.mech.data.result==MECH_SUCCESS);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,25.1f,2.0f,s.now)==MECH_INVALID);
    CHECK(Mechanism_MoveTo(&s.mech,MECH_TRIM,-25.1f,2.0f,s.now)==MECH_INVALID);
    CHECK(Mechanism_StartAcceptance(&s.mech,s.now)==MECH_ACCEPTED);
    id=s.mech.data.request_id; CHECK(Mechanism_HomeAll(&s.mech,s.now)==MECH_BUSY);
    finish(&s,65000u);
    CHECK(s.mech.data.result==MECH_SUCCESS && s.mech.data.done &&
        s.mech.data.stage==8u && s.mech.data.request_id==id);
}
static void stops_and_faults(void)
{
    Sim s; unsigned a;
    for (a=0u;a<3u;++a) {
        init(&s); s.native[a]=0u;
        until_state(&s,MECH_COLLISION_HOME,a);
        while (!s.homes[a+4u]) run(&s,1u);
        s.hold_stop=1u;
        until_state(&s,MECH_CONTACT_STOP,a);
        run(&s,4000u);
        CHECK(s.mech.data.result==MECH_FAILED && s.bus.nodes[a+4u].stop_unconfirmed);
        CHECK(!s.park_at[a+4u]);
        if (a==1u) CHECK(!s.homes[4] && !s.homes[6]);
        if (a==0u) CHECK(!s.homes[6]);
    }
    init(&s); until_state(&s,MECH_COLLISION_HOME,1u);
    while (!s.homes[5]) run(&s,1u);
    Mechanism_Stop(&s.mech,s.now); finish(&s,8000u);
    CHECK(s.mech.data.result==MECH_CANCELLED && !s.homes[4] && !s.homes[6]);
    init(&s); s.reject_enable=6u; finish(&s,20000u);
    CHECK(s.mech.data.result==MECH_FAILED && !s.homes[5] && !s.mech.auto_pending);
    init(&s); while (!s.homes[5]) run(&s,1u);
    s.flags[5]|=4u; finish(&s,10000u);
    CHECK(s.mech.data.result==MECH_FAILED && !s.homes[4]);
    init(&s); s.native[1]=0u; while (!s.homes[5]) run(&s,1u);
    s.freeze_position=6u; run(&s,3500u);
    CHECK(s.mech.data.result==MECH_FAILED && !s.park_at[5]);
}
int main(int argc, char **argv)
{
    if (argc>1 && !strcmp(argv[1],"trial")) { config_and_startup(); return 0; }
    config_and_startup(); home_paths(); progress_observation(); limits_and_moves(); stops_and_faults();
    printf("MECHANISM PASS (%u checks), sizeof(Mechanism)=%zu\n",checks,sizeof(Mechanism));
    return 0;
}
