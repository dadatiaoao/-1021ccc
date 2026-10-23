#include "flow/flow_service.h"
#include "flow/flow_config.h"
#include "flow_init_fixture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FLOW FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
typedef struct {
    uint32_t us, ms, last_cs, address_at, data_at, last_end;
    uint32_t writes_at[100]; uint8_t writes[100][2];
    unsigned writes_n, reads, bursts, exchanges, init_n, phase, fail_at;
    uint8_t high, address, page, id, inv, revision, burst[12];
    int init_ok, delay_ok;
} Fake;
static int init(void *ctx) { Fake *f=ctx; ++f->init_n; f->high=1u; f->page=0u; return f->init_ok; }
static void select_cs(void *ctx,uint8_t high)
{
    Fake *f=ctx;
    if (!high) { CHECK(f->high); f->last_cs=f->us; f->phase=0u; }
    else if (!f->high && f->phase==2u) CHECK(f->us-f->data_at>=50u);
    f->high=high; if (high) f->last_end=f->us;
}
static int delay_us(void *ctx,uint32_t us) { Fake *f=ctx; f->us+=us; return f->delay_ok; }
static uint32_t cycles(void *ctx) { return ((Fake *)ctx)->us; }
static int exchange(void *ctx,const uint8_t *tx,uint8_t *rx,size_t n)
{
    Fake *f=ctx; size_t i;
    CHECK(!f->high); CHECK(n>0u && n<=12u);
    ++f->exchanges;
    if (f->fail_at && f->exchanges==f->fail_at) return 0;
    if (f->phase==0u) {
        CHECK(n==1u && f->us-f->last_cs>=50u);
        f->address=tx[0]; f->address_at=f->us; f->phase=1u; rx[0]=0u;
    } else {
        CHECK(f->phase==1u);
        if (f->address&0x80u) {
            CHECK(n==1u && f->writes_n<100u);
            f->writes[f->writes_n][0]=(uint8_t)(f->address&0x7Fu);
            f->writes[f->writes_n][1]=tx[0]; f->writes_at[f->writes_n++]=f->ms;
            if ((f->address&0x7Fu)==0x7Fu) f->page=tx[0];
            rx[0]=0u;
        } else {
            CHECK(f->us-f->address_at>=50u); ++f->reads;
            for (i=0;i<n;++i) CHECK(tx[i]==0u);
            if (f->address==0x16u) { CHECK(n==12u); memcpy(rx,f->burst,n); ++f->bursts; }
            else {
                CHECK(n==1u && f->page==0u);
                rx[0]=f->address==0x00u ? f->id : f->address==0x5Fu ? f->inv :
                      f->address==0x01u ? f->revision : 0u;
            }
        }
        f->data_at=f->us; f->phase=2u;
    }
    f->us+=(uint32_t)n*12u; return 1;
}
static FlowIo io(Fake *f)
{ FlowIo v={f,init,select_cs,exchange,delay_us,cycles,1u}; return v; }
static void make_fake(Fake *f)
{
    static const uint8_t b[12]={0xB0,0x3F,0,0,0,0,50,40,80,10,1,2};
    memset(f,0,sizeof(*f)); f->id=0x49; f->inv=0xB6; f->revision=7u;
    f->init_ok=f->delay_ok=1; f->high=1u; memcpy(f->burst,b,12u);
}
static void step(FlowService *s,Fake *f,uint32_t now)
{ f->ms=now; Flow_Process(s,now); }
static uint32_t ready(FlowService *s,Fake *f,uint32_t start)
{
    uint32_t t; Flow_Init(s,io(f),start);
    for (t=0;t<1000u && s->data.state==FLOW_INIT;++t) step(s,f,start+t);
    CHECK(s->data.state==FLOW_READY && !s->data.valid && s->data.segments==1u);
    return start+t-1u;
}
static void deltas(Fake *f,int x,int y)
{
    f->burst[0]=(uint8_t)((x||y) ? 0xB0 : 0x30);
    f->burst[2]=(uint8_t)x; f->burst[3]=(uint8_t)((uint16_t)x>>8);
    f->burst[4]=(uint8_t)y; f->burst[5]=(uint8_t)((uint16_t)y>>8);
}
static int32_t mapped_x(int x,int y) { (void)x; (void)y; return FLOW_X_SIGN*(FLOW_SWAP_XY?y:x); }
static int32_t mapped_y(int x,int y) { (void)x; (void)y; return FLOW_Y_SIGN*(FLOW_SWAP_XY?x:y); }
static void test_init_and_decode(void)
{
    Fake f; FlowService s; PmwMotion m; uint8_t b[12]={0}; unsigned i,j=0; uint32_t at;
    make_fake(&f); at=ready(&s,&f,0u);
    CHECK(at>=198u); CHECK(f.bursts==1u && f.writes_n==74u);
    CHECK(f.writes[0][0]==0x3Au && f.writes[0][1]==0x5Au);
    for (i=0;i<sizeof(expected)/sizeof(expected[0]);++i) {
        if (expected[i][2]) {
            CHECK(f.writes_at[j+1u]-f.writes_at[j]>=expected[i][2]);
        } else {
            ++j; CHECK(f.writes[j][0]==expected[i][0] && f.writes[j][1]==expected[i][1]);
        }
    }
    CHECK(j+1u==f.writes_n && s.data.revision==7u);
    for (i=0u;i<65536u;++i) {
        int v=i<32768u ? (int)i : (int)i-65536;
        b[2]=(uint8_t)i; b[3]=(uint8_t)(i>>8); b[4]=b[2]; b[5]=b[3]; b[10]=0x12;b[11]=0x34;
        Pmw_Decode(b,&m); CHECK(m.dx==v && m.dy==v && m.shutter==0x1234u);
    }
}
static void test_service(void)
{
    Fake f; FlowService s; FlowSnapshot d; uint32_t t; unsigned n,i; int64_t x,y; char text[21];
    make_fake(&f); t=ready(&s,&f,0u);
    deltas(&f,-32768,32767); step(&s,&f,t+=10u);
    CHECK(s.data.valid && s.data.total_x==mapped_x(-32768,32767) && s.data.total_y==mapped_y(-32768,32767));
    x=s.data.total_x;y=s.data.total_y;n=f.bursts;
    for (i=0;i<100;++i) { Flow_GetSnapshot(&s,t,&d); step(&s,&f,t); }
    CHECK(d.sequence==1u && s.data.total_x==x && s.data.total_y==y && f.bursts==n);
    Flow_ResetTotals(&s); CHECK(s.data.total_x==0 && s.data.clears==1u && s.data.zero_sequence==1u);
    deltas(&f,0,0); step(&s,&f,t+=10u); CHECK(s.data.valid && s.data.accepted==2u);
    /* Shutter is retained as a raw 16-bit field; do not invent a width limit. */
    f.burst[10]=0xFFu; f.burst[11]=0xFFu; step(&s,&f,t+=10u); CHECK(s.data.valid && s.data.raw.shutter==0xFFFFu);
    f.burst[10]=1u; f.burst[11]=2u;
    Flow_GetSnapshot(&s,t+101u,&d); CHECK(!d.valid && !d.fresh && !strcmp(Flow_StateName(&d),"STALE"));
    f.burst[6]=0u; deltas(&f,123,-12); step(&s,&f,t+=10u);
    CHECK(!s.data.valid && s.data.error==FLOW_LOW_QUALITY && s.data.total_x==0);
    CHECK(s.data.last_rejection==FLOW_LOW_QUALITY && s.data.last_rejected_ms==t && s.data.last_rejected_quality==0u);
    f.burst[6]=50u; f.burst[0]=0x30; step(&s,&f,t+=10u); CHECK(s.data.error==FLOW_STATUS);
    deltas(&f,1,-1); step(&s,&f,t+=120u); CHECK(s.data.error==FLOW_GAP && s.data.missed>=11u);
    step(&s,&f,t+=10u); CHECK(s.data.valid); x=s.data.total_x; y=s.data.total_y;
    CHECK(s.data.last_rejection==FLOW_GAP && s.data.last_rejected_ms==t-10u);
    f.fail_at=f.exchanges+2u; step(&s,&f,t+=10u);
    CHECK(s.data.state==FLOW_ERROR && !s.data.valid && f.high && s.data.total_x==x && s.data.total_y==y);
    CHECK(s.data.last_fault==FLOW_SPI && s.data.errors==1u);
    n=f.init_n; step(&s,&f,t+999u); CHECK(f.init_n==n);
    step(&s,&f,t+=1000u); CHECK(f.init_n==n+1u && s.data.state==FLOW_INIT);
    f.writes_n=0u;
    for (i=1u;i<500u && s.data.state==FLOW_INIT;++i) step(&s,&f,t+i);
    CHECK(s.data.state==FLOW_READY && s.data.segments==2u && s.data.total_x==x);
    t+=i+10u; step(&s,&f,t); CHECK(s.data.valid && s.data.total_x==x+mapped_x(1,-1));
    f.id=0u; step(&s,&f,t+=500u); CHECK(s.data.state==FLOW_ERROR && s.data.error==FLOW_ID);
    Flow_FormatCount(text,INT64_MIN); CHECK(!strcmp(text,"-9223372036854775808"));
    Flow_FormatCount(text,INT64_MAX); CHECK(!strcmp(text,"9223372036854775807"));
    Flow_FormatCount(text,0); CHECK(!strcmp(text,"0"));
    make_fake(&f); t=ready(&s,&f,UINT32_MAX-100u); deltas(&f,7,-8); step(&s,&f,t+=10u);
    CHECK(s.data.valid && s.data.interval_ms==10u);
    s.data.total_x=INT64_MAX; s.data.total_y=INT64_MIN;
    deltas(&f,FLOW_SWAP_XY ? -FLOW_Y_SIGN : FLOW_X_SIGN,FLOW_SWAP_XY ? FLOW_X_SIGN : -FLOW_Y_SIGN);
    step(&s,&f,t+=10u); CHECK(s.data.overflow && !s.data.valid && s.data.total_x==INT64_MAX && s.data.total_y==INT64_MIN);
    Flow_ResetTotals(&s); step(&s,&f,t+=10u); CHECK(!s.data.overflow && s.data.valid);
    printf("FLOW simulated max process %lu us; sample interval %lu ms\n",(unsigned long)s.data.max_process_us,(unsigned long)s.data.interval_ms);
}
static void test_faults(void)
{
    Fake f; FlowService s; FlowSnapshot d; uint32_t t; unsigned i;
    make_fake(&f); f.init_ok=0; Flow_Init(&s,io(&f),0u); CHECK(s.data.state==FLOW_ERROR);
    CHECK(s.retry_ms==2000u); step(&s,&f,999u); CHECK(f.init_n==1u);
    step(&s,&f,1000u); CHECK(f.init_n==2u && s.due==3000u);
    step(&s,&f,1000u); CHECK(f.init_n==2u);
    step(&s,&f,3000u); CHECK(f.init_n==3u && s.due==7000u);
    step(&s,&f,7000u); CHECK(f.init_n==4u && s.due==15000u && s.retry_ms==8000u);
    step(&s,&f,15000u); CHECK(f.init_n==5u && s.due==23000u && s.retry_ms==8000u);
    make_fake(&f); f.id=0u; Flow_Init(&s,io(&f),0u);
    for (i=0;i<60;++i) step(&s,&f,i);
    CHECK(s.data.error==FLOW_ID && !s.data.comm_ok);
    make_fake(&f); f.inv=0xFFu; Flow_Init(&s,io(&f),0u);
    for (i=0;i<60;++i) step(&s,&f,i);
    CHECK(s.data.error==FLOW_ID);
    /* Every initialisation SPI exchange can fail; never publish a sample. */
    for (i=1;i<=172u;++i) {
        unsigned j; make_fake(&f); f.fail_at=i; Flow_Init(&s,io(&f),0u);
        for (j=0;j<500u && s.data.state==FLOW_INIT;++j) step(&s,&f,j);
        if (f.exchanges>=i) CHECK(s.data.state==FLOW_ERROR && !s.data.valid && f.high);
    }
    make_fake(&f); t=ready(&s,&f,0u); memset(f.burst,0,12u); step(&s,&f,t+=10u);
    CHECK(s.data.error==FLOW_BUS_DATA && !s.data.valid);
    make_fake(&f); t=ready(&s,&f,0u); memset(f.burst,0xFF,12u); step(&s,&f,t+=10u);
    CHECK(s.data.error==FLOW_BUS_DATA);
    make_fake(&f); t=ready(&s,&f,0u); f.burst[6]=1u;
    for (i=0;i<200u;++i) step(&s,&f,t+=10u);
    CHECK(s.data.state==FLOW_ERROR && s.data.error==FLOW_NO_VALID);
    Flow_GetSnapshot(&s,t,&d); CHECK(!d.valid && !d.comm_ok);
    make_fake(&f); f.delay_ok=0; Flow_Init(&s,io(&f),0u);
    for (i=0;i<50u;++i) step(&s,&f,i);
    CHECK(s.data.error==FLOW_SPI && f.high);
}
int main(void)
{
    test_init_and_decode(); test_service(); test_faults();
    printf("FLOW PASS %u checks\n",checks); return 0;
}
