#include "vision/vision_service.h"
#include "vision/vision_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vision_golden.h"
static unsigned checks, sends, blocked;
static uint8_t last[VISION_FRAME_MAX], last_size;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"VISION FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static int send_bytes(void *ctx,const uint8_t *bytes,uint8_t n)
{ (void)ctx; CHECK(n<=sizeof(last)); memcpy(last,bytes,n); last_size=n; ++sends; return 1; }
static int busy(void *ctx) { (void)ctx; return (int)blocked; }
static VisionTransport transport={0,send_bytes,busy};
static void frame(VisionService *s,uint8_t cmd,const uint8_t *params,uint8_t n,uint32_t ms)
{
    uint8_t bytes[VISION_FRAME_MAX];
    uint8_t size=VisionProtocol_Build(cmd,params,n,bytes);
    CHECK(size!=0u); VisionService_Receive(s,bytes,size,ms);
}
static void answer(VisionService *s,uint8_t status,uint32_t ms)
{ uint8_t p[2]; p[0]=s->pending_cmd; p[1]=status; frame(s,VISION_ACK,p,2u,ms); }
static void start(VisionService *s,uint8_t mode,uint32_t now)
{
    unsigned i;
    const uint8_t cmds[]={VISION_STOP,VISION_SET_CONTEXT,VISION_SET_COLOR,VISION_SET_TARGET,VISION_SCAN};
    VisionService_Init(s,transport,now);
    CHECK(VisionService_Start(s,mode,1u,1u,2u,2u,now));
    for (i=0u;i<5u;++i) {
        VisionService_Process(s,now+i*2u); CHECK(s->waiting && last[2]==cmds[i]);
        if (i==1u) CHECK(last_size==sizeof(gold_context) && !memcmp(last,gold_context,sizeof(gold_context)));
        answer(s,0u,now+i*2u+1u);
    }
    CHECK(s->state==VISION_ACTIVE && s->ack_count==5u);
    VisionService_Receive(s,gold_heartbeat,sizeof(gold_heartbeat),now+10u);
}
static void obs(VisionService *s,uint16_t req,uint16_t pose,uint16_t id,uint8_t flags,uint8_t target,uint16_t x,uint16_t y,uint32_t ms)
{
    uint8_t p[12];
    VisionProtocol_PutU16(p,req); VisionProtocol_PutU16(p+2,pose); VisionProtocol_PutU16(p+4,id);
    p[6]=flags; p[7]=target; VisionProtocol_PutU16(p+8,x); VisionProtocol_PutU16(p+10,y);
    frame(s,VISION_OBSERVATION,p,sizeof(p),ms);
}
static void parser_tests(void)
{
    VisionParser p;
    VisionFrame f={0};
    uint8_t bytes[16],params[12];
    unsigned n,i,k,count;
    VisionParser_Init(&p);
    CHECK(!VisionProtocol_Build(1u,0,1u,bytes));
    CHECK(!VisionProtocol_Build(1u,params,13u,bytes));
    CHECK(!VisionProtocol_Build(1u,params,1u,0));
    for (n=0u;n<=12u;++n) for(k=0u;k<50u;++k) {
        for(i=0u;i<n;++i) params[i]=(uint8_t)(i*13u+k*17u);
        if (n>1u) params[1]=0xAAu;
        CHECK(VisionProtocol_Build(0x50u,params,(uint8_t)n,bytes)==n+4u);
        for(i=0u;i<n+4u;++i) {
            int result=VisionParser_Push(&p,bytes[i],(uint32_t)(k*1000u+i),&f);
            CHECK(result==(i==n+3u));
        }
        CHECK(f.command==0x50u && f.count==n && !memcmp(f.params,params,n));
    }
    VisionParser_Init(&p);
    for(i=0u;i<sizeof(gold_found);++i) (void)VisionParser_Push(&p,gold_found[i],i,&f);
    CHECK(p.frames==1u && f.count==9u && VisionProtocol_I16(f.params+5)==-123);
    CHECK(VisionProtocol_I16((uint8_t[]){0x80,0})==-32768);
    CHECK(VisionProtocol_I16((uint8_t[]){0x7F,0xFF})==32767);
    CHECK(VisionProtocol_I16((uint8_t[]){0xFF,0xFF})==-1);
    VisionParser_Init(&p); count=0u;
    for(i=0u;i<100u;++i) if (VisionParser_Push(&p,(uint8_t)i,100u,&f)) ++count;
    for(i=0u;i<4u;++i) if (VisionParser_Push(&p,((uint8_t[]){0xAA,0,0xAA,14})[i],101u,&f)) ++count;
    for(i=0u;i<4u;++i) if (VisionParser_Push(&p,gold_heartbeat[i],102u,&f)) ++count;
    CHECK(count==1u && p.bad_lengths==2u);
    (void)VisionParser_Push(&p,0xAAu,UINT32_MAX-40u,&f);
    VisionParser_Expire(&p,59u); CHECK(!p.used && p.partial_timeouts==1u);
    for(i=0u;i<4u;++i) bytes[i]=gold_heartbeat[i]; bytes[3]^=1u;
    for(i=0u;i<4u;++i) (void)VisionParser_Push(&p,bytes[i],200u,&f);
    CHECK(p.bad_checksums==1u);
    for(i=0u;i<4u;++i) (void)VisionParser_Push(&p,gold_heartbeat[i],201u,&f);
    CHECK(p.frames==2u);
    /* Recovery finds both heartbeats inside a checksum-corrupt outer packet. */
    VisionParser_Init(&p);
    bytes[0]=0xAAu; bytes[1]=10u; bytes[2]=0xFFu;
    memcpy(bytes+3,gold_heartbeat,4u); memcpy(bytes+7,gold_heartbeat,4u);
    bytes[11]=0u; bytes[12]=0u;
    for(i=0u;i<13u;++i) {
        if (VisionParser_Push(&p,bytes[i],200u,&f)) while (VisionParser_Next(&p,&f)) {}
    }
    CHECK(p.frames==2u && p.bad_checksums==1u);
}
static void service_tests(void)
{
    VisionService s;
    unsigned i, before;
    char line[43];
    start(&s,2u,100u);
    VisionService_Receive(&s,gold_ring,sizeof(gold_ring),111u);
    CHECK(s.legacy_valid && s.legacy_cx==640u && !VisionService_TargetFresh(&s,111u));
    VisionService_Receive(&s,gold_observation,sizeof(gold_observation),112u);
    CHECK(VisionService_TargetFresh(&s,112u) && s.observation.frame_id==65535u);
    obs(&s,1u,2u,0u,3u,2u,0u,0u,113u); /* wrapping frame and real image origin */
    CHECK(VisionService_TargetFresh(&s,113u) && !s.observation.frame_id);
    obs(&s,1u,2u,0u,3u,2u,40u,50u,114u); CHECK(!s.observation.cx); /* duplicate */
    obs(&s,1u,2u,65535u,3u,2u,40u,50u,114u); CHECK(!s.observation.cx); /* older */
    obs(&s,2u,2u,1u,3u,2u,40u,50u,114u); CHECK(!s.observation.cx); /* wrong request */
    obs(&s,1u,3u,1u,3u,2u,40u,50u,114u); CHECK(!s.observation.cx); /* wrong pose */
    obs(&s,1u,2u,1u,3u,1u,40u,50u,114u); CHECK(!s.observation.cx); /* wrong station */
    obs(&s,1u,2u,1u,3u,2u,1280u,50u,114u); CHECK(s.invalid_frames==1u);
    obs(&s,1u,2u,1u,3u,2u,40u,50u,115u); CHECK(s.observation.cx==40u);
    CHECK(VisionService_TargetFresh(&s,614u)); VisionService_Process(&s,615u);
    CHECK(!VisionService_TargetFresh(&s,615u));
    for(i=0u;i<3u;++i) {
        obs(&s,1u,2u,(uint16_t)(i+2u),(uint8_t[]){0u,17u,129u}[i],2u,0u,0u,616u+i);
        CHECK(!VisionService_TargetFresh(&s,616u+i));
    }
    frame(&s,VISION_PLACE_CHECK,(uint8_t[]){1,2,1},3u,620u);
    CHECK(s.place_valid && s.place_candidate && !s.observation.valid);
    frame(&s,VISION_NOTFOUND,0,0u,621u); CHECK(!s.legacy_valid && !s.place_valid);
    obs(&s,1u,2u,40000u,3u,2u,1u,2u,622u); CHECK(!s.observation.valid); /* out of order */
    obs(&s,1u,2u,5u,3u,2u,1u,2u,623u); CHECK(s.observation.valid);
    frame(&s,VISION_NOTFOUND,0,0u,624u);
    obs(&s,1u,2u,6u,3u,2u,1u,2u,625u); CHECK(s.observation.valid); /* revoke keeps sequence */
    VisionService_Stop(&s,626u); CHECK(!s.observation.valid);
    VisionService_Process(&s,627u); CHECK(last[2]==VISION_STOP && s.waiting);
    before=sends; VisionService_Stop(&s,628u); VisionService_Process(&s,628u); CHECK(sends==before);
    answer(&s,0u,629u); CHECK(s.state==VISION_STOPPED);
    start(&s,4u,1000u);
    obs(&s,1u,2u,1u,3u,2u,640u,360u,1011u); CHECK(!s.observation.valid);
    obs(&s,1u,2u,2u,67u,2u,640u,360u,1012u); CHECK(s.observation.valid);
    VisionService_Process(&s,4010u); CHECK(s.state==VISION_FAULT && !s.online && !s.observation.valid);
    start(&s,1u,5000u);
    VisionService_Receive(&s,gold_found,sizeof(gold_found),5011u);
    CHECK(s.legacy_valid && s.offset_tenths==-123 && !s.distance_cm);
    VisionService_TransportError(&s,VISION_ERROR_UART,5012u);
    CHECK(s.state==VISION_FAULT && !s.legacy_valid && !s.parser.used);
    start(&s,0u,6000u);
    VisionService_Receive(&s,gold_task,sizeof(gold_task),6011u);
    CHECK(s.task_valid && !strcmp(s.task.text,"156+123+516+231"));
    frame(&s,VISION_TASKCODE,(uint8_t[]){1,1,1,1,2,3,1,5,6,2,3,1},12u,6012u);
    CHECK(s.invalid_frames && s.task_valid && !strcmp(s.task.text,"156+123+516+231"));
    for (i=0u;i<9u;++i) { VisionUi_Line(line,i,&s,0u,1u,1u,2u,6012u); CHECK(strlen(line)<=42u); }
    VisionService_Init(&s,transport,7000u); CHECK(!VisionService_Start(&s,5u,1u,1u,1u,0u,7000u));
    CHECK(VisionService_Start(&s,1u,1u,1u,1u,0u,7000u));
    VisionService_Process(&s,7000u); before=sends;
    answer(&s,0u,6999u); CHECK(s.waiting); /* buffered pre-command ACK */
    VisionService_Process(&s,9999u); CHECK(s.waiting && sends==before);
    VisionService_Process(&s,10000u); CHECK(s.state==VISION_FAULT && sends==before);
    CHECK(VisionService_Start(&s,1u,1u,1u,1u,0u,10001u));
    VisionService_Process(&s,10001u); answer(&s,1u,10002u);
    CHECK(s.state==VISION_FAULT && s.error==VISION_ERROR_REJECTED);
    CHECK(VisionService_Start(&s,1u,1u,1u,1u,0u,11000u));
    blocked=1u; VisionService_Process(&s,14000u); blocked=0u;
    CHECK(s.state==VISION_FAULT && s.error==VISION_ERROR_TX_BUSY);
    CHECK(VisionService_Start(&s,1u,1u,1u,1u,0u,15000u));
    VisionService_Process(&s,15000u); answer(&s,0u,18000u);
    CHECK(s.state==VISION_FAULT && s.error==VISION_ERROR_ACK_TIMEOUT); /* late ACK before Process */
    start(&s,3u,UINT32_MAX-100u);
    obs(&s,1u,2u,1u,7u,1u,640u,360u,UINT32_MAX-80u);
    CHECK(VisionService_TargetFresh(&s,418u)); VisionService_Process(&s,419u);
    CHECK(!s.observation.valid);
}
int main(void)
{
    parser_tests(); service_tests();
    printf("PASS: %u vision parser/service checks, real K230 wire fixtures\n",checks);
    return 0;
}
