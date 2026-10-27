/* Scheduler verification with synthetic completion events; no hardware motion. */
#include "mission/mission.h"
#include "mission/mission_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"MISSION FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static const uint8_t perms[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
typedef struct {
    QrTaskCode code;
    MissionAction action;
    uint32_t block;
    unsigned begun, stopped, hold, fail_begin, fail_poll, invalid_qr;
    unsigned puts[2], takes[2], stores[2];
} Fixture;
static uint32_t blocked(void *ctx,uint32_t now)
{ (void)now; return ((Fixture *)ctx)->block; }
static int begin(void *ctx,const MissionAction *a,uint32_t now)
{
    Fixture *f=(Fixture *)ctx;
    (void)now;
    CHECK(a->id>f->action.id && a->batch<2u && a->order<3u);
    f->action=*a; ++f->begun;
    if (a->step==MISSION_SOURCE_TO_SLOT || a->step==MISSION_SLOT_TO_ROUGH ||
        a->step==MISSION_ROUGH_TO_SLOT || a->step==MISSION_SLOT_TO_STORAGE) {
        unsigned i;
        CHECK(a->slot==a->order+1u && a->color==f->code.colors[a->batch][a->order]);
        if (a->step==MISSION_SLOT_TO_STORAGE) {
            for (i=0u;i<3u;++i) if (f->code.colors[0][i]==a->color) break;
            CHECK(i<3u && a->position==f->code.positions[0][i] && a->layer==a->batch+1u);
        } else CHECK(a->position==f->code.positions[a->batch][a->order] && !a->layer);
        if (a->step==MISSION_ROUGH_TO_SLOT) CHECK(f->puts[a->batch]==3u);
    }
    return !f->fail_begin;
}
static MissionActionResult poll(void *ctx,uint32_t id,QrTaskCode *code,uint32_t now)
{
    Fixture *f=(Fixture *)ctx;
    const MissionAction *a=&f->action;
    (void)now; CHECK(id==a->id);
    if (f->hold) return MISSION_WAIT;
    if (f->fail_poll) return MISSION_ACTION_FAILED;
    if (a->step==MISSION_READ_QR) { *code=f->code; if (f->invalid_qr) code->text[0]='9'; }
    if (a->step==MISSION_SLOT_TO_ROUGH) ++f->puts[a->batch];
    if (a->step==MISSION_ROUGH_TO_SLOT) ++f->takes[a->batch];
    if (a->step==MISSION_SLOT_TO_STORAGE) ++f->stores[a->batch];
    return MISSION_SUCCEEDED;
}
static void stop(void *ctx,uint32_t now)
{ (void)now; ++((Fixture *)ctx)->stopped; }
static void init(Mission *m,Fixture *f)
{
    MissionPort port={f,blocked,begin,poll,stop};
    memset(f,0,sizeof(*f));
    CHECK(QrTaskCode_Parse((const uint8_t *)"136+231+613+123",15u,&f->code)==QR_CODE_OK);
    Mission_Init(m,port);
}
static void permutations(void)
{
    Mission m; Fixture f;
    unsigned x,y,z,w,c1,c2,c3,j,runs=0u;
    uint8_t colors[3];
    for (c1=1u;c1<=4u;++c1) for (c2=c1+1u;c2<=5u;++c2) for (c3=c2+1u;c3<=6u;++c3) {
        colors[0]=(uint8_t)c1; colors[1]=(uint8_t)c2; colors[2]=(uint8_t)c3;
        for (x=0u;x<6u;++x) for (y=0u;y<6u;++y) for (z=0u;z<6u;++z) for (w=0u;w<6u;++w) {
            char text[16]; uint32_t now=0u;
            init(&m,&f);
            for (j=0u;j<3u;++j) {
                text[j]=(char)('0'+colors[perms[x][j]]); text[4u+j]=(char)('1'+perms[y][j]);
                text[8u+j]=(char)('0'+colors[perms[z][j]]); text[12u+j]=(char)('1'+perms[w][j]);
            }
            text[3]=text[7]=text[11]='+'; text[15]='\0';
            CHECK(QrTaskCode_Parse((uint8_t *)text,15u,&f.code)==QR_CODE_OK);
            CHECK(Mission_Start(&m,now)); CHECK(!Mission_Start(&m,now));
            while (Mission_IsRunning(&m) && now<1000u) Mission_Process(&m,now++);
            CHECK(m.state==MISSION_FINISHED && f.begun==34u && m.completed_actions==34u);
            CHECK(m.grasp_count==12u && m.place_count==12u && !f.stopped);
            CHECK(f.puts[0]==3u && f.puts[1]==3u && f.takes[0]==3u && f.takes[1]==3u);
            CHECK(f.stores[0]==3u && f.stores[1]==3u);
            CHECK(!m.slots[0] && !m.slots[1] && !m.slots[2] && !m.rough[0] && !m.rough[1] && !m.rough[2]);
            for (j=0u;j<3u;++j) CHECK(m.storage[f.code.positions[0][j]-1u]==f.code.colors[0][j]);
            CHECK(!Mission_Start(&m,now)); Mission_Stop(&m,now); Mission_Process(&m,now);
            CHECK(m.state==MISSION_FINISHED && f.begun==34u); ++runs;
        }
    }
    printf("PASS: %u complete two-batch schedules; same-color stacking and 3-put barrier\n",runs);
}
static void failures(void)
{
    Mission m; Fixture f;
    unsigned i,mode;
    uint32_t now;
    init(&m,&f); f.block=MISSION_BLOCK_ROUTES|MISSION_BLOCK_MECHANISM;
    Mission_Process(&m,0u); CHECK(m.blocked==6u && !Mission_Start(&m,0u));
    CHECK(!m.started && !f.begun && !f.stopped);
    f.block=0u; CHECK(Mission_Start(&m,1u));
    for (mode=0u;mode<3u;++mode) for (i=0u;i<34u;++i) {
        init(&m,&f); CHECK(Mission_Start(&m,0u)); now=0u;
        while (m.completed_actions<i) { Mission_Process(&m,now++); CHECK(now<1000u); }
        if (mode==0u) f.fail_begin=1u;
        else f.hold=1u;
        Mission_Process(&m,now++);
        if (mode==1u) { f.hold=0u; f.fail_poll=1u; Mission_Process(&m,now++); }
        if (mode==2u) Mission_Stop(&m,now++);
        CHECK(m.state==(mode==2u ? MISSION_ABORTED : MISSION_FAULT));
        CHECK(f.stopped==1u && m.completed_actions==i && !Mission_Start(&m,now));
        Mission_Process(&m,now); Mission_Stop(&m,now);
        CHECK(f.stopped==1u);
    }
    init(&m,&f); CHECK(Mission_Start(&m,0u)); Mission_Process(&m,0u); f.hold=1u;
    Mission_Process(&m,89999u); CHECK(m.state==MISSION_RUNNING && !m.completed_actions);
    Mission_Process(&m,90000u); CHECK(m.error==MISSION_TIMEOUT && f.stopped==1u);
    init(&m,&f); f.invalid_qr=1u; CHECK(Mission_Start(&m,0u));
    for (now=0u;now<4u;++now) Mission_Process(&m,now);
    CHECK(m.error==MISSION_BAD_TASK && !m.task_valid && f.begun==2u && f.stopped==1u);
    init(&m,&f); now=UINT32_MAX-100u; CHECK(Mission_Start(&m,now)); Mission_Process(&m,now);
    f.hold=1u; Mission_Process(&m,now+89999u); CHECK(m.state==MISSION_RUNNING);
    Mission_Process(&m,now+90000u); CHECK(m.error==MISSION_TIMEOUT);
    puts("PASS: blocked/repeated starts, all 34 action failures and STOP boundaries, timeout/wrap, bad QR");
}
static void snapshots(void)
{
    Mission m; Fixture f;
    unsigned i; char text[43];
    init(&m,&f); m.blocked=63u; m.grasp_count=m.place_count=UINT32_MAX;
    for (i=0u;i<9u;++i) { MissionUi_Line(text,i,&m,UINT32_MAX); CHECK(strlen(text)<=42u); }
    MissionUi_Line(text,0u,&m,0u); CHECK(strstr(text,"ZONE 2")!=0);
    CHECK(MISSION_START_ZONE==2u);
}
int main(void)
{
    failures(); permutations(); snapshots();
    printf("PASS: %u mission assertions (synthetic completions only)\n",checks); return 0;
}
