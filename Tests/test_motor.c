#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor/motor_service.h"
#include "motor/motor_demo.h"
#include "motor/motor_demo_config.h"

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); return 0; } } while (0)
typedef struct {
    unsigned sends;
    int busy, reject;
    uint8_t last[16], length;
    unsigned by_code[256];
    uint8_t control_order[16];
    unsigned controls;
} FakeTransport;

static int fake_send(void *context, const uint8_t *data, uint8_t length)
{
    FakeTransport *f = (FakeTransport *)context;
    if (f->reject) return 0;
    memcpy(f->last, data, length); f->length = length; ++f->sends; ++f->by_code[data[1]]; f->busy = 1;
    if(data[1]>=0xF3u && f->controls<sizeof(f->control_order)) f->control_order[f->controls++]=data[1];
    return 1;
}
static int fake_busy(void *context) { return ((FakeTransport *)context)->busy; }
static void init(MotorService *s, FakeTransport *f)
{
    MotorTransport transport;
    memset(f, 0, sizeof(*f)); transport.context = f;
    transport.send = fake_send; transport.busy = fake_busy;
    MotorService_Init(s, transport, 1u, 0u);
}
static void capture(void *context, const EmmFrame *frame)
{
    EmmFrame *frames = (EmmFrame *)context;
    frames[0].flags++; frames[frames[0].flags] = *frame;
}

static int test_protocol(void)
{
    const uint8_t pos[] = {1,0x36,1,0xFF,0xFF,0xFF,0xFF,0x6B};
    const uint8_t speed[] = {1,0x35,1,0x03,0xE8,0x6B};
    const uint8_t status[] = {1,0x3A,0x8B,0x6B};
    const uint8_t embedded_tail[] = {1,0x36,0,0,0x6B,0,0x6B,0x6B};
    const uint8_t bad[] = {9,0x36,0,1,2,3,4,0x6B,1,0x36,2,0,0,0,0,0x6B,
                           1,0x3A,3,0xFF,1,0x99,0x6B};
    const uint8_t ack[] = {1,0xFD,2,0x6B,1,0xFD,0x9F,0x6B,1,0xF3,0xE2,0x6B,1,0xFE,0xEE,0x6B};
    const uint8_t expected_move[] = {1,0xFD,0,3,0xE8,0,0,0,0x7D,0,0,0,0x6B};
    const uint8_t expected_velocity[] = {1,0xF6,0,3,0xE8,10,0,0x6B};
    EmmParser p;
    EmmFrame frames[20] = {{0}};
    EmmCommand c;
    unsigned i;
    c = Emm_Move(1,0,1000,0,32000);
    CHECK(c.length == sizeof(expected_move) && !memcmp(c.bytes, expected_move, sizeof(expected_move)));
    c = Emm_Velocity(1,0,1000,10);
    CHECK(c.length == sizeof(expected_velocity) && !memcmp(c.bytes, expected_velocity, sizeof(expected_velocity)));
    c = Emm_Enable(1,1); CHECK(c.length == 6 && c.bytes[2] == 0xAB && c.bytes[3] == 1);
    c = Emm_Stop(1); CHECK(c.length == 5 && c.bytes[2] == 0x98);
    EmmParser_Init(&p, 1);
    for (i=0; i<sizeof(pos); ++i) EmmParser_Feed(&p,pos+i,1,capture,frames);
    CHECK(frames[0].flags == 1 && frames[1].position == -INT64_C(4294967295));
    EmmParser_Feed(&p,speed,sizeof(speed),capture,frames);
    EmmParser_Feed(&p,status,sizeof(status),capture,frames);
    CHECK(frames[2].speed == -1000 && frames[3].flags == 0x8B);
    EmmParser_Feed(&p,embedded_tail,sizeof(embedded_tail),capture,frames);
    CHECK(frames[4].position == INT64_C(0x006B006B));
    EmmParser_Feed(&p,bad,sizeof(bad),capture,frames);
    EmmParser_Feed(&p,status,sizeof(status),capture,frames);
    CHECK(frames[0].flags == 5 && p.rejected > 0);
    EmmParser_Feed(&p,ack,sizeof(ack),capture,frames);
    CHECK(frames[0].flags == 9 && frames[6].result == 2 && frames[7].result == 0x9F);
    CHECK(frames[8].result == 0xE2 && frames[9].result == 0xEE);
    for (i=1; i<sizeof(embedded_tail); ++i) {
        memset(frames,0,sizeof(frames)); EmmParser_Init(&p,1);
        EmmParser_Feed(&p,embedded_tail,i,capture,frames);
        EmmParser_Feed(&p,embedded_tail+i,sizeof(embedded_tail)-i,capture,frames);
        CHECK(frames[0].flags == 1 && frames[1].position == INT64_C(0x006B006B));
    }
    return 1;
}

static int test_service(void)
{
    MotorService s;
    FakeTransport f;
    const uint8_t pos[] = {1,0x36,0,0,1,0,0,0x6B};
    const uint8_t speed[] = {1,0x35,0,0,0,0x6B};
    const uint8_t status[] = {1,0x3A,3,0x6B};
    const uint8_t ack[] = {1,0xF3,2,0x6B};
    const uint8_t async[] = {1,0xFD,0x9F,0x6B};
    EmmCommand c = Emm_Enable(1,1);
    init(&s,&f);
    MotorService_Process(&s,5); CHECK(f.sends == 1 && f.last[1] == EMM_POSITION);
    f.busy = 0; MotorService_Process(&s,6);
    MotorService_Receive(&s,speed,sizeof(speed),6); CHECK(!s.data.valid && !s.data.speed_seq);
    MotorService_Receive(&s,pos,sizeof(pos),7); MotorService_Process(&s,10);
    CHECK(f.sends == 1); /* at least 5 ms after the previous response/TX completion */
    MotorService_Process(&s,12);
    CHECK(f.sends == 2 && f.last[1] == EMM_SPEED);
    f.busy = 0; MotorService_Receive(&s,speed,sizeof(speed),13); MotorService_Process(&s,18);
    CHECK(f.sends == 3 && f.last[1] == EMM_STATUS);
    f.busy = 0; MotorService_Receive(&s,status,sizeof(status),19); MotorService_Process(&s,20);
    CHECK(s.data.online && s.data.position_counts == 65536 && s.data.status_flags == 3);
    CHECK(MotorService_Submit(&s,&c)); CHECK(!MotorService_Submit(&s,&c));
    MotorService_Process(&s,24); CHECK(f.last[1] == EMM_ENABLE && f.sends == 4);
    f.busy = 0; MotorService_Process(&s,25);
    MotorService_Receive(&s,async,sizeof(async),26); CHECK(!s.data.command_seq);
    MotorService_Receive(&s,ack,sizeof(ack),27); CHECK(s.data.command_seq == 1 && s.data.command_result == 2);
    MotorService_Process(&s,700); CHECK(!s.data.online);
    init(&s,&f); f.reject=1; MotorService_Process(&s,5); CHECK(!f.sends && !s.transmitted);
    f.reject=0; MotorService_Process(&s,6); CHECK(f.sends == 1);
    f.busy=0; MotorService_Process(&s,7); MotorService_Process(&s,107);
    CHECK(s.data.timeouts == 1 && f.sends == 1);
    MotorService_Receive(&s,pos,sizeof(pos),108); CHECK(s.pending && !s.transmitted);
    MotorService_Process(&s,206); CHECK(f.sends == 1);
    MotorService_Process(&s,207); CHECK(f.sends == 2);
    f.busy=0; MotorService_Process(&s,208); MotorService_Process(&s,308);
    MotorService_Process(&s,408); CHECK(f.sends == 3);
    f.busy=0; MotorService_Process(&s,409); MotorService_Process(&s,509);
    CHECK(s.data.timeouts == 3 && !s.pending);
    init(&s,&f); CHECK(MotorService_Submit(&s,&c));
    MotorService_Process(&s,5); f.busy=0; MotorService_Process(&s,6);
    MotorService_Process(&s,106); CHECK(s.data.command_result == EMM_ACK_TIMEOUT);
    MotorService_Process(&s,206); CHECK(f.last[1] == EMM_POSITION && f.sends == 2);
    MotorService_Receive(&s,ack,sizeof(ack),207); CHECK(s.data.command_seq == 1 && s.pending);
    MotorService_TransportError(&s,MOTOR_ERROR_RX_OVERFLOW,208);
    CHECK(s.data.last_error == MOTOR_ERROR_RX_OVERFLOW && s.data.valid == 0);
    init(&s,&f); MotorService_Process(&s,5); MotorService_Process(&s,25);
    CHECK(s.data.last_error == MOTOR_ERROR_UART);
    init(&s,&f); f.reject=1; MotorService_Process(&s,5); MotorService_Process(&s,105);
    CHECK(s.data.last_error == MOTOR_ERROR_UART && !s.pending);
    f.reject=0; MotorService_Process(&s,205);
    f.busy=0; MotorService_Receive(&s,speed,sizeof(speed),206);
    MotorService_Process(&s,211); f.busy=0;
    MotorService_Receive(&s,status,sizeof(status),212);
    MotorService_Process(&s,305); f.busy=0;
    MotorService_Receive(&s,pos,sizeof(pos),306);
    MotorService_Process(&s,307); CHECK(s.data.online);
    init(&s,&f);
    MotorService_Init(&s,s.transport,1u,UINT32_MAX-10u);
    MotorService_Process(&s,UINT32_MAX-10u); f.busy=0;
    MotorService_Receive(&s,pos,sizeof(pos),UINT32_MAX-9u);
    MotorService_Process(&s,UINT32_MAX-4u); f.busy=0;
    MotorService_Receive(&s,speed,sizeof(speed),UINT32_MAX-3u);
    MotorService_Process(&s,2u); f.busy=0;
    MotorService_Receive(&s,status,sizeof(status),3u);
    MotorService_Process(&s,4u); CHECK(s.data.online && f.sends==3u);
    return 1;
}

typedef struct {
    unsigned handled;
    int online, drop_velocity_ack, drop_position_ack, reject_enable, refuse_stop, never_arrive;
    int refuse_enable, pretend_move;
    uint8_t flags;
    int32_t rpm;
    int64_t position;
    int64_t move_from, move_to;
    uint32_t move_start, move_end;
} SimMotor;

static void sim_reply(MotorService *s, FakeTransport *f, SimMotor *m, uint32_t now)
{
    uint8_t b[8] = {1,0,0,0,0,0,0,0};
    uint32_t magnitude;
    uint8_t length = 4u;
    if (m->handled == f->sends) return;
    m->handled = f->sends; f->busy=0;
    if (!m->online) return;
    b[1]=f->last[1]; b[2]=2u;
    switch (b[1]) {
    case EMM_POSITION:
        length=8u; b[2]=(uint8_t)(m->position<0);
        magnitude=(uint32_t)(m->position<0 ? -m->position : m->position);
        b[3]=(uint8_t)(magnitude>>24); b[4]=(uint8_t)(magnitude>>16);
        b[5]=(uint8_t)(magnitude>>8); b[6]=(uint8_t)magnitude; break;
    case EMM_SPEED:
        length=6u; b[2]=(uint8_t)(m->rpm<0);
        magnitude=(uint32_t)(m->rpm<0 ? -m->rpm : m->rpm);
        b[3]=(uint8_t)(magnitude>>8); b[4]=(uint8_t)magnitude; break;
    case EMM_STATUS: b[2]=m->flags; break;
    case EMM_ENABLE:
        if (m->reject_enable && f->last[3]) b[2]=0xE2u;
        else if(!m->refuse_enable || !f->last[3]) m->flags=(uint8_t)((m->flags&0xFEu) | f->last[3]);
        break;
    case EMM_VELOCITY:
        m->rpm=1000; m->flags &= 0xFDu;
        if (m->drop_velocity_ack) return;
        break;
    case EMM_STOP:
        if (!m->refuse_stop) { m->rpm=0; m->flags |= 2u; m->move_end=0; }
        break;
    case EMM_MOVE:
        if(m->pretend_move) break;
        magnitude=((uint32_t)f->last[6]<<24) | ((uint32_t)f->last[7]<<16) |
                  ((uint32_t)f->last[8]<<8) | f->last[9];
        m->move_from=m->position;
        m->move_to=m->position+(int64_t)magnitude*65536/3200;
        m->move_start=now; m->move_end=now+600;
        m->rpm=1000; m->flags &= 0xFDu;
        if(m->drop_position_ack) return;
        break;
    default: break;
    }
    b[length-1u]=0x6Bu;
    MotorService_Receive(s,b,length,now);
}

static void sim_tick(MotorService *s, FakeTransport *f, SimMotor *m, MotorDemo *d, uint32_t now)
{
    if(m->move_end) {
        if(now >= m->move_end) {
            m->position=m->move_to; m->rpm=0;
            if(!m->never_arrive) { m->flags |= 2u; m->move_end=0; }
        } else m->position=m->move_from+(m->move_to-m->move_from)*(now-m->move_start)/600;
    }
    sim_reply(s,f,m,now);
    MotorService_Process(s,now);
    MotorDemo_Process(d,s,now);
}

static int test_demo(void)
{
    MotorService s;
    FakeTransport f;
    SimMotor m;
    MotorDemo d;
    uint32_t now;
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1;
    MotorDemo_Init(&d,0);
    for(now=0;now<500;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(f.by_code[EMM_ENABLE]==0 && f.by_code[EMM_VELOCITY]==0);
    for(;now<5000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_DONE && m.rpm==0 && !(m.flags&1u));
    CHECK(f.by_code[EMM_ENABLE]==2 && f.by_code[EMM_VELOCITY]==1 && f.by_code[EMM_STOP]==2);
    CHECK(f.by_code[EMM_MOVE]==1 && d.zero_samples==3 && d.position_changed);
    CHECK(m.position==INT64_C(655360));
    { const uint8_t order[]={0xF3,0xF6,0xFE,0xFD,0xFE,0xF3};
      CHECK(f.controls==sizeof(order) && !memcmp(f.control_order,order,sizeof(order))); }
    for(;now<7000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(f.by_code[EMM_VELOCITY]==1 && d.state==DEMO_DONE);
    init(&s,&f); memset(&m,0,sizeof(m)); MotorDemo_Init(&d,0);
    for(now=0;now<6500;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && !d.cleanup_confirmed && !f.by_code[EMM_VELOCITY]);
    m.online=1;
    for(;now<8500;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && !f.by_code[EMM_VELOCITY]);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.drop_velocity_ack=1; MotorDemo_Init(&d,0);
    for(now=0;now<5000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && d.cleanup_confirmed && f.by_code[EMM_VELOCITY]==1);
    CHECK(f.by_code[EMM_STOP]==1 && !(m.flags&1u));
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.reject_enable=1; MotorDemo_Init(&d,0);
    for(now=0;now<5000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && d.fault==MOTOR_ERROR_REJECTED && !f.by_code[EMM_VELOCITY]);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; MotorDemo_Init(&d,0);
    for(now=0;now<5000;now++) {
        if(d.state==DEMO_SPEED_RUN) m.flags |= 8u;
        sim_tick(&s,&f,&m,&d,now);
    }
    CHECK(d.state==DEMO_FAULT && d.fault==MOTOR_ERROR_STALL && f.by_code[EMM_STOP]==1);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.refuse_stop=1; MotorDemo_Init(&d,0);
    for(now=0;now<8000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && d.fault==MOTOR_ERROR_MOTION_TIMEOUT && !d.cleanup_confirmed);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.never_arrive=1; MotorDemo_Init(&d,0);
    for(now=0;now<15000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && d.fault==MOTOR_ERROR_MOTION_TIMEOUT && f.by_code[EMM_MOVE]==1);
    CHECK(d.cleanup_confirmed && f.by_code[EMM_STOP]==2);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.drop_position_ack=1; MotorDemo_Init(&d,0);
    for(now=0;now<6000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && f.by_code[EMM_MOVE]==1 && f.by_code[EMM_VELOCITY]==1);
    CHECK(d.cleanup_confirmed);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.refuse_enable=1; MotorDemo_Init(&d,0);
    for(now=0;now<5000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && !f.by_code[EMM_VELOCITY] && d.fault==MOTOR_ERROR_TIMEOUT);
    init(&s,&f); memset(&m,0,sizeof(m)); m.online=1; m.pretend_move=1; MotorDemo_Init(&d,0);
    for(now=0;now<15000;now++) sim_tick(&s,&f,&m,&d,now);
    CHECK(d.state==DEMO_FAULT && !d.position_changed && f.by_code[EMM_MOVE]==1);
    init(&s,&f); MotorDemo_Init(&d,0); d.state=DEMO_WAIT_ZERO;
    s.data.online=1; s.data.valid=7; s.data.status_flags=1;
    for(now=0;now<100;now++) MotorDemo_Process(&d,&s,now);
    CHECK(d.zero_samples==0 && d.state==DEMO_WAIT_ZERO);
    s.data.speed_seq=1; s.data.speed_ms=100; MotorDemo_Process(&d,&s,100);
    MotorDemo_Process(&d,&s,101); CHECK(d.zero_samples==1);
    s.data.speed_seq=2; s.data.speed_rpm=6; MotorDemo_Process(&d,&s,102); CHECK(d.zero_samples==0);
    s.data.speed_rpm=0;
    for(now=103;now<106;now++) { ++s.data.speed_seq; s.data.speed_ms=now; MotorDemo_Process(&d,&s,now); }
    CHECK(d.state==DEMO_SEND_MOVE && d.zero_samples==3);
    return 1;
}

int main(void)
{
    if (!test_protocol() || !test_service() || !test_demo()) return 1;
    printf("PASS: %u checks (protocol + service + automatic demo)\n", checks);
    return 0;
}
