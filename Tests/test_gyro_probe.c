#include "gyro_fixture.h"
#include "gyro/gyro_config.h"
#include "gyro/gyro_ui.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"PROBE line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static void step(GyroService *s,GyroMock *m,uint32_t ms)
{
    unsigned tx=m->transactions;
    m->now=ms; Gyro_Process(s,ms,0u);
    CHECK(m->transactions-tx<=1u);
    CHECK(!s->data.raw_valid && !s->data.rate_valid && !s->data.angle_valid);
    CHECK(!m->powered && !m->burst_reads && m->initializations==1u);
}
static void tick(GyroService *s,GyroMock *m) { step(s,m,s->due); }
static void setup(GyroService *s,GyroMock *m,uint32_t now)
{
    GyroMock_Reset(m,now); Gyro_Init(s,GyroMock_Io(m),now);
    CHECK(s->data.probe.mode==GYRO_COMM_TEST && !m->transactions);
    step(s,m,now+2u); CHECK(!m->transactions);
}
static void baseline(GyroService *s,GyroMock *m)
{
    unsigned i;
    for (i=0u;i<GYRO_COMM_TEST_READS;++i) tick(s,m);
    CHECK(s->data.probe.before.total==GYRO_COMM_TEST_READS);
    CHECK(!m->writes);
}
int main(void)
{
    GyroService s; GyroMock m; GyroSnapshot d; char line[43]; unsigned i;
    setup(&s,&m,UINT32_MAX-15u); baseline(&s,&m);
    CHECK(s.data.probe.before.ok==GYRO_COMM_TEST_READS);
    CHECK(!Gyro_RequestCalibration(&s,m.now,0u) && !Gyro_ZeroAngle(&s,m.now));
    if (GYRO_COMM_TEST==2) {
        CHECK(s.data.probe.phase==GYRO_PROBE_BANK);
        tick(&s,&m); CHECK(s.data.probe.bank_seen && s.data.probe.bank==0u && !m.writes);
        tick(&s,&m); CHECK(m.writes==1u && s.data.probe.reset_sent);
        CHECK(s.data.probe.reset_bus==GYRO_BUS_OK);
        i=m.transactions; step(&s,&m,m.now+1u); CHECK(m.transactions==i);
        tick(&s,&m); CHECK(s.data.probe.after.total==1u && s.data.probe.after.ok==1u);
        m.failure=GYRO_BUS_NACK;
        for (i=0u;i<100u;++i) tick(&s,&m);
        CHECK(m.writes==1u && s.data.probe.after.nack==100u);
        CHECK(s.data.probe.before.ok==GYRO_COMM_TEST_READS);
        Gyro_GetSnapshot(&s,m.now,&d);
        CHECK(!d.raw_valid && !d.rate_valid && !d.angle_valid);
        GyroUi_Line(line,2u,1u,&d); CHECK(strstr(line,"1000/1000 100.00%"));
        GyroUi_Line(line,3u,1u,&d); CHECK(strstr(line,"1/101 0.99%"));
        GyroUi_Line(line,4u,1u,&d); CHECK(strstr(line,"RST:OK"));
    } else {
        for (i=0u;i<1100u;++i) tick(&s,&m);
        CHECK(!m.writes && s.data.probe.before.total==2100u && !s.data.probe.after.total);
    }
    /* Every error class and incorrect identity prevents an automatic reset,
       even if later reads recover. No hidden init or reset is allowed. */
    setup(&s,&m,0u);
    for (i=GYRO_BUS_NACK;i<=GYRO_BUS_HW_BUSY;++i) {
        m.failure=(GyroBusError)i; tick(&s,&m);
    }
    m.failure=GYRO_BUS_OK; m.id=0u; tick(&s,&m); m.id=0x47u;
    for (i=0u;i<1100u;++i) tick(&s,&m);
    CHECK(!m.writes && !s.data.probe.after.total);
    CHECK(s.data.probe.before.nack==1u && s.data.probe.before.timeout==1u);
    CHECK(s.data.probe.before.busy==5u && s.data.probe.before.other==1u && s.data.probe.before.bad_id==1u);
    CHECK(s.data.probe.phase==(GYRO_COMM_TEST==2 ? GYRO_PROBE_HOLD : GYRO_PROBE_PRE));
    if (GYRO_COMM_TEST==2) {
        setup(&s,&m,0u); baseline(&s,&m); m.reg[0x76]=1u; tick(&s,&m);
        CHECK(s.data.probe.phase==GYRO_PROBE_HOLD && !m.writes);
        tick(&s,&m); CHECK(!m.writes);
        setup(&s,&m,0u); baseline(&s,&m); m.failure=GYRO_BUS_NACK; tick(&s,&m);
        CHECK(s.data.probe.phase==GYRO_PROBE_HOLD && s.data.probe.bank_bus==GYRO_BUS_NACK);
        m.failure=GYRO_BUS_OK; tick(&s,&m); CHECK(!m.writes);
        for (i=GYRO_BUS_NACK;i<=GYRO_BUS_HW_BUSY;++i) {
            unsigned j;
            setup(&s,&m,0u); baseline(&s,&m); tick(&s,&m);
            m.failure=(GyroBusError)i; tick(&s,&m);
            CHECK(s.data.probe.reset_bus==(GyroBusError)i && m.writes==1u);
            m.failure=GYRO_BUS_OK;
            for (j=0u;j<1100u;++j) tick(&s,&m);
            CHECK(m.writes==1u && s.data.probe.after.ok==1100u);
        }
    }
    GyroMock_Reset(&m,0u); m.init_failure=GYRO_BUS_IO; Gyro_Init(&s,GyroMock_Io(&m),0u);
    step(&s,&m,100000u); CHECK(!m.transactions && s.data.probe.init_bus==GYRO_BUS_IO);
    GyroMock_Reset(&m,0u); Gyro_Init(&s,GyroMock_Io(&m),0u);
    CHECK(!s.data.probe.before.total && !s.data.probe.after.total && !s.data.probe.reset_sent);
    Gyro_GetSnapshot(&s,0u,&d);
    for (i=0;i<=8u;++i) { GyroUi_Line(line,i,1u,&d); CHECK(strlen(line)<43u); }
    printf("Gyro communication test mode %d passed\n",GYRO_COMM_TEST);
    return 0;
}
