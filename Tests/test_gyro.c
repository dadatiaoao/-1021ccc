#include "gyro_fixture.h"
#include "gyro/gyro_config.h"
#include "gyro/gyro_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"GYRO FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
#define NEAR(x,y,e) CHECK(fabs((double)(x)-(double)(y))<(e))
static void step(GyroService *s,GyroMock *m,uint32_t now,uint8_t active)
{
    unsigned tx=m->transactions;
    m->now=now; Gyro_Process(s,now,active);
    tx=m->transactions-tx; if (tx>m->max_transactions) m->max_transactions=tx;
    CHECK(tx<=2u);
}
static void run(GyroService *s,GyroMock *m,uint32_t duration)
{ uint32_t i; for (i=0;i<duration;++i) step(s,m,m->now+1u,0u); }
static void setup(GyroService *s,GyroMock *m,uint32_t now)
{
    unsigned i;
    GyroMock_Reset(m,now); Gyro_Init(s,GyroMock_Io(m),now);
    for (i=0;i<300u && s->data.state==GYRO_INIT;++i) step(s,m,m->now+1u,0u);
    CHECK(s->data.state==GYRO_RUNNING && s->data.who==0x47u);
    CHECK(m->now-m->power_ms>=50u && !s->data.have_sample);
    CHECK(m->reg[0x4F]==0x48u && m->reg[0x4C]==0x32u && m->reg[0x63]==0x20u);
    CHECK(m->reg[0x4E]==0x2Cu && m->reg[0x51]==0x12u && m->reg[0x52]==0x15u);
    CHECK(m->reg[0x76]==0u && m->reg[0x13]==0x09u && m->reg[0x4D]==0x91u);
    CHECK(m->reg1[0x7A]==0x83u && m->reg1[0x7B]==0u && m->reg1[0x7C]==0x5Cu);
    run(s,m,20u); CHECK(s->data.raw_valid && !s->data.calibrated && !s->data.angle_valid);
    CHECK(!Gyro_ZeroAngle(s,m->now));
}
static void cal(GyroService *s,GyroMock *m)
{
    CHECK(Gyro_RequestCalibration(s,m->now,0u));
    CHECK(!s->data.angle_valid && !s->data.rate_valid);
    run(s,m,2200u);
    CHECK(s->data.calibration==GYRO_CAL_DONE && s->data.calibrated && s->data.angle_valid);
    CHECK(s->data.cal_count>=201u && s->data.cal_elapsed_ms>=2000u);
}
static void test_chip(void)
{
    Icm42688 p; GyroMock m; GyroIo io; int r; uint32_t at; int16_t raw[3]; unsigned i;
    GyroMock_Reset(&m,0u); io=GyroMock_Io(&m); Icm_Begin(&p,0u);
    CHECK(Icm_Step(&p,&io,2u)==ICM_NO_DATA && !m.writes);
    m.now=3u; CHECK(Icm_Step(&p,&io,3u)==ICM_NO_DATA && m.writes==1u);
    m.corrupt_reg=0x76; CHECK(Icm_Step(&p,&io,3u)==ICM_CONFIG_ERROR);
    for (i=0u;i<3u;++i) {
        GyroMock_Reset(&m,0u); m.id=(uint8_t)(i==0 ? 0 : i==1 ? 0xFF : 0x42); Icm_Begin(&p,0u);
        for (m.now=3u;m.now<20u;++m.now) { r=Icm_Step(&p,&io,m.now); if (r<0) break; }
        CHECK(r==ICM_ID_ERROR && p.who==m.id && !m.powered);
    }
    GyroMock_Reset(&m,0u); Icm_Begin(&p,0u);
    for (m.now=0u;m.now<100u && p.phase<4u;++m.now) (void)Icm_Step(&p,&io,m.now);
    CHECK(p.phase==4u && m.reset_pending);
    i=m.reads; CHECK(Icm_Step(&p,&io,p.due-1u)==ICM_NO_DATA && m.reads==i);
    m.now=p.due; m.reset_due=m.now+100u;
    CHECK(Icm_Step(&p,&io,m.now)==ICM_RESET_ERROR);
    GyroMock_Reset(&m,0u); Icm_Begin(&p,0u); m.corrupt_reg=0x4F;
    r=0; for (m.now=3u;m.now<300u && r>=0;++m.now) r=Icm_Step(&p,&io,m.now);
    CHECK(r==ICM_CONFIG_ERROR && !m.powered);
    /* Direct DRDY fetch: same values may be new, a repeated read is not. */
    m.failure=GYRO_BUS_OK; m.corrupt_reg=-1; m.powered=1u; m.next_sample=m.now;
    m.raw[0]=-1234; m.raw[1]=2345; m.raw[2]=0;
    CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_READY);
    CHECK(raw[0]==-1234 && raw[1]==2345 && raw[2]==0);
    CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_NO_DATA);
    m.now+=10u; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_READY && raw[0]==-1234);
    m.now+=10u; m.raw[0]=INT16_MIN; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_INVALID);
    m.now+=10u; m.raw[0]=-32767; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_INVALID);
    m.now+=10u; m.raw[0]=-32766; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_SATURATED);
    m.now+=10u; m.raw[0]=32767; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_SATURATED);
    m.now+=10u; m.raw[0]=0; m.reset_flag=1u; CHECK(Icm_Fetch(&p,&io,raw,&at)==ICM_RESET_DETECTED);
}
static void test_service(void)
{
    GyroService s; GyroMock m; GyroSnapshot d; uint32_t seq,at; double angle; unsigned i;
    setup(&s,&m,0u); CHECK(!Gyro_RequestCalibration(&s,m.now,1u));
    CHECK(s.data.calibration==GYRO_CAL_MOTOR && !s.data.cal_count);
    m.raw[GYRO_AXIS]=65; cal(&s,&m);
    NEAR(s.data.bias_dps,(double)GYRO_SIGN*65.0/65.5,0.0001); NEAR(s.data.angle_deg,0.0,0.0001);
    NEAR(s.data.rate_dps,0.0,0.0001);
    seq=s.data.sequence; step(&s,&m,m.now,0u); CHECK(s.data.sequence==seq);
    m.raw[GYRO_AXIS]=6615; run(&s,&m,20u); NEAR(s.data.rate_dps,GYRO_SIGN*100.0,0.0002);
    CHECK(Gyro_ZeroAngle(&s,m.now)); at=s.data.sample_ms; run(&s,&m,4000u);
    NEAR(s.data.angle_deg,(double)GYRO_SIGN*100.0*(double)(s.data.sample_ms-at)/1000.0,0.005);
    CHECK(fabs(s.data.angle_deg)>360.0); angle=s.data.angle_deg;
    seq=s.data.sequence; m.block_drdy=1u; run(&s,&m,25u);
    CHECK(s.data.sequence==seq); NEAR(s.data.angle_deg,angle,0.0001);
    m.block_drdy=0u; run(&s,&m,20u);
    CHECK(!s.data.angle_valid && s.data.angle_gap && s.data.rate_valid);
    angle=s.data.angle_deg; run(&s,&m,100u); NEAR(s.data.angle_deg,angle,0.0001);
    CHECK(Gyro_ZeroAngle(&s,m.now)); at=s.data.sample_ms;
    /* Variable acquisition intervals without a polling blackout; wall time,
       not UI period or a fixed 10ms multiplier, determines the angle. */
    for (i=0u;i<160u;++i) step(&s,&m,m.now+(i%4u)+1u,0u);
    NEAR(s.data.angle_deg,(double)GYRO_SIGN*100.0*(double)(s.data.sample_ms-at)/1000.0,0.01);
    CHECK(s.data.angle_valid);
    m.raw[GYRO_AXIS]=-3210; run(&s,&m,30u); NEAR(s.data.rate_dps,GYRO_SIGN*-50.0,0.0001);
    CHECK(Gyro_ZeroAngle(&s,m.now)); at=s.data.sample_ms; run(&s,&m,1000u);
    NEAR(s.data.angle_deg,(double)GYRO_SIGN*-50.0*(double)(s.data.sample_ms-at)/1000.0,0.002);
    Gyro_GetSnapshot(&s,m.now+31u,&d); CHECK(!d.raw_valid && !d.rate_valid && !d.angle_valid);
    CHECK(!Gyro_ZeroAngle(&s,m.now+31u));
    step(&s,&m,m.now+100u,0u); CHECK(!s.data.angle_valid && s.data.missed>=8u);
    CHECK(s.data.rate_valid && Gyro_ZeroAngle(&s,m.now));
    m.raw[GYRO_AXIS]=32767; run(&s,&m,20u); CHECK(s.data.saturations && !s.data.angle_valid);
    m.raw[GYRO_AXIS]=0; run(&s,&m,20u); CHECK(s.data.rate_valid && !s.data.angle_valid);
    printf("GYRO simulation: max process %lu us, max polling %lu ms, max sample dt %lu ms\n",
        (unsigned long)s.data.max_process_us,(unsigned long)s.data.max_poll_ms,(unsigned long)s.data.max_interval_ms);
}
static void test_cal_failures(void)
{
    GyroService s; GyroMock m; float bias; unsigned i;
    setup(&s,&m,0u); m.raw[GYRO_AXIS]=65; cal(&s,&m); bias=s.data.bias_dps;
    CHECK(Gyro_RequestCalibration(&s,m.now,0u));
    run(&s,&m,200u); CHECK(s.data.calibration==GYRO_CAL_RUNNING && s.data.cal_count<201u);
    m.raw[(GYRO_AXIS+1)%3]=600; run(&s,&m,20u);
    CHECK(s.data.calibration==GYRO_CAL_MOVING && s.data.calibrated && !s.data.angle_valid);
    NEAR(s.data.bias_dps,bias,0.0001);
    memset(m.raw,0,sizeof(m.raw)); run(&s,&m,20u);
    CHECK(Gyro_RequestCalibration(&s,m.now,0u)); step(&s,&m,m.now+1u,1u);
    CHECK(s.data.calibration==GYRO_CAL_MOTOR && !s.data.angle_valid);
    CHECK(Gyro_RequestCalibration(&s,m.now,0u));
    step(&s,&m,m.now+3000u,0u); CHECK(s.data.calibration==GYRO_CAL_TIMEOUT);
    run(&s,&m,400u); CHECK(Gyro_RequestCalibration(&s,m.now,0u));
    m.block_drdy=1u; run(&s,&m,40u); CHECK(s.data.calibration==GYRO_CAL_GAP);
    NEAR(s.data.bias_dps,bias,0.0001);
    m.block_drdy=0u; run(&s,&m,30u); CHECK(Gyro_RequestCalibration(&s,m.now,0u));
    /* Range <=1 dps but high variance: not accepted as a quiet window. */
    for (i=0u;i<2200u;++i) {
        m.raw[0]=(int16_t)((m.now/10u)%2u ? 20 : -20); step(&s,&m,m.now+1u,0u);
    }
    CHECK(s.data.calibration==GYRO_CAL_MOVING && !s.data.angle_valid);
    NEAR(s.data.bias_dps,bias,0.0001);
    memset(m.raw,0,sizeof(m.raw)); m.raw[0]=262; run(&s,&m,20u);
    CHECK(Gyro_RequestCalibration(&s,m.now,0u)); run(&s,&m,2200u);
    CHECK(s.data.calibration==GYRO_CAL_MOVING); /* mean 4 >3, still below abs 5 */
}
static void test_recovery_wrap_ui(void)
{
    GyroService s; GyroMock m; GyroSnapshot d; unsigned i; uint32_t at; char line[43];
    for (i=GYRO_BUS_NACK;i<=GYRO_BUS_HW_BUSY;++i) {
        setup(&s,&m,0u); cal(&s,&m); m.failure=(GyroBusError)i;
        run(&s,&m,5u); CHECK(s.data.state==GYRO_ERROR && s.data.bus_error==(GyroBusError)i);
        at=m.initializations; run(&s,&m,200u); CHECK(m.initializations==at);
        m.failure=GYRO_BUS_OK; run(&s,&m,500u);
        CHECK(s.data.state==GYRO_RUNNING && s.data.raw_valid && !s.data.calibrated && !s.data.angle_valid);
        CHECK(!Gyro_ZeroAngle(&s,m.now));
        CHECK(s.data.bus_error==GYRO_BUS_OK && s.data.last_bus_error==(GyroBusError)i);
    }
    setup(&s,&m,UINT32_MAX-2800u); cal(&s,&m); m.raw[GYRO_AXIS]=6550; run(&s,&m,20u);
    CHECK(Gyro_ZeroAngle(&s,m.now)); at=s.data.sample_ms; run(&s,&m,4000u);
    CHECK(m.now<5000u && s.data.angle_valid);
    NEAR(s.data.angle_deg,(double)GYRO_SIGN*100.0*(double)(uint32_t)(s.data.sample_ms-at)/1000.0,0.005);
    m.reg[0x4Fu]=0x06u;
    for (i=0u;i<1300u && s.data.state!=GYRO_ERROR;++i) step(&s,&m,m.now+1u,0u);
    CHECK(s.data.errors && s.data.state==GYRO_ERROR && s.data.error==GYRO_CONFIG && !s.data.angle_valid);
    run(&s,&m,600u); CHECK(!s.data.calibrated && s.data.raw_valid);
    Gyro_GetSnapshot(&s,m.now,&d); d.raw_valid=d.rate_valid=d.angle_valid=0u;
    GyroUi_Line(line,2u,0u,&d); CHECK(strstr(line,"--"));
    GyroUi_Line(line,3u,0u,&d); CHECK(strstr(line,"--"));
    d.angle_valid=d.rate_valid=1u; d.angle_deg=-12345.67; d.rate_dps=-12.346f;
    GyroUi_Line(line,2u,0u,&d); CHECK(strstr(line,"-12.35"));
    GyroUi_Line(line,3u,0u,&d); CHECK(strstr(line,"-12345.67"));
    d.angle_deg=1e100; GyroUi_Line(line,3u,0u,&d); CHECK(strstr(line,"RANGE"));
    CHECK(sizeof(GyroService)<512u);
}
static void test_startup_and_cal_bus_failure(void)
{
    GyroService s; GyroMock m; unsigned i; float bias; Icm42688 p; GyroIo io;
    for (i=GYRO_BUS_NACK;i<=GYRO_BUS_HW_BUSY;++i) {
        GyroMock_Reset(&m,0u); m.init_failure=(GyroBusError)i;
        Gyro_Init(&s,GyroMock_Io(&m),0u);
        CHECK(s.data.state==GYRO_ERROR && s.data.errors==1u && !s.data.raw_valid);
        run(&s,&m,200u); CHECK(m.initializations==1u);
        m.init_failure=GYRO_BUS_OK; run(&s,&m,500u);
        CHECK(s.data.state==GYRO_RUNNING && s.data.raw_valid && !s.data.calibrated);
    }
    setup(&s,&m,0u); m.raw[GYRO_AXIS]=65; cal(&s,&m); bias=s.data.bias_dps;
    CHECK(Gyro_RequestCalibration(&s,m.now,0u)); run(&s,&m,100u);
    m.failure=GYRO_BUS_TIMEOUT; m.fail_reg=0x25; run(&s,&m,20u);
    CHECK(s.data.calibration==GYRO_CAL_ERROR && !s.data.angle_valid && s.data.state==GYRO_ERROR);
    NEAR(s.data.bias_dps,bias,0.0001);
    /* RESET_DONE and bank-1 readback are checked independently of ID. */
    GyroMock_Reset(&m,0u); io=GyroMock_Io(&m); Icm_Begin(&p,0u);
    for (i=0u;i<20u && p.phase<5u;++i) { m.now=i; (void)Icm_Step(&p,&io,i); }
    CHECK(p.phase==5u); m.reset_flag=0u;
    CHECK(Icm_Step(&p,&io,m.now)==ICM_RESET_ERROR);
    setup(&s,&m,0u); s.data.state=GYRO_ERROR; s.due=m.now;
    m.corrupt_reg=0x7C; run(&s,&m,200u);
    CHECK(s.data.state==GYRO_ERROR && s.data.error==GYRO_CONFIG && !s.data.raw_valid);
}
static void test_visible_diagnostics(void)
{
    GyroSnapshot d={0}; char line[43];
    unsigned i;
    static const char * const busy_names[]={"SCL_LOW","SDA_LOW","BOTH_LOW","HW_BUSY"};
    for (i=0u;i<4u;++i) {
        d.state=GYRO_ERROR; d.error=GYRO_BUS;
        d.bus_error=d.last_bus_error=(GyroBusError)(GYRO_BUS_SCL_LOW+i);
        GyroUi_Line(line,6u,0u,&d);
        CHECK(strstr(line,"ERR:I2C BUS:") && strstr(line,busy_names[i]));
        GyroUi_Line(line,7u,1u,&d);
        CHECK(strstr(line,"LBUS:") && strstr(line,busy_names[i]));
    }
    d.state=GYRO_INIT; GyroUi_Line(line,6u,0u,&d); CHECK(strstr(line,"INIT:"));
    d.state=GYRO_ERROR; d.error=GYRO_BUS; d.bus_error=GYRO_BUS_BUSY;
    GyroUi_Line(line,6u,0u,&d); CHECK(!strcmp(line,"ERR:I2C BUS:BUSY"));
    d.error=GYRO_RESET; d.bus_error=GYRO_BUS_TIMEOUT;
    GyroUi_Line(line,6u,0u,&d); CHECK(!strcmp(line,"ERR:RESET/NO DRDY BUS:TIMEOUT"));
    d.state=GYRO_RUNNING; d.error=GYRO_STALE; d.bus_error=GYRO_BUS_OK;
    GyroUi_Line(line,6u,0u,&d); CHECK(!strcmp(line,"ERR:STALE BUS:OK"));
    d.error=GYRO_OK; d.angle_gap=1u; d.last_fault=GYRO_GAP;
    GyroUi_Line(line,6u,0u,&d); CHECK(!strcmp(line,"ERR:SAMPLE GAP BUS:OK"));
    d.last_bus_error=GYRO_BUS_NACK;
    GyroUi_Line(line,7u,1u,&d); CHECK(strstr(line,"LBUS:NACK"));
    d.angle_gap=0u; d.have_sample=d.calibrated=1u; d.interval_ms=10u; d.age_ms=2u;
    GyroUi_Line(line,6u,0u,&d); CHECK(!strcmp(line,"DT:10 AGE:2 ms"));
}
static void test_bus_failure_records(void)
{
    GyroService s; GyroMock m; GyroSnapshot snap; GyroBusFailure first;
    unsigned phase,verify,n; char line[43];
    /* Fail the next real transaction in every initialization phase,
       including both write and readback in the settings phase. */
    for (phase=0u;phase<=7u;++phase) for (verify=0u;verify<2u;++verify) {
        if (phase!=7u && verify) continue;
        GyroMock_Reset(&m,0u); Gyro_Init(&s,GyroMock_Io(&m),0u);
        for(n=0u;n<300u && (s.chip.phase!=phase || s.chip.verify!=verify);++n)
            step(&s,&m,m.now+1u,0u);
        CHECK(s.chip.phase==phase && s.chip.verify==verify);
        m.failure=GYRO_BUS_NACK;
        for(n=0u;n<60u && s.data.state!=GYRO_ERROR;++n) step(&s,&m,m.now+1u,0u);
        first=s.data.first_bus_failure;
        CHECK(first.valid && first.stage==GYRO_TRACE_INIT && first.phase==phase);
        CHECK(first.op==((phase==0u || phase==3u || (phase==7u && !verify)) ? 'W':'R'));
        CHECK(first.reg==(phase<2u ? 0x76u : (phase==2u || phase==6u ? 0x75u :
                         (phase==3u || phase==4u ? 0x11u : (phase==5u ? 0x2Du : 0x76u)))));
        CHECK(first.bus==GYRO_BUS_NACK);
        if(phase==3u) {
            GyroUi_Line(line,4u,1u,&s.data);
            CHECK(!strcmp(line,"FIRST:INIT3.0 W:11 NACK"));
        }
        /* Repeated failures change RECENT but never FIRST. */
        m.failure=GYRO_BUS_TIMEOUT; run(&s,&m,300u);
        CHECK(s.data.last_bus_failure.bus==GYRO_BUS_TIMEOUT);
        CHECK(!memcmp(&first,&s.data.first_bus_failure,sizeof(first)));
        m.failure=GYRO_BUS_OK; run(&s,&m,800u);
        CHECK(s.data.raw_valid);
        CHECK(!memcmp(&first,&s.data.first_bus_failure,sizeof(first)));
        /* First sample-status read failure after recovery. */
        m.failure=GYRO_BUS_SDA_LOW; m.fail_reg=0x2D;
        run(&s,&m,20u);
        CHECK(s.data.last_bus_failure.stage==GYRO_TRACE_SAMPLE);
        CHECK(s.data.last_bus_failure.op=='R' && s.data.last_bus_failure.reg==0x2Du);
        Gyro_GetSnapshot(&s,m.now,&snap);
        CHECK(!memcmp(&first,&snap.first_bus_failure,sizeof(first)));
        GyroUi_Line(line,6u,1u,&snap); CHECK(!strcmp(line,"RECENT:SAMPLE R:2D SDA_LOW"));
        m.failure=GYRO_BUS_OK; run(&s,&m,1000u); cal(&s,&m);
        CHECK(Gyro_ZeroAngle(&s,m.now));
        CHECK(!memcmp(&first,&s.data.first_bus_failure,sizeof(first)));
        Gyro_Init(&s,GyroMock_Io(&m),m.now);
        CHECK(!s.data.first_bus_failure.valid && !s.data.last_bus_failure.valid);
        GyroUi_Line(line,4u,1u,&s.data); CHECK(!strcmp(line,"FIRST:NONE"));
        GyroUi_Line(line,6u,1u,&s.data); CHECK(!strcmp(line,"RECENT:NONE"));
    }
    GyroMock_Reset(&m,0u); m.init_failure=GYRO_BUS_IO;
    Gyro_Init(&s,GyroMock_Io(&m),0u);
    CHECK(s.data.first_bus_failure.valid && s.data.first_bus_failure.stage==GYRO_TRACE_BUS_INIT);
    GyroUi_Line(line,4u,1u,&s.data); CHECK(!strcmp(line,"FIRST:BUS_INIT -- IO"));
    setup(&s,&m,0u); s.last_check=m.now-GYRO_CHECK_MS; s.last_poll=m.now-GYRO_POLL_MS;
    m.failure=GYRO_BUS_NACK; step(&s,&m,m.now+1u,0u);
    CHECK(s.data.first_bus_failure.stage==GYRO_TRACE_CHECK);
    CHECK(s.data.first_bus_failure.reg==0x75u && s.data.first_bus_failure.op=='R');
    GyroUi_Line(line,4u,1u,&s.data); CHECK(!strcmp(line,"FIRST:CHECK0 R:75 NACK"));
}
static void test_config4_preread(void)
{
    GyroService s; GyroMock m; unsigned n,scenario,writes,reads; char line[43];
    for(scenario=0u;scenario<5u;++scenario) {
        GyroMock_Reset(&m,0u); Gyro_Init(&s,GyroMock_Io(&m),0u);
        for(n=0u;n<300u && !(s.chip.phase==7u && s.chip.index==1u);++n)
            step(&s,&m,m.now+1u,0u);
        CHECK(s.chip.phase==7u && s.chip.index==1u && !s.chip.verify);
        CHECK(m.reg[0x76]==1u); /* Bank write AND its readback completed. */
        writes=m.writes; reads=m.reads;
        if(scenario>=2u) m.reg1[0x7A]=0xC3u;
        if(scenario==1u) { m.fail_reg=0x7A; m.failure=GYRO_BUS_NACK; }
        step(&s,&m,m.now+1u,0u);
        CHECK(m.writes==writes && m.reads==reads+1u);
        if(scenario==1u) {
            CHECK(s.data.state==GYRO_ERROR && !s.data.config4_seen);
            CHECK(s.data.first_bus_failure.op=='R' && s.data.first_bus_failure.reg==0x7Au);
            GyroUi_Line(line,5u,1u,&s.data); CHECK(strstr(line,"LAST7A:--"));
            continue;
        }
        CHECK(s.data.config4_seen && s.data.config4_before==(scenario==0u ? 0x83u:0xC3u));
        GyroUi_Line(line,5u,1u,&s.data);
        CHECK(strstr(line,scenario==0u ? "LAST7A:83 SKIP" : "LAST7A:C3 WRITE"));
        if(scenario==0u) {
            CHECK(s.chip.index==2u && !s.chip.verify);
            step(&s,&m,m.now+1u,0u);
            CHECK(m.writes==writes+1u && s.chip.trace_reg==0x7Bu && s.chip.verify);
            continue;
        }
        CHECK(s.chip.index==1u && !s.chip.verify);
        if(scenario==3u) { m.fail_reg=0x7A; m.failure=GYRO_BUS_TIMEOUT; }
        step(&s,&m,m.now+1u,0u);
        CHECK(m.writes==writes+1u);
        if(scenario==3u) {
            CHECK(s.data.state==GYRO_ERROR && s.data.config4_before==0xC3u);
            CHECK(s.data.first_bus_failure.op=='W' && s.data.first_bus_failure.reg==0x7Au);
            run(&s,&m,300u); /* Failed retry must not erase the prior value. */
            CHECK(s.data.config4_seen && s.data.config4_before==0xC3u);
            continue;
        }
        CHECK(s.chip.verify && m.reg1[0x7A]==0x83u);
        if(scenario==4u) m.corrupt_reg=0x7A;
        step(&s,&m,m.now+1u,0u);
        CHECK(m.reads==reads+2u);
        if(scenario==4u) CHECK(s.data.state==GYRO_ERROR && s.data.error==GYRO_CONFIG);
        else CHECK(s.chip.index==2u && !s.chip.verify);
    }
}
int main(void)
{
    test_config4_preread(); test_bus_failure_records(); test_visible_diagnostics(); test_chip(); test_service(); test_cal_failures(); test_recovery_wrap_ui(); test_startup_and_cal_bus_failure();
    printf("GYRO PASS %u checks, axis %u sign %d, service %u bytes (host ABI)\n",checks,GYRO_AXIS,GYRO_SIGN,(unsigned)sizeof(GyroService));
    return 0;
}
