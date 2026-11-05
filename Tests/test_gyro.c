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
    CHECK(s->data.state==GYRO_RUNNING && s->data.who==0x68u);
    CHECK(m->now-m->power_ms>=50u && !s->data.have_sample);
    CHECK(m->reg[0x1B]==0x08u && m->reg[0x1A]==4u && m->reg[0x19]==9u);
    CHECK(m->reg[0x6B]==9u && m->reg[0x6C]==0x38u);
    CHECK(m->reg[0x37]==0x20u && m->reg[0x38]==1u);
    CHECK(m->reg[0x6A]==0u && m->reg[0x23]==0u);
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
    Mpu6050 p; GyroMock m; GyroIo io; int r; uint32_t at; int16_t raw[3]; unsigned i,j;
    static const uint8_t bad_ids[]={0u,0xFFu,0x47u,0x69u,0x70u};
    static const uint8_t regs[]={0x6Au,0x23u,0x19u,0x1Au,0x1Bu,0x6Cu,0x37u,0x38u,0x6Bu};
    GyroMock_Reset(&m,0u); io=GyroMock_Io(&m); Mpu_Begin(&p,0u);
    CHECK(Mpu_Step(&p,&io,99u)==MPU_NO_DATA && !m.reads && !m.writes);
    m.now=100u; CHECK(Mpu_Step(&p,&io,m.now)==MPU_NO_DATA && p.who==0x68u && !m.writes);
    CHECK(Mpu_Step(&p,&io,m.now)==MPU_NO_DATA && m.writes==1u && m.reset_pending);
    i=m.reads; CHECK(Mpu_Step(&p,&io,p.due-1u)==MPU_NO_DATA && m.reads==i);
    m.now=p.due; m.reset_due=m.now+100u;
    CHECK(Mpu_Step(&p,&io,m.now)==MPU_RESET_ERROR);
    for (i=0u;i<sizeof(bad_ids);++i) {
        GyroMock_Reset(&m,0u); m.id=bad_ids[i]; Mpu_Begin(&p,0u); m.now=100u;
        CHECK(Mpu_Step(&p,&io,m.now)==MPU_ID_ERROR && p.who==m.id && !m.writes);
    }
    for (i=0u;i<sizeof(regs);++i) {
        GyroMock_Reset(&m,0u); Mpu_Begin(&p,0u); m.corrupt_reg=regs[i];
        r=0; for (m.now=0u;m.now<400u && r>=0;++m.now) r=Mpu_Step(&p,&io,m.now);
        CHECK(r==(regs[i]==0x6Bu ? MPU_RESET_ERROR : MPU_CONFIG_ERROR));
    }
    /* Both status clearing races: frame before burst is consumed once; a frame
       during the burst remains pending with different, shadowed data. */
    GyroMock_Reset(&m,0u); m.powered=1u; m.next_sample=0u;
    m.raw[0]=-1234; m.raw[1]=2345;
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_READY);
    CHECK(raw[0]==-1234 && raw[1]==2345 && raw[2]==0);
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_NO_DATA);
    m.now+=10u; CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_READY && raw[0]==-1234);
    m.now+=10u; m.frame_before_burst=1u;
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_READY && raw[0]==-1233);
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_NO_DATA);
    m.now+=10u; m.frame_during_burst=1u;
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_READY && raw[0]==-1233);
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_READY && raw[0]==-1232);
    CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_NO_DATA);
    m.now+=10u; m.raw[0]=INT16_MIN; CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_SATURATED);
    m.now+=10u; m.raw[0]=-32767; CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_SATURATED);
    m.now+=10u; m.raw[0]=32767; CHECK(Mpu_Fetch(&p,&io,raw,&at)==MPU_SATURATED);
    /* Check every runtime setting independently, including ODR/filter/DRDY. */
    for (i=0u;i<sizeof(regs);++i) {
        GyroMock_Reset(&m,0u); Mpu_Begin(&p,0u);
        r=0; for (m.now=0u;m.now<400u && r==0;++m.now) r=Mpu_Step(&p,&io,m.now);
        CHECK(r==MPU_READY); m.reg[regs[i]]^=1u;
        for (j=0u;j<10u;++j) { r=Mpu_Check(&p,&io); if (r<0) break; }
        CHECK(r==MPU_CONFIG_ERROR);
    }
    m.id=0x47u; p.check_index=0u; CHECK(Mpu_Check(&p,&io)==MPU_ID_ERROR);
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
    run(&s,&m,600u); CHECK(Gyro_RequestCalibration(&s,m.now,0u));
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
    for (i=GYRO_BUS_NACK;i<=GYRO_BUS_IO;++i) {
        setup(&s,&m,0u); cal(&s,&m); m.failure=(GyroBusError)i;
        run(&s,&m,5u); CHECK(s.data.state==GYRO_ERROR && s.data.bus_error==(GyroBusError)i);
        at=m.initializations; run(&s,&m,200u); CHECK(m.initializations==at);
        m.failure=GYRO_BUS_OK; run(&s,&m,600u);
        CHECK(s.data.state==GYRO_RUNNING && s.data.raw_valid && !s.data.calibrated && !s.data.angle_valid);
        CHECK(!Gyro_ZeroAngle(&s,m.now));
        CHECK(s.data.bus_error==GYRO_BUS_OK && s.data.last_bus_error==(GyroBusError)i);
    }
    setup(&s,&m,UINT32_MAX-2800u); cal(&s,&m); m.raw[GYRO_AXIS]=6550; run(&s,&m,20u);
    CHECK(Gyro_ZeroAngle(&s,m.now)); at=s.data.sample_ms; run(&s,&m,4000u);
    CHECK(m.now<5000u && s.data.angle_valid);
    NEAR(s.data.angle_deg,(double)GYRO_SIGN*100.0*(double)(uint32_t)(s.data.sample_ms-at)/1000.0,0.005);
    m.reg[0x1Bu]=0x00u;
    for (i=0u;i<2600u && s.data.state!=GYRO_ERROR;++i) step(&s,&m,m.now+1u,0u);
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
    GyroService s; GyroMock m; unsigned i; float bias; Mpu6050 p; GyroIo io;
    for (i=GYRO_BUS_NACK;i<=GYRO_BUS_IO;++i) {
        GyroMock_Reset(&m,0u); m.init_failure=(GyroBusError)i;
        Gyro_Init(&s,GyroMock_Io(&m),0u);
        CHECK(s.data.state==GYRO_ERROR && s.data.errors==1u && !s.data.raw_valid);
        run(&s,&m,200u); CHECK(m.initializations==1u);
        m.init_failure=GYRO_BUS_OK; run(&s,&m,600u);
        CHECK(s.data.state==GYRO_RUNNING && s.data.raw_valid && !s.data.calibrated);
    }
    setup(&s,&m,0u); m.raw[GYRO_AXIS]=65; cal(&s,&m); bias=s.data.bias_dps;
    CHECK(Gyro_RequestCalibration(&s,m.now,0u)); run(&s,&m,100u);
    m.failure=GYRO_BUS_TIMEOUT; m.fail_reg=0x3A; run(&s,&m,20u);
    CHECK(s.data.calibration==GYRO_CAL_ERROR && !s.data.angle_valid && s.data.state==GYRO_ERROR);
    NEAR(s.data.bias_dps,bias,0.0001);
    /* A spontaneous reset must invalidate calibration and recover to UNCAL. */
    setup(&s,&m,0u); cal(&s,&m);
    io=GyroMock_Io(&m); CHECK(mock_write(&m,0x6Bu,0x80u)==GYRO_BUS_OK);
    run(&s,&m,600u); CHECK(s.data.errors && !s.data.rate_valid && !s.data.angle_valid);
    run(&s,&m,600u); CHECK(s.data.state==GYRO_RUNNING && !s.data.calibrated);
    /* Identity is checked again after reset, before any configuration. */
    GyroMock_Reset(&m,0u); Mpu_Begin(&p,0u);
    for (i=0u;i<300u && p.phase<3u;++i) { m.now=i; (void)Mpu_Step(&p,&io,i); }
    CHECK(p.phase==3u); m.id=0x47u;
    CHECK(Mpu_Step(&p,&io,m.now)==MPU_ID_ERROR && !m.powered);

}
static void test_visible_diagnostics(void)
{
    GyroSnapshot d={0}; char line[43];
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
int main(void)
{
    test_visible_diagnostics(); test_chip(); test_service(); test_cal_failures(); test_recovery_wrap_ui(); test_startup_and_cal_bus_failure();
    printf("GYRO PASS %u checks, axis %u sign %d, service %u bytes (host ABI)\n",checks,GYRO_AXIS,GYRO_SIGN,(unsigned)sizeof(GyroService));
    return 0;
}
