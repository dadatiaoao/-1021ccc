/* Register-level MPU6050 substitute, used with the REAL driver/service. */
#ifndef GYRO_FIXTURE_H
#define GYRO_FIXTURE_H
#include "gyro/gyro_service.h"
#include <string.h>
typedef struct {
    uint8_t reg[128], id, powered, reset_pending, ready, block_drdy;
    uint8_t frame_before_burst, frame_during_burst;
    uint32_t now, us, reset_due, next_sample, power_ms, generated, transactions;
    unsigned reads, writes, initializations, burst_reads, max_transactions;
    int corrupt_reg, fail_reg;
    GyroBusError failure, init_failure;
    int16_t raw[3];
} GyroMock;
static void GyroMock_Reset(GyroMock *m,uint32_t now)
{
    memset(m,0,sizeof(*m)); m->id=0x68u; m->reg[0x6B]=0x40u;
    m->now=now; m->corrupt_reg=m->fail_reg=-1;
}
static uint32_t mock_ms(void *ctx) { return ((GyroMock *)ctx)->now; }
static uint32_t mock_cycles(void *ctx) { return ((GyroMock *)ctx)->us; }
static GyroBusError mock_init(void *ctx)
{ GyroMock *m=ctx; ++m->initializations; return m->init_failure; }
static void mock_update(GyroMock *m)
{
    if (m->reset_pending && (int32_t)(m->now-m->reset_due)>=0) {
        m->reset_pending=0u; m->reg[0x6B]=0x40u;
    }
    if (m->powered && (int32_t)(m->now-m->next_sample)>=0) {
        uint32_t n=(m->now-m->next_sample)/10u+1u;
        m->generated+=n; m->next_sample+=n*10u; m->ready=1u;
    }
}
static GyroBusError mock_read(void *ctx,uint8_t reg,uint8_t *v,size_t n)
{
    GyroMock *m=ctx; unsigned i;
    ++m->reads; ++m->transactions; m->us+=(uint32_t)(n+3u)*90u;
    if (m->failure && (m->fail_reg<0 || m->fail_reg==reg)) return m->failure;
    mock_update(m);
    if (reg==0x75u) *v=m->id;
    else if (reg==0x3Au) {
        if (n==15u && m->frame_before_burst) {
            m->frame_before_burst=0u; m->ready=1u; ++m->raw[0];
        }
        memset(v,0,n); *v=(uint8_t)(m->ready && !m->block_drdy);
        m->ready=0u; /* INT_RD_CLEAR=0: status read clears, data read does not. */
        if (n==15u) {
            for (i=0u;i<3u;++i) {
                v[9u+i*2u]=(uint8_t)((uint16_t)m->raw[i]>>8);
                v[10u+i*2u]=(uint8_t)m->raw[i];
            }
            ++m->burst_reads;
            /* Shadowed burst stays old; event after its status byte survives. */
            if (m->frame_during_burst) {
                m->frame_during_burst=0u; ++m->raw[0]; m->ready=1u;
            }
        }
    } else memcpy(v,&m->reg[reg],n);
    if (m->corrupt_reg==reg) *v^=1u;
    return GYRO_BUS_OK;
}
static GyroBusError mock_write(void *ctx,uint8_t reg,uint8_t value)
{
    GyroMock *m=ctx;
    ++m->writes; ++m->transactions; m->us+=270u;
    if (m->failure && (m->fail_reg<0 || m->fail_reg==reg)) return m->failure;
    m->reg[reg]=value;
    if (reg==0x6Bu && (value&0x80u)) {
        memset(m->reg,0,sizeof(m->reg)); m->reg[0x6B]=0x80u;
        m->powered=m->ready=0u; m->reset_pending=1u; m->reset_due=m->now+100u;
    } else if (reg==0x6Bu && !(value&0x40u)) {
        m->powered=1u; m->power_ms=m->now; m->next_sample=m->now+10u;
    }
    return GYRO_BUS_OK;
}
static GyroIo GyroMock_Io(GyroMock *m)
{
    GyroIo io={m,mock_init,mock_read,mock_write,mock_ms,mock_cycles,1u}; return io;
}
#endif
