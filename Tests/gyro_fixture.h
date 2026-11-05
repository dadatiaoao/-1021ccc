/* Register-level ICM42688 substitute, used with the REAL driver/service. */
#ifndef GYRO_FIXTURE_H
#define GYRO_FIXTURE_H
#include "gyro/gyro_service.h"
#include <string.h>
typedef struct {
    uint8_t reg[128], reg1[128], id, powered, reset_pending, reset_flag, ready, block_drdy;
    uint32_t now, us, reset_due, next_sample, power_ms, generated, transactions;
    unsigned reads, writes, initializations, burst_reads, max_transactions;
    int corrupt_reg, fail_reg;
    GyroBusError failure, init_failure;
    int16_t raw[3];
    uint8_t pins, released_pins;
    unsigned releases, line_reads;
} GyroMock;
static void GyroMock_Reset(GyroMock *m,uint32_t now)
{ memset(m,0,sizeof(*m)); m->id=0x47u; m->now=now; m->corrupt_reg=m->fail_reg=-1; }
static uint32_t mock_ms(void *ctx) { return ((GyroMock *)ctx)->now; }
static uint32_t mock_cycles(void *ctx) { return ((GyroMock *)ctx)->us; }
static GyroBusError mock_init(void *ctx)
{ GyroMock *m=ctx; ++m->initializations; return m->init_failure; }
static void mock_update(GyroMock *m)
{
    if (m->reset_pending && (int32_t)(m->now-m->reset_due)>=0) {
        m->reset_pending=0u; m->reset_flag=1u; m->reg[0x11]=0u;
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
    else if (reg==0x2Du) {
        *v=(uint8_t)((m->reset_flag ? 0x10u : 0u)|((m->ready && !m->block_drdy) ? 8u : 0u));
        m->reset_flag=0u;
    } else if (reg==0x25u && n==6u) {
        for (i=0u;i<3u;++i) { v[i*2u]=(uint8_t)((uint16_t)m->raw[i]>>8); v[i*2u+1u]=(uint8_t)m->raw[i]; }
        m->ready=0u; ++m->burst_reads;
    } else memcpy(v,(reg!=0x76u && m->reg[0x76]==1u) ? &m->reg1[reg] : &m->reg[reg],n);
    if (m->corrupt_reg==reg) *v^=1u;
    return GYRO_BUS_OK;
}
static GyroBusError mock_write(void *ctx,uint8_t reg,uint8_t value)
{
    GyroMock *m=ctx;
    ++m->writes; ++m->transactions; m->us+=270u;
    if (m->failure && (m->fail_reg<0 || m->fail_reg==reg)) return m->failure;
    if (reg!=0x76u && m->reg[0x76]==1u) m->reg1[reg]=value; else m->reg[reg]=value;
    if (reg==0x11u && value==1u) {
        memset(m->reg,0,sizeof(m->reg)); m->reg[0x11]=1u;
        memset(m->reg1,0,sizeof(m->reg1)); m->reg1[0x7A]=0x83u; m->reg1[0x7C]=0x5Fu;
        m->powered=m->ready=0u; m->reset_pending=1u; m->reset_due=m->now+2u;
    }
    if (reg==0x4Eu && (value&12u)==12u) {
        m->powered=1u; m->power_ms=m->now; m->next_sample=m->now+10u;
    }
    return GYRO_BUS_OK;
}
static uint8_t mock_lines(void *ctx)
{ GyroMock *m=ctx; ++m->line_reads; return m->releases ? m->released_pins : m->pins; }
static void mock_release(void *ctx) { ++((GyroMock *)ctx)->releases; }
static GyroIo GyroMock_Io(GyroMock *m)
{
    GyroIo io={m,mock_init,mock_read,mock_write,mock_ms,mock_cycles,1u,mock_lines,mock_release,0}; return io;
}
#endif
