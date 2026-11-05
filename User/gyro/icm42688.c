/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 * SPDX-License-Identifier: Apache-2.0
 *
 * Adapted from Zephyr v3.7.0 icm42688.c/icm42688_common.c:
 * reset/configure/sample_fetch. See UPSTREAM.md for revision and changes.
 * SPI/device/sleep/byteorder glue is replaced; no Zephyr runtime is required.
 */
/*
 * ICM-42688 寄存器驱动：经 GyroIo 访问器件，分步复位/配置并读回验证。
 * 只启用陀螺，FIFO 旁路；读取三轴用于静止检查，输出轴选择及相对积分在服务层完成。
 */
#include "icm42688.h"
#include "gyro_config.h"
#include <string.h>

/* Relevant bank-0 register definitions from upstream icm42688_reg.h. */
#define REG_DEVICE_CONFIG 0x11u
#define REG_DRIVE_CONFIG 0x13u
#define REG_INT_CONFIG 0x14u
#define REG_FIFO_CONFIG 0x16u
#define REG_GYRO_DATA_X1 0x25u
#define REG_INT_STATUS 0x2Du
#define REG_INTF_CONFIG0 0x4Cu
#define REG_INTF_CONFIG1 0x4Du
#define REG_PWR_MGMT0 0x4Eu
#define REG_GYRO_CONFIG0 0x4Fu
#define REG_GYRO_CONFIG1 0x51u
#define REG_GYRO_ACCEL_CONFIG0 0x52u
#define REG_TMST_CONFIG 0x54u
#define REG_FSYNC_CONFIG 0x62u
#define REG_INT_CONFIG0 0x63u
#define REG_INT_CONFIG1 0x64u
#define REG_INT_SOURCE0 0x65u
#define REG_WHO_AM_I 0x75u
#define REG_BANK_SEL 0x76u
#define BIT_SOFT_RESET 0x01u
#define BIT_INT_STATUS_RESET_DONE 0x10u
#define BIT_INT_STATUS_DATA_RDY 0x08u
#define WHO_AM_I_ICM42688 0x47u

/* Configure while gyro/accel are OFF (DS-000347 12.9), power on LAST.
 * Keep upstream FIFO bypass / FSYNC off / gyro ODR+FSR / startup logic.
 * Added explicit endian, invalid encoding, filter and clear policy/readback.
 */
static const uint8_t settings[][2] = {
    /* DS-000347 12.3 I2C settings. Reserved bits retain documented reset
       values; do this after a verified soft reset and before gyro power-on. */
    {REG_BANK_SEL,0x01u}, {0x7Au,0x83u}, /* INTF_CONFIG4: I3C_BUS_MODE=0 */
    {0x7Bu,0x00u}, /* INTF_CONFIG5: INT2, not FSYNC/CLKIN */
    {0x7Cu,0x5Cu}, /* INTF_CONFIG6: I3C_EN=1, SDR/DDR=0 */
    {REG_BANK_SEL,0x00u}, {REG_DRIVE_CONFIG,0x09u},
    {REG_INTF_CONFIG1,0x91u}, /* internal clock, PLL when available; RTC off */
    {REG_INT_SOURCE0,0x00u}, {REG_FIFO_CONFIG,0x00u},
    {REG_FSYNC_CONFIG,0x00u}, {REG_TMST_CONFIG,0x21u},
    {REG_INTF_CONFIG0,0x32u}, /* BE, invalid-code mode, SPI disabled (bits 1:0=10) */
    {REG_GYRO_CONFIG0,GYRO_CONFIG0_VALUE},
    {REG_GYRO_CONFIG1,0x12u}, /* 1st-order UI; preserve reset reserved bit 4 */
    {REG_GYRO_ACCEL_CONFIG0,0x15u}, /* gyro BW=23.8Hz, DC delay=10.2ms */
    {REG_INT_CONFIG0,0x20u}, /* DRDY clears on sense read, not status read */
    {REG_INT_CONFIG1,0x00u}, /* INT_ASYNC_RESET=0 */
    {REG_INT_CONFIG,0x04u}, /* latched, active low, open drain; INT1 not wired */
    {REG_INT_SOURCE0,0x08u}, {REG_PWR_MGMT0,0x2Cu} /* gyro LN, accel/temp off */
};
static int rd(Icm42688 *p,const GyroIo *io,uint8_t reg,uint8_t *v,size_t n)
{ p->trace_op='R'; p->trace_reg=reg; p->bus_error=io->read(io->ctx,reg,v,n); return p->bus_error==GYRO_BUS_OK; }
static int wr(Icm42688 *p,const GyroIo *io,uint8_t reg,uint8_t v)
{ p->trace_op='W'; p->trace_reg=reg; p->bus_error=io->write(io->ctx,reg,v); return p->bus_error==GYRO_BUS_OK; }
void Icm_Begin(Icm42688 *p,uint32_t now)
{ memset(p,0,sizeof(*p)); p->due=now+3u; }
/*
 * 按 phase 推进 bank/身份检查、软复位和设置表写入/读回；
 * 等待以 due 期限表示，每次只推进有限事务，上电稳定期不阻塞主循环。
 */
int Icm_Step(Icm42688 *p,const GyroIo *io,uint32_t now)
{
    uint8_t v;
    if ((int32_t)(now-p->due)<0) return ICM_NO_DATA;
    p->trace_stage=GYRO_TRACE_INIT;
    switch (p->phase) {
    case 0u: if (!wr(p,io,REG_BANK_SEL,0u)) return ICM_BUS_ERROR; break;
    case 1u:
        if (!rd(p,io,REG_BANK_SEL,&v,1u)) return ICM_BUS_ERROR;
        if (v!=0u) return ICM_CONFIG_ERROR;
        break;
    case 2u:
    case 6u:
        if (!rd(p,io,REG_WHO_AM_I,&p->who,1u)) return ICM_BUS_ERROR;
        if (p->who!=WHO_AM_I_ICM42688) return ICM_ID_ERROR;
        break;
    case 3u:
        /* Reset bit auto-clears; preserve upstream RESET_DONE verification. */
        if (!wr(p,io,REG_DEVICE_CONFIG,BIT_SOFT_RESET)) return ICM_BUS_ERROR;
        p->due=io->millis(io->ctx)+2u;
        break;
    case 4u:
        if (!rd(p,io,REG_DEVICE_CONFIG,&v,1u)) return ICM_BUS_ERROR;
        if (v&BIT_SOFT_RESET) return ICM_RESET_ERROR;
        break;
    case 5u:
        if (!rd(p,io,REG_INT_STATUS,&v,1u)) return ICM_BUS_ERROR;
        if (!(v&BIT_INT_STATUS_RESET_DONE)) return ICM_RESET_ERROR;
        break;
    case 7u:
        /* Controlled experiment: only Bank 1 INTF_CONFIG4 gets a pre-read.
           Keep one bus transaction per step and all other writes unchanged. */
        if (p->index==1u && !p->verify && !p->config4_read) {
            if (!rd(p,io,settings[p->index][0],&v,1u)) return ICM_BUS_ERROR;
            p->config4_before=v; p->config4_read=1u;
            if (v==settings[p->index][1]) ++p->index; /* Already configured. */
            return ICM_NO_DATA;
        }
        if (!p->verify) {
            if (!wr(p,io,settings[p->index][0],settings[p->index][1])) return ICM_BUS_ERROR;
            p->verify=1u;
            /* No register writes for >=200us after OFF -> ON. Also keep gyro
               ON >=45ms; upstream uses 50ms startup, kept as a deadline. */
            if (settings[p->index][0]==REG_PWR_MGMT0) p->due=io->millis(io->ctx)+GYRO_STARTUP_MS;
        } else {
            if (!rd(p,io,settings[p->index][0],&v,1u)) return ICM_BUS_ERROR;
            if (v!=settings[p->index][1]) return ICM_CONFIG_ERROR;
            p->verify=0u;
            if (++p->index==sizeof(settings)/sizeof(settings[0])) { ++p->phase; return ICM_READY; }
        }
        return ICM_NO_DATA;
    default: return ICM_READY;
    }
    ++p->phase;
    return ICM_NO_DATA;
}
/*
 * 先检查 DRDY，再突发读取一次三轴一致数据并解为大端有符号原始值。
 * 采样时间取数据事务中点近似；复位、非法编码或饱和分别报错，同值的新 DRDY 仍算新样本。
 */
int Icm_Fetch(Icm42688 *p,const GyroIo *io,int16_t raw[3],uint32_t *sample_ms)
{
    uint8_t status, readings[6]; unsigned i; uint32_t began;
    p->trace_stage=GYRO_TRACE_SAMPLE;
    /* Same upstream fetch contract: DRDY gate, ONE coherent burst, BE signed
       parse. Clear on sense read prevents a status->burst update from being
       counted twice. Equal values in separate DRDY events remain new samples. */
    if (!rd(p,io,REG_INT_STATUS,&status,1u)) return ICM_BUS_ERROR;
    if (status&BIT_INT_STATUS_RESET_DONE) return ICM_RESET_DETECTED;
    if (!(status&BIT_INT_STATUS_DATA_RDY)) return ICM_NO_DATA;
    began=io->millis(io->ctx);
    if (!rd(p,io,REG_GYRO_DATA_X1,readings,sizeof(readings))) return ICM_BUS_ERROR;
    *sample_ms=began+(uint32_t)(io->millis(io->ctx)-began)/2u;
    for (i=0u;i<3u;++i) raw[i]=(int16_t)(((uint16_t)readings[i*2u]<<8)|readings[i*2u+1u]);
    for (i=0u;i<3u;++i) if (raw[i]==INT16_MIN || raw[i]==-32767) return ICM_INVALID;
    for (i=0u;i<3u;++i) if (raw[i]>=GYRO_SATURATION_RAW || raw[i]<=-GYRO_SATURATION_RAW) return ICM_SATURATED;
    return ICM_READY;
}
/*
 * 每次轮查一个关键寄存器，检测器件复位或配置漂移；分散事务避免单轮长时间占用总线。
 */
int Icm_Check(Icm42688 *p,const GyroIo *io)
{
    static const uint8_t regs[]={REG_WHO_AM_I,REG_BANK_SEL,REG_PWR_MGMT0,REG_GYRO_CONFIG0,REG_INTF_CONFIG0};
    static const uint8_t values[]={WHO_AM_I_ICM42688,0x00u,0x2Cu,GYRO_CONFIG0_VALUE,0x32u};
    uint8_t v,index=p->check_index;
    p->trace_stage=GYRO_TRACE_CHECK;
    if (!rd(p,io,regs[index],&v,1u)) return ICM_BUS_ERROR;
    if (index==0u) p->who=v;
    if (v!=values[index]) return index==0u ? ICM_ID_ERROR : ICM_CONFIG_ERROR;
    p->check_index=(uint8_t)((index+1u)%5u);
    return ICM_READY;
}
