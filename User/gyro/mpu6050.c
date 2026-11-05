/*
 * Copyright (c) 2022 Intel Corporation
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 * SPDX-License-Identifier: Apache-2.0
 *
 * State-machine structure adapted from the previous ICM driver; see UPSTREAM.md.
 * MPU6050 registers: RM-MPU-6000A-00 rev 4.0.
 */
#include "mpu6050.h"
#include "gyro_config.h"
#include <string.h>

#define REG_SMPLRT_DIV 0x19u
#define REG_CONFIG 0x1Au
#define REG_GYRO_CONFIG 0x1Bu
#define REG_FIFO_EN 0x23u
#define REG_INT_PIN_CFG 0x37u
#define REG_INT_ENABLE 0x38u
#define REG_INT_STATUS 0x3Au
#define REG_USER_CTRL 0x6Au
#define REG_PWR_MGMT_1 0x6Bu
#define REG_PWR_MGMT_2 0x6Cu
#define REG_WHO_AM_I 0x75u
#define WHO_AM_I_MPU6050 0x68u /* Independent of AD0. */
#define BIT_DEVICE_RESET 0x80u
#define BIT_DATA_RDY 0x01u

/* Configure in sleep, wake LAST: gyro XYZ on, accel/temp off, X gyro PLL.
 * INT_RD_CLEAR=0: only INT_STATUS clears DRDY; MCU INT wire not needed.
 * FIFO, auxiliary master and bypass off. */
static const uint8_t settings[][2] = {
    {REG_USER_CTRL,0x00u}, {REG_FIFO_EN,0x00u},
    {REG_SMPLRT_DIV,GYRO_SMPLRT_DIV}, {REG_CONFIG,GYRO_DLPF_VALUE},
    {REG_GYRO_CONFIG,GYRO_CONFIG_VALUE}, {REG_PWR_MGMT_2,0x38u},
    {REG_INT_PIN_CFG,0x20u}, {REG_INT_ENABLE,0x01u},
    {REG_PWR_MGMT_1,0x09u}
};
static int rd(Mpu6050 *p,const GyroIo *io,uint8_t reg,uint8_t *v,size_t n)
{ p->bus_error=io->read(io->ctx,reg,v,n); return p->bus_error==GYRO_BUS_OK; }
static int wr(Mpu6050 *p,const GyroIo *io,uint8_t reg,uint8_t v)
{ p->bus_error=io->write(io->ctx,reg,v); return p->bus_error==GYRO_BUS_OK; }
void Mpu_Begin(Mpu6050 *p,uint32_t now)
{ memset(p,0,sizeof(*p)); p->due=now+GYRO_RESET_WAIT_MS; }

/* One transaction per step; reset/settling deadlines never block the app. */
int Mpu_Step(Mpu6050 *p,const GyroIo *io,uint32_t now)
{
    uint8_t v;
    if ((int32_t)(now-p->due)<0) return MPU_NO_DATA;
    switch (p->phase) {
    case 0u:
    case 3u:
        if (!rd(p,io,REG_WHO_AM_I,&p->who,1u)) return MPU_BUS_ERROR;
        if (p->who!=WHO_AM_I_MPU6050) return MPU_ID_ERROR;
        break;
    case 1u:
        if (!wr(p,io,REG_PWR_MGMT_1,BIT_DEVICE_RESET)) return MPU_BUS_ERROR;
        p->due=io->millis(io->ctx)+GYRO_RESET_WAIT_MS;
        break;
    case 2u:
        if (!rd(p,io,REG_PWR_MGMT_1,&v,1u)) return MPU_BUS_ERROR;
        /* No RESET_DONE interrupt: verify reset bit clear + sleep default. */
        if (v!=0x40u) return MPU_RESET_ERROR;
        break;
    case 4u:
        if (!p->verify) {
            if (!wr(p,io,settings[p->index][0],settings[p->index][1])) return MPU_BUS_ERROR;
            p->verify=1u;
            if (settings[p->index][0]==REG_PWR_MGMT_1)
                p->due=io->millis(io->ctx)+GYRO_STARTUP_MS;
        } else {
            if (!rd(p,io,settings[p->index][0],&v,1u)) return MPU_BUS_ERROR;
            if (v!=settings[p->index][1]) return MPU_CONFIG_ERROR;
            p->verify=0u;
            if (++p->index==sizeof(settings)/sizeof(settings[0])) { ++p->phase; return MPU_READY; }
        }
        return MPU_NO_DATA;
    default: return MPU_READY;
    }
    ++p->phase;
    return MPU_NO_DATA;
}

int Mpu_Fetch(Mpu6050 *p,const GyroIo *io,int16_t raw[3],uint32_t *sample_ms)
{
    uint8_t status, readings[15]; unsigned i; uint32_t began;
    if (!rd(p,io,REG_INT_STATUS,&status,1u)) return MPU_BUS_ERROR;
    if (!(status&BIT_DATA_RDY)) return MPU_NO_DATA;
    began=io->millis(io->ctx);
    /* Re-read status IN THE SAME burst as data (0x3A..0x48). A frame arriving
     * between the poll and burst must have its DRDY consumed too, otherwise
     * it would be published twice. Sensor shadow data is coherent throughout
     * the serial transfer. The second status may be zero (already cleared).
     * Skip accel/temp bytes; gyro starts at offset 9. */
    if (!rd(p,io,REG_INT_STATUS,readings,sizeof(readings))) return MPU_BUS_ERROR;
    *sample_ms=began+(uint32_t)(io->millis(io->ctx)-began)/2u;
    for (i=0u;i<3u;++i)
        raw[i]=(int16_t)(((uint16_t)readings[9u+i*2u]<<8)|readings[10u+i*2u]);
    /* MPU has no ICM invalid sentinel: all signed 16-bit codes are data. */
    for (i=0u;i<3u;++i)
        if (raw[i]>=GYRO_SATURATION_RAW || raw[i]<=-GYRO_SATURATION_RAW) return MPU_SATURATED;
    return MPU_READY;
}

/* One check per call. Reset is detected by configuration loss or no-DRDY timeout. */
int Mpu_Check(Mpu6050 *p,const GyroIo *io)
{
    uint8_t v,index=p->check_index;
    if (!index) {
        if (!rd(p,io,REG_WHO_AM_I,&p->who,1u)) return MPU_BUS_ERROR;
        if (p->who!=WHO_AM_I_MPU6050) return MPU_ID_ERROR;
    } else {
        if (!rd(p,io,settings[index-1u][0],&v,1u)) return MPU_BUS_ERROR;
        if (v!=settings[index-1u][1]) return MPU_CONFIG_ERROR;
    }
    p->check_index=(uint8_t)((index+1u)%(1u+sizeof(settings)/sizeof(settings[0])));
    return MPU_READY;
}
