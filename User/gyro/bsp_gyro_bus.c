/* Software I2C feasibility probe: PB6=SCL, PB7=SDA, external 3.3V pullups.
 * No hardware I2C transfers, sensor writes, bus-clear pulses or auto reset. */
#include "bsp_gyro_bus.h"
#include "gyro_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#define SCL GPIO_PIN_6
#define SDA GPIO_PIN_7
#define HALF_US 10u /* Nominal <=50kHz; GPIO overhead and IRQs make it slower. */
static void line(uint16_t pin,GPIO_PinState v) { HAL_GPIO_WritePin(GPIOB,pin,v); }
static int high(uint16_t pin) { return HAL_GPIO_ReadPin(GPIOB,pin)==GPIO_PIN_SET; }
static void delay_us(uint32_t us)
{
    uint32_t begin=DWT->CYCCNT,limit=(SystemCoreClock/1000000u)*us;
    while ((uint32_t)(DWT->CYCCNT-begin)<limit) { }
}
static void release(void) { line(SDA,GPIO_PIN_SET); line(SCL,GPIO_PIN_SET); }
static GyroBusError clock_high(void)
{
    uint32_t begin=DWT->CYCCNT,limit=(SystemCoreClock/1000000u)*2000u,spins=100000u;
    line(SCL,GPIO_PIN_SET);
    while (!high(SCL)) {
        if ((uint32_t)(DWT->CYCCNT-begin)>=limit || --spins==0u) return GYRO_BUS_TIMEOUT;
    }
    delay_us(HALF_US); return GYRO_BUS_OK;
}
static GyroBusError stop(void)
{
    GyroBusError e;
    line(SCL,GPIO_PIN_RESET); line(SDA,GPIO_PIN_RESET); delay_us(HALF_US);
    e=clock_high(); if (e!=GYRO_BUS_OK) { release(); return e; }
    line(SDA,GPIO_PIN_SET); delay_us(HALF_US);
    return high(SDA) ? GYRO_BUS_OK : GYRO_BUS_BUSY;
}
static GyroBusError start(void)
{
    GyroBusError e;
    line(SDA,GPIO_PIN_SET); delay_us(HALF_US);
    e=clock_high(); if (e!=GYRO_BUS_OK) return e;
    if (!high(SDA)) return GYRO_BUS_BUSY;
    line(SDA,GPIO_PIN_RESET); delay_us(HALF_US); line(SCL,GPIO_PIN_RESET);
    return GYRO_BUS_OK;
}
static GyroBusError send(uint8_t value)
{
    unsigned bit; GyroBusError e; int ack;
    for (bit=0u;bit<8u;++bit) {
        line(SDA,(value&0x80u) ? GPIO_PIN_SET : GPIO_PIN_RESET); delay_us(HALF_US);
        e=clock_high(); if (e!=GYRO_BUS_OK) return e;
        /* Unexpected low while sending 1: release ownership, do not force STOP. */
        if ((value&0x80u) && !high(SDA)) return GYRO_BUS_BUSY;
        line(SCL,GPIO_PIN_RESET); value=(uint8_t)(value<<1);
    }
    line(SDA,GPIO_PIN_SET); delay_us(HALF_US);
    e=clock_high(); if (e!=GYRO_BUS_OK) return e;
    ack=!high(SDA); line(SCL,GPIO_PIN_RESET);
    return ack ? GYRO_BUS_OK : GYRO_BUS_NACK;
}
static GyroBusError receive(uint8_t *value,int last)
{
    unsigned bit; uint8_t v=0u; GyroBusError e;
    line(SDA,GPIO_PIN_SET);
    for (bit=0u;bit<8u;++bit) {
        delay_us(HALF_US); e=clock_high(); if (e!=GYRO_BUS_OK) return e;
        v=(uint8_t)((v<<1)|(high(SDA) ? 1u : 0u)); line(SCL,GPIO_PIN_RESET);
    }
    line(SDA,last ? GPIO_PIN_SET : GPIO_PIN_RESET); delay_us(HALF_US);
    e=clock_high(); if (e!=GYRO_BUS_OK) return e;
    line(SCL,GPIO_PIN_RESET); line(SDA,GPIO_PIN_SET); *value=v;
    return GYRO_BUS_OK;
}
static GyroBusError init(void *ctx)
{
    GPIO_InitTypeDef g; (void)ctx;
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    release(); /* Set output latch HIGH before enabling open-drain GPIO. */
    memset(&g,0,sizeof(g)); g.Pin=SCL|SDA; g.Mode=GPIO_MODE_OUTPUT_OD;
    g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOB,&g);
    return GYRO_BUS_OK;
}
static GyroBusError read_reg(void *ctx,uint8_t reg,uint8_t *v,size_t n)
{
    GyroBusError e,tail; size_t i; (void)ctx;
    if (!v || !n || n>6u) return GYRO_BUS_IO;
    if (!high(SCL) || !high(SDA)) return GYRO_BUS_BUSY;
    e=start();
    if (e==GYRO_BUS_OK) e=send((uint8_t)(GYRO_ADDRESS_7BIT<<1));
    if (e==GYRO_BUS_OK) e=send(reg);
    if (e==GYRO_BUS_OK) e=start();
    if (e==GYRO_BUS_OK) e=send((uint8_t)((GYRO_ADDRESS_7BIT<<1)|1u));
    for (i=0u;e==GYRO_BUS_OK && i<n;++i) e=receive(&v[i],i+1u==n);
    if (e==GYRO_BUS_OK || e==GYRO_BUS_NACK) {
        tail=stop(); if (e==GYRO_BUS_OK) e=tail;
    } else release();
    return e;
}
static GyroBusError write_reg(void *ctx,uint8_t reg,uint8_t value)
{ (void)ctx; (void)reg; (void)value; return GYRO_BUS_IO; }
static uint32_t millis(void *ctx) { (void)ctx; return HAL_GetTick(); }
static uint32_t cycles(void *ctx) { (void)ctx; return DWT->CYCCNT; }
GyroIo BspGyroBus_Io(void)
{
    GyroIo io={0,init,read_reg,write_reg,millis,cycles,0u};
    io.cycles_per_us=SystemCoreClock/1000000u; return io;
}
