#include "stm32f4xx_hal.h"
#include "gyro/bsp_gyro_bus.h"
#include "gyro/gyro_config.h"
#include <stdio.h>
#include <stdlib.h>
static unsigned checks,clock_calls[4],busy,low_pin,hal_calls,init_failed,busy_tail,low_tail;
static HAL_StatusTypeDef response;
static uint32_t response_error;
static I2C_HandleTypeDef *handle;
DwtMock dwt_mock;
CoreMock core_mock;
uint32_t SystemCoreClock=180000000u;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"GYRO BUS FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
void TestGyroClock(unsigned n) { CHECK(n<4u); ++clock_calls[n]; }
unsigned TestGyroFlag(I2C_HandleTypeDef *h,unsigned flag)
{ CHECK(h==handle && flag==I2C_FLAG_BUSY); if (busy_tail) { --busy_tail; return 1u; } return busy; }
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *g)
{
    CHECK(port==GPIOB && g->Pin==(GPIO_PIN_6|GPIO_PIN_7));
    CHECK(g->Mode==GPIO_MODE_AF_OD && g->Pull==GPIO_NOPULL && g->Alternate==GPIO_AF4_I2C1);
}
GPIO_PinState HAL_GPIO_ReadPin(void *port,uint16_t pin)
{ CHECK(port==GPIOB); if (low_tail) { --low_tail; return GPIO_PIN_RESET; } return pin==low_pin ? GPIO_PIN_RESET : GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h)
{
    handle=h; CHECK(h->Instance==I2C1 && h->Init.ClockSpeed==100000u);
    CHECK(h->Init.AddressingMode==I2C_ADDRESSINGMODE_7BIT);
    return init_failed ? HAL_ERROR : HAL_OK;
}
static HAL_StatusTypeDef transfer(I2C_HandleTypeDef *h,uint16_t address,uint16_t size,uint32_t timeout)
{
    CHECK(h==handle && address==(uint16_t)(GYRO_ADDRESS_7BIT<<1));
    CHECK(size==I2C_MEMADD_SIZE_8BIT && timeout==2u);
    ++hal_calls; h->ErrorCode=response_error; return response;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,uint16_t address,uint16_t reg,uint16_t size,uint8_t *v,uint16_t n,uint32_t timeout)
{ CHECK((reg==0x75u && n==1u) || (reg==0x3Au && n==15u));
  *v=0x68; return transfer(h,address,size,timeout); }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,uint16_t address,uint16_t reg,uint16_t size,uint8_t *v,uint16_t n,uint32_t timeout)
{ CHECK(reg==0x1Bu && n==1u && *v==0x08u); return transfer(h,address,size,timeout); }
uint32_t HAL_GetTick(void) { return 123u; }
int main(void)
{
    GyroIo io=BspGyroBus_Io(); uint8_t v,burst[15]; unsigned count,i;
    dwt_mock.CYCCNT=45678u; CHECK(io.init(io.ctx)==GYRO_BUS_OK);
    CHECK(dwt_mock.CYCCNT==45678u && io.cycles_per_us==180u);
    for (i=0u;i<4u;++i) CHECK(clock_calls[i]==1u);
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_OK && v==0x68u);
    CHECK(io.read(io.ctx,0x3Au,burst,15u)==GYRO_BUS_OK);
    CHECK(io.write(io.ctx,0x1Bu,0x08u)==GYRO_BUS_OK);
    busy_tail=3u; CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_OK && !busy_tail);
    low_tail=3u; CHECK(io.write(io.ctx,0x1Bu,0x08u)==GYRO_BUS_OK && !low_tail);
    count=hal_calls; busy=1u;
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_BUSY && hal_calls==count);
    CHECK(io.write(io.ctx,0x1Bu,0x08u)==GYRO_BUS_BUSY && hal_calls==count);
    busy=0u; low_pin=GPIO_PIN_6;
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_BUSY && hal_calls==count);
    low_pin=GPIO_PIN_7;
    CHECK(io.write(io.ctx,0x1Bu,0x08u)==GYRO_BUS_BUSY && hal_calls==count);
    low_pin=0u; response=HAL_ERROR; response_error=HAL_I2C_ERROR_AF;
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_NACK);
    response=HAL_TIMEOUT; CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_TIMEOUT);
    response=HAL_ERROR; response_error=HAL_I2C_ERROR_TIMEOUT;
    CHECK(io.write(io.ctx,0x1Bu,0x08u)==GYRO_BUS_TIMEOUT);
    response=HAL_BUSY; response_error=0u;
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_BUSY);
    response=HAL_ERROR; CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_IO);
    CHECK(io.read(io.ctx,0x75u,&v,0u)==GYRO_BUS_IO);
    CHECK(io.read(io.ctx,0x75u,&v,16u)==GYRO_BUS_IO);
    CHECK(io.read(io.ctx,0x75u,0,1u)==GYRO_BUS_IO);
    response=HAL_OK; CHECK(io.init(io.ctx)==GYRO_BUS_OK && dwt_mock.CYCCNT==45678u);
    CHECK(io.read(io.ctx,0x75u,&v,1u)==GYRO_BUS_OK);
    init_failed=1u; CHECK(io.init(io.ctx)==GYRO_BUS_IO);
    CHECK(io.millis(io.ctx)==123u && io.cycles(io.ctx)==45678u);
    printf("GYRO I2C1 PASS %u checks, address 0x%02X; stuck BUSY/lines skip HAL\n",checks,GYRO_ADDRESS_7BIT);
    return 0;
}
