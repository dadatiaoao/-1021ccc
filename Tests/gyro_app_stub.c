#include "gyro/bsp_gyro_bus.h"
#include "gyro_fixture.h"
#include "stm32f4xx_hal.h"
static GyroMock sensor;
void GyroTest_Fail(unsigned value) { sensor.failure=value ? GYRO_BUS_NACK : GYRO_BUS_OK; }
void GyroTest_Raw(unsigned axis,int16_t value) { sensor.raw[axis]=value; }
static GyroBusError read_app(void *ctx,uint8_t reg,uint8_t *v,size_t n)
{ sensor.now=HAL_GetTick(); return mock_read(ctx,reg,v,n); }
static GyroBusError write_app(void *ctx,uint8_t reg,uint8_t v)
{ sensor.now=HAL_GetTick(); return mock_write(ctx,reg,v); }
static uint32_t ms_app(void *ctx) { (void)ctx; return HAL_GetTick(); }
GyroIo BspGyroBus_Io(void)
{
    GyroIo io;
    GyroMock_Reset(&sensor,HAL_GetTick()); io=GyroMock_Io(&sensor);
    io.read=read_app; io.write=write_app; io.millis=ms_app; return io;
}
