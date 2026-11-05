/*
 * 陀螺硬件适配：专用 I2C1，PB6=SCL、PB7=SDA，开漏且依赖外部上拉。
 * 使用配置的 7 位地址，交给 HAL 时左移一位；触摸使用 I2C2，两者资源独立。
 * 提供有限超时访问与共享 DWT 耗时计数，不在中断里访问器件。
 */
#include "bsp_gyro_bus.h"
#include "gyro_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>
static I2C_HandleTypeDef gyro_i2c;
/* Touch configures I2C2 GPIO itself and uses the HAL weak MSP. Keep that path
 * untouched: initialise ONLY this private I2C1 instance's pins/clock here. */
/*
 * 只复位/配置本陀螺 I2C1 及引脚；启用 DWT 但不清共享计数器，保留光流计时连续性。
 */
static GyroBusError init(void *ctx)
{
    GPIO_InitTypeDef g; (void)ctx;
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET();
    memset(&g,0,sizeof(g)); g.Pin=GPIO_PIN_6|GPIO_PIN_7;
    g.Mode=GPIO_MODE_AF_OD; g.Pull=GPIO_NOPULL;
    g.Speed=GPIO_SPEED_FREQ_HIGH; g.Alternate=GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB,&g);
    memset(&gyro_i2c,0,sizeof(gyro_i2c)); gyro_i2c.Instance=I2C1;
    gyro_i2c.Init.ClockSpeed=GYRO_I2C_HZ; gyro_i2c.Init.DutyCycle=I2C_DUTYCYCLE_2;
    gyro_i2c.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    gyro_i2c.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    gyro_i2c.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    gyro_i2c.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk; /* Shared with flow; never reset CYCCNT. */
    return HAL_I2C_Init(&gyro_i2c)==HAL_OK ? GYRO_BUS_OK : GYRO_BUS_IO;
}
static GyroBusError result(HAL_StatusTypeDef r)
{
    if (r==HAL_OK) return GYRO_BUS_OK;
    if (r==HAL_BUSY) return GYRO_BUS_BUSY;
    if (r==HAL_TIMEOUT || (gyro_i2c.ErrorCode&HAL_I2C_ERROR_TIMEOUT)) return GYRO_BUS_TIMEOUT;
    return (gyro_i2c.ErrorCode&HAL_I2C_ERROR_AF) ? GYRO_BUS_NACK : GYRO_BUS_IO;
}
/*
 * 进入旧 HAL Mem_Read/Write 前检查 BUSY 与 SCL/SDA，
 * 给正常 STOP 尾部有限等待，并用 DWT/自旋双上限避免触发 HAL 内部长 BUSY 等待。
 */
static GyroBusError preflight(void)
{
    uint32_t began=DWT->CYCCNT,limit=(SystemCoreClock/1000000u)*200u,spins=20000u;
    /* HAL 1.7.3 Mem_Read/Write use a fixed 25ms BUSY wait before their supplied
       timeout. Dedicated single-master I2C1 lets us reject it BEFORE HAL.
       Stuck lines are never bit-banged; bounded retries reset I2C1 only. */
    /* HAL returns before its own STOP necessarily clears BUSY. Allow that
       short tail to finish; a stuck bus gets a 200us cycle deadline (plus IRQ
       latency), never HAL's 25ms. If DWT stops, the finite spin cap applies. */
    while (__HAL_I2C_GET_FLAG(&gyro_i2c,I2C_FLAG_BUSY) ||
        HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_RESET) {
        if ((uint32_t)(DWT->CYCCNT-began)>=limit || --spins==0u) return GYRO_BUS_BUSY;
    }
    return GYRO_BUS_OK;
}
/*
 * 有界寄存器读取：先验证长度与总线空闲，再用 HAL 内存访问并映射 NACK/忙/超时等错误。
 */
static GyroBusError read_reg(void *ctx,uint8_t reg,uint8_t *v,size_t n)
{
    GyroBusError e; (void)ctx;
    if (n==0u || n>6u || !v) return GYRO_BUS_IO;
    e=preflight(); if (e!=GYRO_BUS_OK) return e;
    return result(HAL_I2C_Mem_Read(&gyro_i2c,(uint16_t)(GYRO_ADDRESS_7BIT<<1),reg,
        I2C_MEMADD_SIZE_8BIT,v,(uint16_t)n,GYRO_BUS_TIMEOUT_MS));
}
/*
 * 写一个配置寄存器，沿用读操作相同的总线预检和有限 HAL 超时。
 */
static GyroBusError write_reg(void *ctx,uint8_t reg,uint8_t v)
{
    GyroBusError e; (void)ctx;
    e=preflight(); if (e!=GYRO_BUS_OK) return e;
    return result(HAL_I2C_Mem_Write(&gyro_i2c,(uint16_t)(GYRO_ADDRESS_7BIT<<1),reg,
        I2C_MEMADD_SIZE_8BIT,&v,1u,GYRO_BUS_TIMEOUT_MS));
}
static uint32_t millis(void *ctx) { (void)ctx; return HAL_GetTick(); }
static uint32_t cycles(void *ctx) { (void)ctx; return DWT->CYCCNT; }
/*
 * 将专用 I2C、毫秒时钟和周期计数封装为 GyroIo，供器件驱动与服务共同使用。
 */
GyroIo BspGyroBus_Io(void)
{
    GyroIo io={0,init,read_reg,write_reg,millis,cycles,0u};
    io.cycles_per_us=SystemCoreClock/1000000u; return io;
}
