#ifndef GYRO_HAL_STUB_H
#define GYRO_HAL_STUB_H
#include <stdint.h>
typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET,GPIO_PIN_SET } GPIO_PinState;
typedef struct { unsigned ClockSpeed,DutyCycle,OwnAddress1,AddressingMode,DualAddressMode,
    OwnAddress2,GeneralCallMode,NoStretchMode; } I2C_InitTypeDef;
typedef struct { uint32_t SR1,SR2,CR1; } I2C_TypeDef;
extern I2C_TypeDef gyro_test_regs;
typedef struct { I2C_TypeDef *Instance; I2C_InitTypeDef Init; uint32_t ErrorCode; } I2C_HandleTypeDef;
typedef struct { uint32_t Pin,Mode,Pull,Speed,Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t CTRL,CYCCNT; } DwtMock;
typedef struct { uint32_t DEMCR; } CoreMock;
extern DwtMock dwt_mock;
extern CoreMock core_mock;
extern uint32_t SystemCoreClock;
#define DWT (&dwt_mock)
#define CoreDebug (&core_mock)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define I2C1 (&gyro_test_regs)
#define GPIOB ((void *)2)
#define GPIO_PIN_6 0x40u
#define GPIO_PIN_7 0x80u
#define GPIO_MODE_AF_OD 4u
#define GPIO_MODE_INPUT 0u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_HIGH 3u
#define GPIO_AF4_I2C1 4u
#define I2C_DUTYCYCLE_2 0u
#define I2C_ADDRESSINGMODE_7BIT 7u
#define I2C_DUALADDRESS_DISABLE 0u
#define I2C_GENERALCALL_DISABLE 0u
#define I2C_NOSTRETCH_DISABLE 0u
#define I2C_FLAG_BUSY 2u
#define I2C_MEMADD_SIZE_8BIT 1u
#define HAL_I2C_ERROR_TIMEOUT 0x20u
#define HAL_I2C_ERROR_AF 4u
void TestGyroClock(unsigned n);
#define __HAL_RCC_GPIOB_CLK_ENABLE() TestGyroClock(0u)
#define __HAL_RCC_I2C1_CLK_ENABLE() TestGyroClock(1u)
#define __HAL_RCC_I2C1_FORCE_RESET() TestGyroClock(2u)
#define __HAL_RCC_I2C1_RELEASE_RESET() TestGyroClock(3u)
unsigned TestGyroFlag(I2C_HandleTypeDef *h,unsigned flag);
void TestGyroDisable(I2C_HandleTypeDef *h);
#define __HAL_I2C_DISABLE(h) TestGyroDisable(h)
#define __HAL_I2C_GET_FLAG(h,f) TestGyroFlag(h,f)
void HAL_GPIO_Init(void *port,GPIO_InitTypeDef *g);
GPIO_PinState HAL_GPIO_ReadPin(void *port,uint16_t pin);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,uint16_t address,uint16_t reg,uint16_t size,uint8_t *v,uint16_t n,uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,uint16_t address,uint16_t reg,uint16_t size,uint8_t *v,uint16_t n,uint32_t timeout);
uint32_t HAL_GetTick(void);
#endif
