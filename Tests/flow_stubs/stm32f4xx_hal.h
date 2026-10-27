#ifndef FLOW_HAL_STUB_H
#define FLOW_HAL_STUB_H
#include "../stubs/stm32f4xx_hal.h"
typedef struct { unsigned unused; } SPI_TypeDef;
extern SPI_TypeDef fake_spi2;
extern GPIO_TypeDef fake_gpioe;
#define SPI2 (&fake_spi2)
#define GPIOE (&fake_gpioe)
#define GPIO_PIN_13 (1u<<13)
#define GPIO_SPEED_FREQ_HIGH 2u
#define GPIO_AF5_SPI2 5u
#define SPI_MODE_MASTER 1u
#define SPI_DIRECTION_2LINES 2u
#define SPI_DATASIZE_8BIT 8u
#define SPI_POLARITY_HIGH 1u
#define SPI_PHASE_2EDGE 2u
#define SPI_NSS_SOFT 1u
#define SPI_BAUDRATEPRESCALER_64 64u
#define SPI_FIRSTBIT_MSB 0u
#define SPI_TIMODE_DISABLE 0u
#define SPI_CRCCALCULATION_DISABLE 0u
typedef struct { uint32_t Mode,Direction,DataSize,CLKPolarity,CLKPhase,NSS,BaudRatePrescaler,FirstBit,TIMode,CRCCalculation,CRCPolynomial; } SPI_InitTypeDef;
struct SPI_HandleTypeDef { SPI_TypeDef *Instance; SPI_InitTypeDef Init; };
typedef struct { uint32_t CTRL, CYCCNT; } FlowDwt;
typedef struct { uint32_t DEMCR; } FlowDebug;
extern FlowDwt fake_dwt;
extern FlowDebug fake_debug;
extern uint32_t SystemCoreClock;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
uint32_t FakeFlowCycles(void);
#define BSP_FLOW_CYCLES() FakeFlowCycles()
void FakeFlowRcc(unsigned what);
#define __HAL_RCC_GPIOE_CLK_ENABLE() FakeFlowRcc(1u)
#define __HAL_RCC_SPI2_CLK_ENABLE() FakeFlowRcc(2u)
#define __HAL_RCC_SPI2_FORCE_RESET() FakeFlowRcc(3u)
#define __HAL_RCC_SPI2_RELEASE_RESET() FakeFlowRcc(4u)
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,uint8_t *tx,uint8_t *rx,uint16_t size,uint32_t timeout);
#endif
