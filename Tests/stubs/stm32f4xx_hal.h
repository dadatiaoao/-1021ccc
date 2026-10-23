#ifndef HAL_TEST_STUB_H
#define HAL_TEST_STUB_H
#include <stdint.h>
#include <stddef.h>
typedef struct SPI_HandleTypeDef SPI_HandleTypeDef;

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t SR, CR1, CR3; } USART_TypeDef;
typedef struct { uint32_t count, CR; } DMA_Stream_TypeDef;
typedef struct { unsigned unused; } GPIO_TypeDef;
extern USART_TypeDef fake_usart1, fake_usart3, fake_uart5;
extern DMA_Stream_TypeDef fake_stream1, fake_dma2_stream2, fake_dma2_stream7;
extern DMA_Stream_TypeDef fake_stream0, fake_stream7;
extern GPIO_TypeDef fake_gpiob, fake_gpioa;
extern GPIO_TypeDef fake_gpioc, fake_gpiod;
#define USART1 (&fake_usart1)
#define USART3 (&fake_usart3)
#define UART5 (&fake_uart5)
#define GPIOA (&fake_gpioa)
#define DMA2_Stream2 (&fake_dma2_stream2)
#define DMA2_Stream7 (&fake_dma2_stream7)
#define GPIOB (&fake_gpiob)
#define GPIOC (&fake_gpioc)
#define GPIOD (&fake_gpiod)
#define DMA1_Stream1 (&fake_stream1)
#define DMA1_Stream0 (&fake_stream0)
#define DMA1_Stream7 (&fake_stream7)
typedef struct {
    uint32_t Channel, Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment;
    uint32_t Mode, Priority, FIFOMode;
} DMA_InitTypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; DMA_InitTypeDef Init; unsigned State; void *Parent; } DMA_HandleTypeDef;
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct { USART_TypeDef *Instance; UART_InitTypeDef Init; DMA_HandleTypeDef *hdmarx, *hdmatx; uint32_t gState, RxState, ErrorCode; } UART_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define HAL_DMA_STATE_READY 1u
#define HAL_DMA_STATE_BUSY 2u
#define DMA_CHANNEL_4 4u
#define DMA_PERIPH_TO_MEMORY 0u
#define DMA_MEMORY_TO_PERIPH 1u
#define DMA_NORMAL 0u
#define DMA_PRIORITY_HIGH 2u
#define HAL_UART_STATE_READY 0u
#define HAL_UART_STATE_RESET 32u
#define HAL_UART_STATE_BUSY_TX 1u
#define HAL_UART_STATE_BUSY_RX 2u
#define HAL_UART_ERROR_NONE 0u
#define DMA_SxCR_EN 1u
#define USART_CR3_DMAR 64u
#define USART_CR3_DMAT 128u
#define DMA_PINC_DISABLE 0u
#define DMA_MINC_ENABLE 1u
#define DMA_PDATAALIGN_BYTE 0u
#define DMA_MDATAALIGN_BYTE 0u
#define DMA_CIRCULAR 1u
#define DMA_PRIORITY_MEDIUM 1u
#define DMA_FIFOMODE_DISABLE 0u
#define GPIO_PIN_9 (1u << 9)
#define GPIO_AF7_USART1 7u
#define USART1_IRQn 37u
#define DMA2_Stream2_IRQn 58u
#define DMA2_Stream7_IRQn 70u
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA2_CLK_ENABLE() ((void)0)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_2 (1u << 2)
#define GPIO_MODE_AF_PP 2u
#define GPIO_MODE_OUTPUT_PP 1u
#define GPIO_PULLUP 1u
#define GPIO_NOPULL 0u
#define GPIO_SPEED_FREQ_LOW 0u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
#define GPIO_AF7_USART3 7u
#define GPIO_AF8_UART5 8u
#define GPIO_PIN_RESET 0u
#define GPIO_PIN_SET 1u
#define UART_WORDLENGTH_8B 0u
#define UART_STOPBITS_1 0u
#define UART_PARITY_NONE 0u
#define UART_MODE_TX_RX 3u
#define UART_HWCONTROL_NONE 0u
#define UART_OVERSAMPLING_16 0u
#define UART_FLAG_PE 1u
#define UART_FLAG_FE 2u
#define UART_FLAG_NE 4u
#define UART_FLAG_ORE 8u
#define UART_FLAG_IDLE 16u
#define UART_FLAG_TC 256u
#define UART_IT_TC 256u
#define UART_IT_IDLE 16u
#define UART_IT_ERR 32u
#define UART_IT_PE 64u
#define USART3_IRQn 39u
#define DMA1_Stream1_IRQn 12u
#define DMA1_Stream0_IRQn 11u
#define DMA1_Stream7_IRQn 17u
#define UART5_IRQn 53u
#define __HAL_RCC_UART5_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART3_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_ENABLE() ((void)0)
#define __HAL_DMA_GET_COUNTER(d) ((d)->Instance->count)
#define __HAL_LINKDMA(u,field,d) do { (u)->field = &(d); (d).Parent = (u); } while (0)
#define __HAL_UART_CLEAR_OREFLAG(u) ((u)->Instance->SR &= ~(UART_FLAG_PE|UART_FLAG_FE|UART_FLAG_NE|UART_FLAG_ORE|UART_FLAG_IDLE))
#define __HAL_UART_CLEAR_IDLEFLAG(u) __HAL_UART_CLEAR_OREFLAG(u)
#define __HAL_UART_ENABLE_IT(u,it) ((it)==UART_IT_ERR ? ((u)->Instance->CR3|=(it)) : ((u)->Instance->CR1|=(it)))
#define __HAL_UART_DISABLE_IT(u,it) ((it)==UART_IT_ERR ? ((u)->Instance->CR3&=~(it)) : ((u)->Instance->CR1&=~(it)))
#define __HAL_UART_GET_FLAG(u,f) ((u)->Instance->SR & (f))
#define __HAL_UART_GET_IT_SOURCE(u,it) ((it)==UART_IT_ERR ? ((u)->Instance->CR3&(it)) : ((u)->Instance->CR1&(it)))
#define __DMB() ((void)0)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n);
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *u);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *d);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *d);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *d);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state);
void HAL_NVIC_SetPriority(unsigned irq, unsigned preempt, unsigned sub);
void HAL_NVIC_EnableIRQ(unsigned irq);
void HAL_NVIC_DisableIRQ(unsigned irq);
void HAL_NVIC_ClearPendingIRQ(unsigned irq);
void HAL_UART_IRQHandler(UART_HandleTypeDef *u);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *u);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *u);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *u);
#endif
