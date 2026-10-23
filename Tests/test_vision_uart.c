#include "usart/bsp_vision_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"UART5 FAIL %d: %s\n",__LINE__,#x); exit(1); } } while (0)
USART_TypeDef fake_usart1, fake_usart3, fake_uart5;
DMA_Stream_TypeDef fake_dma2_stream2, fake_stream1, fake_stream7;
GPIO_TypeDef fake_gpioa, fake_gpiob, fake_gpioc, fake_gpiod;
DMA_Stream_TypeDef fake_dma2_stream7;
static UART_HandleTypeDef *uart;
static uint8_t *rx, *tx;
static uint16_t rx_size, tx_size;
static uint32_t tick;
static unsigned aborts, qr_events, qr_errors, gpio, irq, init_fail, dma_fail, rx_fail;
static HAL_StatusTypeDef tx_result;
void USART1_IRQHandler(void);
static unsigned rx_starts, rx_fake, tx_fake, tx_aborts, hal_errors, old_busy;
uint32_t __get_PRIMASK(void) { return 0u; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t v) { (void)v; }
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u)
{ uart=u; BspVisionUart_MspInit(u); u->gState=u->RxState=HAL_UART_STATE_READY; return init_fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *d)
{ d->State=old_busy && d->Instance==DMA2_Stream2 ? HAL_DMA_STATE_BUSY : HAL_DMA_STATE_READY;
  d->Instance->CR=0u; return dma_fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *d)
{ d->State=HAL_DMA_STATE_READY; d->Instance->CR=0u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ ++rx_starts; rx=b; rx_size=n; u->hdmarx->Instance->count=n;
  u->RxState=HAL_UART_STATE_BUSY_RX; u->Instance->CR3|=USART_CR3_DMAR;
  __HAL_UART_ENABLE_IT(u,UART_IT_PE); __HAL_UART_ENABLE_IT(u,UART_IT_ERR);
  if(!rx_fake && !rx_fail) { u->hdmarx->State=HAL_DMA_STATE_BUSY; u->hdmarx->Instance->CR|=DMA_SxCR_EN; }
  if(rx_fake>=2u) u->hdmarx->State=HAL_DMA_STATE_BUSY;
  if(rx_fake>=3u) u->hdmarx->Instance->CR|=DMA_SxCR_EN;
  if(rx_fake==3u) u->Instance->CR3&=~USART_CR3_DMAR;
  if(rx_fake==4u) u->RxState=HAL_UART_STATE_READY;
  u->ErrorCode=rx_fake==5u ? 1u : HAL_UART_ERROR_NONE;
  return rx_fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ if (tx_result!=HAL_OK) return tx_result; tx=b; tx_size=n; u->gState=HAL_UART_STATE_BUSY_TX;
  u->Instance->CR3|=USART_CR3_DMAT;
  if(!tx_fake) { u->hdmatx->State=HAL_DMA_STATE_BUSY; u->hdmatx->Instance->CR|=DMA_SxCR_EN; }
  return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ ++aborts; ++tx_aborts; u->gState=HAL_UART_STATE_READY; u->hdmatx->State=HAL_DMA_STATE_READY;
  u->hdmatx->Instance->CR=0u; u->Instance->CR3&=~USART_CR3_DMAT; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ ++aborts; u->RxState=HAL_UART_STATE_READY; u->hdmarx->State=HAL_DMA_STATE_READY;
  u->hdmarx->Instance->CR=0u; u->Instance->CR3&=~USART_CR3_DMAR; return HAL_OK; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g)
{ CHECK(g->Alternate==GPIO_AF7_USART1 && p==GPIOA && g->Pin==(GPIO_PIN_9|GPIO_PIN_10)); ++gpio; }
void HAL_NVIC_SetPriority(unsigned i,unsigned p,unsigned sub)
{ (void)i; CHECK(p==3u && sub==0u); ++irq; }
void HAL_NVIC_EnableIRQ(unsigned i) { (void)i; }
void HAL_NVIC_DisableIRQ(unsigned i) { (void)i; }
void HAL_NVIC_ClearPendingIRQ(unsigned i) { (void)i; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *u)
{
  if((u->Instance->SR & (UART_FLAG_PE|UART_FLAG_FE|UART_FLAG_NE|UART_FLAG_ORE)) &&
     (__HAL_UART_GET_IT_SOURCE(u,UART_IT_PE) || __HAL_UART_GET_IT_SOURCE(u,UART_IT_ERR))) {
    ++hal_errors; HAL_UART_ErrorCallback(u); __HAL_UART_CLEAR_OREFLAG(u); return;
  }
  if((u->Instance->SR & UART_FLAG_TC) && __HAL_UART_GET_IT_SOURCE(u,UART_IT_TC)) {
    u->Instance->SR&=~UART_FLAG_TC; __HAL_UART_DISABLE_IT(u,UART_IT_TC);
    u->gState=HAL_UART_STATE_READY; u->hdmatx->State=HAL_DMA_STATE_READY;
    u->hdmatx->Instance->CR=0u; HAL_UART_TxCpltCallback(u);
  }
}
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *d) { (void)d; }
void BspQrUart_RxEvent(void) { ++qr_events; }
void BspQrUart_Error(void) { ++qr_errors; }
void BspMotorUart_RxEvent(void) {}
void BspMotorUart_Error(void) {}
void BspMotorUart_TxComplete(void) {}
static void inject(const uint8_t *bytes,unsigned n)
{
    unsigned i;
    for (i=0u;i<n;++i) {
        rx[rx_size-fake_dma2_stream2.count]=bytes[i]; --fake_dma2_stream2.count;
        if (fake_dma2_stream2.count==rx_size/2u) HAL_UART_RxHalfCpltCallback(uart);
        if (!fake_dma2_stream2.count) { fake_dma2_stream2.count=rx_size; HAL_UART_RxCpltCallback(uart); }
    }
    fake_usart1.SR|=UART_FLAG_IDLE; USART1_IRQHandler();
}
static void complete(void)
{
    __HAL_UART_ENABLE_IT(uart,UART_IT_TC); fake_usart1.SR|=UART_FLAG_TC; USART1_IRQHandler();
}
int main(int argc,char **argv)
{
    VisionTransport t;
    VisionService service;
    const uint8_t heartbeat[]={0xAA,1,0x15,0x15};
    uint8_t bytes[16],saved[16],flood[800];
    unsigned i,k,before;
    uint32_t frames;
    if (argc>1) {
        init_fail=(unsigned)!strcmp(argv[1],"init"); dma_fail=(unsigned)!strcmp(argv[1],"dma");
        rx_fail=(unsigned)!strcmp(argv[1],"receive"); rx_fake=(unsigned)!strcmp(argv[1],"start");
    }
    t=BspVisionUart_Init(); VisionService_Init(&service,t,0u);
    if (init_fail || dma_fail || rx_fail || rx_fake) {
        CHECK(t.busy(0)); BspVisionUart_Pump(&service,0u); CHECK(service.transport_errors);
        before=rx_starts;
        for (i=1u;i<100u;++i) BspVisionUart_Pump(&service,i);
        CHECK(rx_starts==before && t.busy(0));
        init_fail=dma_fail=rx_fail=rx_fake=0u; tick=100u; BspVisionUart_Pump(&service,tick);
        CHECK(!t.busy(0)); inject(heartbeat,4u); BspVisionUart_Pump(&service,tick);
        CHECK(service.online && service.parser.frames==1u);
        printf("PASS: USART1 init recovery %s\n",argv[1]); return 0;
    }
    CHECK(gpio==1u && irq==3u && uart->Instance==USART1 && uart->hdmarx->Instance==DMA2_Stream2 &&
        uart->hdmatx->Instance==DMA2_Stream7 && rx_size==256u && uart->Init.BaudRate==115200u);
    if (argc>1 && !strcmp(argv[1],"review")) {
        const uint32_t faults[]={UART_FLAG_PE,UART_FLAG_FE,UART_FLAG_NE,UART_FLAG_ORE};
        for (k=0u;k<12u;++k) {
            tick=200u*k; frames=service.parser.frames; before=service.transport_errors;
            service.parser.bytes[0]=0xAA; service.parser.used=1u;
            memcpy(rx,heartbeat,4u); fake_dma2_stream2.count=rx_size-4u;
            fake_usart1.SR=(k<8u ? UART_FLAG_IDLE : 0u)|faults[k%4u];
            if (k>=4u) HAL_UART_RxHalfCpltCallback(uart);
            USART1_IRQHandler(); BspVisionUart_Pump(&service,tick);
            CHECK(service.transport_errors>before && service.parser.frames==frames && !service.parser.used);
            CHECK(!t.busy(0)); inject(heartbeat,4u); BspVisionUart_Pump(&service,tick);
            CHECK(service.parser.frames==frames+1u);
        }
        CHECK(t.send(0,heartbeat,4u)); before=tx_aborts;
        HAL_UART_ErrorCallback(uart); BspVisionUart_Pump(&service,++tick);
        CHECK(tx_aborts==before && t.busy(0)); complete(); CHECK(!t.busy(0));
        CHECK(t.send(0,heartbeat,4u)); before=tx_aborts;
        __HAL_UART_ENABLE_IT(uart,UART_IT_TC); fake_usart1.SR=UART_FLAG_ORE|UART_FLAG_IDLE|UART_FLAG_TC;
        USART1_IRQHandler(); BspVisionUart_Pump(&service,++tick);
        CHECK(tx_aborts==before && !t.busy(0));
        for (k=1u;k<=5u;++k) {
            rx_fake=k; HAL_UART_ErrorCallback(uart); BspVisionUart_Pump(&service,++tick);
            CHECK(t.busy(0)); before=rx_starts;
            for (i=1u;i<100u;++i) BspVisionUart_Pump(&service,tick+i);
            CHECK(before==rx_starts); rx_fake=0u; tick+=100u;
            BspVisionUart_Pump(&service,tick); CHECK(!t.busy(0));
        }
        old_busy=1u; HAL_UART_ErrorCallback(uart); before=rx_starts;
        BspVisionUart_Pump(&service,++tick); CHECK(t.busy(0) && before==rx_starts);
        old_busy=0u; tick+=100u; BspVisionUart_Pump(&service,tick); CHECK(!t.busy(0));
        tx_fake=1u; CHECK(!t.send(0,heartbeat,4u)); BspVisionUart_Pump(&service,++tick); CHECK(!t.busy(0));
        CHECK(hal_errors==0u); printf("PASS: USART1 IRQ errors, DMA false OK, TX preservation (%u)\n",checks);
        return 0;
    }
    for(i=0u;i<16u;++i) bytes[i]=(uint8_t)i;
    memcpy(saved,bytes,16u); CHECK(t.send(0,bytes,16u) && tx_size==16u);
    memset(bytes,0,16u); CHECK(!memcmp(tx,saved,16u));
    CHECK(!t.send(0,bytes,4u) && !memcmp(tx,saved,16u)); complete();
    CHECK(!t.send(0,bytes,17u) && !t.send(0,bytes,0u));
    tx_result=HAL_BUSY; CHECK(!t.send(0,bytes,4u)); tx_result=HAL_OK;
    for(i=0u;i<sizeof(flood);++i) flood[i]=heartbeat[i%4u];
    tick=10u; inject(flood,192u); BspVisionUart_Pump(&service,tick); CHECK(service.parser.frames==32u);
    BspVisionUart_Pump(&service,tick); CHECK(service.parser.frames==48u && service.last_heartbeat_ms==10u);
    tick=20u; inject(heartbeat,4u); tick=5000u; BspVisionUart_Pump(&service,tick);
    VisionService_Process(&service,tick); CHECK(!service.online && service.last_heartbeat_ms==20u);
    tick=5100u; inject(flood,800u); BspVisionUart_Pump(&service,tick);
    CHECK(service.error==VISION_ERROR_OVERFLOW && service.transport_errors==1u && !service.parser.used);
    CHECK(!t.busy(0)); tick=5200u; CHECK(t.send(0,bytes,16u)); before=tx_aborts;
    tick=5221u; BspVisionUart_Pump(&service,tick); CHECK(tx_aborts==before);
    tick=5222u; BspVisionUart_Pump(&service,tick); CHECK(tx_aborts>before && !t.busy(0));
    CHECK(service.error==VISION_ERROR_UART);
    CHECK(!qr_events && !qr_errors);
    printf("PASS: %u real USART1 BSP checks (DMA, timestamps, overflow, watchdog)\n",checks);
    return 0;
}
