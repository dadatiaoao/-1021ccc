#include "usart/bsp_vision_uart.h"
VisionTransport BspVisionUart_Init(void) { VisionTransport t={0}; return t; }
void BspVisionUart_Pump(VisionService *s,uint32_t now) { (void)s; (void)now; }
void BspVisionUart_RxEvent(void) {}
void BspVisionUart_Error(void) {}
void BspVisionUart_TxComplete(void) {}
