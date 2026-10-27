#ifndef BSP_VISION_UART_H
#define BSP_VISION_UART_H
#include "stm32f4xx_hal.h"
#include "../vision/vision_service.h"
VisionTransport BspVisionUart_Init(void);
void BspVisionUart_Pump(VisionService *service,uint32_t now_ms);
void BspVisionUart_MspInit(UART_HandleTypeDef *uart);
void BspVisionUart_RxEvent(void);
void BspVisionUart_Error(void);
void BspVisionUart_TxComplete(void);
#endif
