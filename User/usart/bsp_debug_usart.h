#ifndef BSP_DEBUG_USART_H
#define BSP_DEBUG_USART_H
#include "stm32f4xx_hal.h"
#include <stdio.h>
/* Retired debug API: USART1 is now the binary vision link. */
void DEBUG_USART_Config(void);
void Usart_SendString(uint8_t *text);
#endif
