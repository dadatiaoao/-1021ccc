/* USART1 belongs exclusively to K230. Legacy debug text is intentionally
   discarded, including touch-driver startup prints. No semihosting or UART I/O. */
#include "bsp_debug_usart.h"
#include "bsp_motor_uart.h"
#include "bsp_qr_uart.h"
#include "bsp_vision_uart.h"
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance==UART5) BspMotorUart_MspInit(uart);
    else if (uart->Instance==USART3) BspQrUart_MspInit(uart);
    else if (uart->Instance==USART1) BspVisionUart_MspInit(uart);
}
void DEBUG_USART_Config(void) { /* Retained for source compatibility; do not reinitialize USART1. */ }
void Usart_SendString(uint8_t *text) { (void)text; }
int fputc(int ch,FILE *file) { (void)file; return ch; }
int fgetc(FILE *file) { (void)file; return EOF; }
