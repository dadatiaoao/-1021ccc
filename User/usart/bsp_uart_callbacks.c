/*
 * 工程共享的 HAL UART 回调唯一入口：按 Instance 分发 UART5 电机、USART3 扫码和 USART1 视觉事件。
 * 半满/全满仅通知接收搬运，错误回调仅登记故障，电机 TX 完成只解除在途标志。
 * 新增串口回调应在此路由，避免其它文件重复定义同名强符号。
 */
#include "bsp_motor_uart.h"
#include "bsp_qr_uart.h"
#include "bsp_vision_uart.h"

/* Exactly one strong definition per HAL callback, with explicit instance routing. */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART5) BspMotorUart_RxEvent();
    else if (uart->Instance == USART3) BspQrUart_RxEvent();
    else if (uart->Instance == USART1) BspVisionUart_RxEvent();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART5) BspMotorUart_RxEvent();
    else if (uart->Instance == USART3) BspQrUart_RxEvent();
    else if (uart->Instance == USART1) BspVisionUart_RxEvent();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART5) BspMotorUart_Error();
    else if (uart->Instance == USART3) BspQrUart_Error();
    else if (uart->Instance == USART1) BspVisionUart_Error();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART5) BspMotorUart_TxComplete();
    else if (uart->Instance == USART1) BspVisionUart_TxComplete();
}
