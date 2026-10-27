#include "bsp_limit_gpio.h"
#include "stm32f4xx_hal.h"
void BspLimit_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3; /* ARM J7-41, LIFT J7-43 */
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOG, &gpio);
}
uint8_t BspLimit_Read(unsigned axis)
{
    return (uint8_t)(HAL_GPIO_ReadPin(GPIOG, axis == 0u ? GPIO_PIN_2 : GPIO_PIN_3) == GPIO_PIN_SET);
}
