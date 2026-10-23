#ifndef BSP_LIMIT_GPIO_H
#define BSP_LIMIT_GPIO_H
#include <stdint.h>
void BspLimit_Init(void);
/* NC to GND: released=0, pressed OR open circuit=1. */
uint8_t BspLimit_Read(unsigned axis);
#endif
