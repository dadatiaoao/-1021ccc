/* Standalone N10 UART4 diagnostic. No motor or mechanism control. */
#include "main.h"
#include "./lcd/bsp_lcd.h"
#include <stdio.h>
#include <stdint.h>
#define N 4096u
static volatile uint8_t queue[N];
static volatile uint32_t wr,rd,overrun,errors,bytes;
static uint8_t frame[58]; static unsigned pos;
static uint32_t good,bad,last_valid,last_front; static int front_mm=-1;
void UART4_IRQHandler(void) {
 uint32_t status=UART4->SR;
 if(status & (USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {
  uint8_t b=(uint8_t)UART4->DR; uint32_t next=(wr+1u)%N;
  if(status & (USART_SR_ORE|USART_SR_FE|USART_SR_NE)) errors++;
  if(status & USART_SR_RXNE) { bytes++; if(next!=rd) { queue[wr]=b; wr=next; } else overrun++; }
 }
}
static void consume(uint8_t b) {
 unsigned j; unsigned sum=0; float a,start,end,delta; int dist;
 if(pos==0 && b!=0xa5) return;
 if(pos==1 && b!=0x5a) { pos=(b==0xa5)?1:0; return; }
 if(pos==2 && b!=58) { pos=(b==0xa5)?1:0; return; }
 frame[pos++]=b; if(pos<58) return; pos=0;
 for(j=0;j<57;j++) sum+=frame[j];
 if((uint8_t)sum!=frame[57]) {bad++; return;}
 good++; last_valid=HAL_GetTick();
 start=(frame[5]*256u+frame[6])/100.0f; end=(frame[55]*256u+frame[56])/100.0f;
 delta=end-start; if(delta<0) delta+=360.0f;
 for(j=0;j<16;j++) { a=start+delta*j/15.0f; if(a>=360) a-=360; dist=frame[7+3*j]*256+frame[8+3*j];
  if((a<5.0f || a>355.0f) && dist>=100 && dist<25000) {front_mm=dist; last_front=last_valid;}
 }
}
int main(void) {
 GPIO_InitTypeDef g; char s[90]; uint32_t last=0,now;
 HAL_Init(); SystemClock_Config(); LCD_Init(); LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888); LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); LCD_SetFont(&Font24); LCD_SetTextColor(LCD_COLOR_WHITE); LCD_SetBackColor(LCD_COLOR_BLACK); LCD_Clear(LCD_COLOR_BLACK);
 __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_UART4_CLK_ENABLE();
 g.Pin=GPIO_PIN_10|GPIO_PIN_11; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_HIGH; g.Alternate=GPIO_AF8_UART4; HAL_GPIO_Init(GPIOC,&g);
 UART4->CR1=0; UART4->BRR=(45000000u+115200u)/230400u; UART4->CR1=USART_CR1_UE|USART_CR1_RE|USART_CR1_RXNEIE; HAL_NVIC_SetPriority(UART4_IRQn,2,0); HAL_NVIC_EnableIRQ(UART4_IRQn);
 while(1) { while(rd!=wr) { uint8_t b=queue[rd]; rd=(rd+1u)%N; consume(b); }
  now=HAL_GetTick(); if(now-last>=500u) { last=now;
   LCD_DisplayStringLine(0,(uint8_t*)"N10 UART4 TEST / PC11");
   sprintf(s,"LINK: %-12s", good && now-last_valid<1500u ? "RECEIVING" : "NO DATA"); LCD_DisplayStringLine(1,(uint8_t*)s);
   sprintf(s,"GOOD:%lu BAD:%lu     ",(unsigned long)good,(unsigned long)bad); LCD_DisplayStringLine(2,(uint8_t*)s);
   sprintf(s,"BYTES:%lu ERR:%lu     ",(unsigned long)bytes,(unsigned long)(errors+overrun)); LCD_DisplayStringLine(3,(uint8_t*)s);
   if(good && now-last_front<1500u && front_mm>=0) sprintf(s,"0 DEG: %d mm       ",front_mm); else sprintf(s,"0 DEG: NO VALID POINT     "); LCD_DisplayStringLine(4,(uint8_t*)s);
   LCD_DisplayStringLine(5,(uint8_t*)"0 DEG IS RADAR AXIS"); LCD_DisplayStringLine(6,(uint8_t*)"MOTORS NOT CONTROLLED");
  }
 }
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* ʹ��HSE������HSEΪPLL��ʱ��Դ������PLL�ĸ��ַ�Ƶ����M N P Q
	 * PLLCLK = HSE/M*N/P = 25M / 25 *360 / 2 = 180M
	 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   while(1) {}
  }

  /* ���� OverDrive ģʽ */
  HAL_PWREx_EnableOverDrive();

  /* ѡ��PLLCLK��ΪSYSCLK�������� HCLK, PCLK1 and PCLK2 ��ʱ�ӷ�Ƶ����
	 * SYSCLK = PLLCLK     = 180M
	 * HCLK   = SYSCLK / 1 = 180M
	 * PCLK2  = SYSCLK / 2 = 90M
	 * PCLK1  = SYSCLK / 4 = 45M
	 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {}
  }
}
