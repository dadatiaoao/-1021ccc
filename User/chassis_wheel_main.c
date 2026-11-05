#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./usart/bsp_motor_uart.h"
#include "./lcd/bsp_lcd.h"
#include "./touch/gt9xx.h"
#include "./app/app_runtime.h"
#include "./app/qr_ui.h"
#include "./gyro/bsp_gyro_bus.h"
#include "./chassis/wheel_only_trial.h"
#include <stdio.h>
static const char *state_note(const WheelOnlyTrial *t,const Chassis *c) {
    if(t->fault) return "FAULT / STOP";
    if(t->state==WT_RUNNING) return "RUNNING";
    if(t->state==WT_STOPPING || c->data.state==CHASSIS_STOPPING) return "STOPPING";
    if(t->state==WT_DONE) return "STOPPED";
    if(c->data.state==CHASSIS_READY) return "READY";
    if(c->data.state==CHASSIS_PREPARING) return "PREPARING";
    if(c->data.state==CHASSIS_FAULT) return "CHASSIS FAULT";
    return "PRESS PREPARE";
}
int main(void) {
    static MotorBus bus; static Chassis chassis; static WheelOnlyTrial trial;
    static GyroSnapshot no_gyro; static QrService unused_qr; static QrUi ui;
    static char progress[64]; MotorTransport mt; QrUiAction action; uint32_t now;
    int32_t x,y; uint8_t down; WheelTrialMove move; int request;
    HAL_Init(); SystemClock_Config(); DEBUG_USART_Config(); GTP_Init_Panel(); LCD_Init();
    LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888); LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); GTP_SetRuntimeMode();
    mt=BspMotorUart_Init(); now=AppClock_Now(); MotorBus_Init(&bus,mt,now); MotorBus_SetManual(&bus,1u,now);
    MotorBus_Stop(&bus,MOTOR_WHEELS,now); Chassis_Init(&chassis,&bus,0,now); WheelTrial_Init(&trial,&chassis,&bus);
    QrUi_Init(&ui); ui.page=UI_RUN_ONLY; ui.chassis=&chassis;
    while(1) {
        now=AppClock_Now(); BspMotorUart_PumpBus(&bus,now); MotorBus_Process(&bus,now);
        Chassis_Process(&chassis,&no_gyro,now); WheelTrial_Process(&trial,now);
        AppTouch_Process(now); AppTouch_GetState(&x,&y,&down); now=AppClock_Now();
        action=QrUi_Touch(&ui,x,y,down); request=1; move=WT_FORWARD;
        if(action==QR_UI_STOP_ALL) { WheelTrial_Stop(&trial,now); request=0; }
        else if(action==QR_UI_CHASSIS_START) { (void)Chassis_Prepare(&chassis,now); request=0; }
        else if(action==QR_UI_RUN_FORWARD) move=WT_FORWARD;
        else if(action==QR_UI_RUN_REVERSE) move=WT_BACK;
        else if(action==QR_UI_DEPARTURE_START) move=WT_LEFT;
        else if(action==QR_UI_MISSION_START) move=WT_RIGHT;
        else if(action==QR_UI_VISION_SCAN) move=WT_CCW;
        else if(action==QR_UI_VISION_STOP) move=WT_CW;
        else request=0;
        if(request) (void)WheelTrial_Start(&trial,move,now);
        ui.run_note=state_note(&trial,&chassis);
        snprintf(progress,sizeof(progress),"WHEEL EST:%ld %s",(long)trial.progress,trial.move>=WT_CCW ? "DEG" : "MM");
        ui.run_qr_text=progress; QrUi_Process(&ui,&unused_qr,&bus,&no_gyro,AppClock_Now());
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
