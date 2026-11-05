/* Standalone F429 wheel trial: no QrApp, homing, gyro or servo init. */
#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./usart/bsp_motor_uart.h"
#include "./usart/bsp_qr_uart.h"
#include "./usart/bsp_qr_trigger.h"
#include "./lcd/bsp_lcd.h"
#include "./touch/gt9xx.h"
#include "./app/app_runtime.h"
#include "./app/qr_ui.h"
#include "./motor/wheel_run.h"

int main(void)
{
    static MotorBus bus; static WheelRun run; static QrUi ui;
    static QrService qr; static GyroSnapshot gyro;
    MotorTransport transport; uint32_t now, wait_ms=0; int32_t x,y;
    uint8_t down, zone=0, phase=0; QrUiAction action;
    HAL_Init(); SystemClock_Config(); DEBUG_USART_Config();
    GTP_Init_Panel(); LCD_Init(); LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888);
    LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); GTP_SetRuntimeMode();
    transport=BspMotorUart_Init(); now=HAL_GetTick(); MotorBus_Init(&bus,transport,now);
    MotorBus_SetManual(&bus,1u,now); MotorBus_Stop(&bus,MOTOR_WHEELS,now);
    WheelRun_Init(&run,&bus); QrUi_Init(&ui); ui.page=UI_RUN_ONLY;
    while (1) {
        now=AppClock_Now(); BspMotorUart_PumpBus(&bus,now);
        WheelRun_Process(&run,now); MotorBus_Process(&bus,now);
        AppTouch_Process(now); AppTouch_GetState(&x,&y,&down); action=QrUi_Touch(&ui,x,y,down);
        if (action==QR_UI_STOP_ALL) { phase=0; WheelRun_Stop(&run,now); }
        else if (phase==0 && action==QR_UI_RUN_FORWARD) zone=1;
        else if (phase==0 && action==QR_UI_RUN_REVERSE) zone=2;
        else if (phase==0 && zone && action==QR_UI_MISSION_START) {
            if (WheelRun_Start(&run,(uint8_t)(zone==2),now)) phase=1;
        }
        if (phase==1 && run.state==WHEEL_RUN_DONE) {phase=2;wait_ms=now;}
        if (phase==2 && (uint32_t)(now-wait_ms)>=1000u) {
            if (WheelRun_Start(&run,(uint8_t)(zone==1),now)) phase=3;
            else { phase=0; }
        }
        if (phase==3 && run.state==WHEEL_RUN_DONE) phase=4;
        if (run.state==WHEEL_RUN_FAULT) phase=0;
        if (phase==4 && action==QR_UI_MISSION_START) {phase=0; zone=0;}
        ui.run_note=phase==1 ? "OUTBOUND / 20 RPM / 8 SEC" :
            phase==2 ? "STOPPED / WAIT 1 SEC" : phase==3 ? "RETURN / 20 RPM / 8 SEC" :
            phase==4 ? "DONE / CHECK POSITION / START RESET" :
            run.state==WHEEL_RUN_STOPPING || run.state==WHEEL_RUN_FAULT ? run.note :
            zone==1 ? "ZONE 1 SELECTED / PRESS START" : zone==2 ? "ZONE 2 SELECTED / PRESS START" : "SELECT ZONE 1 OR 2";
        QrUi_Process(&ui,&qr,&bus,&gyro,AppClock_Now());
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
