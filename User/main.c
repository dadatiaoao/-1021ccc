/*
 * 入口：HAL/时钟、触摸与 LCD、电机与扫码、应用初始化。
 * QrApp_Init 建立 K230 视觉通信及固定启停区2的比赛调度；整场比赛等待显式启动；机构上电回零由 MECHANISM_TRIAL_RUN 配置门控。
 * 协作式主循环逐步推进通信、传感器和界面。
 */
#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./lcd/bsp_lcd.h"
#include "./touch/gt9xx.h"
#include "./app/qr_app.h"
#include "./usart/bsp_motor_uart.h"
#include "./usart/bsp_qr_uart.h"
#include "./usart/bsp_qr_trigger.h"
#include "./servo/servo_bench.h"
#if defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH
#include "./servo/servo_dual_bench.h"
#endif

/* 应用对象静态保存；当前上游 TRIAL=1 会自动执行机构回零，比赛仍需显式启动。 */
int main(void)
{
    static QrApp app;
    MotorTransport motor;
    uint8_t qr_ready;
    HAL_Init();
    SystemClock_Config();
    DEBUG_USART_Config();
    GTP_Init_Panel();
    LCD_Init();
    LCD_LayerInit(0, LCD_FB_START_ADDRESS, ARGB8888);
    LCD_SelectLayer(0);
    LCD_SetTransparency(0, 255);
    LCD_DisplayOn();
    GTP_SetRuntimeMode();
    motor = BspMotorUart_Init();
    qr_ready = BspQrUart_Init();
    BspQrTrigger_Init();
#if defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH
    ServoDualBench_Run(); /* Manual test only; does not return. */
#elif SERVO_BENCH_ENABLE
    /* Manual jaw mode never returns; match app and homing are not started. */
    ServoBench_Run();
#endif
    QrApp_Init(&app, motor, qr_ready, HAL_GetTick());
    while (1) {
        /* Communication, sensors, mission and UI advance without blocking. */
        QrApp_Process(&app, HAL_GetTick());
    }
}

/*
 * 配置 HSE=25 MHz、PLL 系统时钟 180 MHz，以及 APB1=45 MHz、APB2=90 MHz。
 * HAL 毫秒时基和传感器 DWT 耗时换算依赖该时钟配置；配置失败时停留在故障循环。
 */
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
