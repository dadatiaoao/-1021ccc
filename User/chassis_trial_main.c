#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./usart/bsp_motor_uart.h"
#include "./lcd/bsp_lcd.h"
#include "./touch/gt9xx.h"
#include "./app/app_runtime.h"
#include "./app/qr_ui.h"
#include "./gyro/bsp_gyro_bus.h"
static uint8_t motor_state(const MotorBus *b,uint32_t now) {
    unsigned i;
    if(b->stop_mask || b->wheels_pending || b->queue_count) return GYRO_MOTOR_STOP;
    for(i=0u;i<4u;++i) {
        const MotorNode *n=&b->nodes[i];
        if(n->stop_unconfirmed) return GYRO_MOTOR_STOP;
        if(!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) || !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || (n->data.status_flags & 12u)) return GYRO_MOTOR_UNKNOWN;
        if(n->motion || n->data.speed_rpm) return GYRO_MOTOR_ACTIVE;
    }
    return GYRO_MOTOR_NONE;
}
int main(void) {
    static MotorBus bus; static Chassis chassis; static GyroService gyro;
    static GyroSnapshot g; static QrService unused_qr; static QrUi ui;
    MotorTransport mt; QrUiAction action; uint32_t now; int32_t x,y; uint8_t down;
    HAL_Init(); SystemClock_Config(); DEBUG_USART_Config(); GTP_Init_Panel(); LCD_Init();
    LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888); LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); GTP_SetRuntimeMode();
    mt=BspMotorUart_Init(); now=AppClock_Now(); MotorBus_Init(&bus,mt,now); MotorBus_SetManual(&bus,1u,now);
    MotorBus_Stop(&bus,MOTOR_WHEELS,now); Chassis_Init(&chassis,&bus,0,now); Gyro_Init(&gyro,BspGyroBus_Io(),now);
    QrUi_Init(&ui); ui.page=UI_RUN_ONLY; ui.chassis=&chassis;
    while(1) {
        now=AppClock_Now(); BspMotorUart_PumpBus(&bus,now); MotorBus_Process(&bus,now);
        now=AppClock_Now(); Gyro_Process(&gyro,now,motor_state(&bus,now)); Gyro_GetSnapshot(&gyro,now,&g);
        Chassis_Process(&chassis,&g,now); AppTouch_Process(now); AppTouch_GetState(&x,&y,&down);
        now=AppClock_Now(); action=QrUi_Touch(&ui,x,y,down);
        if(action==QR_UI_STOP_ALL) Chassis_Stop(&chassis,now);
        else if(action==QR_UI_GYRO_CAL && !Chassis_OwnsWheels(&chassis)) (void)Gyro_RequestCalibration(&gyro,now,motor_state(&bus,now));
        else if(action==QR_UI_GYRO_ZERO && !Chassis_OwnsWheels(&chassis) && motor_state(&bus,now)==GYRO_MOTOR_NONE) (void)Gyro_ZeroAngle(&gyro,now);
        else if(action==QR_UI_CHASSIS_START) (void)Chassis_Prepare(&chassis,now);
        else if(action==QR_UI_RUN_FORWARD) (void)Chassis_MoveRelative(&chassis,300.0f,0.0f,60.0f,now);
        else if(action==QR_UI_RUN_REVERSE) (void)Chassis_MoveRelative(&chassis,-300.0f,0.0f,60.0f,now);
        else if(action==QR_UI_DEPARTURE_START) (void)Chassis_MoveRelative(&chassis,0.0f,300.0f,60.0f,now);
        else if(action==QR_UI_MISSION_START) (void)Chassis_MoveRelative(&chassis,0.0f,-300.0f,60.0f,now);
        else if(action==QR_UI_VISION_SCAN) (void)Chassis_RotateRelative(&chassis,45.0f,15.0f,now);
        else if(action==QR_UI_VISION_STOP) (void)Chassis_RotateRelative(&chassis,-45.0f,15.0f,now);
        QrUi_Process(&ui,&unused_qr,&bus,&g,AppClock_Now());
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
