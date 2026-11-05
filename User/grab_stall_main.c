#include "main.h"
#include "./usart/bsp_debug_usart.h"
#include "./usart/bsp_motor_uart.h"
#include "./usart/bsp_vision_uart.h"
#include "./lcd/bsp_lcd.h"
#include "./touch/gt9xx.h"
#include "./app/app_runtime.h"
#include "./app/qr_ui.h"
#include "./mechanism/bsp_limit_gpio.h"
#include "./mechanism/vision_grab_trial.h"
#include "./servo/servo_dual_bench.h"
#include "./servo/servo_pwm.h"
int GrabStall_SetArmZero(Mechanism *m, uint32_t now);
typedef struct {Mechanism *m; uint32_t close_ms;} Context;
static int lower(void *p,float mm,uint32_t now) {return Mechanism_MoveTo(((Context *)p)->m,MECH_LIFT,mm,5.0f,now)==MECH_ACCEPTED;}
static int closejaw(void *p,uint32_t now) {((Context *)p)->close_ms=now;return Servo_SetTarget(0u,2430)==SERVO_OK;}
static void stoptrial(void *p,uint32_t now) {Mechanism_Stop(((Context *)p)->m,now);}
int main(void) {
 static MotorBus bus; static Mechanism mech; static VisionService vision; static QrUi ui;
 static QrService qr; static GyroSnapshot gyro; static ServoDualState servo;
 static VisionGrabTrial trial; static Context ctx;
 MotorTransport mt; VgtPort port; uint32_t now; int32_t x,y; uint8_t down; QrUiAction action; int lr,cr; const MotorNode *arm;
 HAL_Init(); SystemClock_Config(); DEBUG_USART_Config(); GTP_Init_Panel(); LCD_Init();
 LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888); LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); GTP_SetRuntimeMode();
 mt=BspMotorUart_Init(); now=AppClock_Now(); MotorBus_Init(&bus,mt,now); MotorBus_SetManual(&bus,1u,now); MotorBus_Stop(&bus,MOTOR_WHEELS,now);
 BspLimit_Init(); Mechanism_Init(&mech,&bus,0,now); VisionService_Init(&vision,BspVisionUart_Init(),now);
 (void)ServoDualBench_Init(&servo); QrUi_Init(&ui); ui.page=UI_RUN_ONLY; ui.vision=&vision; ui.mechanism=&mech;
 ctx.m=&mech; port.ctx=&ctx; port.lower=lower; port.close=closejaw; port.stop=stoptrial; Vgt_Init(&trial,port);
 while(1) {
  now=AppClock_Now(); BspMotorUart_PumpBus(&bus,now); /* Trial only: physical switches are not used. */ Mechanism_SampleLimits(&mech,0u,0u,now);
  Mechanism_Process(&mech,now); MotorBus_Process(&bus,now); BspVisionUart_Pump(&vision,now); VisionService_Process(&vision,now); Servo_Process(now);
  AppTouch_Process(now); AppTouch_GetState(&x,&y,&down); action=QrUi_Touch(&ui,x,y,down);
  if(action==QR_UI_STOP_ALL) {Vgt_Stop(&trial,now);MotorBus_Stop(&bus,MOTOR_WHEELS,now);VisionService_Stop(&vision,now);}
  else if(action==QR_UI_RUN_FORWARD && trial.state==VGT_IDLE) (void)Mechanism_HomeAll(&mech,now);
  else if(action==QR_UI_MECH_HOME && trial.state==VGT_IDLE) (void)GrabStall_SetArmZero(&mech,now);
  else if(action==QR_UI_RUN_REVERSE && trial.state==VGT_IDLE) {
   arm=&bus.nodes[4];
   if(mech.data.reference[MECH_LIFT]==MECH_HOMED && mech.data.reference[MECH_ARM]==MECH_HOMED && !Mechanism_OwnsAxes(&mech) && arm->data.online &&
      MotorBus_Fresh(arm,MOTOR_VALID_SPEED,now) && MotorBus_Fresh(arm,MOTOR_VALID_STATUS,now) && !arm->motion &&
      !arm->data.speed_rpm && !arm->stop_unconfirmed && (arm->data.status_flags & 1u) && !(arm->data.status_flags & 12u) &&
      Servo_IsReady() && (Servo_IsEnabled(0u) || Servo_Enable(0u)==SERVO_OK) && Servo_SetTarget(0u,1700)==SERVO_OK &&
      VisionService_Start(&vision,1u,6u,1u,1u,(uint16_t)(vision.pose_id+1u),now)) (void)Vgt_Start(&trial,1u,1u,1u,617u,338u,20u,now);
  }
  lr=mech.data.result==MECH_SUCCESS ? 1 : (mech.data.result==MECH_FAILED || mech.data.result==MECH_CANCELLED ? -1 : 0);
  cr=trial.state==VGT_CLOSING && Servo_IsEnabled(0u) && Servo_CurrentUs(0u)==2430u && (uint32_t)(now-ctx.close_ms)>=1000u ? 1 : 0;
  Vgt_Update(&trial,(uint8_t)(VisionService_TargetFresh(&vision,now) && vision.observation.target_id==6u),vision.observation.frame_id,vision.observation.cx,vision.observation.cy,lr,cr,now);
  ui.run_note=trial.state==VGT_WAIT_TARGET ? "WAIT COLOR IN WINDOW" : trial.state==VGT_LOWERING ? "LOWERING / 100MM" : trial.state==VGT_CLOSING ? "CLOSING JAW" : trial.state==VGT_DONE ? "JAW CLOSED / CHECK OBJECT" : trial.state==VGT_FAULT ? "FAULT / PRESS STOP" : mech.data.reference[MECH_LIFT]==MECH_HOMED ? (mech.data.reference[MECH_ARM]==MECH_HOMED ? "READY / TAP START" : "PLACE ARM / SET ZERO") : "TAP HOME LIFT";
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
