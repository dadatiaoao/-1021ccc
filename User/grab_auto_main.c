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
#include "./mission/ground_pick_parameters.h"
#include <math.h>
int GrabAxes_SetTrimZero(Mechanism *m, uint32_t now);
int GrabStall_SetArmZero(Mechanism *m, uint32_t now);
typedef struct {Mechanism *m; uint32_t close_ms;} Context;
static int lower(void *p,float mm,uint32_t now) {return Mechanism_MoveTo(((Context *)p)->m,MECH_LIFT,mm,5.0f,now)==MECH_ACCEPTED;}
static int closejaw(void *p,uint32_t now) {((Context *)p)->close_ms=now;return Servo_SetTarget(0u,2430)==SERVO_OK;}
static void stoptrial(void *p,uint32_t now) {Mechanism_Stop(((Context *)p)->m,now);}
enum { AUTO_IDLE, AUTO_WAIT_VISION, AUTO_MOVE, AUTO_STOP_VISION, AUTO_LOWER, AUTO_DONE, AUTO_FAIL };
static void auto_abort(Mechanism *m, VisionService *v, uint32_t now) { Mechanism_Stop(m,now); VisionService_Stop(v,now); }
int main(void) {
 static MotorBus bus; static Mechanism mech; static VisionService vision; static QrUi ui;
 static QrService qr; static GyroSnapshot gyro; static ServoDualState servo; static MechanismConfig manual_config;
 MotorTransport mt; uint32_t now; int32_t x,y; uint8_t down; QrUiAction action; static uint8_t aligned=0u, close_pending=0u; static uint32_t close_request=0u; static uint8_t automatic=AUTO_IDLE, steps=0u, search_step=0u; static uint32_t auto_since=0u, auto_request=0u, after_move=0u;
 HAL_Init(); SystemClock_Config(); DEBUG_USART_Config(); GTP_Init_Panel(); LCD_Init();
 LCD_LayerInit(0,LCD_FB_START_ADDRESS,ARGB8888); LCD_SelectLayer(0); LCD_SetTransparency(0,255); LCD_DisplayOn(); GTP_SetRuntimeMode();
 mt=BspMotorUart_Init(); now=AppClock_Now(); MotorBus_Init(&bus,mt,now); MotorBus_SetManual(&bus,1u,now); MotorBus_Stop(&bus,MOTOR_WHEELS,now);
 BspLimit_Init(); Mechanism_DefaultConfig(&manual_config); manual_config.axis[MECH_LIFT].tolerance=0.5f; Mechanism_Init(&mech,&bus,&manual_config,now); VisionService_Init(&vision,BspVisionUart_Init(),now);
 (void)ServoDualBench_Init(&servo); QrUi_Init(&ui); ui.page=UI_RUN_ONLY; ui.vision=&vision; ui.mechanism=&mech;
 while(1) {
  now=AppClock_Now(); BspMotorUart_PumpBus(&bus,now); Mechanism_SampleLimits(&mech,0u,0u,now);
  Mechanism_Process(&mech,now); MotorBus_Process(&bus,now); BspVisionUart_Pump(&vision,now); VisionService_Process(&vision,now); Servo_Process(now);
  AppTouch_Process(now); AppTouch_GetState(&x,&y,&down); action=QrUi_Touch(&ui,x,y,down);
  if(action==QR_UI_STOP_ALL) {automatic=AUTO_IDLE; aligned=0u; close_pending=0u; VisionService_Stop(&vision,now); Mechanism_Stop(&mech,now); MotorBus_Stop(&bus,MOTOR_WHEELS,now); Servo_OffAll();}
  else if(action==QR_UI_DEPARTURE_START && automatic==AUTO_IDLE) {
   if(mech.data.reference[MECH_ARM]==MECH_HOMED && mech.data.reference[MECH_LIFT]==MECH_HOMED && mech.data.reference[MECH_TRIM]==MECH_HOMED &&
      !Mechanism_OwnsAxes(&mech) && fabs(mech.data.position[MECH_LIFT])<=0.5 && Servo_IsReady() &&
      (Servo_IsEnabled(0u) || Servo_Enable(0u)==SERVO_OK) && Servo_SetTarget(0u,1700)==SERVO_OK &&
      VisionService_Start(&vision,1u,6u,1u,1u,(uint16_t)(vision.pose_id+1u),now)) {
     automatic=AUTO_WAIT_VISION; steps=0u; search_step=0u; auto_since=now; after_move=now; ui.run_note="AUTO WAIT VISION";
   } else ui.run_note="AUTO NEED 5 6 7 ZERO";
  }
  else if(automatic!=AUTO_IDLE && automatic!=AUTO_DONE && automatic!=AUTO_FAIL && action!=QR_UI_NONE) { ui.run_note="AUTO BUSY / STOP TO CANCEL"; }
  else if(action==QR_UI_RUN_FORWARD) { aligned=0u; close_pending=0u; (void)Mechanism_HomeAll(&mech,now); }
  else if(action==QR_UI_MECH_HOME) { aligned=0u; if(!GrabStall_SetArmZero(&mech,now)) ui.run_note="ARM ZERO REJECTED"; else ui.run_note="ARM ZERO SET"; }
  else if(action==QR_UI_VISION_SCAN) { aligned=0u; if(VisionService_Start(&vision,1u,6u,1u,1u,(uint16_t)(vision.pose_id+1u),now)) ui.run_note="VISION REQUESTED COLOR 6"; else ui.run_note="VISION REQUEST REJECTED"; }
  else if(action==QR_UI_GYRO_CAL) { aligned=0u; (void)Mechanism_MoveRelative(&mech,MECH_ARM,-5.0f,6.0f,now); }
  else if(action==QR_UI_GYRO_ZERO) { aligned=0u; (void)Mechanism_MoveRelative(&mech,MECH_ARM,5.0f,6.0f,now); }
  else if(action==QR_UI_RUN_REVERSE) { aligned=0u; (void)Mechanism_MoveRelative(&mech,MECH_LIFT,-10.0f,20.0f,now); }
  else if(action==QR_UI_MECH_TEST) {
   if (!aligned) {
     if (VisionService_TargetFresh(&vision,now) && vision.observation.target_id==6u &&
         vision.observation.cx>=626u && vision.observation.cx<=666u &&
         vision.observation.cy>=298u && vision.observation.cy<=338u) {
       aligned=1u; VisionService_Stop(&vision,now); ui.run_note="STOPPING VISION / TAP DN AGAIN";
     } else ui.run_note="ALIGN X646 Y318 FIRST";
   } else if (vision.state!=VISION_STOPPED) ui.run_note="VISION STOP NOT CONFIRMED";
   else {
     float next=(float)mech.data.position[MECH_LIFT]+10.0f;
     if(next>GROUND_PICK_LIFT_MM+0.5f) ui.run_note="GROUND PICK HEIGHT LIMIT";
     else if (Mechanism_MoveRelative(&mech,MECH_LIFT,10.0f,20.0f,now)==MECH_ACCEPTED) {
       close_pending=(uint8_t)(fabs((double)mech.data.target-GROUND_PICK_LIFT_MM)<=0.5);
       close_request=mech.data.request_id; ui.run_note="VISION STOPPED / MANUAL LOWER";
     }
   }
  }
  else if(action==QR_UI_CHASSIS_START) {aligned=0u; if(GrabAxes_SetTrimZero(&mech,now)) ui.run_note="TRIM ZERO SET"; else ui.run_note="TRIM ZERO REJECTED";}
  else if(action==QR_UI_RESCAN || action==QR_UI_MISSION_START) {aligned=0u; if(Mechanism_MoveRelative(&mech,MECH_TRIM,action==QR_UI_RESCAN ? -2.0f : 2.0f,0.5f,now)!=MECH_ACCEPTED) ui.run_note="TRIM MOVE REJECTED";}
  else if(action==QR_UI_SERVO_JAW_LOW || action==QR_UI_SERVO_JAW_HIGH) {
    if(!Mechanism_OwnsAxes(&mech) && Servo_IsReady() && (Servo_IsEnabled(0u) || Servo_Enable(0u)==SERVO_OK))
      (void)Servo_SetTarget(0u,action==QR_UI_SERVO_JAW_LOW ? 1700 : GROUND_PICK_JAW_CLOSE_US);
  }
  if(automatic==AUTO_WAIT_VISION) {
   if(now-auto_since>30000u || vision.state==VISION_FAULT) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO VISION TIMEOUT";}
   else if(VisionService_TargetFresh(&vision,now) && vision.observation.target_id==6u && (vision.observation.flags & VISION_FLAG_STABLE) && (int32_t)(vision.observation.received_ms-after_move)>0) {
    int dx=(int)vision.observation.cx-(int)GROUND_PICK_CENTER_X, dy=(int)vision.observation.cy-(int)GROUND_PICK_CENTER_Y; search_step=0u;
    if(dx>=-20 && dx<=20 && dy>=-20 && dy<=20) {VisionService_Stop(&vision,now); automatic=AUTO_STOP_VISION; auto_since=now; ui.run_note="AUTO STOP VISION";}
    else if(steps>=30u) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO STEP LIMIT";}
    else { MechanismAxis axis; float delta; MechanismRequestResult rr;
      if(dx < -20 || dx > 20) {axis=MECH_ARM; delta=dx>0 ? -1.0f : 1.0f;}
      else {axis=MECH_TRIM; delta=dy>0 ? -1.0f : 1.0f;}
      rr=Mechanism_MoveRelative(&mech,axis,delta,axis==MECH_ARM ? 6.0f : 0.5f,now);
      if(rr==MECH_ACCEPTED) {auto_request=mech.data.request_id; ++steps; automatic=AUTO_MOVE; auto_since=now; ui.run_note="AUTO ALIGN STEP";}
      else {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO MOVE REJECTED";}
    }
   } else if(!VisionService_TargetFresh(&vision,now) && vision.state==VISION_ACTIVE && vision.online && now-vision.last_heartbeat_ms<3000u && now-after_move>=1500u) {
    static const float pattern[5]={1.0f,-2.0f,2.0f,-2.0f,1.0f};
    if(search_step>=5u || steps>=30u) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO TARGET NOT FOUND";}
    else if(Mechanism_MoveRelative(&mech,MECH_ARM,pattern[search_step],6.0f,now)==MECH_ACCEPTED) {
     ++search_step; ++steps; auto_request=mech.data.request_id; automatic=AUTO_MOVE; auto_since=now; ui.run_note="AUTO SEARCH SMALL ARC";
    } else {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO SEARCH REJECTED";}
   }
  } else if(automatic==AUTO_MOVE) {
   if(mech.data.request_id!=auto_request || mech.data.result==MECH_FAILED || mech.data.result==MECH_CANCELLED || now-auto_since>15000u) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO AXIS FAULT";}
   else if(mech.data.result==MECH_SUCCESS && !Mechanism_OwnsAxes(&mech)) {automatic=AUTO_WAIT_VISION; auto_since=now; after_move=now; ui.run_note="AUTO VERIFY POSITION";}
  } else if(automatic==AUTO_STOP_VISION) {
   if(vision.state==VISION_FAULT || now-auto_since>5000u) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO STOP VISION FAULT";}
   else if(vision.state==VISION_STOPPED) {
    if(Mechanism_MoveTo(&mech,MECH_LIFT,GROUND_PICK_LIFT_MM,20.0f,now)==MECH_ACCEPTED) {auto_request=mech.data.request_id; auto_since=now; automatic=AUTO_LOWER; ui.run_note="AUTO LOWER 130MM";}
    else {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO LOWER REJECTED";}
   }
  } else if(automatic==AUTO_LOWER) {
   if(mech.data.request_id!=auto_request || mech.data.result==MECH_FAILED || mech.data.result==MECH_CANCELLED || now-auto_since>20000u) {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO LIFT FAULT";}
   else if(mech.data.result==MECH_SUCCESS && !Mechanism_OwnsAxes(&mech) && mech.data.reference[MECH_LIFT]==MECH_HOMED && fabs(mech.data.position[MECH_LIFT]-GROUND_PICK_LIFT_MM)<=0.5) {
    if(Servo_IsEnabled(0u) && Servo_SetTarget(0u,GROUND_PICK_JAW_CLOSE_US)==SERVO_OK) {automatic=AUTO_DONE; ui.run_note="JAW 2450 / VERIFY GRIP";}
    else {automatic=AUTO_FAIL; auto_abort(&mech,&vision,now); ui.run_note="AUTO JAW FAULT";}
   }
  }
  if(close_pending && mech.data.request_id==close_request && !Mechanism_OwnsAxes(&mech)) {
   close_pending=0u;
   if(mech.data.result==MECH_SUCCESS && mech.data.reference[MECH_LIFT]==MECH_HOMED &&
      fabs(mech.data.position[MECH_LIFT]-GROUND_PICK_LIFT_MM)<=0.5 && Servo_IsReady() &&
      (Servo_IsEnabled(0u) || Servo_Enable(0u)==SERVO_OK) &&
      Servo_SetTarget(0u,GROUND_PICK_JAW_CLOSE_US)==SERVO_OK) ui.run_note="JAW 2450 / VERIFY GRIP";
   else ui.run_note="NO AUTO CLOSE / CHECK LIFT";
  }
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
