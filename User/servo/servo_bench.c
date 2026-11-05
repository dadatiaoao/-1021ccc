/* PM25S manual jaw bench: PA6/TIM3_CH1, 50 Hz; no automatic sweep.
 * Manufacturer PWM range: 500..2500 us, neutral 1500 us.
 * Test window is 1700..2430 us; mounted jaw endpoints remain unknown.
 * DISPLAYED pulse is software output, never measured jaw position.
 */
#include "servo_bench.h"
#include "servo_pwm.h"
#include "app/app_runtime.h"
#include "lcd/bsp_lcd.h"
#include "lcd/lcd_orientation.h"
#include <stdio.h>
#include <string.h>
#if defined(SERVO_TURNTABLE_BENCH) && SERVO_TURNTABLE_BENCH
/* Provisional limits only; verify servo type, wiring and mechanism first. */
static const ServoLimits window={1800u,2000u,1900u,200u,0u};
static const char * const labels[]={"ENABLE","-50 us","+50 us","1800 us","2000 us","PWM OFF"};
#define BENCH_STEP_US 50
#define BENCH_LOW_US 1800
#define BENCH_HIGH_US 2000
#define BENCH_HEADER "TABLE TEST / PC8 TIM3 CH3"
#define BENCH_WINDOW "TEST WINDOW: 1800..2000 / 200 us/s"
#define BENCH_STATUS "TABLE:%s OUTPUT:%u us TARGET:%ld us"
#else
static const ServoLimits window={1700u,2430u,1700u,0u,0u};
static const char * const labels[]={"ENABLE","-730 us","+730 us","1700 us","2430 us","PWM OFF"};
#define BENCH_STEP_US 730
#define BENCH_LOW_US 1700
#define BENCH_HIGH_US 2430
#define BENCH_HEADER "JAW MANUAL TEST / PA6 TIM3 CH1"
#define BENCH_WINDOW "TEST WINDOW: 1700..2430 us / FAST"
#define BENCH_STATUS "JAW:%s  OUTPUT:%u us  TARGET:%ld us"
#endif

int ServoBench_Init(ServoBenchState *s)
{
    memset(s,0,sizeof(*s)); s->was_down=1u;
    Servo_Init();
    if(!Servo_IsReady()) return 0;
    Servo_OffAll();
    return Servo_SetLimits(SERVO_JAW,&window)==SERVO_OK;
}
ServoBenchAction ServoBench_Touch(ServoBenchState *s,int32_t x,int32_t y,uint8_t down)
{
    unsigned i; uint8_t edge=(uint8_t)(down && !s->was_down);
    s->was_down=down;
    if(!edge || x<0 || x>=480 || y<0 || y>=272) return JAW_NONE;
    x=LCD_TOUCH_X(x); y=LCD_TOUCH_Y(y);
    for(i=0u;i<6u;++i) {
        int32_t bx=8+(int32_t)(i%3u)*156, by=136+(int32_t)(i/3u)*56;
        if(x>=bx && x<bx+148 && y>=by && y<by+40) return (ServoBenchAction)(i+1u);
    }
    return JAW_NONE;
}
void ServoBench_Command(ServoBenchState *s,ServoBenchAction action)
{
    if(action==JAW_NONE) return;
    if(action==JAW_OFF) { Servo_Off(SERVO_JAW); s->last_status=SERVO_OK; }
    else if(action==JAW_ENABLE) {
        /* No repeated neutral jumps while already enabled. */
        s->last_status=(uint8_t)(Servo_IsEnabled(SERVO_JAW) ? SERVO_OK : Servo_Enable(SERVO_JAW));
    } else if(action==JAW_MINUS || action==JAW_PLUS)
        s->last_status=Servo_SetTarget(SERVO_JAW,Servo_TargetUs(SERVO_JAW)+(action==JAW_MINUS ? -BENCH_STEP_US : BENCH_STEP_US));
    else if(action==JAW_LOW || action==JAW_HIGH)
        s->last_status=Servo_SetTarget(SERVO_JAW,action==JAW_LOW ? BENCH_LOW_US : BENCH_HIGH_US);
}
void ServoBench_Process(ServoBenchState *s,uint32_t now)
{
    int32_t x,y; uint8_t down;
    AppTouch_Process(now); AppTouch_GetState(&x,&y,&down);
    ServoBench_Command(s,ServoBench_Touch(s,x,y,down));
    Servo_Process(now);
}
static void line(unsigned y,const char *text)
{
    LCD_SetFont(&Font16); LCD_SetTextColor(LCD_COLOR_BLACK);
    LCD_FillRect(8u,(uint16_t)y,464u,22u);
    LCD_SetTextColor(LCD_COLOR_WHITE); LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringAt(8u,(uint16_t)y,(uint8_t *)text,LEFT_MODE);
}
static void controls(void)
{
    unsigned i;
    LCD_Clear(LCD_COLOR_BLACK);
    line(4u,BENCH_HEADER);
    line(28u,"BOOT: OFF  LIFT: OFF  AUTO HOME: OFF");
    for(i=0u;i<6u;++i) {
        uint16_t x=(uint16_t)(8u+(i%3u)*156u),y=(uint16_t)(136u+(i/3u)*56u);
        LCD_SetTextColor(LCD_COLOR_WHITE); LCD_FillRect(x,y,148u,40u);
        LCD_SetTextColor(LCD_COLOR_BLACK); LCD_SetBackColor(LCD_COLOR_WHITE);
        LCD_DisplayStringAt((uint16_t)(x+12u),(uint16_t)(y+12u),(uint8_t *)labels[i],LEFT_MODE);
    }
    line(246u,"FIRST ENABLE, THEN ONE TAP PER ACTION");
}
static void status(const ServoBenchState *s)
{
    char t[56];
    snprintf(t,sizeof(t),BENCH_STATUS,Servo_IsEnabled(SERVO_JAW) ? "ON" : "OFF",
        Servo_IsEnabled(SERVO_JAW) ? (unsigned)Servo_CurrentUs(SERVO_JAW) : 0u,(long)Servo_TargetUs(SERVO_JAW));
    line(60u,t); line(84u,BENCH_WINDOW);
    line(108u,s->last_status==SERVO_REJECT_RANGE ? "LIMIT: command rejected; target unchanged" :
        s->last_status==SERVO_REJECT_STATE ? "OFF: tap ENABLE before movement" :
        "PWM only: actual position is not sensed");
}
void ServoBench_Run(void)
{
    ServoBenchState s; uint32_t now,draw_at;
    if(!ServoBench_Init(&s)) {
        LCD_Clear(LCD_COLOR_BLACK); line(60u,"JAW INIT FAILED / PWM OFF"); for(;;) { }
    }
    controls(); status(&s); draw_at=AppClock_Now();
    for(;;) {
        now=AppClock_Now(); ServoBench_Process(&s,now);
        if((uint32_t)(now-draw_at)>=100u) { draw_at=now; status(&s); }
    }
}
