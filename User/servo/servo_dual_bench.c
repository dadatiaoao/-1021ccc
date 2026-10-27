/* Manual two-servo test: PA6 jaw, PC8 turntable. Never auto-enable. */
#include "servo_dual_bench.h"
#include "servo_pwm.h"
#include "app/app_runtime.h"
#include "lcd/bsp_lcd.h"
#include "lcd/lcd_orientation.h"
#include <stdio.h>
#include <string.h>

enum { DUAL_JAW = 0, DUAL_TABLE = 1 };
static const ServoLimits jaw_limits={1700u,2430u,1700u,0u,0u};
/* User-recorded turntable pulse positions; actual position is not sensed. */
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
static const ServoLimits table_limits={1240u,2600u,1240u,0u,0u};
#else
static const ServoLimits table_limits={1240u,2600u,1900u,0u,0u};
#endif
static const char * const labels[]={"JAW ON","J 1700","J 2430","JAW OFF",
                                    "TABLE ON","T PREV","T NEXT","TABLE OFF"};

int ServoDualBench_Init(ServoDualState *s)
{
    memset(s,0,sizeof(*s)); s->was_down=1u;
    Servo_Init();
    if(!Servo_IsReady()) return 0;
    Servo_OffAll();
    return Servo_SetLimits(DUAL_JAW,&jaw_limits)==SERVO_OK &&
           Servo_SetLimits(DUAL_TABLE,&table_limits)==SERVO_OK;
}
ServoDualAction ServoDualBench_Touch(ServoDualState *s,int32_t x,int32_t y,uint8_t down)
{
    unsigned i; uint8_t edge=(uint8_t)(down && !s->was_down);
    s->was_down=down;
    if(!edge || x<0 || x>=480 || y<0 || y>=272) return DUAL_NONE;
    x=LCD_TOUCH_X(x); y=LCD_TOUCH_Y(y);
    for(i=0u;i<8u;++i) {
        int32_t bx=4+(int32_t)(i%4u)*119, by=112+(int32_t)(i/4u)*48;
        if(x>=bx && x<bx+114 && y>=by && y<by+36) return (ServoDualAction)(i+1u);
    }
    return DUAL_NONE;
}
void ServoDualBench_Command(ServoDualState *s,ServoDualAction action)
{
    uint8_t ch;
    if(action==DUAL_NONE) return;
    ch=(uint8_t)(action>=DUAL_TABLE_ON ? DUAL_TABLE : DUAL_JAW);
    if(action==DUAL_JAW_OFF || action==DUAL_TABLE_OFF) {
        Servo_Off(ch); s->last_status=SERVO_OK;
    } else if(action==DUAL_JAW_ON || action==DUAL_TABLE_ON) {
        s->last_status=(uint8_t)(Servo_IsEnabled(ch) ? SERVO_OK : Servo_Enable(ch));
    } else if(action==DUAL_JAW_LOW || action==DUAL_JAW_HIGH) {
        s->last_status=Servo_SetTarget(ch,action==DUAL_JAW_LOW ? 1700 : 2430);
    } else {
        int32_t target=Servo_TargetUs(ch);
        int32_t next;
        if(!Servo_IsEnabled(ch)) { s->last_status=SERVO_REJECT_STATE; return; }
        if(action==DUAL_TABLE_MINUS) {
            if(target>1900) next=1900;
            else if(target>1240) next=1240;
            else { s->last_status=SERVO_REJECT_RANGE; return; }
        } else {
            if(target<1900) next=1900;
            else if(target<2600) next=2600;
            else { s->last_status=SERVO_REJECT_RANGE; return; }
        }
        s->last_status=Servo_SetTarget(ch,next);
    }
}
void ServoDualBench_Process(ServoDualState *s,uint32_t now)
{
    int32_t x,y; uint8_t down;
    AppTouch_Process(now); AppTouch_GetState(&x,&y,&down);
    ServoDualBench_Command(s,ServoDualBench_Touch(s,x,y,down));
    Servo_Process(now);
}
static void line(unsigned y,const char *text)
{
    LCD_SetFont(&Font16); LCD_SetTextColor(LCD_COLOR_BLACK);
    LCD_FillRect(4u,(uint16_t)y,472u,22u);
    LCD_SetTextColor(LCD_COLOR_WHITE); LCD_SetBackColor(LCD_COLOR_BLACK);
    LCD_DisplayStringAt(4u,(uint16_t)y,(uint8_t *)text,LEFT_MODE);
}
static void controls(void)
{
    unsigned i; LCD_Clear(LCD_COLOR_BLACK);
    line(4u,"SERVO TEST: JAW PA6 / TABLE PC8");
    line(28u,"BOOT: BOTH OFF / AUTO HOME OFF");
    for(i=0u;i<8u;++i) {
        uint16_t x=(uint16_t)(4u+(i%4u)*119u),y=(uint16_t)(112u+(i/4u)*48u);
        LCD_SetTextColor(LCD_COLOR_WHITE); LCD_FillRect(x,y,114u,36u);
        LCD_SetTextColor(LCD_COLOR_BLACK); LCD_SetBackColor(LCD_COLOR_WHITE);
        LCD_DisplayStringAt((uint16_t)(x+4u),(uint16_t)(y+10u),(uint8_t *)labels[i],LEFT_MODE);
    }
}
static void status(const ServoDualState *s)
{
    char t[56]; unsigned i;
    for(i=0;i<2u;++i) {
        snprintf(t,sizeof(t),"%s:%s OUT:%u TGT:%ld us",i ? "TABLE" : "JAW",
            Servo_IsEnabled((uint8_t)i) ? "ON" : "OFF",
            Servo_IsEnabled((uint8_t)i) ? (unsigned)Servo_CurrentUs((uint8_t)i) : 0u,
            (long)Servo_TargetUs((uint8_t)i));
        line(60u+i*24u,t);
    }
    line(246u,s->last_status==SERVO_REJECT_RANGE ? "LIMIT: TARGET UNCHANGED" :
        s->last_status==SERVO_REJECT_STATE ? "TAP THAT SERVO ON FIRST" :
        "TABLE NODES: 1240 / 1900 / 2600");
}
void ServoDualBench_Run(void)
{
    ServoDualState s; uint32_t now,draw_at;
    if(!ServoDualBench_Init(&s)) {
        LCD_Clear(LCD_COLOR_BLACK); line(60u,"SERVO INIT FAILED / PWM OFF"); for(;;) { }
    }
    controls(); status(&s); draw_at=AppClock_Now();
    for(;;) {
        now=AppClock_Now(); ServoDualBench_Process(&s,now);
        if((uint32_t)(now-draw_at)>=100u) { draw_at=now; status(&s); }
    }
}
