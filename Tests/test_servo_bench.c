/* Real bench input/dispatch plus real slew math; hardware calls are recorded. */
#include "servo/servo_bench.h"
#include "servo/servo_dual_bench.h"
#include "servo/servo_pwm.h"
#include "lcd/bsp_lcd.h"
#include "lcd/lcd_orientation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,enables[2],targets[2];
static uint8_t enabled[2],down;
static ServoLimits limits[2];
static int32_t current[2],target[2],tx,ty;
static uint32_t previous,fraction,fraction2;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"JAW BENCH FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
void Servo_Init(void) { memset(enabled,0,sizeof(enabled)); previous=0u; fraction=0u; fraction2=0u; }
uint8_t Servo_IsReady(void) { return 1u; }
void Servo_Off(uint8_t ch) { enabled[ch]=0u; }
void Servo_OffAll(void) { enabled[0]=enabled[1]=0u; }
ServoStatus Servo_SetLimits(uint8_t ch,const ServoLimits *p)
{
    CHECK(!enabled[ch] && ServoMath_LimitsValid(p)); limits[ch]=*p;
    current[ch]=target[ch]=p->neutral_us; return SERVO_OK;
}
ServoStatus Servo_Enable(uint8_t ch)
{ ++enables[ch]; enabled[ch]=1u; current[ch]=target[ch]=limits[ch].neutral_us; return SERVO_OK; }
uint8_t Servo_IsEnabled(uint8_t ch) { return enabled[ch]; }
uint16_t Servo_CurrentUs(uint8_t ch) { return (uint16_t)current[ch]; }
int32_t Servo_TargetUs(uint8_t ch) { return target[ch]; }
ServoStatus Servo_SetTarget(uint8_t ch,int32_t value)
{
    uint16_t resolved;
    if(!enabled[ch]) return SERVO_REJECT_STATE;
    if(!ServoMath_Resolve(&limits[ch],value,&resolved)) return SERVO_REJECT_RANGE;
    ++targets[ch]; target[ch]=resolved; return SERVO_OK;
}
void Servo_Process(uint32_t now)
{
    if(enabled[0]) current[0]=ServoMath_SlewAccumulated(current[0],target[0],now-previous,limits[0].slew_us_per_s,&fraction);
    if(enabled[1]) current[1]=ServoMath_SlewAccumulated(current[1],target[1],now-previous,limits[1].slew_us_per_s,&fraction2);
    previous=now;
}
uint32_t AppClock_Now(void) { return previous; }
void AppTouch_Process(uint32_t now) { (void)now; }
void AppTouch_GetState(int32_t *x,int32_t *y,uint8_t *pressed) { *x=tx; *y=ty; *pressed=down; }
void LCD_Clear(uint32_t c) { (void)c; }
void LCD_SetFont(sFONT *p) { (void)p; }
void LCD_SetTextColor(uint32_t c) { (void)c; }
void LCD_SetBackColor(uint32_t c) { (void)c; }
void LCD_FillRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h)
{ CHECK((unsigned)x+w<=480u && (unsigned)y+h<=272u); }
void LCD_DisplayStringAt(uint16_t x,uint16_t y,uint8_t *t,unsigned mode)
{ (void)t; (void)mode; CHECK(x<480u && y<272u); }
#if !defined(SERVO_DUAL_BENCH) || !SERVO_DUAL_BENCH
static ServoBenchAction tap(ServoBenchState *s,int32_t x,int32_t y)
{
    (void)ServoBench_Touch(s,0,0,0u);
    return ServoBench_Touch(s,LCD_TOUCH_X(x),LCD_TOUCH_Y(y),1u);
}
#endif
#if defined(SERVO_DUAL_BENCH) && SERVO_DUAL_BENCH
int main(void)
{
    ServoDualState s; unsigned i;
    CHECK(ServoDualBench_Init(&s)); CHECK(!enabled[0] && !enabled[1]);
    CHECK(limits[0].neutral_us==1700 && limits[0].max_us==2430 && limits[0].slew_us_per_s==0);
    CHECK(limits[1].min_us==1240 && limits[1].max_us==2600 && limits[1].neutral_us==1900 && limits[1].slew_us_per_s==0);
    CHECK(ServoDualBench_Touch(&s,LCD_TOUCH_X(40),LCD_TOUCH_Y(125),1u)==DUAL_NONE);
    for(i=0;i<8u;++i) {
        int32_t x=20+(int32_t)(i%4u)*119, y=125+(int32_t)(i/4u)*48;
        (void)ServoDualBench_Touch(&s,0,0,0u);
        CHECK(ServoDualBench_Touch(&s,LCD_TOUCH_X(x),LCD_TOUCH_Y(y),1u)==(ServoDualAction)(i+1u));
        CHECK(ServoDualBench_Touch(&s,LCD_TOUCH_X(x),LCD_TOUCH_Y(y),1u)==DUAL_NONE);
    }
    ServoDualBench_Command(&s,DUAL_JAW_HIGH); CHECK(s.last_status==SERVO_REJECT_STATE && !enabled[0]);
    ServoDualBench_Command(&s,DUAL_JAW_ON); CHECK(enabled[0] && current[0]==1700 && !enabled[1]);
    ServoDualBench_Command(&s,DUAL_JAW_HIGH); CHECK(target[0]==2430 && !enabled[1]);
    ServoDualBench_Command(&s,DUAL_JAW_ON); CHECK(enables[0]==1u && target[0]==2430);
    ServoDualBench_Command(&s,DUAL_TABLE_PLUS); CHECK(s.last_status==SERVO_REJECT_STATE && !enabled[1]);
    ServoDualBench_Command(&s,DUAL_TABLE_ON); CHECK(enabled[1] && current[1]==1900 && target[1]==1900);
    ServoDualBench_Command(&s,DUAL_TABLE_PLUS); CHECK(target[1]==2600);
    ServoDualBench_Command(&s,DUAL_TABLE_PLUS); CHECK(s.last_status==SERVO_REJECT_RANGE && target[1]==2600);
    ServoDualBench_Command(&s,DUAL_TABLE_MINUS); CHECK(target[1]==1900);
    ServoDualBench_Command(&s,DUAL_TABLE_MINUS); CHECK(target[1]==1240);
    ServoDualBench_Command(&s,DUAL_TABLE_MINUS); CHECK(s.last_status==SERVO_REJECT_RANGE && target[1]==1240);
    down=0; ServoDualBench_Process(&s,100u); CHECK(current[0]==2430 && current[1]==1240);
    ServoDualBench_Process(&s,300u); CHECK(current[1]==1240);
    ServoDualBench_Command(&s,DUAL_JAW_OFF); CHECK(!enabled[0] && enabled[1]);
    ServoDualBench_Command(&s,DUAL_TABLE_PLUS); CHECK(target[1]==1900);
    ServoDualBench_Process(&s,301u); CHECK(current[1]==1900);
    ServoDualBench_Command(&s,DUAL_TABLE_PLUS); CHECK(target[1]==2600);
    ServoDualBench_Command(&s,DUAL_TABLE_MINUS); CHECK(target[1]==1900);
    ServoDualBench_Command(&s,DUAL_TABLE_MINUS); CHECK(target[1]==1240);
    ServoDualBench_Command(&s,DUAL_TABLE_OFF); CHECK(!enabled[0] && !enabled[1]);
    ServoDualBench_Process(&s,1000u); CHECK(current[1]==1900);
    printf("PASS: %u dual manual checks; separate controls, boot OFF, 1900 start, bounds, OFF\n",checks);
    return 0;
}
#elif defined(SERVO_TURNTABLE_BENCH) && SERVO_TURNTABLE_BENCH
int main(void)
{
    ServoBenchState s; unsigned i;
    CHECK(ServoBench_Init(&s));
    CHECK(!enabled[0] && !enabled[1] && !enables[0]);
    CHECK(limits[0].min_us==1800 && limits[0].max_us==2000 && limits[0].neutral_us==1900);
    CHECK(ServoBench_Touch(&s,LCD_TOUCH_X(40),LCD_TOUCH_Y(150),1u)==JAW_NONE);
    for(i=0;i<6u;++i) CHECK(tap(&s,40+(int32_t)(i%3u)*156,150+(int32_t)(i/3u)*56)==(ServoBenchAction)(i+1u));
    ServoBench_Command(&s,JAW_PLUS); CHECK(s.last_status==SERVO_REJECT_STATE);
    ServoBench_Command(&s,JAW_ENABLE); CHECK(enabled[0] && current[0]==1900 && target[0]==1900 && !enabled[1]);
    ServoBench_Command(&s,JAW_PLUS); CHECK(target[0]==1950);
    down=0; ServoBench_Process(&s,100u); CHECK(current[0]==1920);
    ServoBench_Process(&s,300u); CHECK(current[0]==1950);
    ServoBench_Command(&s,JAW_HIGH); CHECK(target[0]==2000);
    ServoBench_Command(&s,JAW_PLUS); CHECK(s.last_status==SERVO_REJECT_RANGE && target[0]==2000);
    ServoBench_Command(&s,JAW_LOW); CHECK(target[0]==1800);
    ServoBench_Command(&s,JAW_MINUS); CHECK(s.last_status==SERVO_REJECT_RANGE && target[0]==1800);
    ServoBench_Command(&s,JAW_OFF); CHECK(!enabled[0] && !enabled[1]);
    ServoBench_Process(&s,1000u); CHECK(!enabled[0] && current[0]==1950 && !enables[1] && !targets[1]);
    printf("PASS: %u turntable manual checks; boot OFF, start 1900, bounds, slew, OFF\n",checks);
    return 0;
}
#else
int main(void)
{
    ServoBenchState s; unsigned i,n;
    CHECK(ServoBench_Init(&s)); CHECK(!enabled[0] && !enabled[1] && !enables[0] && !enables[1]);
    /* Held-at-boot touch and held gestures must not trigger repeated moves. */
    CHECK(ServoBench_Touch(&s,LCD_TOUCH_X(40),LCD_TOUCH_Y(150),1u)==JAW_NONE);
    for(i=0;i<6u;++i) {
        ServoBenchAction a=tap(&s,40+(int32_t)(i%3u)*156,150+(int32_t)(i/3u)*56);
        CHECK(a==(ServoBenchAction)(i+1u));
        CHECK(ServoBench_Touch(&s,LCD_TOUCH_X(40),LCD_TOUCH_Y(150),1u)==JAW_NONE);
    }
    (void)ServoBench_Touch(&s,0,0,0u); CHECK(ServoBench_Touch(&s,-1,0,1u)==JAW_NONE);
    CHECK(ServoBench_Touch(&s,LCD_TOUCH_X(40),LCD_TOUCH_Y(150),1u)==JAW_NONE);
    ServoBench_Command(&s,JAW_LOW); CHECK(s.last_status==SERVO_REJECT_STATE && !enables[0]);
    ServoBench_Command(&s,JAW_ENABLE); CHECK(enabled[0] && !enabled[1] && enables[0]==1u);
    CHECK(current[0]==1700 && target[0]==1700);
    ServoBench_Command(&s,JAW_HIGH); CHECK(target[0]==2430);
    ServoBench_Command(&s,JAW_ENABLE); CHECK(enables[0]==1u && target[0]==2430);
    n=targets[0]; ServoBench_Command(&s,JAW_PLUS);
    CHECK(s.last_status==SERVO_REJECT_RANGE && target[0]==2430 && targets[0]==n);
    ServoBench_Command(&s,JAW_LOW); CHECK(target[0]==1700);
    ServoBench_Command(&s,JAW_MINUS); CHECK(s.last_status==SERVO_REJECT_RANGE && target[0]==1700);
    ServoBench_Command(&s,JAW_PLUS); CHECK(target[0]==2430);
    down=0; ServoBench_Process(&s,1u);
    CHECK(current[0]==2430 && target[0]==2430 && fraction==0u);
    ServoBench_Command(&s,JAW_MINUS); CHECK(target[0]==1700);
    ServoBench_Process(&s,2u);
    CHECK(current[0]==1700 && fraction==0u);
    ServoBench_Command(&s,JAW_OFF); CHECK(!enabled[0] && !enabled[1]);
    ServoBench_Process(&s,10000u); CHECK(!enabled[0] && !enables[1] && !targets[1]);
    CHECK(ServoBench_Init(&s)); CHECK(!enabled[0] && !enabled[1]);
    /* Direct target changes take effect at the next process call; OFF cancels output. */
    ServoBench_Command(&s,JAW_ENABLE); ServoBench_Command(&s,JAW_HIGH);
    ServoBench_Process(&s,1u);
    CHECK(current[0]==2430 && target[0]==2430);
    ServoBench_Command(&s,JAW_LOW); ServoBench_Command(&s,JAW_OFF);
    ServoBench_Process(&s,2u);
    CHECK(!enabled[0] && current[0]==2430);
    printf("PASS: %u jaw manual checks; boot OFF, one tap, bounds, immediate target, OFF, lift untouched\n",checks);
    return 0;
}
#endif
