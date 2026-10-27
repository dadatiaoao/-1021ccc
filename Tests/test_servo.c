#include "servo/servo_control.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"servo FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
int main(void)
{
    ServoDualState s;
    uint32_t fraction=0u;
    unsigned i;
    CHECK(ServoControl_Init(&s));
    CHECK(!Servo_IsEnabled(0u) && !Servo_IsEnabled(1u));
    ServoControl_Startup(&s,0u,0u,0u); CHECK(!Servo_IsEnabled(0u));
    ServoControl_Startup(&s,1u,1u,0u); CHECK(!Servo_IsEnabled(0u));
    ServoControl_Startup(&s,1u,0u,0u);
    CHECK(Servo_IsEnabled(0u) && Servo_TargetUs(0u)==1700);
    CHECK(Servo_IsEnabled(1u) && Servo_TargetUs(1u)==1240);
    ServoControl_Command(&s,DUAL_JAW_HIGH); CHECK(Servo_TargetUs(0u)==2430);
    ServoControl_Startup(&s,1u,0u,0u); CHECK(Servo_TargetUs(0u)==2430);
    ServoControl_Command(&s,DUAL_TABLE_PLUS); CHECK(Servo_TargetUs(1u)==1900);
    ServoControl_Command(&s,DUAL_TABLE_PLUS); CHECK(Servo_TargetUs(1u)==2600);
    ServoControl_Command(&s,DUAL_TABLE_PLUS); CHECK(s.last_status==SERVO_REJECT_RANGE);
    CHECK(Servo_SetTarget(0u,2600)==SERVO_REJECT_RANGE);
    ServoControl_Command(&s,DUAL_TABLE_OFF); CHECK(!Servo_IsEnabled(1u));
    for (i=0u;i<2u;++i) {
        CHECK(ServoControl_Init(&s));
        if (i) ServoControl_CancelStartup(&s);
        else ServoControl_Startup(&s,0u,0u,1u);
        ServoControl_Startup(&s,1u,0u,0u);
        CHECK(!Servo_IsEnabled(0u) && !Servo_IsEnabled(1u));
    }
    { int32_t v=1700; for(i=0;i<1000u;++i) v=ServoMath_SlewAccumulated(v,2430,1u,300u,&fraction);
      CHECK(v==2000); }
    puts("PASS: startup waits for all axes and rest; failure/STOP cancellation; one-shot, presets, range and fractional slew");
    return 0;
}
