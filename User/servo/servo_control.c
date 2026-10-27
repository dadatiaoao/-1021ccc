#include "servo_control.h"
#include <string.h>
enum { DUAL_JAW = 0, DUAL_TABLE = 1 };
static const ServoLimits jaw_limits={1700u,2430u,1700u,0u};
/* User-recorded turntable pulse positions; actual position is not sensed. */
static const ServoLimits table_limits={1240u,2600u,1240u,0u};
int ServoControl_Init(ServoDualState *s)
{
    memset(s,0,sizeof(*s)); s->boot_pending=1u;
    Servo_Init();
    if(!Servo_IsReady()) return 0;
    Servo_OffAll();
    return Servo_SetLimits(DUAL_JAW,&jaw_limits)==SERVO_OK &&
           Servo_SetLimits(DUAL_TABLE,&table_limits)==SERVO_OK;
}
void ServoControl_Command(ServoDualState *s,ServoDualAction action)
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

void ServoControl_CancelStartup(ServoDualState *s) { s->boot_pending=0u; }
void ServoControl_Startup(ServoDualState *s, uint8_t homed, uint8_t busy, uint8_t failed)
{
    if (!s->boot_pending) return;
    if (failed) { s->boot_pending=0u; return; }
    if (!homed || busy) return;
    s->boot_pending=0u;
    s->last_status=(uint8_t)Servo_Enable(SERVO_JAW);
    if (s->last_status==SERVO_OK) s->last_status=(uint8_t)Servo_Enable(SERVO_TABLE);
    if (s->last_status!=SERVO_OK) Servo_OffAll();
}
