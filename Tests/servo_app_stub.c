/* Host-only stand-in for the two PWM channels; no hardware is controlled. */
#include "servo/servo_pwm.h"
#include <string.h>
static ServoLimits limits[SERVO_CHANNELS];
static int32_t current[SERVO_CHANNELS],target[SERVO_CHANNELS];
static uint8_t enabled[SERVO_CHANNELS],ready;
static uint32_t last_ms,fraction[SERVO_CHANNELS];
void Servo_Init(void)
{
    memset(limits,0,sizeof(limits)); memset(current,0,sizeof(current));
    memset(target,0,sizeof(target)); memset(enabled,0,sizeof(enabled));
    memset(fraction,0,sizeof(fraction)); last_ms=0u; ready=1u;
}
uint8_t Servo_IsReady(void) { return ready; }
ServoStatus Servo_SetLimits(uint8_t ch,const ServoLimits *p)
{
    if(ch>=SERVO_CHANNELS || !p || enabled[ch] || !ServoMath_LimitsValid(p)) return SERVO_REJECT_STATE;
    limits[ch]=*p; current[ch]=target[ch]=p->neutral_us; return SERVO_OK;
}
ServoStatus Servo_Enable(uint8_t ch)
{
    if(!ready || ch>=SERVO_CHANNELS || !ServoMath_LimitsValid(&limits[ch])) return SERVO_REJECT_STATE;
    current[ch]=target[ch]=limits[ch].neutral_us; enabled[ch]=1u; fraction[ch]=0u; return SERVO_OK;
}
void Servo_Off(uint8_t ch) { if(ch<SERVO_CHANNELS) enabled[ch]=0u; }
void Servo_OffAll(void) { unsigned i; for(i=0u;i<SERVO_CHANNELS;++i) Servo_Off((uint8_t)i); }
ServoStatus Servo_SetTarget(uint8_t ch,int32_t value)
{
    uint16_t out;
    if(ch>=SERVO_CHANNELS || !enabled[ch]) return SERVO_REJECT_STATE;
    if(!ServoMath_Resolve(&limits[ch],value,&out)) return SERVO_REJECT_RANGE;
    target[ch]=out; return SERVO_OK;
}
ServoStatus Servo_Jog(uint8_t ch,int32_t delta)
{ return ch<SERVO_CHANNELS ? Servo_SetTarget(ch,current[ch]+delta) : SERVO_REJECT_STATE; }
void Servo_Process(uint32_t now)
{
    unsigned i; uint32_t dt=now-last_ms; last_ms=now;
    for(i=0u;i<SERVO_CHANNELS;++i) if(enabled[i])
        current[i]=ServoMath_SlewAccumulated(current[i],target[i],dt,limits[i].slew_us_per_s,&fraction[i]);
}
const ServoLimits *Servo_GetLimits(uint8_t ch) { return ch<SERVO_CHANNELS ? &limits[ch] : 0; }
uint16_t Servo_CurrentUs(uint8_t ch) { return ch<SERVO_CHANNELS ? (uint16_t)current[ch] : 0u; }
int32_t Servo_TargetUs(uint8_t ch) { return ch<SERVO_CHANNELS ? target[ch] : -1; }
uint8_t Servo_IsEnabled(uint8_t ch) { return ch<SERVO_CHANNELS ? enabled[ch] : 0u; }
