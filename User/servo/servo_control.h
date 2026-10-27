#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H
#include "servo_pwm.h"
typedef enum {
    DUAL_NONE, DUAL_JAW_ON, DUAL_JAW_LOW, DUAL_JAW_HIGH, DUAL_JAW_OFF,
    DUAL_TABLE_ON, DUAL_TABLE_MINUS, DUAL_TABLE_PLUS, DUAL_TABLE_OFF
} ServoDualAction;
typedef struct { uint8_t boot_pending, last_status; } ServoDualState;
int ServoControl_Init(ServoDualState *s);
void ServoControl_Command(ServoDualState *s, ServoDualAction action);
/* All three axes must be homed and stopped. Failure/STOP cancels auto-enable. */
void ServoControl_Startup(ServoDualState *s, uint8_t homed, uint8_t busy, uint8_t failed);
void ServoControl_CancelStartup(ServoDualState *s);
#endif
