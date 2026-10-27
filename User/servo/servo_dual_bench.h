#ifndef SERVO_DUAL_BENCH_H
#define SERVO_DUAL_BENCH_H
#include <stdint.h>

typedef enum {
    DUAL_NONE, DUAL_JAW_ON, DUAL_JAW_LOW, DUAL_JAW_HIGH, DUAL_JAW_OFF,
    DUAL_TABLE_ON, DUAL_TABLE_MINUS, DUAL_TABLE_PLUS, DUAL_TABLE_OFF
} ServoDualAction;
typedef struct { uint8_t was_down, last_status; } ServoDualState;
int ServoDualBench_Init(ServoDualState *s);
ServoDualAction ServoDualBench_Touch(ServoDualState *s,int32_t x,int32_t y,uint8_t down);
void ServoDualBench_Command(ServoDualState *s,ServoDualAction action);
void ServoDualBench_Process(ServoDualState *s,uint32_t now);
void ServoDualBench_Run(void);
#endif
