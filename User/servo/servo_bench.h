#ifndef SERVO_BENCH_H
#define SERVO_BENCH_H
#include <stdint.h>
/* LCD target stays normal. JAW build overrides to 1: manual jaw only.
 * Both outputs remain OFF until an explicit ENABLE button press. */
#ifndef SERVO_BENCH_ENABLE
#define SERVO_BENCH_ENABLE 0
#endif
typedef enum { JAW_NONE, JAW_ENABLE, JAW_MINUS, JAW_PLUS,
    JAW_LOW, JAW_HIGH, JAW_OFF } ServoBenchAction;
typedef struct { uint8_t was_down,last_status; } ServoBenchState;
int ServoBench_Init(ServoBenchState *s);
ServoBenchAction ServoBench_Touch(ServoBenchState *s,int32_t x,int32_t y,uint8_t down);
void ServoBench_Command(ServoBenchState *s,ServoBenchAction action);
void ServoBench_Process(ServoBenchState *s,uint32_t now);
void ServoBench_Run(void);
#endif
