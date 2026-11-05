#ifndef WHEEL_ONLY_TRIAL_H
#define WHEEL_ONLY_TRIAL_H
#include "chassis.h"
typedef enum { WT_IDLE, WT_RUNNING, WT_STOPPING, WT_DONE, WT_FAULT } WheelTrialState;
typedef enum { WT_FORWARD, WT_BACK, WT_LEFT, WT_RIGHT, WT_CCW, WT_CW } WheelTrialMove;
typedef struct {
    Chassis *chassis; MotorBus *bus; WheelTrialState state; WheelTrialMove move;
    int64_t origin[4], last_count[4]; uint32_t seq[4], sample_ms[4];
    uint32_t start_ms, publish_ms, progress_ms; double progress, max_progress;
    const char *note; uint8_t fault;
} WheelOnlyTrial;
double WheelTrial_Estimate(const ChassisConfig *,WheelTrialMove,const int64_t origin[4],const int64_t current[4]);
void WheelTrial_Init(WheelOnlyTrial *,Chassis *,MotorBus *);
int WheelTrial_Start(WheelOnlyTrial *,WheelTrialMove,uint32_t);
void WheelTrial_Stop(WheelOnlyTrial *,uint32_t);
void WheelTrial_Process(WheelOnlyTrial *,uint32_t);
#endif
