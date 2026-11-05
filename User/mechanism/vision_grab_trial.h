#ifndef VISION_GRAB_TRIAL_H
#define VISION_GRAB_TRIAL_H
#include <stdint.h>
/* Test logic only: caller must connect real homing, vision, axis and servo drivers. */
typedef enum { VGT_IDLE, VGT_WAIT_TARGET, VGT_LOWERING, VGT_CLOSING, VGT_DONE, VGT_FAULT } VgtState;
typedef struct {
  void *ctx; int (*lower)(void *,float,uint32_t); int (*close)(void *,uint32_t); void (*stop)(void *,uint32_t);
} VgtPort;
typedef struct {
  VgtPort port; VgtState state; uint16_t x,y,tolerance,last_frame;
  uint32_t stable_start,last_sample,phase_ms; uint8_t calibrated, have_frame, stable;
} VisionGrabTrial;
void Vgt_Init(VisionGrabTrial *,VgtPort);
int Vgt_Start(VisionGrabTrial *,uint8_t lift_homed,uint8_t arm_ready,uint8_t calibrated,uint16_t x,uint16_t y,uint16_t tolerance,uint32_t now);
void Vgt_Update(VisionGrabTrial *,uint8_t fresh,uint16_t frame,uint16_t x,uint16_t y,int lower_result,int close_result,uint32_t now);
void Vgt_Stop(VisionGrabTrial *,uint32_t now);
#endif
