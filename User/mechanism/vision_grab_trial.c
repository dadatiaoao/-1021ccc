#include "vision_grab_trial.h"
#include <string.h>
#define LOWER_MM 100.0f
#define STABLE_MS 300u
#define MAX_GAP_MS 250u
#define LOWER_TIMEOUT_MS 20000u
#define CLOSE_TIMEOUT_MS 3000u
static void fail(VisionGrabTrial *t,uint32_t now) { t->state=VGT_FAULT; if(t->port.stop)t->port.stop(t->port.ctx,now); }
void Vgt_Init(VisionGrabTrial *t,VgtPort p) { memset(t,0,sizeof(*t)); t->port=p; }
int Vgt_Start(VisionGrabTrial *t,uint8_t home,uint8_t arm,uint8_t cal,uint16_t x,uint16_t y,uint16_t tol,uint32_t now) {
 if(!t || t->state!=VGT_IDLE || !home || !arm || !cal || !tol || x>=1280u || y>=720u || !t->port.lower || !t->port.close || !t->port.stop) return 0;
 t->calibrated=cal; t->x=x; t->y=y; t->tolerance=tol; t->have_frame=t->stable=0u; t->phase_ms=now; t->state=VGT_WAIT_TARGET; return 1;
}
void Vgt_Update(VisionGrabTrial *t,uint8_t fresh,uint16_t frame,uint16_t x,uint16_t y,int lr,int cr,uint32_t now) {
 int dx,dy; uint16_t delta;
 if(!t) return;
 if(t->state==VGT_WAIT_TARGET) {
  if(!fresh || x>=1280u || y>=720u || (t->stable && now-t->last_sample>MAX_GAP_MS)) { t->stable=0u; if(!fresh)return; }
  delta=(uint16_t)(frame-t->last_frame); if(t->have_frame && (!delta || delta>=32768u)) return;
  t->last_frame=frame; t->have_frame=1u; t->last_sample=now; dx=(int)x-(int)t->x; dy=(int)y-(int)t->y;
  if(dx<-(int)t->tolerance || dx>(int)t->tolerance || dy<-(int)t->tolerance || dy>(int)t->tolerance) {t->stable=0u; return;}
  if(!t->stable) {t->stable=1u;t->stable_start=now;return;}
  if(now-t->stable_start>=STABLE_MS) { if(!t->port.lower(t->port.ctx,LOWER_MM,now)) {fail(t,now);return;} t->state=VGT_LOWERING;t->phase_ms=now; }
 } else if(t->state==VGT_LOWERING) {
  if(lr<0 || now-t->phase_ms>=LOWER_TIMEOUT_MS) {fail(t,now);return;}
  if(lr>0) {if(!t->port.close(t->port.ctx,now)){fail(t,now);return;} t->state=VGT_CLOSING;t->phase_ms=now;}
 } else if(t->state==VGT_CLOSING) {
  if(cr<0 || now-t->phase_ms>=CLOSE_TIMEOUT_MS) {fail(t,now);return;}
  if(cr>0) t->state=VGT_DONE;
 }
}
void Vgt_Stop(VisionGrabTrial *t,uint32_t now) {if(!t)return; if(t->port.stop)t->port.stop(t->port.ctx,now);t->state=VGT_IDLE;t->stable=0u;}
