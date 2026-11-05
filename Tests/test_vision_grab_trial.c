#include "mechanism/vision_grab_trial.h"
#include <assert.h>
#include <stdio.h>
static int lowers,closes,stops;
static int lower(void *p,float mm,uint32_t now){(void)p;(void)now;assert(mm==100.0f);++lowers;return 1;}
static int closejaw(void *p,uint32_t now){(void)p;(void)now;++closes;return 1;}
static void stop(void *p,uint32_t now){(void)p;(void)now;++stops;}
int main(void){VisionGrabTrial t;VgtPort p={0,lower,closejaw,stop};Vgt_Init(&t,p);
assert(!Vgt_Start(&t,1,1,0,640,360,20,0)); assert(Vgt_Start(&t,1,1,1,640,360,20,0));
Vgt_Update(&t,1,1,640,360,0,0,0);Vgt_Update(&t,1,2,640,360,0,0,150);Vgt_Update(&t,1,3,640,360,0,0,300);assert(lowers==1 && t.state==VGT_LOWERING);
Vgt_Update(&t,1,4,640,360,0,0,400);assert(closes==0);Vgt_Update(&t,1,5,640,360,1,0,500);assert(closes==1 && t.state==VGT_CLOSING);Vgt_Update(&t,1,6,640,360,1,1,600);assert(t.state==VGT_DONE);
Vgt_Stop(&t,700);assert(stops==1);assert(Vgt_Start(&t,1,1,1,640,360,20,800));Vgt_Update(&t,1,7,640,360,0,0,800);Vgt_Update(&t,1,8,700,360,0,0,900);Vgt_Update(&t,1,9,640,360,0,0,1200);assert(lowers==1);puts("vision grab trial tests passed");return 0;}
