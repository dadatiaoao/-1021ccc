#ifndef VISION_UI_H
#define VISION_UI_H
#include "vision_service.h"
void VisionUi_Line(char out[43],unsigned row,const VisionService *s,uint8_t mode,
                   uint8_t color,uint8_t zone,uint8_t position,uint32_t now);
#endif
