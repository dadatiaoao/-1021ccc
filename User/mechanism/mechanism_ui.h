#ifndef MECHANISM_UI_H
#define MECHANISM_UI_H
#include "mechanism.h"
void MechanismUi_Line(char out[43], unsigned row, uint8_t detail,
                      const Mechanism *m, uint32_t now_ms);
#endif
