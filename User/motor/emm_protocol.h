#ifndef EMM_PROTOCOL_H
#define EMM_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#define EMM_POSITION 0x36u
#define EMM_SPEED    0x35u
#define EMM_STATUS   0x3Au
#define EMM_VERSION  0x1Fu
#define EMM_SYNC     0xFFu
#define EMM_MULTI    0xAAu
#define EMM_COMMAND_CAPACITY 128u
#define EMM_ENABLE  0xF3u
#define EMM_VELOCITY 0xF6u
#define EMM_MOVE    0xFDu
#define EMM_STOP    0xFEu
#define EMM_HOME 0x9Au
#define EMM_HOME_ABORT 0x9Cu
#define EMM_HOME_CONFIG 0x4Cu
#define EMM_HOME_BACKOFF 0x5Cu
#define EMM_HOME_STATUS 0x3Cu
#define EMM_ACK_OK  0x02u
#define EMM_ACK_TIMEOUT 0xFFu
#define EMM_ACK_TRANSPORT 0xFCu

typedef enum { EMM_REL_TARGET, EMM_ABSOLUTE, EMM_REL_ACTUAL } EmmPositionMode;
typedef struct { uint8_t bytes[EMM_COMMAND_CAPACITY], length, code; } EmmCommand;
typedef struct {
    uint8_t address, code, result, flags, home_flags;
    uint16_t firmware, hardware;
    int64_t position;
    int32_t speed;
} EmmFrame;
typedef struct { uint8_t address, used, bytes[32]; uint32_t rejected; } EmmParser;
typedef void (*EmmFrameHandler)(void *context, const EmmFrame *frame);

void EmmParser_Init(EmmParser *p, uint8_t address);
void EmmParser_Feed(EmmParser *p, const uint8_t *data, size_t length,
                    EmmFrameHandler handler, void *context);
EmmCommand Emm_Query(uint8_t address, uint8_t code);
EmmCommand Emm_Enable(uint8_t address, uint8_t enabled);
EmmCommand Emm_Velocity(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration);
EmmCommand Emm_Move(uint8_t address, uint8_t direction, uint16_t rpm,
                    uint8_t acceleration, uint32_t pulses);
EmmCommand Emm_Stop(uint8_t address);
/* X42S native sensorless homing; setup is RAM-only, power-on trigger disabled. */
EmmCommand Emm_HomeConfig(uint8_t address, uint8_t direction, uint16_t rpm,
    uint32_t timeout_ms, uint16_t stall_rpm, uint16_t stall_ma, uint16_t stall_ms);
EmmCommand Emm_HomeBackoff(uint8_t address, uint16_t tenths_degree);
EmmCommand Emm_Home(uint8_t address);
EmmCommand Emm_HomeAbort(uint8_t address);
EmmCommand Emm_VelocityEx(uint8_t address, uint8_t direction, uint16_t rpm,
                          uint8_t acceleration, uint8_t cached);
EmmCommand Emm_MoveEx(uint8_t address, uint8_t direction, uint16_t rpm,
    uint8_t acceleration, uint32_t pulses, EmmPositionMode mode, uint8_t cached);
/* Homogeneous motion only. Position notify bit is independent of unicast caching. */
int Emm_Multi(EmmCommand *out, const EmmCommand *members, uint8_t count);
int Emm_Wheels(EmmCommand *out, const int32_t rpm[4], uint8_t acceleration);
EmmCommand Emm_SyncTrigger(void);
#endif
