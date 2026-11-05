#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

/* Same bytes as transport_vision/vision_protocol.py; LEN includes CMD. */
#define VISION_FRAME_MAX 16u
#define VISION_PARAM_MAX 12u
#define VISION_PARTIAL_MS 100u
enum {
    VISION_TASKCODE=0x10, VISION_FOUND=0x11, VISION_NOTFOUND=0x12,
    VISION_PLACE_CHECK=0x13, VISION_HEARTBEAT=0x15, VISION_RING=0x16,
    VISION_OBSERVATION=0x17, VISION_SCAN=0x21, VISION_STOP=0x22,
    VISION_SET_COLOR=0x23, VISION_RESCAN=0x24, VISION_SET_TARGET=0x25,
    VISION_SET_CONTEXT=0x26, VISION_ACK=0x81
};
enum {
    VISION_FLAG_FOUND=1, VISION_FLAG_STABLE=2, VISION_FLAG_PICK_READY=4,
    VISION_FLAG_FULL_BOX=8, VISION_FLAG_AMBIGUOUS=16,
    VISION_FLAG_CALIBRATED=32, VISION_FLAG_REFINED=64, VISION_FLAG_STALE=128
};
typedef struct { uint8_t command, count, params[VISION_PARAM_MAX]; } VisionFrame;
typedef struct {
    uint8_t bytes[VISION_FRAME_MAX], used;
    uint32_t last_byte_ms, frames, bad_checksums, bad_lengths, partial_timeouts;
} VisionParser;
void VisionParser_Init(VisionParser *p);
void VisionParser_Reset(VisionParser *p);
void VisionParser_Expire(VisionParser *p, uint32_t now);
/* Drain Next after Push: a corrupt outer frame may contain several valid frames. */
int VisionParser_Push(VisionParser *p, uint8_t byte, uint32_t received_ms, VisionFrame *out);
int VisionParser_Next(VisionParser *p,VisionFrame *out);
uint8_t VisionProtocol_Build(uint8_t command, const uint8_t *params, uint8_t count,
                             uint8_t out[VISION_FRAME_MAX]);
uint16_t VisionProtocol_U16(const uint8_t *p);
int16_t VisionProtocol_I16(const uint8_t *p);
void VisionProtocol_PutU16(uint8_t *p, uint16_t value);
#endif
