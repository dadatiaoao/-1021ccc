#ifndef VISION_SERVICE_H
#define VISION_SERVICE_H
#include "vision_protocol.h"
#include "../qr/qr_task_code.h"
#define VISION_ACK_MS 3000u
#define VISION_TARGET_MS 500u
#define VISION_LINK_MS 3000u
typedef struct {
    void *context;
    int (*send)(void *context,const uint8_t *bytes,uint8_t length);
    int (*busy)(void *context);
} VisionTransport;
typedef enum { VISION_IDLE, VISION_CONFIGURING, VISION_ACTIVE,
               VISION_STOPPING, VISION_STOPPED, VISION_FAULT } VisionState;
typedef enum { VISION_ERROR_NONE, VISION_ERROR_ACK_TIMEOUT, VISION_ERROR_REJECTED,
               VISION_ERROR_UART, VISION_ERROR_OVERFLOW, VISION_ERROR_LINK,
               VISION_ERROR_TX_BUSY } VisionError;
typedef struct {
    uint16_t request_id, pose_id, frame_id, cx, cy;
    uint8_t valid, flags, target_id;
    uint32_t received_ms;
} VisionObservation;
typedef struct {
    VisionTransport transport;
    VisionParser parser;
    VisionState state;
    VisionError error;
    uint8_t mode, color, zone, position, step, waiting, pending_cmd;
    uint16_t request_id, pose_id, last_frame_id;
    uint8_t online, heartbeat_seen, frame_seen, task_valid;
    uint8_t legacy_valid, legacy_id, place_valid, place_candidate;
    uint16_t legacy_cx, legacy_cy, distance_cm;
    int16_t offset_tenths;
    QrTaskCode task;
    VisionObservation observation;
    uint32_t last_heartbeat_ms, last_rx_ms, command_ms, queued_ms;
    uint32_t legacy_ms, place_ms, task_ms, raw_bytes, invalid_frames;
    uint32_t ignored_frames, ack_count, timeout_count, transport_errors;
} VisionService;
void VisionService_Init(VisionService *s,VisionTransport transport,uint32_t now);
/* Configures STOP -> context -> color -> station -> SCAN, one ACK at a time.
   mode: 0 QR, 1 material, 2 station coarse, 3 pick candidate, 4 geometry candidate.
   position is station 1..3, never a scoring-ring index. No motor commands here. */
int VisionService_Start(VisionService *s,uint8_t mode,uint8_t color,uint8_t zone,
                        uint8_t position,uint16_t camera_pose_id,uint32_t now);
void VisionService_Stop(VisionService *s,uint32_t now);
void VisionService_Receive(VisionService *s,const uint8_t *bytes,size_t count,uint32_t received_ms);
void VisionService_Process(VisionService *s,uint32_t now);
void VisionService_TransportError(VisionService *s,VisionError error,uint32_t now);
int VisionService_TargetFresh(const VisionService *s,uint32_t now);
const char *VisionService_StateName(const VisionService *s);
const char *VisionService_ErrorName(const VisionService *s);
#endif
