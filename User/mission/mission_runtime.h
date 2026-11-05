#ifndef MISSION_RUNTIME_H
#define MISSION_RUNTIME_H
#include "mission.h"
#include "mission_config.h"
#include "../chassis/chassis.h"
#include "../vision/vision_service.h"

/* The mechanism owner implements complete transfers: pose/empty-gripper
   alignment, take, onboard slot, return pose, vertical release, retract.
   ready() means actually homed and stowed, not just a config flag.
   poll success requires normal motor completion + calibrated servo duration.
   ACK or a YOLO box alone must NEVER complete a transfer.
   The owner uses the same Chassis/MotorBus/VisionService, not another UART writer.
   Body alignment may run inside the action; freeze it before grasp/vertical
   release, and return success only after the arm is retracted and wheels rest.
   Dynamic lane/obstacle checking is required through path_clear for motion. */
typedef struct {
    void *context;
    int (*ready)(void *context, uint32_t now);
    int (*path_clear)(void *context, MissionRouteId route, uint8_t waypoint, uint32_t now);
    int (*begin)(void *context, const MissionAction *action, uint32_t now);
    MissionActionResult (*poll)(void *context, uint32_t action_id, uint32_t now);
    void (*stop)(void *context, uint32_t now);
} MissionHardware;
typedef struct {
    Chassis *chassis;
    MotorBus *motor;
    VisionService *vision;
    MissionConfig config;
    MissionHardware hardware;
    MissionAction action;
    QrTaskCode qr_result;
    MissionRouteId route;
    double origin_x, origin_y, origin_angle;
    uint32_t origin_generation, chassis_id;
    uint16_t vision_id;
    uint8_t have_origin, waypoint, nav_phase;
} MissionRuntime;
void MissionRuntime_Init(MissionRuntime *r, Chassis *chassis, MotorBus *motor,
    VisionService *vision, const MissionConfig *config);
MissionPort MissionRuntime_Port(MissionRuntime *r);
/* Bind calibrated hardware before starting. NULL/unimplemented is NOT READY. */
void MissionRuntime_SetHardware(MissionRuntime *r, MissionHardware hardware);
#endif
