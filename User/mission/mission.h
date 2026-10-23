#ifndef MISSION_H
#define MISSION_H
#include "../qr/qr_task_code.h"

/* This build deliberately supports start/finish zone 2 only. */
#define MISSION_START_ZONE 2u
typedef enum { MISSION_IDLE, MISSION_RUNNING, MISSION_FINISHED,
    MISSION_ABORTED, MISSION_FAULT } MissionState;
typedef enum { MISSION_NAV_QR, MISSION_READ_QR, MISSION_NAV_SOURCE,
    MISSION_SOURCE_TO_SLOT, MISSION_NAV_ROUGH, MISSION_SLOT_TO_ROUGH,
    MISSION_ROUGH_TO_SLOT, MISSION_NAV_STORAGE, MISSION_SLOT_TO_STORAGE,
    MISSION_STOW, MISSION_NAV_HOME } MissionStep;
typedef enum { MISSION_WAIT, MISSION_SUCCEEDED, MISSION_ACTION_FAILED } MissionActionResult;
typedef enum { MISSION_NO_ERROR, MISSION_NOT_READY, MISSION_PORT_ERROR,
    MISSION_BAD_TASK, MISSION_ACTION_ERROR, MISSION_TIMEOUT,
    MISSION_CARGO_ERROR, MISSION_USER_STOP } MissionError;
enum { MISSION_BLOCK_CHASSIS=1u, MISSION_BLOCK_ROUTES=2u,
    MISSION_BLOCK_MECHANISM=4u, MISSION_BLOCK_VISION=8u,
    MISSION_BLOCK_FEEDBACK=16u, MISSION_BLOCK_BUSY=32u };

typedef struct {
    uint32_t id;
    MissionStep step;
    uint8_t batch, order, color, slot, position, layer;
} MissionAction;
typedef struct {
    void *context;
    uint32_t (*blocked)(void *context, uint32_t now);
    int (*begin)(void *context, const MissionAction *action, uint32_t now);
    MissionActionResult (*poll)(void *context, uint32_t id, QrTaskCode *task, uint32_t now);
    void (*stop)(void *context, uint32_t now);
} MissionPort;
typedef struct {
    MissionPort port;
    MissionState state;
    MissionError error;
    MissionAction action;
    QrTaskCode task;
    uint32_t blocked, next_id, started_ms, action_ms;
    uint32_t completed_actions, grasp_count, place_count;
    uint8_t started, active, task_valid;
    uint8_t slots[3], rough[3], storage[3], base_position[7];
} Mission;

void Mission_Init(Mission *m, MissionPort port);
/* One accepted start per boot. Repeated START never restarts an old run. */
int Mission_Start(Mission *m, uint32_t now);
void Mission_Process(Mission *m, uint32_t now);
void Mission_Stop(Mission *m, uint32_t now);
int Mission_IsRunning(const Mission *m);
const char *Mission_StateName(MissionState state);
const char *Mission_StepName(MissionStep step);
const char *Mission_ErrorName(MissionError error);
#endif
