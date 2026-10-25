#ifndef MECHANISM_H
#define MECHANISM_H
#include "mechanism_config.h"
#include "./motor/motor_bus.h"
#include "./chassis/chassis.h"
typedef enum { MECH_ARM, MECH_LIFT, MECH_TRIM } MechanismAxis;
typedef enum { MECH_ACCEPTED, MECH_BUSY, MECH_NOT_READY, MECH_INVALID } MechanismRequestResult;
typedef enum { MECH_NONE, MECH_RUNNING, MECH_SUCCESS, MECH_FAILED, MECH_CANCELLED } MechanismResult;
typedef enum { MECH_UNREFERENCED, MECH_HOMED, MECH_SESSION_REFERENCE } MechanismReference;
typedef enum { MECH_IDLE, MECH_PREPARE, MECH_CHECK, MECH_INITIAL_RELEASE,
    MECH_FAST_SEEK, MECH_BACKOFF, MECH_SLOW_SEEK, MECH_PARK_RELEASE,
    MECH_MOVING, MECH_BETWEEN, MECH_STOPPING, MECH_FAULT,
    MECH_LIFT_MEASURE } MechanismState;
typedef enum { MECH_OK, MECH_CONFIG, MECH_FEEDBACK, MECH_AXIS_BUSY,
    MECH_INTERLOCK, MECH_ENABLE_FAILED, MECH_TIMEOUT, MECH_BUS_REJECTED,
    MECH_LIMIT, MECH_LIMIT_UNSTABLE, MECH_SAMPLE_GAP, MECH_NOT_FOUND,
    MECH_NOT_RELEASED, MECH_FINAL_ERROR, MECH_RESOLUTION,
    MECH_STOP_UNCONFIRMED, MECH_BAD_REQUEST, MECH_REFERENCE, MECH_DRIVER_FAULT } MechanismReason;
typedef struct {
    uint8_t raw, stable, valid, sampled;
    uint32_t changed_ms;
} MechanismLimit;
typedef struct {
    MechanismState state;
    MechanismResult result;
    MechanismResult lift_measure_result;
    int64_t lift_measure_start, lift_measure_counts;
    MechanismReason reason, rejection;
    MechanismReference reference[3];
    MechanismLimit limit[2];
    double position[3], target;
    int64_t origin_counts[3], approach_counts[2], quantized_target;
    uint32_t config_issues[3], request_id, bus_request_id;
    uint32_t trigger_ms, stop_request_ms, stop_submit_ms, stop_confirm_ms;
    uint32_t max_sample_gap_ms;
    uint8_t axis, stage, acceptance, done, stop_pending, trim_ready, homing;
} MechanismSnapshot;
typedef struct {
    MotorBus *bus;
    const Chassis *chassis; /* optional peer; independent bench needs no wheels */
    MechanismConfig config;
    MechanismSnapshot data;
    uint32_t last_sample_ms, deadline, stable_since, position_seq, wait_position_ms;
    uint32_t segment_request, trigger_at, stop_at;
    uint32_t measure_refresh_ms;
    double segment_target, trim_start;
    uint8_t owned, auto_pending, operation, segment, stable, enable_wait;
    uint8_t triggered, touched, stop_mask, waiting_position;
} Mechanism;
void Mechanism_Init(Mechanism *m, MotorBus *bus, const MechanismConfig *config, uint32_t now_ms);
/* Call once per foreground loop BEFORE MotorBus_Process. No GPIO in service. */
void Mechanism_SampleLimits(Mechanism *m, uint8_t arm_high, uint8_t lift_high, uint32_t now_ms);
void Mechanism_Process(Mechanism *m, uint32_t now_ms);
MechanismRequestResult Mechanism_HomeAll(Mechanism *m, uint32_t now_ms);
MechanismRequestResult Mechanism_MoveTo(Mechanism *m, MechanismAxis axis, float target, float speed, uint32_t now_ms);
MechanismRequestResult Mechanism_MoveRelative(Mechanism *m, MechanismAxis axis, float delta, float speed, uint32_t now_ms);
MechanismRequestResult Mechanism_StartAcceptance(Mechanism *m, uint32_t now_ms);
/* Explicit bottom-to-top run; count feedback directly, independent of mm/rev.
 * Success is a measurement only, not a HOME reference or automatic calibration. */
MechanismRequestResult Mechanism_MeasureLift(Mechanism *m, uint32_t now_ms);
void Mechanism_Stop(Mechanism *m, uint32_t now_ms);
void Mechanism_GetSnapshot(const Mechanism *m, MechanismSnapshot *out);
int Mechanism_OwnsAxes(const Mechanism *m);
const char *Mechanism_ReasonName(MechanismReason reason);
const char *Mechanism_StateName(MechanismState state);
#endif
