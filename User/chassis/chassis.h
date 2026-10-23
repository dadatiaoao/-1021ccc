#ifndef CHASSIS_H
#define CHASSIS_H
#include "chassis_kinematics.h"
#include "chassis_odometry.h"
#include "./motor/motor_bus.h"
typedef enum { CHASSIS_DISARMED, CHASSIS_PREPARING, CHASSIS_READY, CHASSIS_VELOCITY,
    CHASSIS_MOVING, CHASSIS_ROTATING, CHASSIS_STOPPING, CHASSIS_FAULT } ChassisState;
typedef enum { CHASSIS_NONE, CHASSIS_RUNNING, CHASSIS_SUCCESS, CHASSIS_CANCELLED, CHASSIS_FAILED } ChassisResult;
typedef enum { CHASSIS_ACCEPTED, CHASSIS_BUSY, CHASSIS_NOT_READY, CHASSIS_INVALID } ChassisRequestResult;
typedef enum { CHASSIS_OK, CHASSIS_CONFIG, CHASSIS_FEEDBACK, CHASSIS_MOTOR_FAULT,
    CHASSIS_STOP_UNCONFIRMED, CHASSIS_AXIS_BUSY, CHASSIS_ENABLE_FAILED, CHASSIS_PREPARE_TIMEOUT,
    CHASSIS_INPUT_EXPIRED, CHASSIS_GYRO_INVALID, CHASSIS_ODOMETRY_INVALID, CHASSIS_SENSOR_CHANGED,
    CHASSIS_ACTION_TIMEOUT, CHASSIS_STOP_TIMEOUT, CHASSIS_FINAL_ERROR, CHASSIS_BAD_REQUEST,
    CHASSIS_BUS_REJECTED } ChassisReason;
typedef struct {
    ChassisOdometryIssue issue;
    uint32_t at_ms, sample_age_ms;
    uint8_t valid, wheel;
} ChassisFault;
typedef struct {
    ChassisState state;
    ChassisResult result;
    ChassisReason reason, rejection;
    ChassisVelocity requested, output;
    ChassisWheels wheels;
    double x_mm, y_mm, angle_deg;
    float target_x_mm, target_y_mm, target_angle_deg;
    float error_mm, error_deg;
    int32_t measured_rpm[4];
    uint32_t request_id, config_issues, publish_accepted, publish_rejected;
    uint32_t submitted_frames, last_submit_ms, max_submit_gap_ms, feedback_age_ms;
    uint8_t sequence_active, stage, done, pose_valid, gyro_valid, odometry_valid, stop_pending;
    ChassisFault fault; /* first failure evidence; live recovery never clears it */
} ChassisSnapshot;
typedef struct {
    MotorBus *bus;
    ChassisConfig config;
    ChassisSnapshot data;
    ChassisOdometry odometry;
    GyroSnapshot gyro, action_gyro;
    double origin_x, origin_y, origin_angle;
    float action_limit;
    uint32_t input_deadline, deadline, control_ms, publish_ms, stop_ms;
    uint32_t stable_since, stable_odometry_seq, stable_gyro_seq, odometry_generation;
    uint32_t verify_ms, between_due, last_submitted, observed_frames;
    ChassisResult stop_result;
    uint8_t finite, stable, verifying, between, enable_wait, have_origin;
    uint8_t enable_index;
} Chassis;
void Chassis_Init(Chassis *c, MotorBus *bus, const ChassisConfig *config, uint32_t now_ms);
/* Config is copied. Keep it unchanged while armed; apply measured changes by
   initializing a disarmed instance after verified STOP. Snapshots are diagnostic
   state, not writable command interfaces. */
/* Translation is a wheel-based estimate; slip is not measured. No optical-flow input. */
void Chassis_Process(Chassis *c, const GyroSnapshot *gyro, uint32_t now_ms);
ChassisRequestResult Chassis_Prepare(Chassis *c, uint32_t now_ms);
ChassisRequestResult Chassis_SetBodyVelocity(Chassis *c, float vx_mm_s, float vy_mm_s, float omega_deg_s, uint32_t now_ms);
ChassisRequestResult Chassis_MoveRelative(Chassis *c, float dx_mm, float dy_mm, float max_speed_mm_s, uint32_t now_ms);
ChassisRequestResult Chassis_RotateRelative(Chassis *c, float angle_deg, float max_rate_deg_s, uint32_t now_ms);
void Chassis_Stop(Chassis *c, uint32_t now_ms);
void Chassis_GetSnapshot(const Chassis *c, ChassisSnapshot *out);
int Chassis_OwnsWheels(const Chassis *c);
ChassisRequestResult Chassis_StartAcceptance(Chassis *c, uint32_t now_ms);
const char *Chassis_StateName(ChassisState state);
const char *Chassis_ReasonName(ChassisReason reason);
const char *Chassis_StageName(uint8_t stage);
#endif
