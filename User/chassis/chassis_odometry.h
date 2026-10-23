#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H
#include "chassis_config.h"
#include "./flow/flow_service.h"
#include "./gyro/gyro_service.h"
typedef struct {
    uint32_t sequence, sample_ms;
    double angle;
} ChassisGyroPoint;
typedef enum {
    CHASSIS_ODOM_OK, CHASSIS_ODOM_CONFIG, CHASSIS_ODOM_GYRO_INVALID,
    CHASSIS_ODOM_GYRO_CHANGED, CHASSIS_ODOM_FLOW_INVALID,
    CHASSIS_ODOM_FLOW_STALE, CHASSIS_ODOM_FLOW_REJECTED,
    CHASSIS_ODOM_FLOW_GAP, CHASSIS_ODOM_FLOW_RESTART,
    CHASSIS_ODOM_FLOW_CLEAR, CHASSIS_ODOM_FLOW_OVERFLOW,
    CHASSIS_ODOM_FLOW_RESET, CHASSIS_ODOM_SYNC_TIMEOUT, CHASSIS_ODOM_NUMERIC
} ChassisOdometryIssue;
typedef struct {
    ChassisGyroPoint gyro[2];
    GyroSnapshot gyro_guard;
    FlowSnapshot flow_guard, pending_flow;
    double x, y, flow_angle;
    uint32_t generation;
    ChassisOdometryIssue last_issue;
    uint32_t break_ms, break_flow_age_ms, break_paired_age_ms;
    uint32_t break_rejected, break_gaps, break_segments, break_clears;
    FlowError break_flow_error;
    uint8_t gyro_count, baseline, valid, pending, break_quality;
} ChassisOdometry;
int Chassis_GyroValid(const GyroSnapshot *g, uint32_t now);
int Chassis_GyroContinuous(const GyroSnapshot *a, const GyroSnapshot *b);
void Chassis_OdometryProcess(ChassisOdometry *o, const ChassisConfig *c,
    const FlowSnapshot *f, const GyroSnapshot *g, uint32_t now);
const char *Chassis_OdometryIssueName(ChassisOdometryIssue issue);
const char *Chassis_FlowFaultName(ChassisOdometryIssue issue, FlowError error);
#endif
