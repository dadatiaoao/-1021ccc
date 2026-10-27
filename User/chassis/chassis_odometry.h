#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H
#include "chassis_config.h"
#include "./motor/motor_bus.h"
#include "./gyro/gyro_service.h"

/* Staggered position replies are interpolated to a common timestamp.
   Gyro history covers the unchanged 1600ms motor freshness limit at 10ms. */
#define CHASSIS_WHEEL_HISTORY 4u
#define CHASSIS_GYRO_HISTORY 176u
typedef struct { double counts; uint32_t sample_ms, sequence; } ChassisWheelPoint;
typedef struct { double angle; uint32_t sample_ms; } ChassisGyroPoint;
typedef enum {
    CHASSIS_ODOM_OK, CHASSIS_ODOM_CONFIG, CHASSIS_ODOM_GYRO_INVALID,
    CHASSIS_ODOM_GYRO_CHANGED, CHASSIS_ODOM_WHEEL_STALE,
    CHASSIS_ODOM_WHEEL_RESET, CHASSIS_ODOM_WHEEL_JUMP,
    CHASSIS_ODOM_SYNC_TIMEOUT, CHASSIS_ODOM_NUMERIC
} ChassisOdometryIssue;
typedef struct {
    ChassisWheelPoint wheel[4][CHASSIS_WHEEL_HISTORY];
    ChassisGyroPoint gyro[CHASSIS_GYRO_HISTORY];
    GyroSnapshot gyro_guard;
    double previous_counts[4], x, y, angle;
    uint32_t generation, sequence, sample_ms;
    ChassisOdometryIssue last_issue;
    uint32_t break_ms, break_age_ms;
    uint16_t gyro_head, gyro_count;
    uint8_t wheel_count[4], baseline, valid, break_wheel;
} ChassisOdometry;
int Chassis_GyroValid(const GyroSnapshot *g, uint32_t now);
int Chassis_GyroContinuous(const GyroSnapshot *a, const GyroSnapshot *b);
void Chassis_OdometryProcess(ChassisOdometry *o, const ChassisConfig *c,
    const MotorBus *bus, const GyroSnapshot *g, uint32_t now);
const char *Chassis_OdometryIssueName(ChassisOdometryIssue issue);
#endif
