#ifndef GYRO_SERVICE_H
#define GYRO_SERVICE_H
/*
 * 陀螺对外数据契约：raw_dps 为安装轴原始角速度，bias_dps 为校准偏置，
 * rate_dps 为扣偏置角速度，angle_deg 为相对积分角；分别检查有效位及 age_ms。
 * GYRO_MOTOR_* 参数表示校准拒绝原因枚举，不能当成实物静止证明。
 */
#include "icm42688.h"
typedef enum { GYRO_INIT, GYRO_RUNNING, GYRO_ERROR } GyroState;
typedef enum { GYRO_OK, GYRO_BUS, GYRO_ID, GYRO_RESET, GYRO_CONFIG,
    GYRO_BAD_SAMPLE, GYRO_SATURATED, GYRO_GAP, GYRO_STALE } GyroError;
typedef enum { GYRO_CAL_NONE, GYRO_CAL_RUNNING, GYRO_CAL_DONE,
    GYRO_CAL_MOVING, GYRO_CAL_TIMEOUT, GYRO_CAL_ERROR, GYRO_CAL_MOTOR,
    GYRO_CAL_GAP, GYRO_CAL_NOT_READY, GYRO_CAL_UNKNOWN, GYRO_CAL_STOP } GyroCalibration;
enum { GYRO_MOTOR_NONE, GYRO_MOTOR_ACTIVE, GYRO_MOTOR_UNKNOWN, GYRO_MOTOR_STOP };
typedef struct {
    GyroState state; GyroError error, last_fault; GyroBusError bus_error, last_bus_error;
    GyroCalibration calibration;
    uint32_t probe_total, probe_ok, probe_nack, probe_timeout, probe_busy, probe_bad;
    int16_t raw[3];
    float raw_dps, bias_dps, rate_dps;
    double angle_deg;
    uint32_t sequence, sample_ms, interval_ms, max_interval_ms, age_ms;
    uint32_t max_poll_ms, max_process_us, missed, gaps, errors, saturations, invalid;
    uint32_t restarts, zeros, cal_requests, cal_count, cal_elapsed_ms;
    uint8_t who, axis, calibrated, raw_valid, rate_valid, angle_valid, angle_gap, have_sample;
    int8_t sign;
} GyroSnapshot;
typedef struct {
    GyroIo io; Icm42688 chip; GyroSnapshot data;
    uint32_t due, retry_ms, last_poll, last_check, ready_ms, cal_start, cal_first;
    float mean[3], m2[3], minimum[3], maximum[3], previous_rate;
    uint8_t have_previous;
} GyroService;
void Gyro_Init(GyroService *s, GyroIo io, uint32_t now);
void Gyro_Process(GyroService *s, uint32_t now, uint8_t motor_active);
void Gyro_GetSnapshot(const GyroService *s, uint32_t now, GyroSnapshot *out);
/* GYRO_MOTOR_* is auxiliary rejection evidence, never proof of rest. */
int Gyro_RequestCalibration(GyroService *s, uint32_t now, uint8_t motor_active);
int Gyro_ZeroAngle(GyroService *s, uint32_t now);
const char *Gyro_StateName(const GyroSnapshot *d);
const char *Gyro_ErrorName(GyroError e);
const char *Gyro_CalName(GyroCalibration c);
#endif
