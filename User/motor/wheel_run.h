#ifndef WHEEL_RUN_H
#define WHEEL_RUN_H
#include "motor_bus.h"
#include "../qr/qr_service.h"
/* Timed wheel trial or forward-until-QR. No homing/gyro/odometry. */
#define WHEEL_RUN_RPM 20
#define WHEEL_RUN_MS 2000u
#define WHEEL_RUN_LEASE_MS 500u
typedef enum { WHEEL_RUN_READY, WHEEL_RUN_ENABLE, WHEEL_RUN_MOVING,
    WHEEL_RUN_STOPPING, WHEEL_RUN_DONE, WHEEL_RUN_FAULT } WheelRunState;
typedef struct {
    MotorBus *bus;
    WheelRunState state;
    const char *note;
    QrService *qr;
    void (*new_scan)(QrService *qr,uint32_t now);
    QrTaskCode task;
    uint32_t phase_ms, publish_ms;
    uint32_t scan_ms, scan_session, scan_seq, scan_errors, scan_overflows;
    uint8_t index, waiting, reverse, fault, until_qr, task_valid;
} WheelRun;
void WheelRun_Init(WheelRun *r, MotorBus *bus);
int WheelRun_Start(WheelRun *r, uint8_t reverse, uint32_t now);
void WheelRun_BindQr(WheelRun *r, QrService *qr, void (*new_scan)(QrService *,uint32_t));
int WheelRun_StartUntilQr(WheelRun *r, uint32_t now);
void WheelRun_Stop(WheelRun *r, uint32_t now);
void WheelRun_Process(WheelRun *r, uint32_t now);
#endif
