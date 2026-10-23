#ifndef QR_UI_H
#define QR_UI_H
/*
 * 界面作业状态：target 是本次冻结的九行文本，previous 是已完成显示的内容；
 * drawing/row/offset/strip 记录跨主循环的绘图进度，redraw 用于取消旧作业并切换布局。
 */
#include "./qr/qr_service.h"
#include "./motor/motor_bus.h"
#include "./flow/flow_service.h"
#include "./gyro/gyro_service.h"
#include "./chassis/chassis.h"
#include "./mechanism/mechanism.h"
typedef enum { QR_UI_NONE, QR_UI_RESCAN, QR_UI_MODE, QR_UI_RUN,
               QR_UI_STOP_AXIS, QR_UI_STOP_WHEELS, QR_UI_STOP_ALL, QR_UI_FLOW_CLEAR,
               QR_UI_GYRO_CAL, QR_UI_GYRO_ZERO, QR_UI_CHASSIS_START,
               QR_UI_MECH_HOME, QR_UI_MECH_TEST } QrUiAction;
typedef struct {
    char previous[9][43];
    char target[9][43]; /* one coherent refresh snapshot, frozen until painted */
    uint32_t last_draw, max_step_ms;
    uint16_t clear_y;
    uint8_t drawing, row, offset, strip, button, job_flags;
    const Chassis *chassis;
    const Mechanism *mechanism;
    uint8_t mechanism_page, mechanism_detail;
    uint8_t chassis_page, chassis_detail;
    uint8_t flow_page, flow_detail;
    uint8_t gyro_page, gyro_detail;
    uint8_t motor_page, was_down, redraw, selected, detail, test_action;
} QrUi;
void QrUi_Init(QrUi *ui);
QrUiAction QrUi_Touch(QrUi *ui, int32_t x, int32_t y, uint8_t down);
void QrUi_Process(QrUi *ui, const QrService *qr, const MotorBus *motor, const FlowSnapshot *flow,
                  const GyroSnapshot *gyro, uint32_t now_ms);
#endif
