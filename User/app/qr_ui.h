#ifndef QR_UI_H
#define QR_UI_H
#include "./qr/qr_service.h"
#include "./motor/motor_bus.h"
#include "./gyro/gyro_service.h"
#include "./chassis/chassis.h"
#include "./mechanism/mechanism.h"
#include "./vision/vision_service.h"
#include "./mission/mission.h"
#include "./servo/servo_dual_bench.h"

typedef enum { QR_UI_NONE, QR_UI_RESCAN, QR_UI_STOP_ALL,
    QR_UI_GYRO_CAL, QR_UI_GYRO_ZERO, QR_UI_CHASSIS_START,
    QR_UI_MECH_HOME, QR_UI_MECH_TEST, QR_UI_VISION_SCAN, QR_UI_VISION_STOP, QR_UI_MISSION_START,
    QR_UI_SERVO_JAW_ON, QR_UI_SERVO_JAW_LOW, QR_UI_SERVO_JAW_HIGH, QR_UI_SERVO_JAW_OFF,
    QR_UI_SERVO_TABLE_ON, QR_UI_SERVO_TABLE_PREV, QR_UI_SERVO_TABLE_NEXT, QR_UI_SERVO_TABLE_OFF } QrUiAction;
typedef enum { UI_HOME, UI_MOTOR, UI_GYRO, UI_QR, UI_SERVO, UI_VISION,
    UI_CHASSIS, UI_MECHANISM, UI_LIMITS, UI_RACE } QrUiPage;
#define UI_WIDGETS 22u
#define UI_TEXT_BYTES 96u
typedef struct {
    uint16_t x, y, width, height;
    uint8_t button, large;
    char text[UI_TEXT_BYTES];
} UiWidget;
/* Freeze each draw job, then yield after <=480x4 pixels or <=2 glyphs. */
typedef struct {
    UiWidget target[UI_WIDGETS];
    char previous[UI_WIDGETS][UI_TEXT_BYTES];
    const Chassis *chassis;
    const Mechanism *mechanism;
    const VisionService *vision;
    const Mission *mission;
    const ServoDualState *servo;
    uint8_t servo_ready, servo_blocked;
    uint8_t vision_mode, vision_color, vision_zone, vision_position;
    uint32_t last_draw, max_step_ms;
    uint16_t clear_y, strip, text_x, text_y;
    uint8_t page, more, selected, axis, was_down, redraw;
    uint8_t drawing, count, widget, offset;
} QrUi;
void QrUi_Init(QrUi *ui);
/* Input uses native panel coordinates; map once for the upside-down mount. */
QrUiAction QrUi_Touch(QrUi *ui, int32_t x, int32_t y, uint8_t down);
void QrUi_Process(QrUi *ui, const QrService *qr, const MotorBus *motor,
    const GyroSnapshot *gyro, uint32_t now_ms);
#endif
