/*
 * 应用编排层：串联七电机总线、扫码、触摸、光流、陀螺与统一界面。
 * 本层负责把按键意图分发给服务，并在耗时步骤之间重新读取时钟、服务陀螺；
 * 协议解析、硬件恢复和绘图细节分别由对应模块维护。
 */
#include "qr_app.h"
#include "app_runtime.h"
#include "./flow/bsp_flow_spi.h"
#include "./gyro/bsp_gyro_bus.h"
#include "./usart/bsp_qr_uart.h"
#include "./usart/bsp_qr_trigger.h"
#include "./usart/bsp_motor_uart.h"
#include "./usart/bsp_vision_uart.h"
#include "./motor/motor_test_config.h"
#include <string.h>

/* Explicit manual requests share the existing bus. STOP cancels chassis intent
   first; diagnostic pages never acquire wheel control. */
static void manual_action(QrApp *a, QrUiAction action, uint32_t now)
{
    uint8_t address = (uint8_t)(a->ui.selected + 1u);
    int owns=Chassis_OwnsWheels(&a->chassis);
    if (action==QR_UI_STOP_ALL || action==QR_UI_STOP_WHEELS ||
        (action==QR_UI_STOP_AXIS && address<=4u)) Chassis_Stop(&a->chassis,now);
    else if (owns && (action==QR_UI_MODE || (action==QR_UI_RUN &&
        (address<=4u || a->ui.test_action>=3u)))) {
        a->motor.message="CHASSIS BUSY"; return;
    }
    if (action == QR_UI_MODE) MotorBus_SetManual(&a->motor, (uint8_t)!a->motor.manual, now);
    else if (action == QR_UI_STOP_ALL) MotorBus_Stop(&a->motor, MOTOR_ALL, now);
    else if (action == QR_UI_STOP_WHEELS) MotorBus_Stop(&a->motor, MOTOR_WHEELS, now);
    else if (action == QR_UI_STOP_AXIS) MotorBus_Stop(&a->motor, (uint8_t)(1u << a->ui.selected), now);
    else if (action == QR_UI_RUN && a->motor.manual) {
        if (a->ui.test_action == 0u) { (void)MotorBus_Enable(&a->motor, address, 1u, now); return; }
#if MOTOR_TEST_CONFIGURED || MOTOR_BENCH_JOG_ENABLED
        {
            MotorAction m;
            memset(&m, 0, sizeof(m));
            m.lease_ms = MOTOR_TEST_LEASE_MS; m.timeout_ms = MOTOR_TEST_POSITION_TIMEOUT_MS;
            m.pulses_per_rev = MOTOR_TEST_PULSES_PER_REV;
#if MOTOR_TEST_CONFIGURED
            if (a->ui.test_action == 1u) {
                m.command = Emm_VelocityEx(address, MOTOR_TEST_DIRECTION, MOTOR_TEST_RPM, MOTOR_TEST_ACCELERATION, 0u);
                (void)MotorBus_Submit(&a->motor, &m, now);
            } else if (a->ui.test_action == 2u) {
                if (address < 5u) { a->motor.message = "POSITION: SELECT AXIS 5..7"; return; }
                m.command = Emm_MoveEx(address, MOTOR_TEST_DIRECTION, MOTOR_TEST_RPM, MOTOR_TEST_ACCELERATION,
                    MOTOR_TEST_PULSES, MOTOR_TEST_POSITION_MODE, 0u);
                (void)MotorBus_Submit(&a->motor, &m, now);
            } else {
                const int32_t rpm[4] = MOTOR_TEST_WHEEL_RPM;
                if (a->ui.test_action == 3u) (void)MotorBus_Wheels(&a->motor, rpm, MOTOR_TEST_ACCELERATION, MOTOR_TEST_LEASE_MS, now);
                else {
                    MotorAction group[4];
                    unsigned i;
                    for (i = 0u; i < 4u; ++i) {
                        group[i] = m;
                        group[i].command = Emm_VelocityEx((uint8_t)(i + 1u), (uint8_t)(rpm[i] < 0),
                            (uint16_t)(rpm[i] < 0 ? -rpm[i] : rpm[i]), MOTOR_TEST_ACCELERATION, 1u);
                    }
                    (void)MotorBus_Sync(&a->motor, group, 4u, now);
                }
            }
#else
            /* Bench jog: an explicit MANUAL + RUN ONCE with a short low-speed
               lease remains available while MOTOR_TEST_CONFIGURED=0. Position,
               0xAA wheel group and sync stay gated by MOTOR_TEST_CONFIGURED. */
            if (a->ui.test_action == 1u) {
                m.command = Emm_VelocityEx(address, MOTOR_TEST_DIRECTION, MOTOR_TEST_RPM,
                    MOTOR_TEST_ACCELERATION, 0u);
                (void)MotorBus_Submit(&a->motor, &m, now);
            } else {
                a->motor.message = "CONFIGURE: POSITION/GROUP/SYNC";
            }
#endif
        }
#else
        a->motor.message = "CONFIGURE motor_test_config.h";
#endif
    }
}

/* Initialization starts diagnostics only; motion always requires explicit intent. */
void QrApp_Init(QrApp *a, MotorTransport motor, uint8_t ready, uint32_t now)
{
    MotorBus_Init(&a->motor, motor, now);
    QrService_Init(&a->qr, ready, now); QrUi_Init(&a->ui);
    Flow_Init(&a->flow, BspFlowSpi_Io(), now);
    Gyro_Init(&a->gyro, BspGyroBus_Io(), now);
    Chassis_Init(&a->chassis,&a->motor,0,now);
    a->ui.chassis=&a->chassis;
    VisionService_Init(&a->vision,BspVisionUart_Init(),now);
    a->ui.vision=&a->vision;
    MissionRuntime_Init(&a->mission_runtime,&a->chassis,&a->motor,&a->vision,0);
    Mission_Init(&a->mission,MissionRuntime_Port(&a->mission_runtime));
    a->ui.mission=&a->mission;
    Mission_Process(&a->mission,now);
}
int QrApp_StartMission(QrApp *a, uint32_t now)
{ return Mission_Start(&a->mission,now); }
/*
 * 给陀螺校准提供拒绝原因：活动、未知、未确认停止或无已知阻碍。
 * 新鲜度使用当前时间；任何轴未确认 STOP 都优先拦截。配置范围内已发现/命令的轴
 * 若反馈不足则视为未知，从未出现的节点可缺省；返回 NONE 本身不能证明实物静止。
 */
static uint8_t motor_active(const QrApp *a, uint32_t now)
{
    unsigned i;
    uint8_t condition = GYRO_MOTOR_NONE;
    if (Chassis_OwnsWheels(&a->chassis) || a->motor.queue_count || a->motor.wheels_pending ||
        a->motor.sync_state==MOTOR_SYNC_PREPARE || a->motor.sync_state==MOTOR_SYNC_TRIGGER ||
        a->motor.sync_state==MOTOR_SYNC_VERIFY) condition = GYRO_MOTOR_ACTIVE;
    for (i=0u;i<MOTOR_COUNT;++i) {
        const MotorNode *n=&a->motor.nodes[i];
        if (n->stop_unconfirmed || n->action==MOTOR_STOP_PENDING) return GYRO_MOTOR_STOP;
        if (!(MOTOR_CAL_AXIS_MASK & (1u << i))) continue;
        if (n->motion || n->action==MOTOR_QUEUED || n->action==MOTOR_SENT ||
            n->action==MOTOR_ACCEPTED || n->action==MOTOR_EXECUTING ||
            (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) && n->data.speed_rpm!=0)) {
            if (condition != GYRO_MOTOR_UNKNOWN) condition = GYRO_MOTOR_ACTIVE;
        } else if (n->action==MOTOR_UNKNOWN ||
            ((n->data.rx_count || n->data.valid || n->request_id || n->stop_sent) &&
             (!n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
              !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || (n->data.status_flags & 0x0Cu))))
            condition = GYRO_MOTOR_UNKNOWN;
    }
    return condition;
}
/*
 * 主循环的一轮：接收电机字节并推进事务/底盘，处理触摸动作，
 * 更新扫码触发与解码、光流和陀螺，最后提交快照并推进一个绘图片段。
 * 在触摸、传感器、绘图前后刷新 AppClock，避免沿用已过时的时间；
 * 多次调用 Gyro_Process 由其内部轮询间隔限频，不会把同一样本重复积分。
 */
void QrApp_Process(QrApp *a, uint32_t now)
{
    int32_t x, y;
    FlowSnapshot flow;
    GyroSnapshot gyro;
    uint8_t down;
    QrUiAction action;
    BspMotorUart_PumpBus(&a->motor, now);
    MotorBus_Process(&a->motor, now);
    BspVisionUart_Pump(&a->vision,now);
    VisionService_Process(&a->vision,AppClock_Now());
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
    AppTouch_Process(now); AppTouch_GetState(&x, &y, &down);
    now = AppClock_Now();
    action = QrUi_Touch(&a->ui, x, y, down);
    /* A running mission owns all action intent, even when diagnostics are shown.
       Every STOP cancels the mission first; other commands cannot steal it. */
    if (action==QR_UI_STOP_ALL || action==QR_UI_STOP_WHEELS || action==QR_UI_STOP_AXIS ||
        (action==QR_UI_VISION_STOP && Mission_IsRunning(&a->mission))) Mission_Stop(&a->mission,now);
    if (Mission_IsRunning(&a->mission) && action!=QR_UI_NONE && action!=QR_UI_STOP_ALL &&
        action!=QR_UI_STOP_WHEELS && action!=QR_UI_STOP_AXIS) {
        a->motor.message="RACE BUSY - STOP FIRST"; action=QR_UI_NONE;
    }
    if (action==QR_UI_MISSION_START) (void)QrApp_StartMission(a,now);
    else if (action==QR_UI_VISION_SCAN) {
        (void)VisionService_Start(&a->vision,a->ui.vision_mode,a->ui.vision_color,
            a->ui.vision_zone,a->ui.vision_position,(uint16_t)(a->vision.pose_id+1u),now);
    } else if (action==QR_UI_VISION_STOP) VisionService_Stop(&a->vision,now);
    else if (action == QR_UI_RESCAN)
        BspQrUart_BeginSession(&a->qr, now);
    else if (action==QR_UI_CHASSIS_START) (void)Chassis_StartAcceptance(&a->chassis,now);
    else if ((action==QR_UI_FLOW_CLEAR || action==QR_UI_GYRO_CAL || action==QR_UI_GYRO_ZERO) &&
        Chassis_OwnsWheels(&a->chassis)) {
        a->motor.message="CHASSIS BUSY"; a->chassis.data.rejection=CHASSIS_AXIS_BUSY;
    }
    else if (action == QR_UI_FLOW_CLEAR) Flow_ResetTotals(&a->flow);
    else if (action == QR_UI_GYRO_CAL) {
        (void)Gyro_RequestCalibration(&a->gyro, now, motor_active(a, now));
    } else if (action == QR_UI_GYRO_ZERO) {
        (void)Gyro_ZeroAngle(&a->gyro, now);
    } else if (action==QR_UI_MODE || action==QR_UI_RUN || action==QR_UI_STOP_AXIS ||
        action==QR_UI_STOP_WHEELS || action==QR_UI_STOP_ALL) {
        manual_action(a, action, now);
        if (action==QR_UI_STOP_ALL) VisionService_Stop(&a->vision,now);
    }
    BspQrTrigger_Poll(now);
    BspQrUart_Pump(&a->qr, now);
    now = AppClock_Now(); /* Capture IRQs/touch I2C can advance SysTick during a step. */
    QrService_Process(&a->qr, now);
    Flow_Process(&a->flow, now);
    now = AppClock_Now();
    Gyro_Process(&a->gyro, now, motor_active(a, now));
    now = AppClock_Now();
    Flow_GetSnapshot(&a->flow, now, &flow);
    Gyro_GetSnapshot(&a->gyro, now, &gyro);
    Chassis_Process(&a->chassis,&flow,&gyro,now);
    Mission_Process(&a->mission,AppClock_Now());
    now = AppClock_Now();
    QrUi_Process(&a->ui, &a->qr, &a->motor, &flow, &gyro, now);
    if (AppClock_Now() - now > a->ui.max_step_ms) a->ui.max_step_ms = AppClock_Now() - now;
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
}
