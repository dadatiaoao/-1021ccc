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
#include "./mechanism/bsp_limit_gpio.h"
#include "./motor/motor_test_config.h"

/* The screen only issues scoped service actions and a global STOP.
   Old raw motor/manual test menus have been removed. */
static void stop_all(QrApp *a, uint32_t now)
{
    Mechanism_Stop(&a->mechanism,now);
    Chassis_Stop(&a->chassis,now);
    MotorBus_Stop(&a->motor,MOTOR_ALL,now);
}

/* Calibration confirmations gate the one-shot power-on mechanism home. */
void QrApp_Init(QrApp *a, MotorTransport motor, uint8_t ready, uint32_t now)
{
    MotorBus_Init(&a->motor, motor, now);
    QrService_Init(&a->qr, ready, now); QrUi_Init(&a->ui);
    Flow_Init(&a->flow, BspFlowSpi_Io(), now);
    Gyro_Init(&a->gyro, BspGyroBus_Io(), now);
    Chassis_Init(&a->chassis,&a->motor,0,now);
    a->ui.chassis=&a->chassis;
    BspLimit_Init();
    Mechanism_Init(&a->mechanism,&a->motor,0,now);
    a->mechanism.chassis=&a->chassis;
    a->chassis.external_busy=&a->mechanism.owned;
    a->ui.mechanism=&a->mechanism;
    if (MECHANISM_LIFT_CALIBRATION_MODE) a->ui.page=UI_LIFT_CAL;
}
/*
 * 给陀螺校准提供拒绝原因：活动、未知、未确认停止或无已知阻碍。
 * 新鲜度使用当前时间；任何轴未确认 STOP 都优先拦截。配置范围内已发现/命令的轴
 * 若反馈不足则视为未知，从未出现的节点可缺省；返回 NONE 本身不能证明实物静止。
 */
static uint8_t motor_active(const QrApp *a, uint32_t now)
{
    unsigned i;
    uint8_t condition = GYRO_MOTOR_NONE;
    if (Mechanism_OwnsAxes(&a->mechanism) || Chassis_OwnsWheels(&a->chassis) || a->motor.queue_count || a->motor.wheels_pending ||
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
    now=AppClock_Now();
    Mechanism_SampleLimits(&a->mechanism,BspLimit_Read(0u),BspLimit_Read(1u),now);
    Mechanism_Process(&a->mechanism,now); /* limits/STOP before selecting next UART5 transaction */
    MotorBus_Process(&a->motor, now);
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
    AppTouch_Process(now); AppTouch_GetState(&x, &y, &down);
    now = AppClock_Now();
    action = QrUi_Touch(&a->ui, x, y, down);
    if (action == QR_UI_RESCAN)
        BspQrUart_BeginSession(&a->qr, now);
    else if (action==QR_UI_CHASSIS_START) (void)Chassis_StartAcceptance(&a->chassis,now);
    else if (action==QR_UI_MECH_HOME) (void)Mechanism_HomeAll(&a->mechanism,now);
    else if (action==QR_UI_MECH_TEST) (void)Mechanism_StartAcceptance(&a->mechanism,now);
    else if (action==QR_UI_LIFT_MEASURE) (void)Mechanism_MeasureLift(&a->mechanism,now);
    else if ((action==QR_UI_GYRO_CAL || action==QR_UI_GYRO_ZERO) &&
        (Chassis_OwnsWheels(&a->chassis) || Mechanism_OwnsAxes(&a->mechanism))) {
        a->motor.message=Mechanism_OwnsAxes(&a->mechanism) ? "MECHANISM BUSY" : "CHASSIS BUSY";
        a->chassis.data.rejection=CHASSIS_AXIS_BUSY;
    }
    else if (action == QR_UI_FLOW_CLEAR) Flow_ResetTotals(&a->flow);
    else if (action == QR_UI_GYRO_CAL) {
        (void)Gyro_RequestCalibration(&a->gyro, now, motor_active(a, now));
    } else if (action == QR_UI_GYRO_ZERO) {
        (void)Gyro_ZeroAngle(&a->gyro, now);
    } else if (action==QR_UI_STOP_ALL) stop_all(a,now);
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
    Chassis_Process(&a->chassis,&gyro,now);
    now = AppClock_Now();
    QrUi_Process(&a->ui, &a->qr, &a->motor, &flow, &gyro, now);
    if (AppClock_Now() - now > a->ui.max_step_ms) a->ui.max_step_ms = AppClock_Now() - now;
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
}
