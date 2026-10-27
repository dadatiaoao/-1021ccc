/*
 * 应用编排层：串联七电机总线、扫码、触摸、陀螺与统一界面。
 * 本层负责把按键意图分发给服务，并在耗时步骤之间重新读取时钟、服务陀螺；
 * 协议解析、硬件恢复和绘图细节分别由对应模块维护。
 */
#include "qr_app.h"
#include "app_runtime.h"
#include "./gyro/bsp_gyro_bus.h"
#include "./usart/bsp_qr_uart.h"
#include "./usart/bsp_qr_trigger.h"
#include "./usart/bsp_motor_uart.h"
#include "./usart/bsp_vision_uart.h"
#include "./mechanism/bsp_limit_gpio.h"
#include "./motor/motor_test_config.h"
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
#include "./servo/servo_pwm.h"
#endif

/* The screen only issues scoped service actions and a global STOP.
   Old raw motor/manual test menus have been removed. */
static void stop_all(QrApp *a, uint32_t now)
{
    VisionService_Stop(&a->vision,now);
    Mechanism_Stop(&a->mechanism,now);
    Chassis_Stop(&a->chassis,now);
    MotorBus_Stop(&a->motor,MOTOR_ALL,now);
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    if (a->ui.servo_ready) Servo_OffAll();
#endif
}

/* Calibration confirmations gate the one-shot power-on mechanism home. */
void QrApp_Init(QrApp *a, MotorTransport motor, uint8_t ready, uint32_t now)
{
    MotorBus_Init(&a->motor, motor, now);
    QrService_Init(&a->qr, ready, now); QrUi_Init(&a->ui);
    Gyro_Init(&a->gyro, BspGyroBus_Io(), now);
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    a->ui.servo=&a->servo;
    a->ui.servo_ready=(uint8_t)ServoDualBench_Init(&a->servo);
    /* Power-on position commands: jaw 1700 us, turntable 1240 us.
       These servos have no position feedback; this does not confirm arrival. */
    if (a->ui.servo_ready &&
        (Servo_Enable(0u)!=SERVO_OK || Servo_Enable(1u)!=SERVO_OK)) {
        Servo_OffAll();
        a->ui.servo_ready=0u;
    }
#endif
    Chassis_Init(&a->chassis,&a->motor,0,now);
    a->ui.chassis=&a->chassis;
    VisionService_Init(&a->vision,BspVisionUart_Init(),now);
    a->ui.vision=&a->vision;
    MissionRuntime_Init(&a->mission_runtime,&a->chassis,&a->motor,&a->vision,0);
    Mission_Init(&a->mission,MissionRuntime_Port(&a->mission_runtime));
    a->ui.mission=&a->mission;
    BspLimit_Init();
    Mechanism_Init(&a->mechanism,&a->motor,0,now);
    a->mechanism.chassis=&a->chassis;
    a->chassis.external_busy=&a->mechanism.owned;
    a->ui.mechanism=&a->mechanism;
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
 * 更新扫码触发与解码、舵机和陀螺，最后提交快照并推进一个绘图片段。
 * 在触摸、传感器、绘图前后刷新 AppClock，避免沿用已过时的时间；
 * 多次调用 Gyro_Process 由其内部轮询间隔限频，不会把同一样本重复积分。
 */
void QrApp_Process(QrApp *a, uint32_t now)
{
    int32_t x, y;
    GyroSnapshot gyro;
    uint8_t down;
    QrUiAction action;
    BspMotorUart_PumpBus(&a->motor, now);
    now=AppClock_Now();
    Mechanism_SampleLimits(&a->mechanism,BspLimit_Read(0u),BspLimit_Read(1u),now);
    Mechanism_Process(&a->mechanism,now); /* limits/STOP before selecting next UART5 transaction */
    MotorBus_Process(&a->motor, now);
    BspVisionUart_Pump(&a->vision,now);
    VisionService_Process(&a->vision,AppClock_Now());
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
    AppTouch_Process(now); AppTouch_GetState(&x, &y, &down);
    now = AppClock_Now();
    action = QrUi_Touch(&a->ui, x, y, down);
    /* A running mission owns all action intent, even when diagnostics are shown.
       Every STOP cancels the mission first; other commands cannot steal it. */
    if (action==QR_UI_STOP_ALL ||
        (action==QR_UI_VISION_STOP && Mission_IsRunning(&a->mission))) Mission_Stop(&a->mission,now);
    if (Mission_IsRunning(&a->mission) && action!=QR_UI_NONE && action!=QR_UI_STOP_ALL) {
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
    else if (action==QR_UI_MECH_HOME) (void)Mechanism_HomeAll(&a->mechanism,now);
    else if (action==QR_UI_MECH_TEST) (void)Mechanism_StartAcceptance(&a->mechanism,now);
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    else if (action>=QR_UI_SERVO_JAW_ON && action<=QR_UI_SERVO_TABLE_OFF) {
        ServoDualAction servo_action=(ServoDualAction)(action-QR_UI_SERVO_JAW_ON+DUAL_JAW_ON);
        uint8_t is_off=(uint8_t)(servo_action==DUAL_JAW_OFF || servo_action==DUAL_TABLE_OFF);
        a->ui.servo_blocked=0u;
        if (!a->ui.servo_ready) a->servo.last_status=SERVO_REJECT_STATE;
        else if (!is_off && (a->mechanism.auto_pending || Mechanism_OwnsAxes(&a->mechanism) ||
                 Chassis_OwnsWheels(&a->chassis) || a->motor.manual || a->motor.wheels_pending))
            a->ui.servo_blocked=1u;
        else ServoDualBench_Command(&a->servo,servo_action);
    }
#endif
    else if ((action==QR_UI_GYRO_CAL || action==QR_UI_GYRO_ZERO) &&
        (Chassis_OwnsWheels(&a->chassis) || Mechanism_OwnsAxes(&a->mechanism))) {
        a->motor.message=Mechanism_OwnsAxes(&a->mechanism) ? "MECHANISM BUSY" : "CHASSIS BUSY";
        a->chassis.data.rejection=CHASSIS_AXIS_BUSY;
    }
    else if (action == QR_UI_GYRO_CAL) {
        (void)Gyro_RequestCalibration(&a->gyro, now, motor_active(a, now));
    } else if (action == QR_UI_GYRO_ZERO) {
        (void)Gyro_ZeroAngle(&a->gyro, now);
    } else if (action==QR_UI_STOP_ALL) stop_all(a,now);
    BspQrTrigger_Poll(now);
    BspQrUart_Pump(&a->qr, now);
    now = AppClock_Now(); /* Capture IRQs/touch I2C can advance SysTick during a step. */
    QrService_Process(&a->qr, now);
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    if (a->ui.servo_ready) Servo_Process(now);
#endif
    now = AppClock_Now();
    Gyro_Process(&a->gyro, now, motor_active(a, now));
    now = AppClock_Now();
    Gyro_GetSnapshot(&a->gyro, now, &gyro);
    Chassis_Process(&a->chassis,&gyro,now);
    Mission_Process(&a->mission,AppClock_Now());
    now = AppClock_Now();
    QrUi_Process(&a->ui, &a->qr, &a->motor, &gyro, now);
    if (AppClock_Now() - now > a->ui.max_step_ms) a->ui.max_step_ms = AppClock_Now() - now;
    now = AppClock_Now(); Gyro_Process(&a->gyro, now, motor_active(a, now));
}
