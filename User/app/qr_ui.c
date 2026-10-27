/* Dashboard and module pages. Navigation never owns motion. */
#include "qr_ui.h"
#include "./lcd/bsp_lcd.h"
#include "./lcd/lcd_orientation.h"
#include "./fonts/ui_glyphs.h"
#include "./gyro/gyro_ui.h"
#include "./vision/vision_ui.h"
#include "./mission/mission_ui.h"
#include "./servo/servo_pwm.h"
#include <stdio.h>
#include <string.h>

enum { DRAW_IDLE, DRAW_CLEAR, DRAW_WIDGET, DRAW_FILL, DRAW_TEXT };
static const uint16_t nav_x[5]={8u,104u,200u,296u,384u};
static const uint16_t nav_w[5]={88u,88u,88u,80u,88u};
static const char * const titles[]={/* 运行总览 */ "\350\277\220\350\241\214\346\200\273\350\247\210",/* 电机状态 */ "\347\224\265\346\234\272\347\212\266\346\200\201",/* 陀螺仪 */ "\351\231\200\350\236\272\344\273\252",/* 扫码任务 */ "\346\211\253\347\240\201\344\273\273\345\212\241",/* 舵机 */ "\350\210\265\346\234\272",
    /* 视觉通信 */ "\350\247\206\350\247\211\351\200\232\344\277\241",/* 底盘位置解算 */ "\345\272\225\347\233\230\344\275\215\347\275\256\350\247\243\347\256\227",/* 机械臂三轴 */ "\346\234\272\346\242\260\350\207\202\344\270\211\350\275\264",/* 限位开关 */ "\351\231\220\344\275\215\345\274\200\345\205\263","RACE / ZONE 2"};

static UiWidget *item(QrUi *u,unsigned x,unsigned y,unsigned w,unsigned h,
    unsigned button,const char *text)
{
    UiWidget *v=&u->target[u->count++];
    v->x=(uint16_t)x; v->y=(uint16_t)y; v->width=(uint16_t)w; v->height=(uint16_t)h;
    v->button=(uint8_t)button; v->large=0u;
    strncpy(v->text,text,UI_TEXT_BYTES-1u); v->text[UI_TEXT_BYTES-1u]='\0';
    return v;
}
static void row(QrUi *u,unsigned index,const char *text)
{ (void)item(u,8u,44u+index*24u,464u,22u,0u,text); }
static void open_page(QrUi *u,uint8_t page)
{ u->page=page; u->more=0u; u->redraw=1u; }
void QrUi_Init(QrUi *u)
{ memset(u,0,sizeof(*u)); u->redraw=1u; u->vision_color=u->vision_zone=u->vision_position=1u; }
static int hit(int32_t x,int32_t y,int32_t bx,int32_t by,int32_t w,int32_t h)
{ return x>=bx && x<bx+w && y>=by && y<by+h; }

QrUiAction QrUi_Touch(QrUi *u,int32_t x,int32_t y,uint8_t down)
{
    unsigned i;
    uint8_t edge=(uint8_t)(down && !u->was_down);
    u->was_down=down;
    if (!edge || x<0 || y<0 || x>=480 || y>=272) return QR_UI_NONE;
    x=LCD_TOUCH_X(x); y=LCD_TOUCH_Y(y);
    for (i=0u;i<5u;++i) if (hit(x,y,nav_x[i],232,nav_w[i],32)) {
        static const uint8_t pages[]={UI_HOME,UI_CHASSIS,UI_MECHANISM,UI_LIMITS};
        if (i==4u) return QR_UI_STOP_ALL;
        open_page(u,pages[i]); return QR_UI_NONE;
    }
    if (hit(x,y,400,4,72,32)) { u->more=(uint8_t)!u->more; u->redraw=1u; return QR_UI_NONE; }
    if (u->page==UI_HOME && u->more && hit(x,y,8,196,216,30)) { open_page(u,UI_RACE); return QR_UI_NONE; }
    if (u->page==UI_RACE && hit(x,y,8,196,216,30)) return QR_UI_MISSION_START;
    if (u->page==UI_SERVO) {
        for (i=0u;i<8u;++i) {
            int bx=4+(int)(i%4u)*119, by=112+(int)(i/4u)*48;
            if (hit(x,y,bx,by,114,36)) return (QrUiAction)(QR_UI_SERVO_JAW_ON+i);
        }
        return QR_UI_NONE;
    }
    if (u->page==UI_VISION) {
        if (hit(x,y,8,196,140,30)) return QR_UI_VISION_SCAN;
        if (hit(x,y,160,196,140,30)) return QR_UI_VISION_STOP;
        if (hit(x,y,8,68,464,22)) u->vision_mode=(uint8_t)((u->vision_mode+1u)%5u);
        if (hit(x,y,8,92,464,22)) {
            if (x<160) u->vision_color=(uint8_t)(u->vision_color%6u+1u);
            else if (x<312) u->vision_zone=(uint8_t)(u->vision_zone%2u+1u);
            else u->vision_position=(uint8_t)(u->vision_position%3u+1u);
        }
        return QR_UI_NONE;
    }
    if (u->page==UI_HOME && !u->more) {
        static const uint8_t pages[]={UI_MOTOR,UI_GYRO,UI_QR,UI_SERVO,UI_VISION,UI_MECHANISM};
        for (i=0u;i<6u;++i) if (hit(x,y,8+(int)(i%3u)*158,104+(int)(i/3u)*62,148,52)) {
            open_page(u,pages[i]); return QR_UI_NONE;
        }
    } else if (u->page==UI_MOTOR) {
        if (!u->more) {
            for (i=0u;i<7u;++i) if (hit(x,y,8,44+(int)i*24,464,22)) {
                u->selected=(uint8_t)i; u->more=1u; u->redraw=1u; return QR_UI_NONE;
            }
        } else if (hit(x,y,8,196,140,30) || hit(x,y,160,196,140,30)) {
            u->selected=(uint8_t)((u->selected+(x<152 ? 6u : 1u))%7u); u->redraw=1u;
        }
    } else if (u->page==UI_MECHANISM && u->more) {
        if (hit(x,y,8,44,464,22)) { u->axis=(uint8_t)((u->axis+1u)%3u); u->redraw=1u; }
        else if (hit(x,y,8,196,140,30)) return QR_UI_MECH_HOME;
        else if (hit(x,y,160,196,140,30)) return QR_UI_MECH_TEST;
    } else if (u->page==UI_CHASSIS && u->more && hit(x,y,8,196,216,30)) return QR_UI_CHASSIS_START;
    else if (u->page==UI_GYRO && !u->more && hit(x,y,8,196,140,30)) return QR_UI_GYRO_CAL;
    else if (u->page==UI_GYRO && !u->more && hit(x,y,160,196,140,30)) return QR_UI_GYRO_ZERO;
    else if (u->page==UI_QR && !u->more && hit(x,y,8,196,216,30)) return QR_UI_RESCAN;
    return QR_UI_NONE;
}

static void decimal(char out[20],double value)
{
    long v; unsigned long a;
    if (!(value>=-999999.0 && value<=999999.0)) { strcpy(out,"RANGE"); return; }
    v=(long)(value*100.0+(value<0.0 ? -0.5 : 0.5)); a=(unsigned long)(v<0 ? -v : v);
    snprintf(out,20,"%s%lu.%02lu",v<0 ? "-" : "",a/100u,a%100u);
}
static const char *node_state(const MotorNode *n,uint32_t now)
{
    if (n->stop_unconfirmed) return "STOP?";
    if (!n->data.online) return n->data.rx_count ? "OFFLINE" : "WAIT";
    if (!MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) || !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now)) return "STALE";
    if (n->data.status_flags & 12u) return "FAULT";
    if (n->action==MOTOR_UNKNOWN) return "UNKNOWN";
    return n->data.speed_rpm ? "MOVING" : "ONLINE";
}
static const char *phase(const QrUi *u,const MotorBus *b)
{
    const Mechanism *m=u->mechanism; const Chassis *c=u->chassis; unsigned i;
    if ((m && m->data.stop_pending) || (c && c->data.stop_pending)) return /* 停车确认中 */ "\345\201\234\350\275\246\347\241\256\350\256\244\344\270\255";
    for (i=0u;i<MOTOR_COUNT;++i) if (b->nodes[i].stop_unconfirmed) return /* 停车未确认 */ "\345\201\234\350\275\246\346\234\252\347\241\256\350\256\244";
    if ((m && m->data.state==MECH_FAULT) || (c && c->data.state==CHASSIS_FAULT)) return /* 故障待处理 */ "\346\225\205\351\232\234\345\276\205\345\244\204\347\220\206";
    if (m && Mechanism_OwnsAxes(m)) return m->data.homing ? /* 机构回零中 */ "\346\234\272\346\236\204\345\233\236\351\233\266\344\270\255" : /* 机构动作中 */ "\346\234\272\346\236\204\345\212\250\344\275\234\344\270\255";
    if (c && Chassis_OwnsWheels(c)) return c->data.sequence_active ? /* 底盘验收中 */ "\345\272\225\347\233\230\351\252\214\346\224\266\344\270\255" : /* 底盘运动中 */ "\345\272\225\347\233\230\350\277\220\345\212\250\344\270\255";
    if (m && m->auto_pending) return /* 上电准备中 */ "\344\270\212\347\224\265\345\207\206\345\244\207\344\270\255";
    if ((m && m->data.result==MECH_CANCELLED) || (c && c->data.result==CHASSIS_CANCELLED)) return /* 已停止 */ "\345\267\262\345\201\234\346\255\242";
    if (m && (!m->data.reference[0] || !m->data.reference[1] || !m->data.trim_ready)) return /* 等待机构就绪 */ "\347\255\211\345\276\205\346\234\272\346\236\204\345\260\261\347\273\252";
    return /* 待机 */ "\345\276\205\346\234\272";
}
static void home(QrUi *u,const QrService *q,const MotorBus *b,const GyroSnapshot *g,uint32_t now)
{
    char t[UI_TEXT_BYTES],status[6][40]; unsigned i,online=0u,stops=0u;
    for (i=0u;i<MOTOR_COUNT;++i) {
        const char *s=node_state(&b->nodes[i],now);
        if (!strcmp(s,"ONLINE") || !strcmp(s,"MOVING")) ++online;
        if (b->nodes[i].stop_unconfirmed) ++stops;
    }
    {
        const char *p=phase(u,b);
        char step[24]="";
        if (u->mechanism && Mechanism_OwnsAxes(u->mechanism) &&
            !u->mechanism->data.stop_pending && !stops && u->mechanism->data.state!=MECH_FAULT)
            snprintf(step,sizeof(step)," [A%u]",u->mechanism->data.axis+5u);
        else if (u->chassis && u->chassis->data.sequence_active &&
            !u->chassis->data.stop_pending && !stops && u->chassis->data.state!=CHASSIS_FAULT)
            snprintf(step,sizeof(step)," [%u/8]",u->chassis->data.stage);
        snprintf(t,sizeof(t),/* 阶段: %s%s%s */ "\351\230\266\346\256\265: %s%s%s",p,step,MECHANISM_TRIAL_RUN ? " / TRIAL" : ""); row(u,0u,t);
    }
    if (u->more) {
        row(u,1u,"RACE / ZONE 2: see START gate"); row(u,2u,"K230 UART1: linked protocol");
        snprintf(t,sizeof(t),"UART5 TX:%lu RX:%lu",(unsigned long)b->tx_frames,(unsigned long)b->rx_frames); row(u,3u,t);
        snprintf(t,sizeof(t),"STOP?:%u UI MAX:%lu ms",stops,(unsigned long)u->max_step_ms); row(u,4u,t);
        row(u,5u,/* 位姿为估算值，回零参数仍需标定 */ "\344\275\215\345\247\277\344\270\272\344\274\260\347\256\227\345\200\274\357\274\214\345\233\236\351\233\266\345\217\202\346\225\260\344\273\215\351\234\200\346\240\207\345\256\232");
        (void)item(u,8,196,216,30,1,"RACE / ZONE 2"); return;
    }
    snprintf(t,sizeof(t),/* 任务: %s */ "\344\273\273\345\212\241: %s",q->data.valid ? q->data.current.text : /* 等待扫码 */ "\347\255\211\345\276\205\346\211\253\347\240\201"); row(u,1u,t);
    snprintf(status[0],40,"%u/7 %s",online,stops ? "STOP?" : "ONLINE");
    snprintf(status[1],40,"%.12s",g ? Gyro_StateName(g) : "--");
    snprintf(status[2],40,"%s",q->data.valid ? "CODE LOCKED" : q->data.uart_ready ? "WAIT CODE" : "UART ERROR");
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    if (u->servo_ready) snprintf(status[3],40,"J:%s T:%s",
        Servo_IsEnabled(0u) ? "ON" : "OFF",Servo_IsEnabled(1u) ? "ON" : "OFF");
    else strcpy(status[3],"NOT READY");
#else
    snprintf(status[3],40,"NOT READY");
#endif
    snprintf(status[4],40,"%.12s",u->vision ? VisionService_StateName(u->vision) : "--");
    snprintf(status[5],40,"%.12s",u->mechanism ? Mechanism_StateName(u->mechanism->data.state) : "--");
    for (i=0u;i<6u;++i) {
        static const char * const names[]={/* 电机 */ "\347\224\265\346\234\272",/* 陀螺仪 */ "\351\231\200\350\236\272\344\273\252",/* 扫码 */ "\346\211\253\347\240\201",/* 舵机 */ "\350\210\265\346\234\272",/* 视觉 */ "\350\247\206\350\247\211",/* 机械臂 */ "\346\234\272\346\242\260\350\207\202"};
        snprintf(t,sizeof(t),"%s\n%s",names[i],status[i]);
        (void)item(u,8u+(i%3u)*158u,104u+(i/3u)*62u,148u,52u,1u,t);
    }
}
static void motors(QrUi *u,const MotorBus *b,uint32_t now)
{
    unsigned i; char t[UI_TEXT_BYTES],p[24],v[20];
    const MotorNode *n=&b->nodes[u->selected]; const MotorSnapshot *d=&n->data;
    if (!u->more) {
        for (i=0u;i<MOTOR_COUNT;++i) {
            n=&b->nodes[i];
            if (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now)) snprintf(v,sizeof(v),"%ld",(long)n->data.speed_rpm); else strcpy(v,"--");
            snprintf(t,sizeof(t),"%u %-4s %-7s %s RPM",i+1u,MotorBus_Role((uint8_t)(i+1u)),node_state(n,now),v);
            (void)item(u,8u,44u+i*24u,464u,22u,1u,t);
        }
        snprintf(t,sizeof(t),"%.42s",b->message); row(u,7u,t); return;
    }
    snprintf(t,sizeof(t),"%u %s %s %s",u->selected+1u,MotorBus_Role((uint8_t)(u->selected+1u)),node_state(n,now),MotorBus_ActionName(n->action)); row(u,0u,t);
    if (MotorBus_Fresh(n,MOTOR_VALID_POSITION,now)) snprintf(p,sizeof(p),"%lld",(long long)d->position_counts); else strcpy(p,"-- / STALE");
    snprintf(t,sizeof(t),"POSITION: %s counts",p); row(u,1u,t);
    if (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now)) snprintf(v,sizeof(v),"%ld",(long)d->speed_rpm); else strcpy(v,"--");
    snprintf(t,sizeof(t),"SPEED: %s RPM   STOP:%s",v,n->stop_unconfirmed ? "UNCONFIRMED" : "OK"); row(u,2u,t);
    if (MotorBus_Fresh(n,MOTOR_VALID_STATUS,now)) snprintf(t,sizeof(t),"ENABLE:%u INPOS:%u FAULT:%02X",d->status_flags&1u,(d->status_flags>>1)&1u,d->status_flags&12u);
    else strcpy(t,"STATUS: -- / STALE");
    row(u,3u,t);
    snprintf(t,sizeof(t),"RX:%lu TIMEOUT:%lu ERR:%u",(unsigned long)d->rx_count,(unsigned long)d->timeouts,d->last_error); row(u,4u,t);
    snprintf(t,sizeof(t),"%.42s",b->message); row(u,5u,t);
    (void)item(u,8,196,140,30,1,/* 上一轴 */ "\344\270\212\344\270\200\350\275\264"); (void)item(u,160,196,140,30,1,/* 下一轴 */ "\344\270\213\344\270\200\350\275\264");
}
static void qr(QrUi *u,const QrService *q,uint32_t now)
{
    char t[UI_TEXT_BYTES]; const QrSnapshot *d=&q->data;
    snprintf(t,sizeof(t),"%s / SESSION %lu",QrService_StateName(q),(unsigned long)d->session); row(u,0u,t);
    if (!u->more) {
        UiWidget *v=item(u,8,76,464,32,0,d->valid ? d->current.text : "---+---+---+---"); v->large=1u;
        row(u,3u,d->valid ? /* 任务码已锁定 */ "\344\273\273\345\212\241\347\240\201\345\267\262\351\224\201\345\256\232" : /* 等待有效任务码 */ "\347\255\211\345\276\205\346\234\211\346\225\210\344\273\273\345\212\241\347\240\201");
        snprintf(t,sizeof(t),"UART:%s WAIT:%lu s",d->uart_ready ? "OK" : "ERROR",(unsigned long)((now-d->session_ms)/1000u)); row(u,4u,t);
        (void)item(u,8,196,216,30,1,/* 重新扫码 */ "\351\207\215\346\226\260\346\211\253\347\240\201"); return;
    }
    snprintf(t,sizeof(t),"RX:%lu RECORD:%lu",(unsigned long)d->raw_bytes,(unsigned long)d->records); row(u,1u,t);
    snprintf(t,sizeof(t),"ACCEPT:%lu DUP:%lu",(unsigned long)d->accepted_seq,(unsigned long)d->duplicates); row(u,2u,t);
    snprintf(t,sizeof(t),"BAD:%lu PARTIAL:%lu",(unsigned long)d->invalid_records,(unsigned long)d->frame_timeouts); row(u,3u,t);
    snprintf(t,sizeof(t),"OVF:%lu UART:%lu",(unsigned long)d->overflows,(unsigned long)d->uart_errors); row(u,4u,t);
    snprintf(t,sizeof(t),"ERR:%u CODE:%s",d->last_error,QrTaskCode_ErrorName(d->code_error)); row(u,5u,t);
    row(u,6u,d->valid ? d->current.text : "NO CURRENT CODE");
}
static void chassis(QrUi *u,uint32_t now)
{
    const Chassis *c=u->chassis; const ChassisSnapshot *d;
    char t[UI_TEXT_BYTES],a[20],b[20],v[20]; unsigned i;
    if (!c) { row(u,0u,"UNAVAILABLE"); return; } d=&c->data;
    if (!u->more) {
        const ChassisOdometry *o=&c->odometry;
        uint8_t valid=(uint8_t)(o->valid && now-o->sample_ms<=MOTOR_BUS_FRESH_MS);
        row(u,0u,/* 轮端里程计 / 当前连续坐标段 */ "\350\275\256\347\253\257\351\207\214\347\250\213\350\256\241 / \345\275\223\345\211\215\350\277\236\347\273\255\345\235\220\346\240\207\346\256\265");
        if (valid) { decimal(a,o->x); decimal(b,o->y); decimal(v,o->angle); }
        else { strcpy(a,"--"); strcpy(b,"--"); strcpy(v,"--"); }
        snprintf(t,sizeof(t),"X: %s mm",a); row(u,1u,t);
        snprintf(t,sizeof(t),"Y: %s mm",b); row(u,2u,t);
        snprintf(t,sizeof(t),"YAW: %s deg",v); row(u,3u,t);
        snprintf(t,sizeof(t),"%s  SEG:%lu",valid ? "VALID" : "INVALID / WAIT",(unsigned long)o->generation); row(u,4u,t);
        snprintf(t,sizeof(t),"%s %u/8 %s",Chassis_StateName(d->state),d->stage,Chassis_StageName(d->stage)); row(u,5u,t);
        row(u,6u,Chassis_ReasonName(d->reason!=CHASSIS_OK ? d->reason : d->rejection));
        row(u,7u,/* 打滑不可观测，断链后重新建立坐标段 */ "\346\211\223\346\273\221\344\270\215\345\217\257\350\247\202\346\265\213\357\274\214\346\226\255\351\223\276\345\220\216\351\207\215\346\226\260\345\273\272\347\253\213\345\235\220\346\240\207\346\256\265"); return;
    }
    snprintf(t,sizeof(t),"%s %u/8 %s",Chassis_StateName(d->state),d->stage,Chassis_StageName(d->stage)); row(u,0u,t);
    for (i=0u;i<4u;++i) {
        const MotorNode *n=&c->bus->nodes[i];
        if (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now)) snprintf(v,sizeof(v),"%ld",(long)n->data.speed_rpm); else strcpy(v,"--");
        snprintf(t,sizeof(t),"%s CMD:%ld RPM:%s %s",MotorBus_Role((uint8_t)(i+1u)),(long)d->wheels.quantized[i],v,node_state(n,now)); row(u,i+1u,t);
    }
    snprintf(t,sizeof(t),"%.20s / %.18s",Chassis_ReasonName(d->reason!=CHASSIS_OK ? d->reason : d->rejection),
        d->fault.valid ? Chassis_OdometryIssueName(d->fault.issue) : Chassis_OdometryIssueName(c->odometry.last_issue)); row(u,5u,t);
    (void)item(u,8,196,216,30,1,/* 底盘八段验收 */ "\345\272\225\347\233\230\345\205\253\346\256\265\351\252\214\346\224\266");
}
static void mechanism(QrUi *u,uint32_t now)
{
    const Mechanism *m=u->mechanism; const MechanismSnapshot *d;
    char t[UI_TEXT_BYTES],v[20]; unsigned i;
    static const char * const refs[]={"NO ZERO","HOMED"};
    if (!m) { row(u,0u,"UNAVAILABLE"); return; } d=&m->data;
    if (!u->more) {
        snprintf(t,sizeof(t),"%s%s",Mechanism_StateName(d->state),MECHANISM_TRIAL_RUN ? " / TRIAL" : ""); row(u,0u,t);
        for (i=0u;i<3u;++i) {
            static const char * const names[]={/* 5 旋转 */ "5 \346\227\213\350\275\254",/* 6 升降 */ "6 \345\215\207\351\231\215",/* 7 伸缩 */ "7 \344\274\270\347\274\251"};
            const MotorNode *n=&m->bus->nodes[i+4u];
            if (d->reference[i] && MotorBus_Fresh(n,MOTOR_VALID_POSITION,now) && n->data.online) decimal(v,d->position[i]); else strcpy(v,"--");
            snprintf(t,sizeof(t),"%s: %s %s / %s",names[i],v,i ? "mm" : "deg",refs[d->reference[i]]); row(u,i+1u,t);
        }
        row(u,4u,Mechanism_ReasonName(d->reason!=MECH_OK ? d->reason : d->rejection));
        row(u,5u,"5/6 HOME; 7 MIDPOINT");
        row(u,6u,MECHANISM_TRIAL_RUN ? /* TRIAL: 临时换算参数，尚未标定 */ "TRIAL: \344\270\264\346\227\266\346\215\242\347\256\227\345\217\202\346\225\260\357\274\214\345\260\232\346\234\252\346\240\207\345\256\232" : /* 换算参数需实物标定 */ "\346\215\242\347\256\227\345\217\202\346\225\260\351\234\200\345\256\236\347\211\251\346\240\207\345\256\232");
        row(u,7u,/* MORE 查看单轴与回零操作 */ "MORE \346\237\245\347\234\213\345\215\225\350\275\264\344\270\216\345\233\236\351\233\266\346\223\215\344\275\234"); return;
    }
    i=u->axis;
    snprintf(t,sizeof(t),/* 轴 %u %s / 点击切换轴 */ "\350\275\264 %u %s / \347\202\271\345\207\273\345\210\207\346\215\242\350\275\264",i+5u,MotorBus_Role((uint8_t)(i+5u)));
    (void)item(u,8,44,464,22,1,t);
    snprintf(t,sizeof(t),"REF:%s CFG:%lu",refs[d->reference[i]],(unsigned long)d->config_issues[i]); row(u,1u,t);
    snprintf(t,sizeof(t),"COUNTS:%lld%s",(long long)m->bus->nodes[i+4u].data.position_counts,
        MotorBus_Fresh(&m->bus->nodes[i+4u],MOTOR_VALID_POSITION,now) ? "" : " STALE"); row(u,2u,t);
    snprintf(t,sizeof(t),"%s AXIS:%u STAGE:%u/8",Mechanism_StateName(d->state),d->axis+5u,d->stage); row(u,3u,t);
    snprintf(t,sizeof(t),"%s STOP:%s",Mechanism_ReasonName(d->reason!=MECH_OK ? d->reason : d->rejection),d->stop_pending ? "UNCONF" : "OK"); row(u,4u,t);
    row(u,5u,MECHANISM_TRIAL_RUN ? /* TRIAL: 临时参数 */ "TRIAL: \344\270\264\346\227\266\345\217\202\346\225\260" : /* 动作仍受标定与限位门控 */ "\345\212\250\344\275\234\344\273\215\345\217\227\346\240\207\345\256\232\344\270\216\351\231\220\344\275\215\351\227\250\346\216\247");
    (void)item(u,8,196,140,30,1,/* 重新回零 */ "\351\207\215\346\226\260\345\233\236\351\233\266"); (void)item(u,160,196,140,30,1,/* 机构验收 */ "\346\234\272\346\236\204\351\252\214\346\224\266");
}
static void limits(QrUi *u,uint32_t now)
{
    const Mechanism *m=u->mechanism; unsigned i; char t[UI_TEXT_BYTES];
    if (!m) { row(u,0u,"UNAVAILABLE"); return; }
    for (i=0u;i<2u;++i) {
        const MechanismLimit *l=&m->data.limit[i];
        snprintf(t,sizeof(t),"%u %s: %s",i+5u,i ? /* 升降最上端 */ "\345\215\207\351\231\215\346\234\200\344\270\212\347\253\257" : /* 旋转收拢端 */ "\346\227\213\350\275\254\346\224\266\346\213\242\347\253\257",!l->valid ? /* 采样待稳定 */ "\351\207\207\346\240\267\345\276\205\347\250\263\345\256\232" : l->stable ? /* 按下 / 断路 */ "\346\214\211\344\270\213 / \346\226\255\350\267\257" : /* 释放 */ "\351\207\212\346\224\276"); row(u,i*2u,t);
        snprintf(t,sizeof(t),"PG%u RAW:%u STABLE:%s",i+2u,l->raw,!l->valid ? "--" : l->stable ? "HIGH" : "LOW"); row(u,i*2u+1u,t);
    }
    row(u,4u,/* 7 伸缩轴: 未安装限位 */ "7 \344\274\270\347\274\251\350\275\264: \346\234\252\345\256\211\350\243\205\351\231\220\344\275\215");
    if (u->more) {
        row(u,5u,/* COM 接地 / NC 输入上拉 / 高有效 */ "COM \346\216\245\345\234\260 / NC \350\276\223\345\205\245\344\270\212\346\213\211 / \351\253\230\346\234\211\346\225\210");
        snprintf(t,sizeof(t),"MAX SAMPLE GAP:%lu ms",(unsigned long)m->data.max_sample_gap_ms); row(u,6u,t);
        snprintf(t,sizeof(t),"SAMPLE AGE:%lu ms",(unsigned long)(now-m->last_sample_ms)); row(u,7u,t);
    } else { row(u,5u,/* 高电平也可能表示接线断开 */ "\351\253\230\347\224\265\345\271\263\344\271\237\345\217\257\350\203\275\350\241\250\347\244\272\346\216\245\347\272\277\346\226\255\345\274\200"); row(u,6u,/* 原始电平与消抖结果分别显示 */ "\345\216\237\345\247\213\347\224\265\345\271\263\344\270\216\346\266\210\346\212\226\347\273\223\346\236\234\345\210\206\345\210\253\346\230\276\347\244\272"); }
}
static void servos(QrUi *u)
{
    static const char * const labels[]={"JAW ON","J 1700","J 2430","JAW OFF",
                                        "TABLE ON","T PREV","T NEXT","TABLE OFF"};
    unsigned i;
#if defined(SERVO_DUAL_RUNTIME) && SERVO_DUAL_RUNTIME
    char t[UI_TEXT_BYTES];
    if (u->servo_ready) {
        for (i=0u;i<2u;++i) {
            snprintf(t,sizeof(t),"%s:%s OUT:%u TGT:%ld us",i ? "TABLE" : "JAW",
                Servo_IsEnabled((uint8_t)i) ? "ON" : "OFF",
                Servo_IsEnabled((uint8_t)i) ? (unsigned)Servo_CurrentUs((uint8_t)i) : 0u,
                (long)Servo_TargetUs((uint8_t)i));
            row(u,i,t);
        }
    } else row(u,0u,"SERVO INIT FAILED / PWM OFF");
#else
    row(u,0u,"SERVO NOT AVAILABLE");
#endif
    for (i=0u;i<8u;++i)
        (void)item(u,4u+(i%4u)*119u,112u+(i/4u)*48u,114u,36u,1u,labels[i]);
    (void)item(u,8u,196u,464u,24u,0u,u->servo_blocked ? "ARM/CHASSIS BUSY" :
        u->servo && u->servo->last_status==SERVO_REJECT_RANGE ? "LIMIT: TARGET UNCHANGED" :
        u->servo && u->servo->last_status==SERVO_REJECT_STATE ? "TAP THAT SERVO ON FIRST" :
        "TABLE: 1240 / 1900 / 2600 us");
}
static void snapshot(QrUi *u,const QrService *q,const MotorBus *b,const GyroSnapshot *g,uint32_t now)
{
    unsigned i; char t[43]; u->count=0u;
    (void)item(u,8,4,384,32,0,titles[u->page]);
    switch (u->page) {
    case UI_HOME: home(u,q,b,g,now); break;
    case UI_MOTOR: motors(u,b,now); break;
    case UI_QR: qr(u,q,now); break;
    case UI_CHASSIS: chassis(u,now); break;
    case UI_MECHANISM: mechanism(u,now); break;
    case UI_LIMITS: limits(u,now); break;
    case UI_SERVO: servos(u); break;
    case UI_VISION:
        if (u->vision) for(i=1u;i<=6u;++i) {
            VisionUi_Line(t,i,u->vision,u->vision_mode,u->vision_color,u->vision_zone,u->vision_position,now); row(u,i-1u,t);
        }
        (void)item(u,8,196,140,30,1,"SCAN"); (void)item(u,160,196,140,30,1,"VISION STOP");
        break;
    case UI_RACE:
        if (u->mission) for(i=1u;i<=6u;++i) { MissionUi_Line(t,i==6u ? 7u : i,u->mission,now); row(u,i-1u,t); }
        (void)item(u,8,196,216,30,1,"START / ZONE 2");
        break;
    default:
        for (i=1u;i<=(u->more ? 8u : 6u);++i) {
            GyroUi_Line(t,i,u->more,g);
            row(u,i-1u,t);
        }
        if (!u->more) {
            if (u->page==UI_GYRO) { (void)item(u,8,196,140,30,1,/* 校准 */ "\346\240\241\345\207\206"); (void)item(u,160,196,140,30,1,/* 角度清零 */ "\350\247\222\345\272\246\346\270\205\351\233\266"); }
        }
        break;
    }
    for (i=0u;i<5u;++i) {
        static const char * const labels[]={/* 首页 */ "\351\246\226\351\241\265",/* 底盘 */ "\345\272\225\347\233\230",/* 机械臂 */ "\346\234\272\346\242\260\350\207\202",/* 限位 */ "\351\231\220\344\275\215",/* 停止 */ "\345\201\234\346\255\242"};
        (void)item(u,nav_x[i],232,nav_w[i],32,1,labels[i]);
    }
    (void)item(u,400,4,72,32,1,u->more ? "LESS" : "MORE");
}

/* Built-in UTF-8 subset; no external font flash is needed. */
static uint16_t codepoint(const char *s,uint8_t *bytes)
{
    const uint8_t *p=(const uint8_t *)s;
    if (p[0]<0x80u) { *bytes=1u; return p[0]; }
    if ((p[0]&0xF0u)==0xE0u && p[1] && p[2]) {
        *bytes=3u; return (uint16_t)(((p[0]&15u)<<12)|((p[1]&63u)<<6)|(p[2]&63u));
    }
    *bytes=1u; return '?';
}
static unsigned text_width(const char *s,unsigned large)
{
    unsigned width=0u; uint8_t n;
    while (*s && *s!='\n') { uint16_t c=codepoint(s,&n); width+=c<128u ? (large ? Font24.Width : Font16.Width) : 20u; s+=n; }
    return width;
}
static void start_line(QrUi *u,const UiWidget *v)
{
    unsigned width=text_width(v->text+u->offset,v->large);
    u->text_x=(uint16_t)(v->x+(v->button && width<v->width ? (v->width-width)/2u : 0u));
}
static void glyph(uint16_t x,uint16_t y,uint16_t c,uint32_t fg,uint32_t bg)
{
    unsigned i,j,k; const uint8_t *bits=0;
    for (k=0u;k<UI_GLYPH_COUNT;++k) if (ui_glyphs[k].code==c) { bits=ui_glyphs[k].bits; break; }
    for (i=0u;i<20u;++i) for (j=0u;j<20u;++j)
        LCD_DrawPixel((uint16_t)(x+j),(uint16_t)(y+i),bits && (bits[i*3u+j/8u] & (0x80u>>(j%8u))) ? fg : bg);
}
static void draw_step(QrUi *u)
{
    UiWidget *v; unsigned h,k;
    if (u->drawing==DRAW_IDLE) return;
    if (u->drawing==DRAW_CLEAR) {
        LCD_SetTextColor(LCD_COLOR_BLACK); LCD_FillRect(0,u->clear_y,480,4); u->clear_y+=4u;
        if (u->clear_y==272u) { u->widget=0u; u->drawing=DRAW_WIDGET; } return;
    }
    if (u->drawing==DRAW_WIDGET) {
        while (u->widget<u->count && !strcmp(u->previous[u->widget],u->target[u->widget].text)) ++u->widget;
        if (u->widget==u->count) { u->drawing=DRAW_IDLE; return; }
        u->strip=0u; u->drawing=DRAW_FILL;
    }
    v=&u->target[u->widget];
    if (u->drawing==DRAW_FILL) {
        h=v->height-u->strip; if (h>4u) h=4u;
        LCD_SetTextColor(v->button ? LCD_COLOR_WHITE : LCD_COLOR_BLACK);
        LCD_FillRect(v->x,(uint16_t)(v->y+u->strip),v->width,(uint16_t)h); u->strip=(uint16_t)(u->strip+h);
        if (u->strip==v->height) {
            u->offset=0u; u->text_y=(uint16_t)(v->y+(v->height==52u ? 3u : v->height>20u ? (v->height-20u)/2u : 0u));
            start_line(u,v); u->drawing=DRAW_TEXT;
        } return;
    }
    LCD_SetBackColor(v->button ? LCD_COLOR_WHITE : LCD_COLOR_BLACK);
    LCD_SetTextColor(v->button ? LCD_COLOR_BLACK : LCD_COLOR_WHITE);
    for (k=0u;k<2u && v->text[u->offset];++k) {
        uint8_t n; uint16_t c; unsigned width;
        if (v->text[u->offset]=='\n') { ++u->offset; u->text_y+=24u; start_line(u,v); }
        c=codepoint(v->text+u->offset,&n); if (!c) break;
        width=c<128u ? (v->large ? Font24.Width : Font16.Width) : 20u;
        if (u->text_x+width>(unsigned)v->x+v->width || u->text_y+(v->large ? 24u : 20u)>(unsigned)v->y+v->height) {
            while (v->text[u->offset] && v->text[u->offset]!='\n') ++u->offset;
            continue;
        }
        if (c<128u) {
            char part[2]; part[0]=(char)c; part[1]='\0'; LCD_SetFont(v->large ? &Font24 : &Font16);
            LCD_DisplayStringAt(u->text_x,(uint16_t)(u->text_y+(v->large ? 0u : 2u)),(uint8_t *)part,LEFT_MODE);
        } else glyph(u->text_x,u->text_y,c,v->button ? LCD_COLOR_BLACK : LCD_COLOR_WHITE,v->button ? LCD_COLOR_WHITE : LCD_COLOR_BLACK);
        u->text_x=(uint16_t)(u->text_x+width); u->offset=(uint8_t)(u->offset+n);
    }
    if (!v->text[u->offset]) { strcpy(u->previous[u->widget],v->text); ++u->widget; u->drawing=DRAW_WIDGET; }
}
void QrUi_Process(QrUi *u,const QrService *q,const MotorBus *b,const GyroSnapshot *g,uint32_t now)
{
    if (u->redraw || (!u->drawing && now-u->last_draw>=100u)) {
        if (u->redraw) { memset(u->previous,0,sizeof(u->previous)); u->clear_y=0u; u->drawing=DRAW_CLEAR; }
        else { u->widget=0u; u->drawing=DRAW_WIDGET; }
        u->redraw=0u; u->last_draw=now; snapshot(u,q,b,g,now);
    }
    draw_step(u);
}
