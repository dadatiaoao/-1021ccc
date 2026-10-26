#include "mechanism.h"
#include "./motor/motor_test_config.h"
#include <math.h>
#include <float.h>
#include <string.h>

enum { OP_NONE, OP_HOME, OP_MOVE, OP_TEST };
static int finite_number(double v) { return v==v && v<=DBL_MAX && v>=-DBL_MAX; }
static int positive(double v) { return finite_number(v) && v>0.0; }
static int due(uint32_t now, uint32_t at) { return (int32_t)(now-at)>=0; }
static MotorNode *node(Mechanism *m, unsigned a) { return &m->bus->nodes[a+4u]; }
static uint8_t axis_bit(unsigned a) { return (uint8_t)(1u<<(a+4u)); }
static int busy_node(const MotorNode *n)
{
    return n->motion || n->action==MOTOR_QUEUED || n->action==MOTOR_SENT ||
        n->action==MOTOR_ACCEPTED || n->action==MOTOR_EXECUTING || n->stop_unconfirmed;
}
static int feedback_healthy(const MotorNode *n, uint32_t now, uint8_t enabled)
{
    return n->data.online && MotorBus_Fresh(n,MOTOR_VALID_POSITION,now) &&
        MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) && MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) &&
        !(n->data.status_flags & 12u) && (!enabled || (n->data.status_flags & 1u)) &&
        n->action!=MOTOR_UNKNOWN && n->action!=MOTOR_REJECTED;
}
static int healthy(const MotorNode *n, uint32_t now, uint8_t enabled)
{ return feedback_healthy(n,now,enabled) && !n->stop_unconfirmed; }
static MechanismReason motor_reason(const MotorNode *n)
{
    if (n->data.last_error==MOTOR_ERROR_MOTION_TIMEOUT) return MECH_TIMEOUT;
    if ((n->data.status_flags & 12u) || n->data.last_error==MOTOR_ERROR_STALL) return MECH_DRIVER_FAULT;
    return MECH_FEEDBACK;
}
static double position(const Mechanism *m, unsigned a)
{
    return (double)(m->bus->nodes[a+4u].data.position_counts-m->data.origin_counts[a]) *
        m->config.axis[a].positive_sign*m->config.axis[a].units_per_rev/65536.0;
}
void Mechanism_DefaultConfig(MechanismConfig *c)
{
    static const MechanismConfig measured=MECHANISM_CALIBRATION;
    *c=measured;
}
uint32_t Mechanism_ConfigIssues(const MechanismConfig *c, unsigned a)
{
    const MechanismAxisConfig *p;
    uint32_t r=0u;
    if (!c || a>2u) return 63u;
    p=&c->axis[a];
    if (!positive(p->units_per_rev) || !p->pulses_per_rev || p->pulses_per_rev>1000000u) r|=MECH_CFG_SCALE;
    if (p->positive_sign!=1 && p->positive_sign!=-1) r|=MECH_CFG_DIRECTION;
    if (!p->calibrated) r|=MECH_CFG_CONFIRM;
    if (!positive(p->max_rpm) || p->max_rpm>3000.0f || !positive(p->move_speed) ||
        !positive(p->tolerance) || p->tolerance<p->units_per_rev/p->pulses_per_rev ||
        !c->stable_ms || c->stable_ms>1000u || c->prepare_ms<100u || c->prepare_ms>60000u ||
        c->between_ms<500u || c->between_ms>10000u) r|=MECH_CFG_MOTION;
    if (a<2u) {
        const MechanismHomeConfig *h=&c->home[a];
        if (!h->wiring_confirmed || !h->home_confirmed) r|=MECH_CFG_CONFIRM;
        if (!positive(h->max_position) || !positive(h->initial_release) || !positive(h->fast_seek) ||
            !positive(h->backoff) || !positive(h->slow_seek) || !positive(h->clearance) ||
            !positive(h->fast_rpm) || !positive(h->slow_rpm) || h->slow_rpm>=h->fast_rpm ||
            h->fast_rpm>p->max_rpm || h->slow_rpm<1.0f ||
            h->backoff<=p->tolerance || h->clearance<=p->tolerance) r|=MECH_CFG_HOME;
        if (!positive(c->outside_angle) || c->outside_angle>c->home[0].max_position ||
            !positive(c->top_window) || !positive(c->work_window) ||
            c->top_window>=c->home[1].max_position || c->work_window>=c->outside_angle ||
            !positive(c->test_down_mm) || c->test_down_mm>c->home[1].max_position) r|=MECH_CFG_POSE;
    }
    if (a==2u) {
        const MechanismTrimHomeConfig *h=&c->trim_home;
        double backoff=h->backoff_tenths/3600.0*p->units_per_rev;
        if (!positive(c->test_trim_mm) || c->test_trim_mm>h->travel_mm/2.0f) r|=MECH_CFG_MOTION;
        if (!positive(h->travel_mm) || !h->rpm || h->rpm>p->max_rpm ||
            !h->stall_rpm || !h->stall_ma || !h->stall_ms || !h->backoff_tenths ||
            backoff>=h->travel_mm/2.0 || h->timeout_ms<100u || h->timeout_ms>60000u)
            r|=MECH_CFG_HOME;
    }
    return r;
}
static MechanismRequestResult reject(Mechanism *m, MechanismRequestResult r, MechanismReason why)
{ m->data.rejection=why; return r; }
int Mechanism_OwnsAxes(const Mechanism *m) { return m && m->owned; }
static int peer_busy(const Mechanism *m)
{ return m->chassis && Chassis_OwnsWheels(m->chassis); }
static int stops_pending(const Mechanism *m)
{
    unsigned a;
    for (a=0u;a<3u;++a) if (m->bus->nodes[a+4u].stop_unconfirmed) return 1;
    return 0;
}
static void begin(Mechanism *m, uint8_t op, uint32_t now)
{
    if (!++m->data.request_id) ++m->data.request_id;
    m->data.reason=m->data.rejection=MECH_OK; m->data.result=MECH_RUNNING;
    m->data.done=m->data.stage=m->data.stop_pending=0u;
    m->data.acceptance=(uint8_t)(op==OP_TEST);
    m->data.homing=(uint8_t)(op==OP_HOME);
    m->operation=op; m->owned=1u; m->touched=m->segment=m->stable=m->enable_wait=0u;
    m->triggered=m->waiting_position=0u; m->deadline=now+m->config.prepare_ms;
    m->progress_request=0u;
}
static void fail(Mechanism *m, MechanismReason why, uint32_t now)
{
    if (m->data.reason==MECH_OK) m->data.reason=why; /* first failure survives recovery */
    if (why==MECH_FEEDBACK || why==MECH_DRIVER_FAULT || why==MECH_LIMIT || why==MECH_SAMPLE_GAP ||
        why==MECH_ENABLE_FAILED || why==MECH_FINAL_ERROR || why==MECH_BUS_REJECTED || why==MECH_TIMEOUT)
        m->data.reference[m->data.axis]=MECH_UNREFERENCED;
    m->data.result=MECH_FAILED; m->data.acceptance=0u; m->operation=OP_NONE;
    m->stop_mask=m->touched; m->stop_at=now;
    if (m->stop_mask) MotorBus_Stop(m->bus,m->stop_mask,now);
    m->data.state=MECH_STOPPING; m->data.stop_pending=(uint8_t)(m->stop_mask!=0u);
}
void Mechanism_Init(Mechanism *m, MotorBus *b, const MechanismConfig *c, uint32_t now)
{
    unsigned a;
    memset(m,0,sizeof(*m)); m->bus=b;
    if (c) m->config=*c; else Mechanism_DefaultConfig(&m->config);
    for (a=0u;a<3u;++a) m->data.config_issues[a]=Mechanism_ConfigIssues(&m->config,a);
    m->auto_pending=1u; m->last_sample_ms=now;
}
void Mechanism_SampleLimits(Mechanism *m, uint8_t arm, uint8_t lift, uint32_t now)
{
    unsigned a; uint32_t gap=now-m->last_sample_ms;
    uint8_t raw[2]; raw[0]=(uint8_t)(arm!=0u); raw[1]=(uint8_t)(lift!=0u);
    if (gap>m->data.max_sample_gap_ms) m->data.max_sample_gap_ms=gap;
    m->last_sample_ms=now;
    for (a=0u;a<2u;++a) {
        MechanismLimit *l=&m->data.limit[a];
        if (!l->sampled || gap>10u || l->raw!=raw[a]) { l->changed_ms=now; l->valid=0u; }
        l->sampled=1u; l->raw=raw[a];
        if (now-l->changed_ms>=20u) { l->stable=l->raw; l->valid=1u; }
    }
    if (gap>10u && m->owned && m->data.state!=MECH_STOPPING) fail(m,MECH_SAMPLE_GAP,now);
}
MechanismRequestResult Mechanism_HomeAll(Mechanism *m, uint32_t now)
{
    unsigned a;
    if (m->owned || peer_busy(m)) return reject(m,MECH_BUSY,MECH_AXIS_BUSY);
    if (stops_pending(m)) return reject(m,MECH_NOT_READY,MECH_STOP_UNCONFIRMED);
    if (m->data.config_issues[0] || m->data.config_issues[1]) return reject(m,MECH_INVALID,MECH_CONFIG);
    if (!m->data.limit[0].valid || !m->data.limit[1].valid || now-m->last_sample_ms>10u)
        return reject(m,MECH_NOT_READY,MECH_SAMPLE_GAP);
    for (a=0u;a<3u;++a) {
        const MotorNode *n=node(m,a);
        if (busy_node(n) || (MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) && n->data.speed_rpm))
            return reject(m,MECH_BUSY,MECH_AXIS_BUSY);
    }
    begin(m,OP_HOME,now); m->auto_pending=0u;
    m->data.reference[0]=m->data.reference[1]=m->data.reference[2]=MECH_UNREFERENCED;
    m->data.axis=MECH_LIFT; m->data.state=MECH_PREPARE;
    return MECH_ACCEPTED;
}
static MechanismReason interlock(Mechanism *m, unsigned a, double target, uint32_t now)
{
    if (a==MECH_ARM && (m->data.reference[MECH_LIFT]!=MECH_HOMED ||
        !healthy(node(m,MECH_LIFT),now,1u) || busy_node(node(m,MECH_LIFT)) ||
        node(m,MECH_LIFT)->data.speed_rpm || fabs(position(m,MECH_LIFT))>m->config.top_window)) return MECH_INTERLOCK;
    if (a==MECH_LIFT && target>position(m,a) &&
        (m->data.reference[MECH_ARM]!=MECH_HOMED || !healthy(node(m,MECH_ARM),now,1u) ||
        busy_node(node(m,MECH_ARM)) || node(m,MECH_ARM)->data.speed_rpm ||
        fabs(position(m,MECH_ARM)-m->config.outside_angle)>m->config.work_window)) return MECH_INTERLOCK;
    return MECH_OK;
}
/* Conversion uses wide arithmetic, rounds pulses once and never clips a motion.
   Return 2 for a true no-op. The requested absolute target is checked again on completion. */
static MechanismReason submit(Mechanism *m, unsigned a, double delta, double speed, uint32_t now, uint8_t *noop)
{
    const MechanismAxisConfig *c=&m->config.axis[a];
    MotorNode *n=node(m,a); MotorAction action;
    double p,r,seconds; uint32_t pulses,timeout; uint16_t rpm; int64_t counts;
    *noop=0u;
    if (!finite_number(delta) || !positive(speed)) return MECH_BAD_REQUEST;
    p=fabs(delta)/c->units_per_rev*c->pulses_per_rev;
    r=speed/c->units_per_rev*60.0;
    if (!finite_number(p) || p>4294967295.0-0.5 || !positive(r) || r>c->max_rpm || r>3000.0) return MECH_BAD_REQUEST;
    pulses=(uint32_t)(p+0.5); rpm=(uint16_t)(r+0.5);
    if (!rpm || rpm>c->max_rpm) return MECH_RESOLUTION;
    if (!pulses) {
        if (fabs(delta)>c->tolerance) return MECH_RESOLUTION;
        *noop=1u; return MECH_OK;
    }
    counts=(int64_t)pulses*65536/c->pulses_per_rev;
    if (!counts) return MECH_RESOLUTION;
    if ((delta<0.0 ? -1 : 1)*c->positive_sign<0) counts= -counts;
    if (n->data.position_counts+counts>INT64_C(4294967295) ||
        n->data.position_counts+counts< -INT64_C(4294967295)) return MECH_BAD_REQUEST;
    seconds=1.0+3.0*((double)pulses/c->pulses_per_rev)*60.0/rpm;
    if (!finite_number(seconds) || seconds>60.0) return MECH_BAD_REQUEST;
    timeout=(uint32_t)(seconds*1000.0);
    memset(&action,0,sizeof(action)); action.pulses_per_rev=c->pulses_per_rev; action.timeout_ms=timeout;
    action.command=Emm_MoveEx((uint8_t)(a+5u),(uint8_t)(counts<0),rpm,c->acceleration,pulses,EMM_REL_ACTUAL,0u);
    if (!MotorBus_Submit(m->bus,&action,now)) return MECH_BUS_REJECTED;
    m->segment_request=n->request_id; m->data.bus_request_id=n->request_id;
    m->position_seq=n->data.position_seq; m->deadline=now+timeout+1000u;
    m->touched|=axis_bit(a); m->data.quantized_target=n->data.position_counts+counts;
    m->segment_target=(double)n->data.position_counts*c->positive_sign*c->units_per_rev/65536.0+delta;
    m->segment=1u; m->stable=0u;
    return MECH_OK;
}
static int target_in_range(const Mechanism *m, unsigned a, double target)
{
    return a==MECH_TRIM ? fabs(target)<=m->config.trim_home.travel_mm/2.0 :
        target>=0.0 && target<=m->config.home[a].max_position;
}
static MechanismRequestResult ready(Mechanism *m, unsigned a, uint32_t now)
{
    if (m->owned || peer_busy(m)) return reject(m,MECH_BUSY,MECH_AXIS_BUSY);
    if (m->data.state==MECH_FAULT || stops_pending(m)) return reject(m,MECH_NOT_READY,MECH_STOP_UNCONFIRMED);
    if (a>2u) return reject(m,MECH_INVALID,MECH_BAD_REQUEST);
    if (m->data.config_issues[a]) return reject(m,MECH_INVALID,MECH_CONFIG);
    if (!m->data.reference[a]) return reject(m,MECH_NOT_READY,MECH_REFERENCE);
    if (!healthy(node(m,a),now,1u)) return reject(m,MECH_NOT_READY,MECH_FEEDBACK);
    if (busy_node(node(m,a)) || node(m,a)->data.speed_rpm) return reject(m,MECH_BUSY,MECH_AXIS_BUSY);
    if (now-m->last_sample_ms>10u || !m->data.limit[0].valid || !m->data.limit[1].valid)
        return reject(m,MECH_NOT_READY,MECH_SAMPLE_GAP);
    if (m->data.limit[0].raw || m->data.limit[1].raw) return reject(m,MECH_NOT_READY,MECH_LIMIT);
    return MECH_ACCEPTED;
}
MechanismRequestResult Mechanism_MoveTo(Mechanism *m, MechanismAxis a, float target, float speed, uint32_t now)
{
    MechanismRequestResult r; MechanismReason why; uint8_t noop;
    if ((unsigned)a>2u || !finite_number(target) || !positive(speed)) return reject(m,MECH_INVALID,MECH_BAD_REQUEST);
    r=ready(m,(unsigned)a,now); if (r!=MECH_ACCEPTED) return r;
    if (!target_in_range(m,(unsigned)a,target)) return reject(m,MECH_INVALID,MECH_LIMIT);
    why=interlock(m,(unsigned)a,target,now); if (why!=MECH_OK) return reject(m,MECH_NOT_READY,why);
    if ((a<MECH_TRIM && m->data.limit[a].raw) || m->data.limit[0].raw || m->data.limit[1].raw)
        return reject(m,MECH_NOT_READY,MECH_LIMIT);
    /* Validate/submit before changing logical request state, so invalid requests do not acquire ownership. */
    why=submit(m,(unsigned)a,target-position(m,a),speed,now,&noop);
    if (why!=MECH_OK) return reject(m,MECH_INVALID,why);
    {
        uint32_t deadline=m->deadline;
        begin(m,OP_MOVE,now); m->data.axis=(uint8_t)a; m->data.target=target;
        m->data.state=MECH_MOVING; m->segment=(uint8_t)(noop ? 0u : 1u);
        m->touched=noop ? 0u : axis_bit((unsigned)a); m->deadline=deadline;
        if (noop) { m->data.result=MECH_SUCCESS; m->data.state=MECH_IDLE; m->owned=0u; }
    }
    return MECH_ACCEPTED;
}
MechanismRequestResult Mechanism_MoveRelative(Mechanism *m, MechanismAxis a, float delta, float speed, uint32_t now)
{
    double target;
    if ((unsigned)a>2u || !finite_number(delta)) return reject(m,MECH_INVALID,MECH_BAD_REQUEST);
    target=position(m,(unsigned)a)+delta;
    if (!finite_number(target) || fabs(target)>FLT_MAX) return reject(m,MECH_INVALID,MECH_BAD_REQUEST);
    return Mechanism_MoveTo(m,a,(float)target,speed,now);
}
MechanismRequestResult Mechanism_StartAcceptance(Mechanism *m, uint32_t now)
{
    unsigned a; MechanismRequestResult r;
    for (a=0u;a<3u;++a) { r=ready(m,a,now); if (r!=MECH_ACCEPTED) return r; }
    if (fabs(position(m,MECH_ARM))>m->config.axis[0].tolerance ||
        fabs(position(m,MECH_LIFT))>m->config.top_window) return reject(m,MECH_NOT_READY,MECH_INTERLOCK);
    if (!target_in_range(m,MECH_TRIM,position(m,MECH_TRIM)+m->config.test_trim_mm) ||
        !target_in_range(m,MECH_TRIM,position(m,MECH_TRIM)-m->config.test_trim_mm))
        return reject(m,MECH_INVALID,MECH_LIMIT);
    begin(m,OP_TEST,now); m->trim_start=position(m,MECH_TRIM);
    m->data.state=MECH_BETWEEN; m->deadline=now;
    return MECH_ACCEPTED;
}
void Mechanism_Stop(Mechanism *m, uint32_t now)
{
    m->auto_pending=0u; /* a STOP never schedules a later automatic retry */
    if (m->data.state==MECH_STOPPING || m->data.state==MECH_FAULT) return;
    m->data.rejection=MECH_OK;
    m->data.result=MECH_CANCELLED; m->data.acceptance=0u; m->operation=OP_NONE;
    m->stop_mask=m->owned ? m->touched : 0u; m->stop_at=now;
    if (m->stop_mask) MotorBus_Stop(m->bus,m->stop_mask,now);
    m->data.state=MECH_STOPPING; m->data.stop_pending=(uint8_t)(m->stop_mask!=0u); m->owned=1u;
}
static void complete(Mechanism *m)
{
    m->owned=0u; m->operation=OP_NONE; m->data.state=MECH_IDLE;
    m->data.result=MECH_SUCCESS; m->data.done=m->data.acceptance; m->data.acceptance=0u;
}
static void next_step(Mechanism *m, MechanismState state, uint32_t now)
{
    m->data.state=state; m->segment=m->stable=m->triggered=m->waiting_position=0u;
    m->deadline=now+m->config.prepare_ms;
}
static int settled(Mechanism *m, const MotorNode *n, uint32_t now)
{
    if (!healthy(n,now,1u) || busy_node(n) || n->data.speed_rpm) { m->stable=0u; return 0; }
    if (!m->stable) { m->stable=1u; m->stable_since=now; }
    return now-m->stable_since>=m->config.stable_ms;
}
static void home_axis_done(Mechanism *m, uint32_t now)
{
    unsigned a=m->data.axis;
    m->data.origin_counts[a]=node(m,a)->data.position_counts; m->data.reference[a]=MECH_HOMED;
    if (a==MECH_LIFT) { m->data.axis=MECH_ARM; next_step(m,MECH_PREPARE,now); }
    else {
        m->data.axis=MECH_TRIM;
        if (!m->data.config_issues[2] && healthy(node(m,2u),now,0u)) next_step(m,MECH_PREPARE,now);
        else complete(m); /* absence of 7 never invents an unconfirmable STOP */
    }
}
static int new_position_after(Mechanism *m, const MotorNode *n, uint32_t now)
{
    if (!m->waiting_position) {
        m->waiting_position=1u; m->position_seq=n->data.position_seq; m->wait_position_ms=now; return 0;
    }
    return n->data.position_seq!=m->position_seq && due(n->data.position_ms,m->wait_position_ms+1u);
}
static void prepare(Mechanism *m, uint32_t now)
{
    unsigned i,a=m->data.axis; uint8_t known_wheels=0u; MotorNode *n=node(m,a);
    if (due(now,m->deadline)) { fail(m,MECH_ENABLE_FAILED,now); return; }
    /* Known wheels must stop; nodes never seen/commanded are optional on a bench. */
    for (i=0u;i<4u;++i) {
        const MotorNode *w=&m->bus->nodes[i];
        if (w->data.valid || w->request_id || w->data.rx_count || w->stop_unconfirmed) known_wheels|=(uint8_t)(1u<<i);
    }
    for (i=0u;i<4u;++i) {
        MotorNode *w=&m->bus->nodes[i];
        if (!(known_wheels & (1u<<i))) continue;
        if (w->stop_unconfirmed) return;
        if (busy_node(w) || (MotorBus_Fresh(w,MOTOR_VALID_SPEED,now) && w->data.speed_rpm)) {
            if (!w->stop_unconfirmed) MotorBus_Stop(m->bus,known_wheels,now);
            return;
        }
        /* During power-on discovery fields arrive separately. Missing position
           is not evidence of wheel motion; wait for speed/status rather than
           cancelling their queries with repeated STOP requests. */
        if (!w->data.online || !MotorBus_Fresh(w,MOTOR_VALID_SPEED,now) ||
            !MotorBus_Fresh(w,MOTOR_VALID_STATUS,now) || (w->data.status_flags & 12u) ||
            w->action==MOTOR_UNKNOWN) return;
    }
    if (a!=MECH_TRIM && (!healthy(node(m,0u),now,0u) || !healthy(node(m,1u),now,0u))) return;
    if (m->enable_wait) {
        if (n->request_id!=m->segment_request || n->action==MOTOR_REJECTED || n->action==MOTOR_UNKNOWN || n->stop_unconfirmed) {
            fail(m,MECH_ENABLE_FAILED,now); return;
        }
        if (n->action!=MOTOR_COMPLETE) return;
        m->enable_wait=0u; m->stable=m->waiting_position=0u;
    }
    if (!healthy(n,now,0u)) return;
    if (busy_node(n) || n->data.speed_rpm) { fail(m,MECH_AXIS_BUSY,now); return; }
#if MOTOR_DRIVER_REQUIRE_MANUAL
    MotorBus_SetManual(m->bus,1u,now);
#endif
    if (!(n->data.status_flags & 1u)) {
        if (MotorBus_Enable(m->bus,(uint8_t)(a+5u),1u,now)) {
            m->enable_wait=1u; m->segment_request=n->request_id; m->touched|=axis_bit(a);
        }
        return;
    }
    if (!settled(m,n,now) || !new_position_after(m,n,now)) return;
    if (a==MECH_TRIM) {
        next_step(m,MECH_COLLISION_HOME,now);
    } else next_step(m,MECH_CHECK,now);
}
/* User-validated endpoint detection, ONLY during TRIM homing search.
   New position samples and a displacement deadband reject encoder jitter.
   First abort native homing and verify STOP; then retreat 25 mm from contact. */
static int trim_progress(Mechanism *m, const MotorNode *n, uint32_t now)
{
    double travel;
    if (!n->motion) { m->progress_request=0u; return 1; }
    if (m->progress_request!=n->request_id) {
        m->progress_request=n->request_id; m->progress_ms=n->action_ms;
        m->progress_counts=n->data.position_counts; m->progress_seq=n->data.position_seq;
        m->progress_retreating=0u;
    }
    if (n->data.position_seq==m->progress_seq || !due(n->data.position_ms,n->action_ms+1u)) return 1;
    m->progress_seq=n->data.position_seq;
    travel=(double)(n->data.position_counts-m->progress_counts)*m->config.axis[MECH_TRIM].units_per_rev/65536.0;
    /* Once native backoff has been observed, a later jam is not the search endpoint. */
    if (travel*m->config.axis[MECH_TRIM].positive_sign<=-MECHANISM_TRIM_PROGRESS_MM) m->progress_retreating=1u;
    if (m->progress_retreating) return 1;
    if (fabs(travel)>=MECHANISM_TRIM_PROGRESS_MM) {
        m->progress_counts=n->data.position_counts; m->progress_ms=now;
    } else if (now-m->progress_ms>=MECHANISM_TRIM_NO_PROGRESS_MS) {
        MotorBus_Stop(m->bus,axis_bit(MECH_TRIM),now);
        m->data.state=MECH_CONTACT_STOP; m->data.stop_pending=1u;
        m->data.stop_request_ms=now; m->stop_at=now;
        m->deadline=now+3000u; m->waiting_position=m->stable=0u;
        return 0;
    }
    return 1;
}
/* Native homing owns collision success detection. */
static void trim_home_process(Mechanism *m, uint32_t now)
{
    const MechanismTrimHomeConfig *h=&m->config.trim_home;
    const MechanismAxisConfig *c=&m->config.axis[MECH_TRIM];
    MotorNode *n=node(m,MECH_TRIM); EmmCommand command; MechanismReason why; uint8_t noop;
    double delta= -(h->travel_mm/2.0-h->backoff_tenths/3600.0*c->units_per_rev);
    if (m->data.state==MECH_CONTACT_STOP) {
        if (n->stop_sent) m->data.stop_submit_ms=n->stop_first_ms;
        if (due(now,m->deadline)) { fail(m,MECH_STOP_UNCONFIRMED,now); return; }
        if (n->stop_unconfirmed) return;
        if (!healthy(n,now,1u)) { fail(m,motor_reason(n),now); return; }
        if (!settled(m,n,now) || !new_position_after(m,n,now)) return;
        m->data.stop_confirm_ms=now; m->data.stop_pending=0u;
        /* Native backoff was aborted: full half-travel, NOT the native 24 mm remainder. */
        why=submit(m,MECH_TRIM,-h->travel_mm/2.0,h->rpm*c->units_per_rev/60.0,now,&noop);
        if (why!=MECH_OK || noop) { fail(m,why!=MECH_OK ? why : MECH_RESOLUTION,now); return; }
        m->data.target=0.0; m->data.state=MECH_CENTERING; m->waiting_position=0u;
        return;
    }
    if (due(now,m->deadline)) { fail(m,MECH_TIMEOUT,now); return; }
    if (m->segment) {
        if (n->request_id!=m->segment_request || n->stop_unconfirmed || n->action==MOTOR_CANCELLED) {
            fail(m,MECH_BUS_REJECTED,now); return;
        }
        if (n->action!=MOTOR_COMPLETE) return;
    }
    if (m->data.state==MECH_CENTERING) {
        if (!settled(m,n,now) || !new_position_after(m,n,now)) return;
        if (fabs((double)n->data.position_counts*c->positive_sign*c->units_per_rev/65536.0-
            m->segment_target)>c->tolerance) { fail(m,MECH_FINAL_ERROR,now); return; }
        m->data.origin_counts[MECH_TRIM]=n->data.position_counts;
        m->data.reference[MECH_TRIM]=MECH_HOMED; m->data.trim_ready=1u; complete(m); return;
    }
    if (m->segment==3u) {
        if (!settled(m,n,now) || !new_position_after(m,n,now)) return;
        why=submit(m,MECH_TRIM,delta,h->rpm*c->units_per_rev/60.0,now,&noop);
        if (why!=MECH_OK || noop) { fail(m,why!=MECH_OK ? why : MECH_RESOLUTION,now); return; }
        m->data.target=0.0; m->data.state=MECH_CENTERING; m->waiting_position=0u; return;
    }
    command=m->segment==0u ? Emm_HomeConfig(7u,(uint8_t)(c->positive_sign<0),h->rpm,
        h->timeout_ms,h->stall_rpm,h->stall_ma,h->stall_ms) :
        m->segment==1u ? Emm_HomeBackoff(7u,h->backoff_tenths) : Emm_Home(7u);
    if (!MotorBus_HomeCommand(m->bus,&command,h->timeout_ms,now)) { fail(m,MECH_BUS_REJECTED,now); return; }
    m->segment_request=n->request_id; m->data.bus_request_id=n->request_id;
    m->touched|=axis_bit(MECH_TRIM); ++m->segment;
    m->deadline=now+(m->segment==3u ? h->timeout_ms : m->config.prepare_ms)+1000u;
}
static void home_process(Mechanism *m, uint32_t now)
{
    unsigned a=m->data.axis; MotorNode *n=node(m,a);
    MechanismLimit *l=&m->data.limit[a]; const MechanismHomeConfig *h=&m->config.home[a];
    MechanismState state=m->data.state; MechanismReason why;
    double delta=0.0,rpm=h->slow_rpm; uint8_t noop;
    int seeking=(state==MECH_FAST_SEEK || state==MECH_SLOW_SEEK);
    if (state==MECH_CHECK) {
        if (l->valid) next_step(m,l->stable ? MECH_INITIAL_RELEASE : MECH_FAST_SEEK,now);
        return;
    }
    if (due(now,m->deadline)) { fail(m,MECH_TIMEOUT,now); return; }
    if (m->segment==2u) {
        if (m->triggered && now-m->trigger_at>=200u && (!l->valid || !l->stable)) {
            fail(m,MECH_LIMIT_UNSTABLE,now); return;
        }
        if (n->stop_sent) m->data.stop_submit_ms=n->stop_first_ms;
        if (n->stop_unconfirmed) {
            if (now-m->stop_at>=3000u) fail(m,MECH_STOP_UNCONFIRMED,now);
            return;
        }
        if (!healthy(n,now,1u) || n->data.speed_rpm) { fail(m,MECH_FEEDBACK,now); return; }
        if (!m->waiting_position) m->data.stop_confirm_ms=now;
        if (!new_position_after(m,n,now) || !l->valid || (seeking ? !l->stable : l->stable)) return;
        m->data.stop_pending=0u;
        if (state==MECH_SLOW_SEEK) m->data.approach_counts[a]=n->data.position_counts;
        next_step(m,state==MECH_INITIAL_RELEASE ? MECH_FAST_SEEK :
            state==MECH_FAST_SEEK ? MECH_BACKOFF : MECH_PARK_RELEASE,now);
        return;
    }
    if (m->segment==3u) {
        if (l->valid && !l->stable && settled(m,n,now) && new_position_after(m,n,now)) home_axis_done(m,now);
        return;
    }
    if ((m->segment<=1u && seeking && l->raw) ||
        (m->segment==1u && state==MECH_INITIAL_RELEASE && l->valid && !l->stable)) {
        m->triggered=(uint8_t)seeking; m->trigger_at=now; m->stop_at=now;
        if (seeking) m->data.trigger_ms=now;
        m->data.stop_request_ms=now; m->data.stop_pending=1u;
        m->touched|=axis_bit(a);
        MotorBus_Stop(m->bus,axis_bit(a),now); m->segment=2u; m->waiting_position=0u;
        return;
    }
    if (!m->segment) {
        if (state==MECH_INITIAL_RELEASE) delta=h->initial_release;
        else if (state==MECH_FAST_SEEK) { delta= -h->fast_seek; rpm=h->fast_rpm; }
        else if (state==MECH_BACKOFF) delta=h->backoff;
        else if (state==MECH_SLOW_SEEK) delta= -h->slow_seek;
        else if (state==MECH_PARK_RELEASE) delta=h->clearance;
        why=submit(m,a,delta,rpm*m->config.axis[a].units_per_rev/60.0,now,&noop);
        if (why!=MECH_OK || noop) fail(m,why!=MECH_OK ? why : MECH_RESOLUTION,now);
        return;
    }
    if (n->request_id!=m->segment_request || n->stop_unconfirmed || n->action==MOTOR_CANCELLED) {
        fail(m,MECH_BUS_REJECTED,now); return;
    }
    if (n->action!=MOTOR_COMPLETE || !settled(m,n,now)) return;
    if (seeking) { fail(m,MECH_NOT_FOUND,now); return; }
    if (state==MECH_INITIAL_RELEASE || !l->valid || l->stable) { fail(m,MECH_NOT_RELEASED,now); return; }
    if (fabs((double)n->data.position_counts*m->config.axis[a].positive_sign*
        m->config.axis[a].units_per_rev/65536.0-m->segment_target)>m->config.axis[a].tolerance) {
        fail(m,MECH_FINAL_ERROR,now); return;
    }
    if (state==MECH_BACKOFF) next_step(m,MECH_SLOW_SEEK,now);
    else { m->segment=3u; m->waiting_position=0u; }
}
static void test_next(Mechanism *m, uint32_t now)
{
    static const uint8_t axes[8]={0u,1u,1u,2u,2u,2u,2u,0u};
    unsigned stage=m->data.stage,a; double target; MechanismReason why; uint8_t noop=0u;
    if (stage>=8u) { complete(m); return; }
    a=axes[stage]; m->data.axis=(uint8_t)a;
    target=stage==0u ? m->config.outside_angle : stage==1u ? m->config.test_down_mm :
        stage==3u ? m->trim_start+m->config.test_trim_mm : stage==5u ? m->trim_start-m->config.test_trim_mm :
        (stage>=3u && stage<=6u) ? m->trim_start : 0.0;
    why=interlock(m,a,target,now);
    if (why==MECH_OK && !target_in_range(m,a,target)) why=MECH_LIMIT;
    if (why==MECH_OK) why=submit(m,a,target-position(m,a),m->config.axis[a].move_speed,now,&noop);
    if (why!=MECH_OK) { fail(m,why,now); return; }
    m->data.target=target; m->data.state=MECH_MOVING;
    if (noop) { ++m->data.stage; next_step(m,MECH_BETWEEN,now); m->deadline=now+m->config.between_ms; }
}
void Mechanism_Process(Mechanism *m, uint32_t now)
{
    unsigned a; MotorNode *n;
    for (a=0u;a<3u;++a) {
        if (m->data.reference[a]) {
            if (!feedback_healthy(node(m,a),now,1u)) {
                m->data.reference[a]=MECH_UNREFERENCED;
                if (m->owned && m->data.state!=MECH_STOPPING) fail(m,motor_reason(node(m,a)),now);
            }
        }
        m->data.position[a]=position(m,a);
    }
    m->data.trim_ready=(uint8_t)(m->data.reference[2]==MECH_HOMED);
    if (m->auto_pending && m->data.limit[0].valid && m->data.limit[1].valid) {
        m->auto_pending=0u; (void)Mechanism_HomeAll(m,now);
    }
    if (m->owned && m->data.state!=MECH_STOPPING && now-m->last_sample_ms>10u) fail(m,MECH_SAMPLE_GAP,now);
    /* Unexpected limit outside explicit homing aborts even while parked. */
    for (a=0u;a<2u;++a) if (m->data.limit[a].raw && m->data.reference[a] &&
        !(m->operation==OP_HOME && m->data.axis==a)) {
        m->data.axis=(uint8_t)a; m->touched|=axis_bit(a); m->owned=1u; fail(m,MECH_LIMIT,now);
    }
    if (!m->owned) return;
    if (m->data.state==MECH_STOPPING) {
        m->data.stop_pending=0u;
        for (a=0u;a<3u;++a) if (m->stop_mask & axis_bit(a)) {
            n=node(m,a);
            if (n->stop_unconfirmed || !n->data.online || !MotorBus_Fresh(n,MOTOR_VALID_SPEED,now) ||
                !MotorBus_Fresh(n,MOTOR_VALID_STATUS,now) || (n->data.status_flags & 12u) ||
                busy_node(n) || n->data.speed_rpm) m->data.stop_pending=1u;
        }
        if (!m->data.stop_pending) { m->owned=0u; m->data.state=m->data.result==MECH_FAILED ? MECH_FAULT : MECH_IDLE; }
        else if (now-m->stop_at>=3000u && m->data.reason==MECH_OK) m->data.reason=MECH_STOP_UNCONFIRMED;
        return;
    }
    if (m->data.state==MECH_PREPARE) { prepare(m,now); return; }
    a=m->data.axis; n=node(m,a);
    if (m->operation==OP_HOME && a==MECH_TRIM && m->data.state==MECH_CONTACT_STOP) {
        trim_home_process(m,now); return;
    }
    if (a==MECH_ARM && interlock(m,a,m->data.target,now)!=MECH_OK) { fail(m,MECH_INTERLOCK,now); return; }
    if (!healthy(n,now,1u) && !(m->operation==OP_HOME && m->segment==2u && n->stop_unconfirmed)) {
        fail(m,motor_reason(n),now); return;
    }
    if (m->operation==OP_HOME && a==MECH_TRIM && m->data.state==MECH_COLLISION_HOME &&
        m->segment==3u && !trim_progress(m,n,now)) return;
    if (m->operation==OP_HOME) {
        if (a==MECH_TRIM) trim_home_process(m,now); else home_process(m,now);
        return;
    }
    if (m->data.state==MECH_BETWEEN) { if (due(now,m->deadline)) test_next(m,now); return; }
    if (due(now,m->deadline)) { fail(m,MECH_TIMEOUT,now); return; }
    if (n->request_id!=m->segment_request || n->stop_unconfirmed || n->action==MOTOR_CANCELLED) {
        fail(m,MECH_BUS_REJECTED,now); return;
    }
    if (n->motion && n->target_known) m->data.quantized_target=n->target_counts;
    if (n->action!=MOTOR_COMPLETE || !settled(m,n,now) || n->data.position_seq==m->position_seq) return;
    if (fabs(position(m,a)-m->data.target)>m->config.axis[a].tolerance) { fail(m,MECH_FINAL_ERROR,now); return; }
    if (m->operation==OP_TEST) { ++m->data.stage; next_step(m,MECH_BETWEEN,now); m->deadline=now+m->config.between_ms; }
    else complete(m);
}
void Mechanism_GetSnapshot(const Mechanism *m, MechanismSnapshot *out) { if (out) *out=m->data; }
const char *Mechanism_StateName(MechanismState s)
{
    static const char * const names[]={"IDLE","PREPARE","CHECK","INITIAL RELEASE","FAST SEEK","BACKOFF",
        "SLOW SEEK","PARK RELEASE","MOVING","BETWEEN","STOPPING","FAULT","COLLISION HOME","CENTERING","CONTACT STOP"};
    return (unsigned)s<sizeof(names)/sizeof(names[0]) ? names[s] : "?";
}
const char *Mechanism_ReasonName(MechanismReason r)
{
    static const char * const names[]={"OK","CONFIG REQUIRED","FEEDBACK","AXIS BUSY","POSE INTERLOCK",
        "ENABLE FAILED","ACTION TIMEOUT","BUS REJECTED","LIMIT","LIMIT UNSTABLE","LIMIT SAMPLE GAP",
        "LIMIT NOT FOUND","LIMIT NOT RELEASED","FINAL ERROR","RESOLUTION","STOP UNCONFIRMED","BAD REQUEST","NEED HOME","MOTOR STALL/PROTECTION"};
    return (unsigned)r<sizeof(names)/sizeof(names[0]) ? names[r] : "?";
}
