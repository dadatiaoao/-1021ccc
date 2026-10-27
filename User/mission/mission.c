#include "mission.h"
#include <string.h>

static void fail(Mission *m, MissionError error, uint32_t now)
{
    m->state=MISSION_FAULT; m->error=error; m->active=0u;
    if (m->port.stop) m->port.stop(m->port.context,now);
}
void Mission_Init(Mission *m, MissionPort port)
{ memset(m,0,sizeof(*m)); m->port=port; }
int Mission_IsRunning(const Mission *m) { return m && m->state==MISSION_RUNNING; }
int Mission_Start(Mission *m, uint32_t now)
{
    if (m->started || m->state!=MISSION_IDLE) return 0;
    m->blocked=m->port.blocked ? m->port.blocked(m->port.context,now) : MISSION_BLOCK_MECHANISM;
    if (m->blocked || !m->port.begin || !m->port.poll || !m->port.stop) {
        m->error=MISSION_NOT_READY; return 0;
    }
    m->started=1u; m->state=MISSION_RUNNING; m->error=MISSION_NO_ERROR;
    m->started_ms=now; m->action.step=MISSION_NAV_QR;
    return 1;
}
void Mission_Stop(Mission *m, uint32_t now)
{
    if (!Mission_IsRunning(m)) return;
    m->state=MISSION_ABORTED; m->error=MISSION_USER_STOP; m->active=0u;
    m->port.stop(m->port.context,now);
}
static int describe(Mission *m)
{
    MissionAction *a=&m->action;
    unsigned i=a->order, b=a->batch;
    a->color=a->slot=a->position=a->layer=0u;
    if (a->step!=MISSION_SOURCE_TO_SLOT && a->step!=MISSION_SLOT_TO_ROUGH &&
        a->step!=MISSION_ROUGH_TO_SLOT && a->step!=MISSION_SLOT_TO_STORAGE) return 1;
    if (!m->task_valid || b>1u || i>2u) return 0;
    a->color=m->task.colors[b][i]; a->slot=(uint8_t)(i+1u);
    a->position=m->task.positions[b][i];
    if (a->step==MISSION_SLOT_TO_STORAGE) {
        a->position=m->base_position[a->color]; a->layer=(uint8_t)(b+1u);
        if (!a->position || a->position>3u ||
            (b==0u ? m->storage[a->position-1u]!=0u : m->storage[a->position-1u]!=a->color)) return 0;
    }
    if (a->step==MISSION_SOURCE_TO_SLOT) return m->slots[i]==0u;
    if (a->step==MISSION_ROUGH_TO_SLOT)
        return !m->slots[i] && m->rough[a->position-1u]==a->color;
    if (m->slots[i]!=a->color) return 0;
    return a->step!=MISSION_SLOT_TO_ROUGH || m->rough[a->position-1u]==0u;
}
static int accept_task(Mission *m, const QrTaskCode *task)
{
    QrTaskCode checked;
    unsigned i;
    if (QrTaskCode_Parse((const uint8_t *)task->text,QR_TASK_LENGTH,&checked)!=QR_CODE_OK) return 0;
    m->task=checked; m->task_valid=1u;
    for (i=0u;i<3u;++i) m->base_position[checked.colors[0][i]]=checked.positions[0][i];
    return 1;
}
static void next(Mission *m)
{
    MissionAction *a=&m->action;
    unsigned i=a->order, p=a->position ? a->position-1u : 0u;
    switch (a->step) {
    case MISSION_NAV_QR: a->step=MISSION_READ_QR; break;
    case MISSION_READ_QR: a->step=MISSION_NAV_SOURCE; break;
    case MISSION_NAV_SOURCE: a->step=MISSION_SOURCE_TO_SLOT; a->order=0u; break;
    case MISSION_SOURCE_TO_SLOT:
        m->slots[i]=a->color;
        if (++a->order==3u) { a->order=0u; a->step=MISSION_NAV_ROUGH; } break;
    case MISSION_NAV_ROUGH: a->step=MISSION_SLOT_TO_ROUGH; a->order=0u; break;
    case MISSION_SLOT_TO_ROUGH:
        m->slots[i]=0u; m->rough[p]=a->color;
        /* This barrier is reached only after ALL THREE put operations complete. */
        if (++a->order==3u) { a->order=0u; a->step=MISSION_ROUGH_TO_SLOT; } break;
    case MISSION_ROUGH_TO_SLOT:
        m->rough[p]=0u; m->slots[i]=a->color;
        if (++a->order==3u) { a->order=0u; a->step=MISSION_NAV_STORAGE; } break;
    case MISSION_NAV_STORAGE: a->step=MISSION_SLOT_TO_STORAGE; a->order=0u; break;
    case MISSION_SLOT_TO_STORAGE:
        m->slots[i]=0u; m->storage[p]=a->color;
        if (++a->order==3u) {
            a->order=0u;
            if (!a->batch) { a->batch=1u; a->step=MISSION_NAV_SOURCE; }
            else a->step=MISSION_STOW;
        } break;
    case MISSION_STOW: a->step=MISSION_NAV_HOME; break;
    case MISSION_NAV_HOME: m->state=MISSION_FINISHED; break;
    default: break;
    }
}
void Mission_Process(Mission *m, uint32_t now)
{
    MissionActionResult result;
    QrTaskCode task;
    uint32_t timeout;
    if (m->state==MISSION_IDLE) {
        m->blocked=m->port.blocked ? m->port.blocked(m->port.context,now) : MISSION_BLOCK_MECHANISM;
        return;
    }
    if (!Mission_IsRunning(m)) return;
    if (!m->active) {
        if (!describe(m)) { fail(m,MISSION_CARGO_ERROR,now); return; }
        if (!++m->next_id) ++m->next_id;
        m->action.id=m->next_id; m->action_ms=now;
        if (!m->port.begin(m->port.context,&m->action,now)) { fail(m,MISSION_PORT_ERROR,now); return; }
        m->active=1u; return;
    }
    timeout=m->action.step==MISSION_READ_QR ? 15000u : 90000u;
    if ((uint32_t)(now-m->action_ms)>=timeout) { fail(m,MISSION_TIMEOUT,now); return; }
    memset(&task,0,sizeof(task));
    result=m->port.poll(m->port.context,m->action.id,&task,now);
    if (result==MISSION_WAIT) return;
    if (result!=MISSION_SUCCEEDED) { fail(m,MISSION_ACTION_ERROR,now); return; }
    if (m->action.step==MISSION_READ_QR && !accept_task(m,&task)) { fail(m,MISSION_BAD_TASK,now); return; }
    /* Count external picks and external placements, not onboard slot transfers. */
    if (m->action.step==MISSION_SOURCE_TO_SLOT || m->action.step==MISSION_ROUGH_TO_SLOT)
        ++m->grasp_count;
    if (m->action.step==MISSION_SLOT_TO_ROUGH || m->action.step==MISSION_SLOT_TO_STORAGE)
        ++m->place_count;
    ++m->completed_actions; m->active=0u; next(m);
}
const char *Mission_StateName(MissionState s)
{
    static const char *names[]={"WAIT START","RUNNING","FINISHED","ABORTED","FAULT"};
    return (unsigned)s<5u ? names[s] : "UNKNOWN";
}
const char *Mission_StepName(MissionStep s)
{
    static const char *names[]={"GO QR","READ QR","GO SOURCE","SOURCE -> SLOT",
        "GO ROUGH","SLOT -> ROUGH","ROUGH -> SLOT","GO STORAGE",
        "SLOT -> STORAGE","STOW ARM","RETURN ZONE 2"};
    return (unsigned)s<11u ? names[s] : "UNKNOWN";
}
const char *Mission_ErrorName(MissionError e)
{
    static const char *names[]={"NONE","NOT READY","ACTION NOT CONNECTED","BAD TASK CODE",
        "ACTION FAILED","ACTION TIMEOUT","CARGO ORDER ERROR","USER STOP"};
    return (unsigned)e<8u ? names[e] : "UNKNOWN";
}
