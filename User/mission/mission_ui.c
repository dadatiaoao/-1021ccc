#include "mission_ui.h"
#include <stdio.h>
#include <string.h>
void MissionUi_Line(char out[43], unsigned row, const Mission *m, uint32_t now)
{
    switch (row) {
    case 0u: strcpy(out,"RACE ZONE 2 / TAP: EXIT"); break;
    case 1u: snprintf(out,43,"%s%s",Mission_StateName(m->state),
        m->state==MISSION_IDLE ? (m->blocked ? " / NOT READY" : " / READY") : ""); break;
    case 2u: snprintf(out,43,"QR:%s",m->task_valid ? m->task.text : "--"); break;
    case 3u: snprintf(out,43,"B%u ITEM%u %s",(unsigned)(m->action.batch+1u),
        (unsigned)(m->action.order+1u),Mission_StepName(m->action.step)); break;
    case 4u: snprintf(out,43,"COLOR:%u SLOT:%u POS:%u LAYER:%u",m->action.color,
        m->action.slot,m->action.position,m->action.layer); break;
    case 5u: snprintf(out,43,"SLOT:%u%u%u ROUGH:%u%u%u STORE:%u%u%u",
        m->slots[0],m->slots[1],m->slots[2],m->rough[0],m->rough[1],m->rough[2],
        m->storage[0],m->storage[1],m->storage[2]); break;
    case 6u: snprintf(out,43,"INFER GRASP:%lu PLACE:%lu T:%lus",(unsigned long)m->grasp_count,
        (unsigned long)m->place_count,(unsigned long)(m->started ? (now-m->started_ms)/1000u : 0u)); break;
    case 7u:
        if (m->state==MISSION_IDLE && m->blocked) snprintf(out,43,"NEED:%s%s%s%s%s%s",
            m->blocked & MISSION_BLOCK_CHASSIS ? " BASE" : "",
            m->blocked & MISSION_BLOCK_ROUTES ? " ROUTE" : "",
            m->blocked & MISSION_BLOCK_MECHANISM ? " ARM" : "",
            m->blocked & MISSION_BLOCK_VISION ? " VIS" : "",
            m->blocked & MISSION_BLOCK_FEEDBACK ? " SENSOR" : "",
            m->blocked & MISSION_BLOCK_BUSY ? " BUSY" : "");
        else snprintf(out,43,"ERR:%s",Mission_ErrorName(m->error)); break;
    default: snprintf(out,43,"DONE:%lu REQ:%lu (counts inferred)",
        (unsigned long)m->completed_actions,(unsigned long)m->action.id); break;
    }
}
