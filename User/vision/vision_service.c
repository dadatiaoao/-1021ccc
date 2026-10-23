#include "vision_service.h"
#include <string.h>
static void revoke(VisionService *s)
{
    memset(&s->observation,0,sizeof(s->observation));
    s->legacy_valid=s->place_valid=0u;
}
static void fault(VisionService *s,VisionError error)
{
    revoke(s); s->task_valid=0u; s->waiting=0u; s->state=VISION_FAULT; s->error=error;
}
void VisionService_Init(VisionService *s,VisionTransport transport,uint32_t now)
{
    memset(s,0,sizeof(*s)); s->transport=transport; s->state=VISION_IDLE;
    s->color=s->zone=s->position=1u; s->queued_ms=now;
}
int VisionService_Start(VisionService *s,uint8_t mode,uint8_t color,uint8_t zone,
                        uint8_t position,uint16_t pose,uint32_t now)
{
    if (mode>4u || color<1u || color>6u || zone<1u || zone>2u || position<1u || position>3u ||
        s->waiting || s->state==VISION_CONFIGURING || s->state==VISION_STOPPING) return 0;
    revoke(s); s->task_valid=0u; s->frame_seen=0u;
    s->mode=mode; s->color=color; s->zone=zone; s->position=position; s->pose_id=pose;
    ++s->request_id; if (!s->request_id) ++s->request_id;
    s->state=VISION_CONFIGURING; s->error=VISION_ERROR_NONE; s->step=0u; s->queued_ms=now;
    return 1;
}
void VisionService_Stop(VisionService *s,uint32_t now)
{
    revoke(s); s->task_valid=0u;
    /* Existing STOP is already on the wire; do not create an ambiguous duplicate. */
    if (s->waiting && s->pending_cmd==VISION_STOP) { s->state=VISION_STOPPING; return; }
    s->waiting=0u; s->state=VISION_STOPPING; s->queued_ms=now; s->error=VISION_ERROR_NONE;
}
void VisionService_TransportError(VisionService *s,VisionError error,uint32_t now)
{
    (void)now; ++s->transport_errors; VisionParser_Reset(&s->parser);
    s->online=s->heartbeat_seen=0u; fault(s,error);
}
static int xy(uint16_t x,uint16_t y) { return x<1280u && y<720u; }
static int ack(VisionService *s,const VisionFrame *f,uint32_t ms)
{
    if (f->count!=2u || f->params[1]>1u) return 0;
    if (!s->waiting || f->params[0]!=s->pending_cmd || (int32_t)(ms-s->command_ms)<0) {
        ++s->ignored_frames; return 1;
    }
    if ((uint32_t)(ms-s->command_ms)>=VISION_ACK_MS) {
        ++s->timeout_count; fault(s,VISION_ERROR_ACK_TIMEOUT); return 1;
    }
    s->waiting=0u; ++s->ack_count;
    if (f->params[1]) { fault(s,VISION_ERROR_REJECTED); return 1; }
    if (s->state==VISION_STOPPING) s->state=VISION_STOPPED;
    else if (s->state==VISION_CONFIGURING) {
        ++s->step; s->queued_ms=ms;
        if (s->step==5u) s->state=VISION_ACTIVE;
    }
    return 1;
}
static int observation(VisionService *s,const VisionFrame *f,uint32_t ms)
{
    VisionObservation o;
    uint16_t delta;
    if (f->count!=12u) return 0;
    memset(&o,0,sizeof(o));
    o.request_id=VisionProtocol_U16(f->params); o.pose_id=VisionProtocol_U16(f->params+2);
    o.frame_id=VisionProtocol_U16(f->params+4); o.flags=f->params[6]; o.target_id=f->params[7];
    o.cx=VisionProtocol_U16(f->params+8); o.cy=VisionProtocol_U16(f->params+10);
    if (o.flags & VISION_FLAG_FOUND) {
        if (!xy(o.cx,o.cy)) return 0;
    } else if (o.cx || o.cy) return 0;
    delta=(uint16_t)(o.frame_id-s->last_frame_id);
    if (s->state!=VISION_ACTIVE || !s->mode || o.request_id!=s->request_id || o.pose_id!=s->pose_id ||
        o.target_id!=((s->mode==1u || s->mode==3u) ? s->color : s->position) ||
        (s->frame_seen && (!delta || delta>=32768u))) { ++s->ignored_frames; return 1; }
    s->frame_seen=1u; s->last_frame_id=o.frame_id; o.received_ms=ms;
    o.valid=(uint8_t)((o.flags & VISION_FLAG_FOUND) &&
        !(o.flags & (VISION_FLAG_AMBIGUOUS|VISION_FLAG_STALE)));
    /* Mode 4 must be a refined geometry candidate. No coarse fallback. */
    if (s->mode==4u && !(o.flags & VISION_FLAG_REFINED)) o.valid=0u;
    s->observation=o; return 1;
}
static int handle(VisionService *s,const VisionFrame *f,uint32_t ms)
{
    uint16_t x,y;
    unsigned i,j;
    uint8_t text[QR_TASK_LENGTH];
    QrTaskCode task;
    if (f->command==VISION_ACK) return ack(s,f,ms);
    if (f->command==VISION_HEARTBEAT) {
        if (f->count) return 0;
        s->heartbeat_seen=s->online=1u; s->last_heartbeat_ms=ms; return 1;
    }
    if (f->command==VISION_OBSERVATION) return observation(s,f,ms);
    if (f->command==VISION_NOTFOUND) {
        if (f->count) return 0;
        /* Revoke even while changing mode: absence must never preserve an old target. */
        revoke(s); return 1;
    }
    if (f->command==VISION_TASKCODE) {
        if (f->count!=12u) return 0;
        for (i=0u,j=0u;i<12u;++i) {
            if (f->params[i]>9u) return 0;
            if (i && i%3u==0u) text[j++]='+';
            text[j++]=(uint8_t)('0'+f->params[i]);
        }
        if (QrTaskCode_Parse(text,sizeof(text),&task)!=QR_CODE_OK) return 0;
        if (s->state==VISION_ACTIVE && !s->mode && !s->task_valid) {
            s->task=task; s->task_valid=1u; s->task_ms=ms;
        } else ++s->ignored_frames;
        return 1;
    }
    if (f->command==VISION_FOUND) {
        if (f->count!=9u || f->params[0]<1u || f->params[0]>6u) return 0;
        x=VisionProtocol_U16(f->params+1); y=VisionProtocol_U16(f->params+3);
        if (!xy(x,y)) return 0;
        if (s->state==VISION_ACTIVE && (s->mode==1u || s->mode==3u) && f->params[0]==s->color) {
            s->legacy_valid=1u; s->legacy_id=f->params[0]; s->legacy_cx=x; s->legacy_cy=y;
            s->offset_tenths=VisionProtocol_I16(f->params+5); s->distance_cm=VisionProtocol_U16(f->params+7);
            s->legacy_ms=ms;
        } else ++s->ignored_frames;
        return 1;
    }
    if (f->command==VISION_RING) {
        if (f->count!=5u || f->params[0]<1u || f->params[0]>3u) return 0;
        x=VisionProtocol_U16(f->params+1); y=VisionProtocol_U16(f->params+3);
        if (!xy(x,y)) return 0;
        if (s->state==VISION_ACTIVE && (s->mode==2u || s->mode==4u) && f->params[0]==s->position) {
            s->legacy_valid=1u; s->legacy_id=f->params[0]; s->legacy_cx=x; s->legacy_cy=y; s->legacy_ms=ms;
        } else ++s->ignored_frames;
        return 1;
    }
    if (f->command==VISION_PLACE_CHECK) {
        if (f->count!=3u || f->params[0]<1u || f->params[0]>2u ||
            f->params[1]<1u || f->params[1]>3u || f->params[2]>1u) return 0;
        if (s->state==VISION_ACTIVE && s->mode==2u && f->params[0]==s->zone && f->params[1]==s->position) {
            s->place_valid=1u; s->place_candidate=f->params[2]; s->place_ms=ms;
        } else ++s->ignored_frames;
        return 1;
    }
    return 0;
}
void VisionService_Receive(VisionService *s,const uint8_t *b,size_t n,uint32_t ms)
{
    VisionFrame f;
    size_t i;
    if (!b) return;
    for (i=0u;i<n;++i) {
        ++s->raw_bytes;
        if (VisionParser_Push(&s->parser,b[i],ms,&f)) {
            do {
                if (!handle(s,&f,ms)) ++s->invalid_frames;
                else s->last_rx_ms=ms;
            } while (VisionParser_Next(&s->parser,&f));
        }
    }
}
int VisionService_TargetFresh(const VisionService *s,uint32_t now)
{
    return s->state==VISION_ACTIVE && s->online && s->observation.valid &&
        (uint32_t)(now-s->last_heartbeat_ms)<VISION_LINK_MS &&
        (uint32_t)(now-s->observation.received_ms)<VISION_TARGET_MS;
}
void VisionService_Process(VisionService *s,uint32_t now)
{
    uint8_t params[4],cmd,n=0u,bytes[VISION_FRAME_MAX],length;
    VisionParser_Expire(&s->parser,now);
    if (s->legacy_valid && (uint32_t)(now-s->legacy_ms)>=VISION_TARGET_MS) s->legacy_valid=0u;
    if (s->place_valid && (uint32_t)(now-s->place_ms)>=VISION_TARGET_MS) s->place_valid=0u;
    if (s->observation.valid && (uint32_t)(now-s->observation.received_ms)>=VISION_TARGET_MS)
        s->observation.valid=0u;
    if (s->heartbeat_seen && (uint32_t)(now-s->last_heartbeat_ms)>=VISION_LINK_MS) {
        s->online=0u;
        if (s->state==VISION_ACTIVE) fault(s,VISION_ERROR_LINK);
    }
    if (s->waiting) {
        if ((uint32_t)(now-s->command_ms)>=VISION_ACK_MS) {
            ++s->timeout_count; fault(s,VISION_ERROR_ACK_TIMEOUT);
        }
        return; /* No blind retries: ACK has no sequence number. */
    }
    if (s->state!=VISION_CONFIGURING && s->state!=VISION_STOPPING) return;
    if ((uint32_t)(now-s->queued_ms)>=VISION_ACK_MS) { fault(s,VISION_ERROR_TX_BUSY); return; }
    if (!s->transport.send || (s->transport.busy && s->transport.busy(s->transport.context))) return;
    cmd=VISION_STOP;
    if (s->state==VISION_CONFIGURING) switch (s->step) {
    case 1u: cmd=VISION_SET_CONTEXT; n=4u; VisionProtocol_PutU16(params,s->request_id);
             VisionProtocol_PutU16(params+2,s->pose_id); break;
    case 2u: cmd=VISION_SET_COLOR; n=1u; params[0]=s->color; break;
    case 3u: cmd=VISION_SET_TARGET; n=2u; params[0]=s->zone; params[1]=s->position; break;
    case 4u: cmd=VISION_SCAN; n=1u; params[0]=s->mode; break;
    default: break;
    }
    length=VisionProtocol_Build(cmd,params,n,bytes);
    if (s->transport.send(s->transport.context,bytes,length)) {
        s->waiting=1u; s->pending_cmd=cmd; s->command_ms=now;
    }
}
const char *VisionService_StateName(const VisionService *s)
{
    static const char *names[]={"IDLE","CONFIG","ACTIVE","STOPPING","STOPPED","FAULT"};
    return names[s->state];
}
const char *VisionService_ErrorName(const VisionService *s)
{
    static const char *names[]={"NONE","ACK TIMEOUT","REJECTED","UART","OVERFLOW","LINK LOST","TX BUSY"};
    return names[s->error];
}
