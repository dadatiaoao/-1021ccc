#include "vision_ui.h"
#include <stdio.h>
#include <string.h>
void VisionUi_Line(char out[43],unsigned row,const VisionService *s,uint8_t mode,
                   uint8_t color,uint8_t zone,uint8_t position,uint32_t now)
{
    const VisionObservation *o=&s->observation;
    static const char *modes[]={"QR","COLOR","STATION","PICK","GEOMETRY"};
    switch (row) {
    case 0u: strcpy(out,"VISION / TAP TITLE: EXIT"); break;
    case 1u: snprintf(out,43,"HB:%s %s M%u ACK:%lu",s->online ? "ON" : "OFF",
               VisionService_StateName(s),(unsigned)s->mode,(unsigned long)s->ack_count); break;
    case 2u: snprintf(out,43,"NEXT M%u:%s [tap, then SCAN]",(unsigned)mode,modes[mode]); break;
    case 3u: snprintf(out,43,"NEXT COLOR:%u Z:%u POS:%u [tap thirds]",(unsigned)color,
               (unsigned)zone,(unsigned)position); break;
    case 4u: snprintf(out,43,"REQ:%u POSE:%u FRAME:%u",(unsigned)s->request_id,
               (unsigned)s->pose_id,(unsigned)o->frame_id); break;
    case 5u:
        if (!s->mode && s->task_valid) snprintf(out,43,"QR:%s",s->task.text);
        else if (VisionService_TargetFresh(s,now)) snprintf(out,43,"ID:%u X:%u Y:%u FLAGS:%02X",
            (unsigned)o->target_id,(unsigned)o->cx,(unsigned)o->cy,(unsigned)o->flags);
        else strcpy(out,"TARGET: -- (no fresh tagged result)");
        break;
    case 6u: snprintf(out,43,"ERR:%s BAD:%lu",VisionService_ErrorName(s),
        (unsigned long)(s->invalid_frames+s->parser.bad_checksums+s->parser.bad_lengths)); break;
    case 7u: snprintf(out,43,"RX:%lu UART:%lu LOST:%lu",(unsigned long)s->raw_bytes,
        (unsigned long)s->transport_errors,(unsigned long)s->ignored_frames); break;
    default: strcpy(out,"STATION/PICK: candidate, verify on bench"); break;
    }
}
