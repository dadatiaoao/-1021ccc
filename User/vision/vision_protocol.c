#include "vision_protocol.h"
#include <string.h>
void VisionParser_Init(VisionParser *p) { memset(p,0,sizeof(*p)); }
void VisionParser_Reset(VisionParser *p) { p->used=0u; }
void VisionParser_Expire(VisionParser *p, uint32_t now)
{
    if (p->used && (uint32_t)(now-p->last_byte_ms)>=VISION_PARTIAL_MS) {
        p->used=0u; ++p->partial_timeouts;
    }
}
static void drop(VisionParser *p, unsigned n)
{
    p->used=(uint8_t)(p->used-n);
    memmove(p->bytes,p->bytes+n,p->used);
}
int VisionParser_Next(VisionParser *p,VisionFrame *out)
{
    unsigned length,total,i;
    uint8_t sum;
    while (p->used) {
        if (p->bytes[0]!=0xAAu) { drop(p,1u); continue; }
        if (p->used<2u) break;
        length=p->bytes[1];
        if (length<1u || length>13u) { ++p->bad_lengths; drop(p,1u); continue; }
        total=length+3u;
        if (p->used<total) break;
        sum=0u;
        for (i=2u;i<2u+length;++i) sum=(uint8_t)(sum+p->bytes[i]);
        if (sum!=p->bytes[total-1u]) { ++p->bad_checksums; drop(p,1u); continue; }
        out->command=p->bytes[2]; out->count=(uint8_t)(length-1u);
        memcpy(out->params,p->bytes+3u,out->count);
        drop(p,total); ++p->frames; return 1;
    }
    return 0;
}
int VisionParser_Push(VisionParser *p,uint8_t byte,uint32_t ms,VisionFrame *out)
{
    VisionParser_Expire(p,ms); p->last_byte_ms=ms;
    p->bytes[p->used++]=byte;
    return VisionParser_Next(p,out);
}
uint8_t VisionProtocol_Build(uint8_t cmd,const uint8_t *params,uint8_t n,uint8_t out[VISION_FRAME_MAX])
{
    unsigned i;
    uint8_t sum=cmd;
    if (!out || n>VISION_PARAM_MAX || (n && !params)) return 0u;
    out[0]=0xAAu; out[1]=(uint8_t)(n+1u); out[2]=cmd;
    for (i=0u;i<n;++i) { out[3u+i]=params[i]; sum=(uint8_t)(sum+params[i]); }
    out[3u+n]=sum; return (uint8_t)(n+4u);
}
uint16_t VisionProtocol_U16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8)|p[1]); }
int16_t VisionProtocol_I16(const uint8_t *p)
{
    uint16_t value=VisionProtocol_U16(p);
    return value<=32767u ? (int16_t)value : (int16_t)((int32_t)value-65536);
}
void VisionProtocol_PutU16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
