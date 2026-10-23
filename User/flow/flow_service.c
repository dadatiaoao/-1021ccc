/*
 * 光流服务：分阶段初始化 PMW3901，按周期读取运动突发，检查质量/间隔后累计轴向 counts。
 * 通信正常、跟踪有效、数据新鲜是不同状态；尚未换算毫米，也未与陀螺/轮速融合定位。
 */
#include "flow_service.h"
#include "flow_config.h"
#include <string.h>
#include <limits.h>
static int reached(uint32_t now,uint32_t at) { return (int32_t)(now-at)>=0; }
static FlowError pmw_error(int r)
{ return r==PMW_ID_ERROR ? FLOW_ID : r==PMW_DATA_ERROR ? FLOW_BUS_DATA : FLOW_SPI; }
/*
 * 记录故障、清有效性并释放片选，按递增退避安排重新初始化；保留累计值供诊断。
 */
static void fault(FlowService *s, FlowError error, uint32_t now)
{
    if (s->data.state==FLOW_READY) ++s->data.gaps;
    s->data.state=FLOW_ERROR; s->data.error=s->data.last_fault=error;
    s->data.comm_ok=s->data.tracking=s->data.valid=0u; ++s->data.errors;
    s->io.select(s->io.ctx,1u);
    s->due=now+s->retry_ms;
    if (s->retry_ms<FLOW_RETRY_MAX_MS) s->retry_ms*=2u;
}
static void start(FlowService *s,uint32_t now)
{
    if (!s->io.init(s->io.ctx)) { fault(s,FLOW_SPI,now); return; }
    s->io.select(s->io.ctx,1u); Pmw_Begin(&s->chip,now);
    s->data.state=FLOW_INIT; s->data.error=FLOW_OK;
}
/*
 * 绑定可替换的 SPI/时基接口，建立初始状态并发起传感器初始化。
 */
void Flow_Init(FlowService *s,FlowIo io,uint32_t now)
{
    memset(s,0,sizeof(*s)); s->io=io; s->retry_ms=FLOW_RETRY_MIN_MS;
    start(s,now);
}
/*
 * 用饱和检查累计有符号 64 位计数，避免溢出回绕；溢出后由服务锁存无效标志。
 */
static int add_count(int64_t a,int32_t b,int64_t *out)
{
    if (b>0 && a>INT64_MAX-b) { *out=INT64_MAX; return 0; }
    if (b<0 && a<INT64_MIN-b) { *out=INT64_MIN; return 0; }
    *out=a+b; return 1;
}
/*
 * 单步推进初始化/恢复或周期采样；将原始增量按配置交换轴与符号，
 * 只有采样间隔、质量和状态均合法时才累计 counts，长时间无有效数据则重新初始化。
 */
static void process(FlowService *s,uint32_t now)
{
    FlowSnapshot *d=&s->data; int r;
    if (d->state==FLOW_ERROR) { if (reached(now,s->due)) start(s,now); return; }
    if (d->state==FLOW_INIT) {
        r=Pmw_Step(&s->chip,&s->io,now);
        d->product=s->chip.product; d->inverse=s->chip.inverse; d->revision=s->chip.revision;
        if (r==PMW_READY) {
            d->state=FLOW_READY; d->comm_ok=1u; d->error=FLOW_OK; ++d->segments;
            s->last_attempt=s->last_id=s->last_valid=now; s->retry_ms=FLOW_RETRY_MIN_MS;
        } else if (r!=PMW_PENDING) fault(s,pmw_error(r),now);
        return;
    }
    if ((uint32_t)(now-s->last_attempt)<FLOW_SAMPLE_MS) return;
    d->interval_ms=now-s->last_attempt; s->last_attempt=now;
    if (d->interval_ms>d->max_interval_ms) d->max_interval_ms=d->interval_ms;
    if (d->interval_ms>=2u*FLOW_SAMPLE_MS) d->missed+=d->interval_ms/FLOW_SAMPLE_MS-1u;
    if ((uint32_t)(now-s->last_id)>=FLOW_ID_CHECK_MS) {
        r=Pmw_CheckId(&s->chip,&s->io); s->last_id=now;
        d->product=s->chip.product; d->inverse=s->chip.inverse; d->revision=s->chip.revision;
        if (r!=PMW_READY) { fault(s,pmw_error(r),now); return; }
    }
    r=Pmw_Burst(&s->io,&d->raw);
    if (r!=PMW_READY) {
        ++d->rejected; d->last_rejection=pmw_error(r);
        d->last_rejected_ms=now; d->last_rejected_quality=d->raw.squal;
        fault(s,pmw_error(r),now); return;
    }
    ++d->sequence; d->have_sample=1u; d->sample_ms=now; d->comm_ok=1u;
    d->dx=FLOW_X_SIGN*(int32_t)(FLOW_SWAP_XY ? d->raw.dy : d->raw.dx);
    d->dy=FLOW_Y_SIGN*(int32_t)(FLOW_SWAP_XY ? d->raw.dx : d->raw.dy);
    d->error=FLOW_OK;
    if (d->interval_ms>FLOW_STALE_MS) { d->error=FLOW_GAP; ++d->gaps; }
    else if (d->raw.squal<FLOW_MIN_SQUAL) d->error=FLOW_LOW_QUALITY;
    else if (d->raw.raw_min>d->raw.raw_max ||
        (!(d->raw.motion&0x80u) && (d->raw.dx!=0 || d->raw.dy!=0))) d->error=FLOW_STATUS;
    d->tracking=(uint8_t)(d->error==FLOW_OK);
    if (d->tracking && !d->overflow) {
        int x=add_count(d->total_x,d->dx,&d->total_x);
        int y=add_count(d->total_y,d->dy,&d->total_y);
        if (!x || !y) d->overflow=1u;
        else ++d->accepted;
    }
    if (d->overflow) d->error=FLOW_OVERFLOW;
    d->valid=(uint8_t)(d->tracking && !d->overflow);
    if (d->valid) s->last_valid=now;
    else {
        ++d->rejected; d->last_rejection=d->error;
        d->last_rejected_ms=now; d->last_rejected_quality=d->raw.squal;
    }
    if ((uint32_t)(now-s->last_valid)>=FLOW_INVALID_RESTART_MS) fault(s,FLOW_NO_VALID,now);
}
/*
 * 主循环调用入口：执行一次服务步骤，并用共享周期计数器记录最大处理耗时。
 */
void Flow_Process(FlowService *s,uint32_t now)
{
    uint32_t began=s->io.cycles(s->io.ctx), cost;
    process(s,now);
    cost=(uint32_t)(s->io.cycles(s->io.ctx)-began)/s->io.cycles_per_us;
    if (cost>s->data.max_process_us) s->data.max_process_us=cost;
}
/*
 * 复制当前状态并按读取时刻计算 age/fresh/valid；不会修改内部累计或启动硬件事务。
 */
void Flow_GetSnapshot(const FlowService *s,uint32_t now,FlowSnapshot *out)
{
    *out=s->data;
    out->age_ms=out->have_sample ? now-out->sample_ms : UINT32_MAX;
    out->fresh=(uint8_t)(out->have_sample && out->age_ms<=FLOW_STALE_MS);
    out->valid=(uint8_t)(out->valid && out->fresh && out->state==FLOW_READY);
}
/*
 * 只清累计位移与累计溢出锁存，记录清零序号；不重置传感器、采样序号或通信统计。
 */
void Flow_ResetTotals(FlowService *s)
{
    s->data.total_x=s->data.total_y=0; s->data.overflow=0u;
    s->data.zero_sequence=s->data.sequence; ++s->data.clears;
}
const char *Flow_StateName(const FlowSnapshot *d)
{
    if (d->state==FLOW_INIT) return "INIT";
    if (d->state==FLOW_ERROR) return d->error==FLOW_ID ? "NOT FOUND" : "ERROR";
    if (!d->fresh) return "STALE";
    if (!d->valid) return "INVALID";
    return "READY";
}
const char *Flow_ErrorName(FlowError e)
{
    static const char *names[]={"NONE","SPI/TIMEBASE","BAD ID","BUS 00/FF","LOW QUALITY",
        "BAD STATUS","SAMPLE GAP","NO VALID DATA","TOTAL OVERFLOW"};
    return (unsigned)e<sizeof(names)/sizeof(names[0]) ? names[e] : "UNKNOWN";
}
/*
 * 输出完整 int64 十进制，兼容 AC5 的 32 位 long，并避免对 INT64_MIN 直接取负溢出。
 */
void Flow_FormatCount(char out[21],int64_t value)
{
    char reverse[20]; unsigned n=0u,i=0u;
    uint64_t v=value<0 ? (uint64_t)(-(value+1))+1u : (uint64_t)value;
    do { reverse[n++]=(char)('0'+v%10u); v/=10u; } while (v);
    if (value<0) out[i++]='-';
    while (n) out[i++]=reverse[--n];
    out[i]='\0';
}
