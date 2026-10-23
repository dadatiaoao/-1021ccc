#ifndef FLOW_SERVICE_H
#define FLOW_SERVICE_H
/*
 * 光流对外数据契约：raw 为器件原始突发，dx/dy 已按配置映射轴向，total_x/y 为累计 counts。
 * comm_ok 只表示通信，tracking 表示样本质量，fresh 表示时效，valid 综合可用性；
 * 快照时间字段单位 ms，max_process_us 单位 us。
 */
#include "pmw3901.h"
typedef enum { FLOW_INIT, FLOW_READY, FLOW_ERROR } FlowState;
typedef enum { FLOW_OK, FLOW_SPI, FLOW_ID, FLOW_BUS_DATA, FLOW_LOW_QUALITY,
    FLOW_STATUS, FLOW_GAP, FLOW_NO_VALID, FLOW_OVERFLOW } FlowError;
typedef struct {
    FlowState state; FlowError error, last_fault, last_rejection;
    PmwMotion raw;
    int32_t dx, dy;
    int64_t total_x, total_y;
    uint32_t sequence, accepted, rejected, errors, missed, gaps, segments, clears, zero_sequence;
    uint32_t sample_ms, interval_ms, max_interval_ms, age_ms, max_process_us;
    uint32_t last_rejected_ms;
    uint8_t last_rejected_quality;
    uint8_t product, inverse, revision, comm_ok, tracking, fresh, valid, have_sample, overflow;
} FlowSnapshot;
typedef struct {
    FlowIo io; Pmw3901 chip; FlowSnapshot data;
    uint32_t due, last_attempt, last_id, last_valid, retry_ms;
} FlowService;
void Flow_Init(FlowService *s, FlowIo io, uint32_t now);
void Flow_Process(FlowService *s, uint32_t now);
void Flow_GetSnapshot(const FlowService *s, uint32_t now, FlowSnapshot *out);
void Flow_ResetTotals(FlowService *s);
const char *Flow_StateName(const FlowSnapshot *d);
const char *Flow_ErrorName(FlowError error);
/* Decimal conversion supports INT64_MIN and AC5's 32-bit long. */
void Flow_FormatCount(char out[21], int64_t value);
#endif
