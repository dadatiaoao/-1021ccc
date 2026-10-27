/*
 * Emm 协议纯数据层：构造查询/使能/速度/位置/STOP/组帧，以及从任意分片字节流提取应答。
 * 本层不操作 UART、不等待 ACK，也不判定动作完成；调度和反馈时效由服务层负责。
 */
#include "emm_protocol.h"
#include <string.h>

static EmmCommand command(uint8_t address, uint8_t code, uint8_t length)
{
    EmmCommand c;
    memset(&c, 0, sizeof(c));
    c.bytes[0] = address; c.bytes[1] = code;
    c.bytes[length - 1u] = 0x6Bu; c.code = code; c.length = length;
    return c;
}

EmmCommand Emm_Query(uint8_t address, uint8_t code) { return command(address, code, 3u); }
EmmCommand Emm_Enable(uint8_t address, uint8_t enabled)
{
    EmmCommand c = command(address, EMM_ENABLE, 6u);
    c.bytes[2] = 0xABu; c.bytes[3] = enabled ? 1u : 0u; return c;
}
EmmCommand Emm_Velocity(uint8_t address, uint8_t direction, uint16_t rpm, uint8_t acceleration)
{
    EmmCommand c = command(address, EMM_VELOCITY, 8u);
    c.bytes[2] = direction; c.bytes[3] = (uint8_t)(rpm >> 8);
    c.bytes[4] = (uint8_t)rpm; c.bytes[5] = acceleration; return c;
}
EmmCommand Emm_Move(uint8_t address, uint8_t direction, uint16_t rpm,
                    uint8_t acceleration, uint32_t pulses)
{
    EmmCommand c = command(address, EMM_MOVE, 13u);
    c.bytes[2] = direction; c.bytes[3] = (uint8_t)(rpm >> 8);
    c.bytes[4] = (uint8_t)rpm; c.bytes[5] = acceleration;
    c.bytes[6] = (uint8_t)(pulses >> 24); c.bytes[7] = (uint8_t)(pulses >> 16);
    c.bytes[8] = (uint8_t)(pulses >> 8); c.bytes[9] = (uint8_t)pulses;
    return c; /* relative movement, no synchronous cache */
}
/*
 * 构造 Addr FE 98 00 6B 非同步立即停止帧；不发送失能命令。
 */
EmmCommand Emm_Stop(uint8_t address)
{
    EmmCommand c = command(address, EMM_STOP, 5u); c.bytes[2] = 0x98u; return c;
}

/*
 * 带参数校验和同步缓存位的速度帧；非法参数用 length=0 表示不可提交。
 */
EmmCommand Emm_VelocityEx(uint8_t address, uint8_t direction, uint16_t rpm,
                          uint8_t acceleration, uint8_t cached)
{
    EmmCommand c = Emm_Velocity(address, direction, rpm, acceleration);
    if (!address || address > 7u || direction > 1u || rpm > 3000u || cached > 1u) c.length = 0u;
    c.bytes[6] = cached; return c;
}
/*
 * 构造带位置模式和同步缓存位的位置帧，脉冲多字节字段按协议高字节在前编码。
 */
EmmCommand Emm_MoveEx(uint8_t address, uint8_t direction, uint16_t rpm,
    uint8_t acceleration, uint32_t pulses, EmmPositionMode mode, uint8_t cached)
{
    EmmCommand c = Emm_Move(address, direction, rpm, acceleration, pulses);
    if (!address || address > 7u || direction > 1u || rpm > 3000u ||
        (unsigned)mode > 2u || cached > 1u) c.length = 0u;
    c.bytes[10] = (uint8_t)mode; c.bytes[11] = cached; return c;
}
/*
 * 将不重复地址、同类且非同步的速度/位置子帧打包为 0xAA 多机帧，校验长度和参数。
 */
int Emm_Multi(EmmCommand *out, const EmmCommand *m, uint8_t count)
{
    EmmCommand c;
    unsigned i, length = 5u, seen = 0u;
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!m || !count || count > 7u) return 0;
    c = command(0u, EMM_MULTI, 5u);
    for (i = 0u; i < count; ++i) {
        unsigned a = m[i].bytes[0];
        if (!a || a > 7u || (seen & (1u << a)) || m[i].code != m[0].code ||
            (m[i].code != EMM_VELOCITY && m[i].code != EMM_MOVE) ||
            m[i].bytes[1] != m[i].code || m[i].bytes[2] > 1u ||
            (((unsigned)m[i].bytes[3] << 8) | m[i].bytes[4]) > 3000u ||
            (m[i].code == EMM_MOVE && m[i].bytes[10] > 2u) ||
            m[i].length != (m[i].code == EMM_MOVE ? 13u : 8u) ||
            m[i].bytes[m[i].length - 1u] != 0x6Bu ||
            m[i].bytes[m[i].length - 2u] != 0u ||
            length + m[i].length > sizeof(c.bytes)) return 0;
        seen |= 1u << a;
        memcpy(c.bytes + length - 1u, m[i].bytes, m[i].length);
        length += m[i].length;
    }
    c.bytes[2] = (uint8_t)(length >> 8); c.bytes[3] = (uint8_t)length;
    c.bytes[length - 1u] = 0x6Bu; c.length = (uint8_t)length;
    *out = c; return 1;
}
/*
 * 按固定地址 1～4 构造四轮速度组帧；RPM 符号转为协议方向位，幅值转为转速。
 */
int Emm_Wheels(EmmCommand *out, const int32_t rpm[4], uint8_t acceleration)
{
    EmmCommand m[4];
    unsigned i;
    if (!out || !rpm) return 0;
    memset(out, 0, sizeof(*out));
    for (i = 0u; i < 4u; ++i) {
        if (rpm[i] < -3000 || rpm[i] > 3000) return 0;
        m[i] = Emm_VelocityEx((uint8_t)(i + 1u), (uint8_t)(rpm[i] < 0),
            (uint16_t)(rpm[i] < 0 ? -rpm[i] : rpm[i]), acceleration, 0u);
    }
    return Emm_Multi(out, m, 4u);
}
EmmCommand Emm_SyncTrigger(void)
{ EmmCommand c = command(0u, EMM_SYNC, 4u); c.bytes[2] = 0x66u; return c; }

EmmCommand Emm_HomeConfig(uint8_t address, uint8_t direction, uint16_t rpm,
    uint32_t timeout_ms, uint16_t stall_rpm, uint16_t stall_ma, uint16_t stall_ms)
{
    EmmCommand c=command(address,EMM_HOME_CONFIG,20u);
    c.bytes[2]=0xAEu; c.bytes[4]=2u; c.bytes[5]=direction;
    c.bytes[6]=(uint8_t)(rpm>>8); c.bytes[7]=(uint8_t)rpm;
    c.bytes[8]=(uint8_t)(timeout_ms>>24); c.bytes[9]=(uint8_t)(timeout_ms>>16);
    c.bytes[10]=(uint8_t)(timeout_ms>>8); c.bytes[11]=(uint8_t)timeout_ms;
    c.bytes[12]=(uint8_t)(stall_rpm>>8); c.bytes[13]=(uint8_t)stall_rpm;
    c.bytes[14]=(uint8_t)(stall_ma>>8); c.bytes[15]=(uint8_t)stall_ma;
    c.bytes[16]=(uint8_t)(stall_ms>>8); c.bytes[17]=(uint8_t)stall_ms;
    return c;
}
EmmCommand Emm_HomeBackoff(uint8_t address, uint16_t tenths_degree)
{
    EmmCommand c=command(address,EMM_HOME_BACKOFF,7u);
    c.bytes[2]=0xACu; c.bytes[4]=(uint8_t)(tenths_degree>>8); c.bytes[5]=(uint8_t)tenths_degree;
    return c;
}
EmmCommand Emm_Home(uint8_t address)
{ EmmCommand c=command(address,EMM_HOME,5u); c.bytes[2]=2u; return c; }
EmmCommand Emm_HomeAbort(uint8_t address)
{ EmmCommand c=command(address,EMM_HOME_ABORT,4u); c.bytes[2]=0x48u; return c; }

void EmmParser_Init(EmmParser *p, uint8_t address)
{
    memset(p, 0, sizeof(*p)); p->address = address;
}

static uint8_t frame_length(uint8_t code)
{
    switch (code) {
    case EMM_VERSION: return 7u;
    case EMM_POSITION: return 8u;
    case EMM_SPEED: return 6u;
    case EMM_HOME_STATUS: return 5u;
    case EMM_HOME: case EMM_HOME_ABORT: case EMM_HOME_CONFIG: case EMM_HOME_BACKOFF:
    case EMM_STATUS: case EMM_ENABLE: case EMM_VELOCITY:
    case EMM_MOVE: case EMM_STOP: case EMM_SYNC: return 4u;
    default: return 0u;
    }
}

static void discard(EmmParser *p, uint8_t count)
{
    p->used = (uint8_t)(p->used - count);
    memmove(p->bytes, p->bytes + count, p->used);
}

/*
 * 把已满足长度/帧尾要求的应答解为结构体，检查符号位及允许的结果码。
 * 速度/位置采用独立符号位和无符号幅值，避免把位置幅值误当有符号 32 位数。
 */
static int decode(const uint8_t *b, EmmFrame *f)
{
    uint32_t magnitude;
    memset(f, 0, sizeof(*f)); f->address = b[0]; f->code = b[1];
    if (b[1] == EMM_POSITION || b[1] == EMM_SPEED) {
        if (b[2] > 1u) return 0;
        if (b[1] == EMM_POSITION) {
            magnitude = ((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16) |
                        ((uint32_t)b[5] << 8) | b[6];
            f->position = b[2] ? -(int64_t)magnitude : (int64_t)magnitude;
        } else {
            magnitude = ((uint32_t)b[3] << 8) | b[4];
            f->speed = b[2] ? -(int32_t)magnitude : (int32_t)magnitude;
        }
    } else if (b[1] == EMM_HOME_STATUS) { f->home_flags=b[2]; f->flags=b[3]; }
    else if (b[1] == EMM_STATUS) f->flags = b[2];
    else if (b[1] == EMM_VERSION) {
        f->firmware = (uint16_t)(((uint16_t)b[2] << 8) | b[3]);
        f->hardware = (uint16_t)(((uint16_t)b[4] << 8) | b[5]);
    }
    else {
        if (b[2] != 2u && b[2] != 0x12u && b[2] != 0x22u &&
            b[2] != 0xE2u && b[2] != 0xEEu && b[2] != 0x9Fu) return 0;
        f->result = b[2];
    }
    return 1;
}

/*
 * 支持半帧、粘帧及噪声：按地址/功能码确定长度，不完整则保留；
 * 坏候选逐字节丢弃重新同步，完整合法帧通过回调交给服务层做事务匹配。
 */
void EmmParser_Feed(EmmParser *p, const uint8_t *data, size_t length,
                    EmmFrameHandler handler, void *context)
{
    size_t i;
    uint8_t wanted;
    EmmFrame frame;
    for (i = 0; i < length; ++i) {
        if (p->used == sizeof(p->bytes)) { discard(p, 1u); ++p->rejected; }
        p->bytes[p->used++] = data[i];
        while (p->used >= 2u) {
            wanted = frame_length(p->bytes[1]);
            if ((p->address ? p->bytes[0] != p->address :
                 (p->bytes[0] < 1u || p->bytes[0] > 7u)) || !wanted) {
                discard(p, 1u); ++p->rejected; continue;
            }
            if (p->used < wanted) break;
            if (p->bytes[wanted - 1u] != 0x6Bu || !decode(p->bytes, &frame)) {
                discard(p, 1u); ++p->rejected; continue;
            }
            discard(p, wanted);
            if (handler) handler(context, &frame);
        }
    }
}
