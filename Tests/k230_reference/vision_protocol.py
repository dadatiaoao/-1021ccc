"""K230 RPIP wire format; the current F429 project still needs its peer.
AA LEN CMD PARAMS CHECKSUM; LEN includes CMD; checksum is SUM, not XOR.
Pure Python / MicroPython so the real host decoder can test its output.
"""
TASKCODE = 0x10
FOUND = 0x11
NOTFOUND = 0x12
PLACE_CHECK = 0x13
HEARTBEAT = 0x15
RING = 0x16
OBSERVATION = 0x17  # Opt-in only after SET_CONTEXT; 12 parameter bytes.
SCAN = 0x21
STOP = 0x22
SET_COLOR = 0x23
RESCAN = 0x24
SET_TARGET = 0x25
SET_CONTEXT = 0x26  # u16 request_id, u16 camera_pose_id; enables OBSERVATION.
ACK = 0x81

FLAG_FOUND = 1
FLAG_STABLE = 2
FLAG_PICK_READY = 4
FLAG_FULL_BOX = 8
FLAG_AMBIGUOUS = 16
FLAG_CALIBRATED = 32
FLAG_REFINED = 64
FLAG_STALE = 128


def frame(cmd, params=b''):
    payload = bytes([cmd]) + bytes(params)
    if not 1 <= len(payload) <= 13:
        raise ValueError('RPIP payload length must be 1..13')
    return bytes([0xAA, len(payload)]) + payload + bytes([sum(payload) & 255])


def u16(value):
    if not 0 <= value <= 65535:
        raise ValueError('unsigned field out of range')
    return bytes([value >> 8, value & 255])


def found(color, cx, cy, offset_tenths=0, distance_cm=0):
    if not -32768 <= offset_tenths <= 32767:
        raise ValueError('signed field out of range')
    return frame(FOUND, bytes([color]) + u16(cx) + u16(cy) +
                 u16(offset_tenths & 65535) + u16(distance_cm))


class Parser:
    def __init__(self):
        self.buf = bytearray()

    def reset(self):
        self.buf = bytearray()

    def feed(self, data):
        result = []
        for byte in data:
            self.buf.append(byte)
            while self.buf:
                if self.buf[0] != 0xAA:
                    self.buf = self.buf[1:]
                    continue
                if len(self.buf) < 2:
                    break
                length = self.buf[1]
                if not 1 <= length <= 13:
                    self.buf = self.buf[1:]
                    continue
                total = length + 3
                if len(self.buf) < total:
                    break
                payload = self.buf[2:2 + length]
                if sum(payload) & 255 == self.buf[total - 1]:
                    result.append((payload[0], bytes(payload[1:])))
                    self.buf = self.buf[total:]
                else:
                    self.buf = self.buf[1:]  # MicroPython bytearray cannot delete items.
        return result
