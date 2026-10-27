"""Generate C golden bytes from the checked-in K230 protocol reference."""
import importlib.util
from pathlib import Path
source = Path(__file__).resolve().parent / 'k230_reference/vision_protocol.py'
spec = importlib.util.spec_from_file_location('k230_protocol', source)
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
cases = {
    'gold_heartbeat': p.frame(p.HEARTBEAT),
    'gold_found': p.found(1, 640, 360, -123, 0),
    'gold_task': p.frame(p.TASKCODE, [1,5,6,1,2,3,5,1,6,2,3,1]),
    'gold_ring': p.frame(p.RING, bytes([2])+p.u16(640)+p.u16(360)),
    'gold_context': p.frame(p.SET_CONTEXT, p.u16(1)+p.u16(2)),
    'gold_observation': p.frame(p.OBSERVATION, p.u16(1)+p.u16(2)+p.u16(65535)+
                              bytes([p.FLAG_FOUND|p.FLAG_STABLE|p.FLAG_REFINED,2])+p.u16(640)+p.u16(360)),
}
dest = Path(__file__).resolve().parents[1] / 'Output/host-tests/vision_golden.h'
dest.parent.mkdir(parents=True, exist_ok=True)
dest.write_text('/* Generated from actual K230 source; do not edit. */\n'+''.join(
    'static const uint8_t %s[]={%s};\n' % (k, ','.join('0x%02X'%x for x in v))
    for k,v in cases.items()), encoding='ascii')
print('PASS: golden wire fixtures generated from K230 source')
