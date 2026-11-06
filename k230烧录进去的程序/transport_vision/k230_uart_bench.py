"""Run only with the standalone F429 vision_uart_bench firmware.
Uses the integrated YOLO app; traces TX and test-only host RX confirmation.
Does not install startup files or control actuators.
"""
import sys
import time
import machine
sys.path.insert(0, '/sdcard/transport_vision')
import vision_protocol as wire
from vision_app import run

RealUART = machine.UART


class TracedUART:
    UART1 = RealUART.UART1
    UART2 = RealUART.UART2

    def __init__(self, *args, **kwargs):
        self.inner = RealUART(*args, **kwargs)
        self.parser = wire.Parser()
        self.last_tx_log = time.ticks_ms()
        self.last_rx_log = time.ticks_ms()
        self.tx_found = 0

    def write(self, packet):
        count = self.inner.write(packet)
        now = time.ticks_ms()
        if len(packet) >= 4 and packet[2] == wire.FOUND:
            self.tx_found += 1
            if time.ticks_diff(now, self.last_tx_log) >= 1000:
                print('K230_TX_FOUND', self.tx_found, 'color', packet[3],
                      'xy', (packet[4] << 8) | packet[5], (packet[6] << 8) | packet[7])
                self.last_tx_log = now
        return count

    def read(self):
        data = self.inner.read()
        if data:
            for cmd, params in self.parser.feed(data):
                # Standalone host constructs this AFTER the actual C decoder.
                # It is deliberately different from K230's transmitted FOUND.
                if cmd == 0x91 and len(params) == 6:
                    now = time.ticks_ms()
                    if time.ticks_diff(now, self.last_rx_log) >= 1000:
                        print('HOST_RX_CONFIRMED', 'color', params[0],
                              'xy', (params[1] << 8) | params[2], (params[3] << 8) | params[4],
                              'found_seq_mod256', params[5], 'valid', params[0] != 0)
                        self.last_rx_log = now
        return data

    def deinit(self):
        self.inner.deinit()


run(bench=False, test_color=6, uart_factory=TracedUART)
