"""Explicit UART entry, waits for STM32 commands. Not a boot installer."""
import sys
PACKAGE_DIR = '/sdcard/transport_vision'
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)
from vision_app import run
run(bench=False)
