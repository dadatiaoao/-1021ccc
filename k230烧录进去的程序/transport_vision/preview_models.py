"""Run THIS file in CanMV to compare both models. No serial transmission."""
import sys
# Explicit preview-only entry; unlike /sdcard/main.py this never opens UART.
PACKAGE_DIR = '/sdcard/transport_vision'
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)
from vision_app import run
run(bench=True)
