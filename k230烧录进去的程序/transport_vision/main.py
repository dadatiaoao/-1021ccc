"""Migration vision entry. Keep the whole package in /sdcard/transport_vision."""
import sys
PACKAGE_DIR = '/sdcard/transport_vision'
if PACKAGE_DIR not in sys.path:
    sys.path.insert(0, PACKAGE_DIR)
from vision_app import run

if __name__ == '__main__':
    run(bench=False)
