"""PC only: explicit deployment allowlist; no legacy/locked/test scripts on SD."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def build(destination):
    destination = Path(destination).resolve()
    if destination.exists():
        raise ValueError('Choose a new output directory; existing outputs are preserved')
    subprocess.run([sys.executable,'-B',str(ROOT/'tools/check_install.py'),str(ROOT)],check=True)
    files = [ROOT/'main.py',ROOT/'README.md']
    for pattern in ('transport_vision/*.py','transport_vision/models/*','libs/*.py',
                    'res/font/*','tools/*.py','docs/*.md','calibration/*.json'):
        files.extend(ROOT.glob(pattern))
    files = sorted({p for p in files if p.is_file()})
    destination.mkdir(parents=True)
    sd = destination/'sdcard'
    hashes = {}
    for source in files:
        relative = source.relative_to(ROOT)
        target = sd/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
        hashes[relative.as_posix()] = hashlib.sha256(target.read_bytes()).hexdigest()
    archive = destination/'K230_YOLO_视觉完善版.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for relative in hashes:z.write(sd/relative,relative)
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:raise ValueError('Corrupt deployment ZIP')
        for relative,digest in hashes.items():
            if hashlib.sha256(z.read(relative)).hexdigest() != digest:
                raise ValueError('Deployment mismatch: '+relative)
    (destination/'deployment-manifest.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8')
    print('PACKAGE_OK',len(files),'files',str(archive))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out',required=True)
    build(parser.parse_args().out)
