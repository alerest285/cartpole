"""Build the portable Windows/MATLAB lab folder with a SHA-256 manifest."""
import argparse
import hashlib
import json
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent


def build(course=None):
    files={}
    for folder in ('cartpole','web','tests','docs','matlab','tools','data/policies'):
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc','.log') and 'results' not in p.parts and p.name!='checkpoint.npz':
                files[p.relative_to(ROOT).as_posix()]=p.read_bytes()
    for name in ('README.md','requirements.txt','setup_lab.cmd','start_sim.cmd','start_hardware.cmd','start_sim.command','data/bench.json'):
        files[name]=(ROOT/name).read_bytes()
    for name in ('data/hardware.json','data/params_fitted.json'):
        p=ROOT/name
        if p.exists(): files[name]=p.read_bytes()
    source=Path(course) if course else ROOT/'course_original'
    if not source.is_dir():
        source=Path.home()/'Downloads/Mission Impossible Cartpole 4.0'
    if source.is_dir():
        for p in source.iterdir():
            if p.is_file() and p.suffix in ('.m','.slx','.pdf'):
                files['course_original/'+p.name]=p.read_bytes()
    manifest={name:hashlib.sha256(body).hexdigest() for name,body in sorted(files.items())}
    files['MANIFEST.sha256.json']=(json.dumps(manifest,indent=2)+'\n').encode()
    out=ROOT/'dist/cartpole-lab.zip'; out.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,body in sorted(files.items()): archive.writestr('cartpole-lab/'+name,body)
    with zipfile.ZipFile(out) as archive:
        assert archive.testzip() is None
        for name,digest in manifest.items():
            assert hashlib.sha256(archive.read('cartpole-lab/'+name)).hexdigest()==digest
    print(f'{out}: {len(files)} files, {out.stat().st_size:,} bytes; manifest verified')


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--course')
    build(parser.parse_args().course)
