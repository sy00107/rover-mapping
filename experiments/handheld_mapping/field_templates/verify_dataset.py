"""Verify a received dataset using Python standard library only."""
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
for record in manifest['files']:
    path=(root/record['path']).resolve()
    if not path.is_relative_to(root) or not path.is_file():raise SystemExit('Invalid/missing file: '+record['path'])
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024*1024),b''):digest.update(chunk)
    if path.stat().st_size!=record['size'] or digest.hexdigest()!=record['sha256']:
        raise SystemExit('Checksum mismatch: '+record['path'])
print('Verified',len(manifest['files']),'files')
