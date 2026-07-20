import hashlib, json, os, tempfile
from datetime import datetime, timezone

def now(): return datetime.now(timezone.utc).isoformat()
def digest(path):
    h=hashlib.sha256()
    with open(os.path.expanduser(path),'rb') as f:
        for b in iter(lambda:f.read(65536),b''): h.update(b)
    return h.hexdigest()
def atomic_json(path, value):
    folder=os.path.dirname(path)
    if not os.path.isdir(folder): os.makedirs(folder)
    fd,tmp=tempfile.mkstemp(prefix='.tmp-', dir=folder)
    try:
        with os.fdopen(fd,'w') as f: json.dump(value,f,ensure_ascii=False,indent=2,sort_keys=True); f.flush(); os.fsync(f.fileno())
        os.rename(tmp,path)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
def load_json(path, default=None):
    try:
        with open(path) as f:return json.load(f)
    except (IOError,ValueError): return default
